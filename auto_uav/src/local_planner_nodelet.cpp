#include "auto_uav/local_planner_nodelet.h"

#include <mutex>

#include "auto_uav/waypoint_generator.h"

namespace avoidance {

LocalPlannerNodelet::LocalPlannerNodelet() : spin_dt_(0.1) {}

void LocalPlannerNodelet::onInit() {
    NODELET_DEBUG("Initializing nodelet...");
    InitializeNodelet();
    startNode();
}

void LocalPlannerNodelet::InitializeNodelet() {
    nh_ = ros::NodeHandle("~");
    nh_private_ = ros::NodeHandle("");

    wp_generator_.reset(new WaypointGenerator());
    avoidance_node_.reset(new AvoidanceNode(nh_, nh_private_));

    readParams();

    // initialize standard subscribers
    pose_sub_ = nh_.subscribe<const geometry_msgs::PoseStamped&>(
        "/mavros/local_position/pose", 1, &LocalPlannerNodelet::positionCallback, this);
    velocity_sub_ = nh_.subscribe<const geometry_msgs::TwistStamped&>(
        "/mavros/local_position/velocity_local", 1, &LocalPlannerNodelet::velocityCallback, this);
    state_sub_ = nh_.subscribe("/mavros/state", 1, &LocalPlannerNodelet::stateCallback, this);
    fcu_input_sub_ = nh_.subscribe("/mavros/trajectory/desired", 1,
                                   &LocalPlannerNodelet::fcuInputGoalCallback, this);
                                   
    mavros_vel_setpoint_pub_ =
        nh_.advertise<geometry_msgs::Twist>("/mavros/setpoint_velocity/cmd_vel_unstamped", 10);
    mavros_pos_setpoint_pub_ =
        nh_.advertise<geometry_msgs::PoseStamped>("/mavros/setpoint_position/local", 10);
    mavros_obstacle_free_path_pub_ =
        nh_.advertise<mavros_msgs::Trajectory>("/mavros/trajectory/generated", 10);
}

void LocalPlannerNodelet::startNode() {
    ros::TimerOptions timer_options(ros::Duration(spin_dt_),
                                    boost::bind(&LocalPlannerNodelet::cmdLoopCallback, this, _1),
                                    &cmdloop_queue_);
    cmdloop_timer_ = nh_.createTimer(timer_options);

    cmdloop_spinner_.reset(new ros::AsyncSpinner(1, &cmdloop_queue_));
    cmdloop_spinner_->start();

    avoidance_node_->init();
}

void LocalPlannerNodelet::readParams() {
    // Parameter from launch file
    Eigen::Vector3d goal_d = goal_position_.cast<double>();
    nh_private_.param<double>(nodelet::Nodelet::getName() + "/goal_x_param", goal_d.x(), 0.0);
    nh_private_.param<double>(nodelet::Nodelet::getName() + "/goal_y_param", goal_d.y(), 0.0);
    nh_private_.param<double>(nodelet::Nodelet::getName() + "/lgoal_z_param", goal_d.z(), 0.0);
    nh_private_.param<bool>(nodelet::Nodelet::getName() + "/accept_goal_input_topic",
                            accept_goal_input_topic_, false);
    goal_position_ = goal_d.cast<float>();

    new_goal_ = true;
}

void LocalPlannerNodelet::positionCallback(const geometry_msgs::PoseStamped& msg) {
    last_position_ = newest_position_;
    newest_position_ = toEigen(msg.pose.position);
    newest_orientation_ = toEigen(msg.pose.orientation);

    position_received_ = true;
}

void LocalPlannerNodelet::velocityCallback(const geometry_msgs::TwistStamped& msg) {
    velocity_ = toEigen(msg.twist.linear);
}

void LocalPlannerNodelet::stateCallback(const mavros_msgs::State& msg) {
    armed_ = msg.armed;

    if (msg.mode == "AUTO.MISSION") {
        nav_state_ = NavigationState::mission;
    } else if (msg.mode == "AUTO.TAKEOFF") {
        nav_state_ = NavigationState::auto_takeoff;
    } else if (msg.mode == "AUTO.LAND") {
        nav_state_ = NavigationState::auto_land;
    } else if (msg.mode == "AUTO.RTL") {
        nav_state_ = NavigationState::auto_rtl;
    } else if (msg.mode == "AUTO.RTGS") {
        nav_state_ = NavigationState::auto_rtgs;
    } else if (msg.mode == "AUTO.LOITER") {
        nav_state_ = NavigationState::auto_loiter;
    } else if (msg.mode == "OFFBOARD") {
        nav_state_ = NavigationState::offboard;
    } else {
        nav_state_ = NavigationState::none;
    }
}

void LocalPlannerNodelet::cmdLoopCallback(const ros::TimerEvent& event) {
    std::lock_guard<std::mutex> lock(waypoints_mutex_);
    hover_ = false;

    // Process callbacks & wait for a position update
    ros::Time start_query_position = ros::Time::now();

    while (!position_received_ && ros::ok()) {
        ros::getGlobalCallbackQueue()->callAvailable(ros::WallDuration(0.1));
        ros::Duration since_query = ros::Time::now() - start_query_position;
        if (since_query > ros::Duration(local_planner_->timeout_termination_)) {
            setSystemStatus(MAV_STATE::MAV_STATE_FLIGHT_TERMINATION);
            if (!position_not_received_error_sent_) {
                // clang-format off
        ROS_WARN("\033[1;33m Planner abort: missing required data from FCU \n \033[0m");
        ROS_WARN("----------------------------- Debugging Info -----------------------------");
        ROS_WARN("Local planner has not received a position from FCU, check the following: ");
        ROS_WARN("1. Check cables connecting PX4 autopilot with onboard computer");
        ROS_WARN("2. Set PX4 parameter MAV_1_MODE to onbard or external vision");
        ROS_WARN("3. Set correct fcu_url in local_planner launch file:");
        ROS_WARN("   Example direct connection to serial port: /dev/ttyUSB0:921600");
        ROS_WARN("   Example connection over mavlink router: udp://:14540@localhost:14557");
        ROS_WARN("--------------------------------------------------------------------------");
                // clang-format on
                position_not_received_error_sent_ = true;
            }
        }
    }

    // Check if all information was received
    ros::Time now = ros::Time::now();
    ros::Duration since_last_cloud = now - last_wp_time_;
    ros::Duration since_start = now - start_time_;

    checkFailsafe(since_last_cloud, since_start, hover_);

    // send waypoint
    if (avoidance_node_->getSystemStatus() == MAV_STATE::MAV_STATE_ACTIVE) {
        calculateWaypoints(hover_);
    }

    position_received_ = false;

    return;
}

void LocalPlannerNodelet::calculateWaypoints(bool hover) {
    bool is_airborne = armed_ && (nav_state_ != NavigationState::none);

    wp_generator_->updateState(newest_position_, newest_orientation_, goal_position_,
                               prev_goal_position_, velocity_, hover, is_airborne, nav_state_,
                               is_land_waypoint_, is_takeoff_waypoint_, desired_velocity_);
    waypointResult result = wp_generator_->getWaypoints();

    Eigen::Vector3f closest_pt = Eigen::Vector3f(NAN, NAN, NAN);
    Eigen::Vector3f deg60_pt = Eigen::Vector3f(NAN, NAN, NAN);
    wp_generator_->getOfftrackPointsForVisualization(closest_pt, deg60_pt);

    last_waypoint_position_ = newest_waypoint_position_;
    newest_waypoint_position_ = result.smoothed_goto_position;
    last_adapted_waypoint_position_ = newest_adapted_waypoint_position_;
    newest_adapted_waypoint_position_ = result.adapted_goto_position;

    // send waypoints to mavros
    mavros_msgs::Trajectory obst_free_path = {};
    transformToTrajectory(obst_free_path, toPoseStamped(result.position_wp, result.orientation_wp),
                          toTwist(result.linear_velocity_wp, result.angular_velocity_wp));
    mavros_pos_setpoint_pub_.publish(toPoseStamped(result.position_wp, result.orientation_wp));

    mavros_obstacle_free_path_pub_.publish(obst_free_path);
}

void LocalPlannerNodelet::fcuInputGoalCallback(const mavros_msgs::Trajectory& msg) {
    bool update = ((avoidance::toEigen(msg.point_2.position) -
                    avoidance::toEigen(goal_mission_item_msg_.pose.position))
                       .norm() > 0.01) ||
                  !std::isfinite(goal_position_(0)) || !std::isfinite(goal_position_(1));
    if ((msg.point_valid[0] == true) && update) {
        new_goal_ = true;
        prev_goal_position_ = goal_position_;
        goal_position_ = toEigen(msg.point_1.position);
        desired_velocity_ = toEigen(msg.point_1.velocity);
        is_land_waypoint_ = (msg.command[0] == static_cast<int>(MavCommand::MAV_CMD_NAV_LAND));
        is_takeoff_waypoint_ =
            (msg.command[0] == static_cast<int>(MavCommand::MAV_CMD_NAV_TAKEOFF));
    }
    if (msg.point_valid[1] == true) {
        goal_mission_item_msg_.pose.position = msg.point_2.position;
        if (msg.command[1] == UINT16_MAX) {
            goal_position_ = toEigen(msg.point_2.position);
            desired_velocity_ << NAN, NAN, NAN;
        }
        desired_yaw_setpoint_ = msg.point_2.yaw;
        desired_yaw_speed_setpoint_ = msg.point_2.yaw_rate;
    }
}

void LocalPlannerNodelet::setSystemStatus(MAV_STATE state) {
    avoidance_node_->setSystemStatus(state);
}

void LocalPlannerNodelet::checkFailsafe(ros::Duration since_last_cloud, ros::Duration since_start,
                                        bool& hover) {
    avoidance_node_->checkFailsafe(since_last_cloud, since_start, hover);
}

}  // namespace avoidance