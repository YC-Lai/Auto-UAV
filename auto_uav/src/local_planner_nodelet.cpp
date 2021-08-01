#include "auto_uav/local_planner_nodelet.h"

#include "auto_uav/waypoint_generator.h"

namespace avoidance {

void LocalPlannerNodelet::onInit() {
    NODELET_DEBUG("Initializing nodelet...");
    InitializeNodelet();

    startNode();
}

void LocalPlannerNodelet::InitializeNodelet() {
    nh_ = ros::NodeHandle("~");

    wp_generator_.reset(new WaypointGenerator());

    readParams();

    tf_listener_ = new tf::TransformListener(
        ros::Duration(tf::Transformer::DEFAULT_CACHE_TIME), true);

    // initialize standard subscribers
    pose_sub_ = nh_.subscribe<const geometry_msgs::PoseStamped&>(
        "/mavros/local_position/pose", 1, &LocalPlannerNodelet::positionCallback,
        this);
    velocity_sub_ = nh_.subscribe<const geometry_msgs::TwistStamped&>(
        "/mavros/local_position/velocity_local", 1,
        &LocalPlannerNodelet::velocityCallback, this);
    state_sub_ = nh_.subscribe("/mavros/state", 1,
                               &LocalPlannerNodelet::stateCallback, this);
    fcu_input_sub_ = nh_.subscribe(
        "/mavros/trajectory/desired", 1, &LocalPlannerNodelet::fcuInputGoalCallback, this);
    goal_topic_sub_ = nh_.subscribe(
        "/input/goal_position", 1, &LocalPlannerNodelet::updateGoalCallback, this);

    mavros_vel_setpoint_pub_ = nh_.advertise<geometry_msgs::Twist>(
        "/mavros/setpoint_velocity/cmd_vel_unstamped", 10);
    mavros_pos_setpoint_pub_ = nh_.advertise<geometry_msgs::PoseStamped>(
        "/mavros/setpoint_position/local", 10);
    mavros_obstacle_free_path_pub_ = nh_.advertise<mavros_msgs::Trajectory>(
        "/mavros/trajectory/generated", 10);
}

void LocalPlannerNodelet::startNode() {
    ros::TimerOptions timer_options(ros::Duration(spin_dt_), boost::bind(&LocalPlannerNodelet::cmdLoopCallback, this, _1),
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
    nh_private_.param<bool>(nodelet::Nodelet::getName() + "/accept_goal_input_topic", accept_goal_input_topic_, false);
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

}  // namespace avoidance