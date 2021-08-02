#ifndef LOCAL_PLANNER_LOCAL_PLANNER_NODE_H
#define LOCAL_PLANNER_LOCAL_PLANNER_NODE_H

#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/TransformStamped.h>
#include <mavros_msgs/Altitude.h>
#include <mavros_msgs/CompanionProcessStatus.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/Trajectory.h>
#include <nodelet/nodelet.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>

#include <Eigen/Core>
#include <Eigen/Dense>

#include "utils/avoidance_node.h"
#include "utils/avoidance_output.h"
#include "utils/common.h"

namespace avoidance {

class WaypointGenerator;

class LocalPlannerNodelet : public nodelet::Nodelet {
   public:
    LocalPlannerNodelet();
    virtual ~LocalPlannerNodelet();

    /**
     * @brief     Initializer for nodeletes
     **/
    virtual void onInit();

    /**
     * @brief     Initialize ROS components
     **/
    void InitializeNodelet();

    /**
     * @brief     start spinners
     **/
    void startNode();

    std::unique_ptr<WaypointGenerator> wp_generator_;
    std::unique_ptr<ros::AsyncSpinner> cmdloop_spinner_;
    std::unique_ptr<avoidance::AvoidanceNode> avoidance_node_;
    bool position_received_ = false;

   private:
    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;

    ros::Timer cmdloop_timer_;
    ros::CallbackQueue cmdloop_queue_;
    double spin_dt_ = 0.1;

    // Publishers
    ros::Publisher mavros_pos_setpoint_pub_;
    ros::Publisher mavros_vel_setpoint_pub_;
    ros::Publisher mavros_obstacle_free_path_pub_;
    ros::Publisher mavros_system_status_pub_;

    // Subscribers
    ros::Subscriber pose_sub_;
    ros::Subscriber velocity_sub_;
    ros::Subscriber state_sub_;
    ros::Subscriber fcu_input_sub_;
    ros::Subscriber goal_topic_sub_;

    bool new_goal_ = false;

    std::mutex waypoints_mutex_;
    Eigen::Vector3f newest_waypoint_position_;
    Eigen::Vector3f last_waypoint_position_;
    Eigen::Vector3f newest_adapted_waypoint_position_;
    Eigen::Vector3f last_adapted_waypoint_position_;
    Eigen::Vector3f newest_position_;
    Eigen::Quaternionf newest_orientation_;
    Eigen::Vector3f last_position_;
    Eigen::Quaternionf last_orientation_;
    Eigen::Vector3f velocity_;
    Eigen::Vector3f desired_velocity_;
    Eigen::Vector3f goal_position_;
    Eigen::Vector3f prev_goal_position_;

    NavigationState nav_state_ = NavigationState::none;
    bool armed_ = false;
    bool hover_;
    bool accept_goal_input_topic_;
    ros::Time start_time_;
    ros::Time last_wp_time_;
    bool is_land_waypoint_{false};
    bool is_takeoff_waypoint_{false};
    geometry_msgs::PoseStamped goal_mission_item_msg_;
    mavros_msgs::Altitude ground_distance_msg_;

    float desired_yaw_setpoint_{NAN};
    float desired_yaw_speed_setpoint_{NAN};

    /**
     * @brief     reads parameters from launch file and yaml file
     **/
    void readParams();

    /**
     * @brief     callaback for vehicle position and orientation
     * @param[in] msg, vehicle position and orientation in ENU frame
     **/
    void positionCallback(const geometry_msgs::PoseStamped& msg);

    /**
     * @brief     callaback for vehicle velocity
     * @param[in] msg, vehicle velocity message
     **/
    void velocityCallback(const geometry_msgs::TwistStamped& msg);

    /**
     * @brief     callaback for vehicle state
     * @param[in] msg, vehicle position and orientation in ENU frame
     **/
    void stateCallback(const mavros_msgs::State& msg);
    void cmdLoopCallback(const ros::TimerEvent& event);

    /**
     * @brief      calculates position and velocity setpoints and sends to the FCU
     * @param[in]  hover, true if the vehicle is loitering
     **/
    void calculateWaypoints(bool hover);

    /**
     * @brief      set avoidance system status
     **/
    void setSystemStatus(MAV_STATE state);

    /**
     * @brief      check healthiness of the avoidance system to trigger failsafe in
     *             the FCU
     * @param[in]  since_last_cloud, time elapsed since the last waypoint was
     *             published to the FCU
     * @param[in]  since_start, time elapsed since staring the node
     * @param[out] planner_is_healthy, true if the planner is running without
     *errors
     * @param[out] hover, true if the vehicle is hovering
     **/
    void checkFailsafe(ros::Duration since_last_cloud, ros::Duration since_start, bool& hover);

    /**
     * @brief     callaback for setting the goal from the FCU Mission Waypoints
     * @param[in] msg, current and next position goals
     **/
    void fcuInputGoalCallback(const mavros_msgs::Trajectory& msg);
};

}  // namespace avoidance

#endif  // LOCAL_PLANNER_LOCAL_PLANNER_NODE_H