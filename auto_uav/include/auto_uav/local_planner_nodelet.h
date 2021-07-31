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

   private:
    ros::NodeHandle nh_;

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

    /**
     * @brief     reads parameters from launch file and yaml file
     **/
    void readParams();
};

}  // namespace avoidance

#endif  // LOCAL_PLANNER_LOCAL_PLANNER_NODE_H