#ifndef GCSGUI_GCSLOGI_H
#define GCSGUI_GCSLOGI_H

#include <auto_uav_msgs/Connect.h>
#include <auto_uav_msgs/Land.h>
#include <auto_uav_msgs/Takeoff.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <ros/ros.h>

#include <string>

#define armServiceName "mavros/cmd/arming"
#define modeSwitchName "mavros/set_mode"
#define uavStateTopic "mavros/state"
#define uavPoseTopic "mavros/local_position/pose"  // Local position from FCU. (FCU: flight control unit)
#define uavPubPoseTopic "mavros/setpoint_position/local"
#define initServiceName "auto_uav/init"
#define takeoffServiceName "auto_uav/takeoff"
#define landServiceName "auto_uav/land"

namespace gcsGui {

class gcsLogi {
   public:
    gcsLogi();

    std::string sendMessage();

    // some service clients
    ros::ServiceClient connect_client;
    ros::ServiceClient arm_client;
    ros::ServiceClient set_mode_client;
    ros::ServiceClient takeoff_client;
    ros::ServiceClient land_client;
    //ros::ServiceClient keyboard_client_;
    //ros::ServiceClient planner_client_;
    //ros::ServiceClient kill_client_;

    bool is_connected;
    // define flight altitude and flight speed
    const double altitude;
    const double speed;

   private:
    ros::NodeHandle nh_;

    ros::Subscriber state_sub_;

    void cb_state(const mavros_msgs::State::ConstPtr& msgPtr);
    mavros_msgs::State current_state_;
};

}  // namespace gcsGui

#endif