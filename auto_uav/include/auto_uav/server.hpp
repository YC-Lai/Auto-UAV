#ifndef GCSGUI_SERVER_H
#define GCSGUI_SERVER_H

#include <auto_uav_msgs/Connect.h>
#include <auto_uav_msgs/Land.h>
#include <auto_uav_msgs/Takeoff.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
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

namespace Server {

class server {
    struct Status {
        bool isInit = false;
    };

    struct State {
        geometry_msgs::PoseStamped curPose;  // This pose is from EKF2
        geometry_msgs::PoseStamped curSetPose;
        mavros_msgs::State curState;
    };

    struct Param {
        std::string worldFrame = "/map";
        double yawFromPX4toSensor = 0;
    };

    struct SubscriberSet {
        ros::Subscriber subState;
        ros::Subscriber subPose;
    };

    struct PublisherSet {
        ros::Publisher pubSetPose;
    };

    struct ServerSet {
        ros::ServiceServer initServer;
        ros::ServiceServer takeoffServer;
        ros::ServiceServer landServer;
    };

   private:
    Status status_;
    State state_;
    Param param_;

    SubscriberSet subSet_;
    PublisherSet pubSet_;
    ServerSet servSet_;
    ros::NodeHandle nh_;

    ros::Rate rate_;

    void cbPose(const geometry_msgs::PoseStamped::ConstPtr& msgPtr);
    void cbTwist(const geometry_msgs::TwistStamped::ConstPtr& msgPtr);
    void cbState(const mavros_msgs::State::ConstPtr& msgPtr);

    bool cbInit(auto_uav_msgs::Connect::Request& req, auto_uav_msgs::Connect::Response& resp);
    bool cbTakeoff(auto_uav_msgs::Takeoff::Request& req, auto_uav_msgs::Takeoff::Response& resp);
    bool cbLand(auto_uav_msgs::Land::Request& req, auto_uav_msgs::Land::Response& resp);

    void sys_connect();
    void publish();
    bool ckState();

   public:
    server(/* args */);
    void run();
};

}  // namespace Server

#endif