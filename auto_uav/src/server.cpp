#include <auto_uav/server.hpp>

namespace Server {

server::server() : nh_(),
                   rate_(20.0) {
    // Publicher
    pubSet_.pubSetPose = nh_.advertise<geometry_msgs::PoseStamped>(uavPubPoseTopic, 10);

    // Subscriber
    subSet_.subPose = nh_.subscribe(uavPoseTopic, 10, &server::cbPose, this);
    subSet_.subState = nh_.subscribe(uavStateTopic, 10, &server::cbState, this);

    // Service
    servSet_.initServer = nh_.advertiseService(initServiceName, &server::cbInit, this);
    servSet_.landServer = nh_.advertiseService(landServiceName, &server::cbLand, this);
    servSet_.takeoffServer = nh_.advertiseService(takeoffServiceName, &server::cbTakeoff, this);
}

void server::cbPose(const geometry_msgs::PoseStamped::ConstPtr& msgPtr) {
    state_.curPose = *msgPtr;
}

void server::cbState(const mavros_msgs::State::ConstPtr& msgPtr) {
    state_.curState = *msgPtr;
}

bool server::cbInit(auto_uav_msgs::Connect::Request& req, auto_uav_msgs::Connect::Response& resp) {
    sys_connect();
    if (status_.isInit) {
        resp.is_success = true;
        return true;
    }
    return false;
}

bool server::cbTakeoff(auto_uav_msgs::Takeoff::Request& req, auto_uav_msgs::TakeoffResponse& resp) {
    // Takeoff parameters
    double height = req.height;
    double speedToAir = req.speed;
    double takeoffTime = height / speedToAir;

    state_.curSetPose.pose.position.x = 0;
    state_.curSetPose.pose.position.y = 0;
    state_.curSetPose.pose.position.z = height;

    resp.is_success = true;
    return true;
}

bool server::cbLand(auto_uav_msgs::LandRequest& req, auto_uav_msgs::LandResponse& resp) {
    // Takeoff parameters
    double height = req.ground;
    double speedToAir = req.speed;
    double takeoffTime = height / speedToAir;

    state_.curSetPose.pose.position.x = 0;
    state_.curSetPose.pose.position.y = 0;
    state_.curSetPose.pose.position.z = height;

    resp.is_success = true;
    return true;
}

void server::sys_connect() {
    // wait for FCU connection
    while (ros::ok() && !state_.curState.connected) {
        ROS_INFO_NAMED("Server", "Wait");
        ros::spinOnce();
        rate_.sleep();
    }
    ROS_INFO_NAMED("Server", "FCU connected");

    //send a few setpoints before starting
    for (int i = 50; ros::ok() && i > 0; --i) {
        pubSet_.pubSetPose.publish(state_.curPose);
        ros::spinOnce();
        rate_.sleep();
    }
    ROS_INFO_NAMED("Server", "System enabled");
    status_.isInit = true;
}

void server::publish() {
    pubSet_.pubSetPose.publish(state_.curSetPose);
}

bool server::ckState() {}

void server::run() {
    while (ros::ok()) {
        if (status_.isInit) {
            publish();
        }        
        ros::spinOnce();
        rate_.sleep();
    }
}

}  // namespace Server