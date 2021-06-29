#include <auto_uav/gcsLogi.hpp>

namespace gcsGui {

gcsLogi::gcsLogi() : altitude(2.0),
                     speed(0.05),
                     is_connected(false) {
    connect_client = nh_.serviceClient<auto_uav_msgs::Connect>(initServiceName);
    arm_client = nh_.serviceClient<mavros_msgs::CommandBool>(armServiceName);
    set_mode_client = nh_.serviceClient<mavros_msgs::SetMode>(modeSwitchName);
    takeoff_client = nh_.serviceClient<auto_uav_msgs::Takeoff>(takeoffServiceName);
    land_client = nh_.serviceClient<auto_uav_msgs::Land>(landServiceName);

    // subscriber
    state_sub_ = nh_.subscribe(uavStateTopic, 10, &gcsLogi::cb_state, this);
}

void gcsLogi::cb_state(const mavros_msgs::State::ConstPtr& msgPtr) {
    current_state_ = *msgPtr;
}

}  // namespace gcsGui