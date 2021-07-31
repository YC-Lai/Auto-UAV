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

    
}

}  // namespace avoidance