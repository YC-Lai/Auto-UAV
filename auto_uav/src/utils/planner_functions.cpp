#include "utils/planner_functions.h"

#include "utils/common.h"

namespace avoidance {

bool getSetpointFromPath(const std::vector<Eigen::Vector3f>& path, const ros::Time& path_generation_time,
                         float velocity, const ros::Time& current_time, Eigen::Vector3f& setpoint) {
    int i = path.size();
    // path contains nothing meaningful
    if (i < 2) {
        return false;
    }

    // path only has one segment: return end of that segment as setpoint
    if (i == 2) {
        setpoint = path[0];
        return true;
    }

    // step through the path until the point where we should be if we had traveled perfectly with velocity along it
    Eigen::Vector3f path_segment = path[i - 3] - path[i - 2];
    float distance_left = (current_time - path_generation_time).toSec() * velocity;
    setpoint = path[i - 2] + (distance_left / path_segment.norm()) * path_segment;
    for (i = path.size() - 3; i > 0 && distance_left > path_segment.norm(); --i) {
        distance_left -= path_segment.norm();
        path_segment = path[i - 1] - path[i];
        setpoint = path[i] + (distance_left / path_segment.norm()) * path_segment;
    }
    // If we excited because we're past the last node of the path, the path is no longer valid!
    return distance_left < path_segment.norm();
}

}  // namespace avoidance