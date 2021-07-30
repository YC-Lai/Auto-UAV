#ifndef LOCAL_PLANNER_FUNCTIONS_H
#define LOCAL_PLANNER_FUNCTIONS_H

#include <Eigen/Dense>
#include <vector>

#include "utils/common.h"

namespace avoidance {

/**
* @brief      Returns a setpoint that lies on the given path
* @param[in]  vector of nodes defining the path, with the last node of the path at index 0
* @param[in]  ros time of path generation
* @param[in]  velocity, scalar value for the norm of the current vehicle velocity
* @param[out] setpoint on the tree toward which the drone should fly
* @returns    boolean indicating whether the tree was valid
**/
bool getSetpointFromPath(const std::vector<Eigen::Vector3f>& path, const ros::Time& path_generation_time,
                         float velocity, const ros::Time& current_time, Eigen::Vector3f& setpoint);

}  // namespace avoidance

#endif  // LOCAL_PLANNER_FUNCTIONS_H