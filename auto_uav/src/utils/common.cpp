#include "utils/common.h"

#include <cmath>
#include <fstream>
#include <iostream>
#include <limits>
#include <vector>

namespace avoidance {

float getYawFromQuaternion(const Eigen::Quaternionf q) {
    float siny_cosp = 2.f * (q.w() * q.z() + q.x() * q.y());
    float cosy_cosp = 1.f - 2.f * (q.y() * q.y() + q.z() * q.z());
    float yaw = atan2(siny_cosp, cosy_cosp);

    return yaw * RAD_TO_DEG;
}

float getPitchFromQuaternion(const Eigen::Quaternionf q) {
    float pitch = 0.f;
    float sinp = 2.f * (q.w() * q.y() - q.z() * q.x());
    if (fabs(sinp) >= 1.f) {
        pitch = copysign(M_PI / 2.f, sinp);  // use PI/2 if out of range
    } else {
        pitch = asin(sinp);
    }
    return pitch * RAD_TO_DEG;
}

float wrapAngleToPlusMinusPI(float angle) {
    return angle - 2.0f * M_PI_F * std::floor(angle / (2.0f * M_PI_F) + 0.5f);
}

float wrapAngleToPlusMinus180(float angle) {
    return angle - 360.f * std::floor(angle / 360.f + 0.5f);
}

double getAngularVelocity(float desired_yaw, float curr_yaw) {
    desired_yaw = wrapAngleToPlusMinusPI(desired_yaw);
    float yaw_vel1 = desired_yaw - curr_yaw;
    float yaw_vel2;
    // finds the yaw vel for the other yaw direction
    if (yaw_vel1 > 0.0f) {
        yaw_vel2 = -(2.0f * M_PI_F - yaw_vel1);
    } else {
        yaw_vel2 = 2.0f * M_PI_F + yaw_vel1;
    }

    // check which yaw direction is shorter
    float vel = (std::abs(yaw_vel1) <= std::abs(yaw_vel2)) ? yaw_vel1 : yaw_vel2;
    return 0.5 * static_cast<double>(vel);
}

// calculate the yaw for the next waypoint
float nextYaw(const Eigen::Vector3f& u, const Eigen::Vector3f& v) {
    float dx = v.x() - u.x();
    float dy = v.y() - u.y();

    return atan2(dy, dx);
}

// PolarPoint cartesianToPolarFCU(const Eigen::Vector3f& pos, const Eigen::Vector3f& origin) {
//     PolarPoint p = cartesianToPolarHistogram(pos, origin);
//     p.z = -p.z + 90.0f;
//     p.e = -p.e;
//     wrapPolar(p);
//     return p;
// }

// PolarPoint cartesianToPolarFCU(const pcl::PointXYZ& p) {
//     return cartesianToPolarFCU(Eigen::Vector3f(p.x, p.y, p.z), Eigen::Vector3f(0.0f, 0.0f,
//     0.0f));
// }

void createPoseMsg(Eigen::Vector3f& out_waypt, Eigen::Quaternionf& out_q,
                   const Eigen::Vector3f& in_waypt, float yaw) {
    out_waypt = in_waypt;
    float roll = 0.0f, pitch = 0.0f;
    out_q = Eigen::AngleAxisf(roll, Eigen::Vector3f::UnitX()) *
            Eigen::AngleAxisf(pitch, Eigen::Vector3f::UnitY()) *
            Eigen::AngleAxisf(yaw, Eigen::Vector3f::UnitZ());
}

void transformToTrajectory(mavros_msgs::Trajectory& obst_avoid, geometry_msgs::PoseStamped pose,
                           geometry_msgs::Twist vel) {
    obst_avoid.header.stamp = ros::Time::now();
    obst_avoid.type = 0;  // MAV_TRAJECTORY_REPRESENTATION::WAYPOINTS
    obst_avoid.point_1.position.x = pose.pose.position.x;
    obst_avoid.point_1.position.y = pose.pose.position.y;
    obst_avoid.point_1.position.z = pose.pose.position.z;
    obst_avoid.point_1.velocity.x = vel.linear.x;
    obst_avoid.point_1.velocity.y = vel.linear.y;
    obst_avoid.point_1.velocity.z = vel.linear.z;
    obst_avoid.point_1.acceleration_or_force.x = NAN;
    obst_avoid.point_1.acceleration_or_force.y = NAN;
    obst_avoid.point_1.acceleration_or_force.z = NAN;
    obst_avoid.point_1.yaw = tf::getYaw(pose.pose.orientation);
    obst_avoid.point_1.yaw_rate = -vel.angular.z;

    fillUnusedTrajectoryPoint(obst_avoid.point_2);
    fillUnusedTrajectoryPoint(obst_avoid.point_3);
    fillUnusedTrajectoryPoint(obst_avoid.point_4);
    fillUnusedTrajectoryPoint(obst_avoid.point_5);

    obst_avoid.time_horizon = {NAN, NAN, NAN, NAN, NAN};

    obst_avoid.point_valid = {true, false, false, false, false};
}

void transformToTrajectory(mavros_msgs::Trajectory& obst_avoid,
                           trajectory_msgs::MultiDOFJointTrajectory path) {
    const size_t n_commands = path.points.size();
    
    if (n_commands < 1) {
        ROS_WARN_STREAM("Got MultiDOFJointTrajectory message, but message has no points.");
        return;
    }

    geometry_msgs::Transform transform = path.points[0].transforms[0];
    geometry_msgs::PoseStamped pose = toPoseStamped(transform.translation, transform.rotation);
    geometry_msgs::Twist vel = path.points[0].velocities[0];

    transformToTrajectory(obst_avoid, pose, vel);
}

void fillUnusedTrajectoryPoint(mavros_msgs::PositionTarget& point) {
    point.position.x = NAN;
    point.position.y = NAN;
    point.position.z = NAN;
    point.velocity.x = NAN;
    point.velocity.y = NAN;
    point.velocity.z = NAN;
    point.acceleration_or_force.x = NAN;
    point.acceleration_or_force.y = NAN;
    point.acceleration_or_force.z = NAN;
    point.yaw = NAN;
    point.yaw_rate = NAN;
}

}  // namespace avoidance