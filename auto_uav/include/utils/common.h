#ifndef COMMON_H
#define COMMON_H

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <Eigen/Core>
#include <Eigen/Dense>

#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <mavros_msgs/Trajectory.h>
#include <tf/transform_listener.h>

enum class MAV_STATE {
  MAV_STATE_UNINIT,
  MAV_STATE_BOOT,
  MAV_STATE_CALIBRATIN,
  MAV_STATE_STANDBY,
  MAV_STATE_ACTIVE,
  MAV_STATE_CRITICAL,
  MAV_STATE_EMERGENCY,
  MAV_STATE_POWEROFF,
  MAV_STATE_FLIGHT_TERMINATION,
};

enum class NavigationState {
  mission,
  auto_takeoff,
  auto_land,
  auto_rtl,
  auto_rtgs,
  offboard,
  auto_loiter,
  none,
};

enum class MavCommand {
  MAV_CMD_NAV_LAND = 21,
  MAV_CMD_NAV_TAKEOFF,
  MAV_CMD_DO_CHANGE_SPEED = 178,
};

struct PolarPoint {
  PolarPoint(float e_, float z_, float r_) : e(e_), z(z_), r(r_){};
  PolarPoint() : e(0.0f), z(0.0f), r(0.0f){};
  float e;
  float z;
  float r;
};

#define M_PI_F 3.14159265358979323846f
#define WARN_UNUSED __attribute__((warn_unused_result))

const float DEG_TO_RAD = M_PI_F / 180.f;
const float RAD_TO_DEG = 180.0f / M_PI_F;

/**
* @brief     Compute the yaw angle from a quaternion
* @returns   yaw angle in degrees
**/
float getYawFromQuaternion(const Eigen::Quaternionf q);

/**
* @brief     Compute the pitch angle from a quaternion
* @returns   pitch angle in degrees
**/
float getPitchFromQuaternion(const Eigen::Quaternionf q);

/**
* @brief     wrappes the input angle in to plus minus PI space
* @param[in] angle to be wrapped  [rad]
* @returns   wrapped angle [rad]
**/
float WARN_UNUSED wrapAngleToPlusMinusPI(float angle);
/**
* @brief     wrappes the input angle in to plus minus 180 deg space
* @param[in] angle to be wrapped  [deg]
* @returns   wrapped angle [deg]
**/
float WARN_UNUSED wrapAngleToPlusMinus180(float angle);

/**
* @brief     computes an angular velocity to reach the desired_yaw
* @param[in] adesired_yaw  [rad]
* @param[in] curr_yaw  [rad]
* @returns   a scaled angular velocity to reach the desired yaw[rad/s]
**/
double getAngularVelocity(float desired_yaw, float curr_yaw);

/**
* @brief     Compute the yaw angle between current position and point
* @returns   angle between two points in rad
**/
float nextYaw(const Eigen::Vector3f& u, const Eigen::Vector3f& v);

/**
* @brief     Compute the polar vector in FCU convention between two cartesian
*            points
* @param[in] endpoint of the polar vector
* @param[in] origin of the polar vector
* @returns   polar point in FCU convention that points from the given origin to
*            the given point
* @warning   the output adheres to the FCU convention: positive yaw is measured
*            CCW from the positive x-axis, and positve pitch is measured CCW
*            from the positve x-axis. (Positive pitch is pitching forward)
* @note      An overloaded function taking a pcl::PointXYZ assumes the origin
*            (0, 0, 0)
**/
PolarPoint cartesianToPolarFCU(const Eigen::Vector3f& pos, const Eigen::Vector3f& origin);
PolarPoint cartesianToPolarFCU(const pcl::PointXYZ& p);

// todo: is this used?
void createPoseMsg(Eigen::Vector3f& out_waypt, Eigen::Quaternionf& out_q, const Eigen::Vector3f& in_waypt, float yaw);

inline Eigen::Vector3f toEigen(const geometry_msgs::Point& p) {
  Eigen::Vector3f ev3(p.x, p.y, p.z);
  return ev3;
}

inline Eigen::Vector3f toEigen(const geometry_msgs::Vector3& v3) {
  Eigen::Vector3f ev3(v3.x, v3.y, v3.z);
  return ev3;
}

inline Eigen::Vector3f toEigen(const pcl::PointXYZ& p) {
  Eigen::Vector3f ev3(p.x, p.y, p.z);
  return ev3;
}

inline Eigen::Vector3f toEigen(const pcl::PointXYZI& p) {
  Eigen::Vector3f ev3(p.x, p.y, p.z);
  return ev3;
}

inline Eigen::Quaternionf toEigen(const geometry_msgs::Quaternion& gmq) {
  Eigen::Quaternionf eqf;
  eqf.x() = gmq.x;
  eqf.y() = gmq.y;
  eqf.z() = gmq.z;
  eqf.w() = gmq.w;
  return eqf;
}

inline geometry_msgs::Point toPoint(const Eigen::Vector3f& ev3) {
  geometry_msgs::Point gmp;
  gmp.x = ev3.x();
  gmp.y = ev3.y();
  gmp.z = ev3.z();
  return gmp;
}

inline geometry_msgs::Vector3 toVector3(const Eigen::Vector3f& ev3) {
  geometry_msgs::Vector3 gmv3;
  gmv3.x = ev3.x();
  gmv3.y = ev3.y();
  gmv3.z = ev3.z();
  return gmv3;
}

inline geometry_msgs::Quaternion toQuaternion(const Eigen::Quaternionf& eqf) {
  geometry_msgs::Quaternion q;
  q.x = eqf.x();
  q.y = eqf.y();
  q.z = eqf.z();
  q.w = eqf.w();
  return q;
}

inline pcl::PointXYZ toXYZ(const Eigen::Vector3f& ev3) {
  pcl::PointXYZ xyz;
  xyz.x = ev3.x();
  xyz.y = ev3.y();
  xyz.z = ev3.z();
  return xyz;
}

inline pcl::PointXYZI toXYZI(const Eigen::Vector3f& ev3, float intensity) {
  pcl::PointXYZI p;
  p.x = ev3.x();
  p.y = ev3.y();
  p.z = ev3.z();
  p.intensity = intensity;
  return p;
}

inline pcl::PointXYZI toXYZI(float x, float y, float z, float intensity) {
  pcl::PointXYZI p;
  p.x = x;
  p.y = y;
  p.z = z;
  p.intensity = intensity;
  return p;
}

inline pcl::PointXYZI toXYZI(const pcl::PointXYZ& xyz, float intensity) {
  pcl::PointXYZI p;
  p.x = xyz.x;
  p.y = xyz.y;
  p.z = xyz.z;
  p.intensity = intensity;
  return p;
}

inline geometry_msgs::Twist toTwist(const Eigen::Vector3f& l, const Eigen::Vector3f& a) {
  geometry_msgs::Twist gmt;
  gmt.linear = toVector3(l);
  gmt.angular = toVector3(a);
  return gmt;
}

inline geometry_msgs::PoseStamped toPoseStamped(const Eigen::Vector3f& ev3, const Eigen::Quaternionf& eq) {
  geometry_msgs::PoseStamped gmps;
  gmps.header.stamp = ros::Time::now();
  gmps.header.frame_id = "/local_origin";
  gmps.pose.position = toPoint(ev3);
  gmps.pose.orientation = toQuaternion(eq);
  return gmps;
}

inline Eigen::Vector3f toNED(const Eigen::Vector3f& xyz_enu) {
  Eigen::Vector3f xyz_ned;
  xyz_ned.x() = xyz_enu.y();
  xyz_ned.y() = xyz_enu.x();
  xyz_ned.z() = -xyz_enu.z();
  return xyz_ned;
}

inline Eigen::Vector3f toENU(const Eigen::Vector3f& xyz_ned) {
  Eigen::Vector3f xyz_enu;
  xyz_enu.x() = xyz_ned.y();
  xyz_enu.y() = xyz_ned.x();
  xyz_enu.z() = -xyz_ned.z();
  return xyz_enu;
}

#endif  // COMMON_H