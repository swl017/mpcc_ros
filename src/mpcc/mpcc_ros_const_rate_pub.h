#include "Tests/spline_test.h"
#include "Tests/model_integrator_test.h"
#include "Tests/constratins_test.h"
#include "Tests/cost_test.h"

#include "MPC/mpc.h"
#include "Model/integrator.h"
#include "Params/track.h"

#include <nlohmann/json.hpp>

#include <math.h>

#include "ros/ros.h"
#include <ros/package.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Quaternion.h>

#include <std_msgs/Float64.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>

#include <ackermann_msgs/AckermannDriveStamped.h>

std::string config_path_;
std::string package_path_;

// struct Quaternion
// {
//     double w, x, y, z;
// };

struct EulerAngles {
    double roll, pitch, yaw;
};

geometry_msgs::Quaternion ToQuaternion(double yaw, double pitch, double roll) // yaw (Z), pitch (Y), roll (X)
{
    // Abbreviations for the various angular functions
    double cy = cos(yaw * 0.5);
    double sy = sin(yaw * 0.5);
    double cp = cos(pitch * 0.5);
    double sp = sin(pitch * 0.5);
    double cr = cos(roll * 0.5);
    double sr = sin(roll * 0.5);

    geometry_msgs::Quaternion q;
    q.w = cr * cp * cy + sr * sp * sy;
    q.x = sr * cp * cy - cr * sp * sy;
    q.y = cr * sp * cy + sr * cp * sy;
    q.z = cr * cp * sy - sr * sp * cy;

    return q;
}

EulerAngles ToEulerAngles(geometry_msgs::Quaternion q) {
    EulerAngles angles;

    // roll (x-axis rotation)
    double sinr_cosp = 2 * (q.w * q.x + q.y * q.z);
    double cosr_cosp = 1 - 2 * (q.x * q.x + q.y * q.y);
    angles.roll = std::atan2(sinr_cosp, cosr_cosp);

    // pitch (y-axis rotation)
    double sinp = 2 * (q.w * q.y - q.z * q.x);
    if (std::abs(sinp) >= 1)
        angles.pitch = std::copysign(M_PI / 2, sinp); // use 90 degrees if out of range
    else
        angles.pitch = std::asin(sinp);

    // yaw (z-axis rotation)
    double siny_cosp = 2 * (q.w * q.z + q.x * q.y);
    double cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z);
    angles.yaw = std::atan2(siny_cosp, cosy_cosp);

    return angles;
}

namespace mpcc
{
class MpccRos
{
public:
    MpccRos();
    ~MpccRos();

    void stateCallback(const nav_msgs::OdometryConstPtr& msg);
    void publishControl();
    State x_;
    Input u_;
private:
    ros::Subscriber ego_odom_sub_;
    ros::Publisher control_pub_;

    nav_msgs::Odometry ego_odom_;

};
}