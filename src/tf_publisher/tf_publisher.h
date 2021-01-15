#ifndef TF_PUBLISHER_H
#define TF_PUBLISHER_H

#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>

class TFPublisher
{
public:
    TFPublisher();
    ~TFPublisher();

    void egoPoseSubCallback(const nav_msgs::Odometry::ConstPtr &msg);

    std::string ns_;
    std::string world_frame_id_;
    std::string ego_frame_id_;
    std::string ego_pose_topic_;

private:
    ros::Subscriber ego_pose_sub_;
};

#endif /* TF_PUBLISHER_H */