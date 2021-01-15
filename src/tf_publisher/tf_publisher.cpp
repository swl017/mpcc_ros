/**
 * @file tf_publisher.cpp
 * @author Seungwook Lee
 * @date 2021-01-15
 **/

#include "tf_publisher.h"

TFPublisher::TFPublisher()
{
    ros::NodeHandle nh;
    ros::NodeHandle nhp("~");

    // Namespace
    ns_ = ros::this_node::getNamespace();
    if (ns_.size() > 1)
    {
        ns_ = ns_.substr(1, ns_.size());
        ns_.append("/");
    }
    else
    {
        ns_ = "";
    }

    // Parameters
    nhp.param<std::string>("world_frame_id", world_frame_id_, "odom");
    nhp.param<std::string>("ego_frame_id", ego_frame_id_, "base_link");
    nhp.param<std::string>("ego_pose_topic", ego_pose_topic_, "sim_pose");
    ego_frame_id_ = ns_ + ego_frame_id_;

    // Subscribers and Publishers
    ego_pose_sub_ = nh.subscribe(ego_pose_topic_.c_str(), 1, &TFPublisher::egoPoseSubCallback, this);

    ROS_INFO("namesapce: %s", ns_.c_str());
    ROS_INFO("ego_frame_id_: %s", ego_frame_id_.c_str());
}
TFPublisher::~TFPublisher()
{
}

void TFPublisher::egoPoseSubCallback(const nav_msgs::Odometry::ConstPtr &msg)
{
    static tf2_ros::TransformBroadcaster br;
    geometry_msgs::TransformStamped transformStamped;

    transformStamped.header.stamp = ros::Time::now();
    transformStamped.header.frame_id = world_frame_id_.c_str();
    transformStamped.child_frame_id = ego_frame_id_.c_str();
    transformStamped.transform.translation.x = msg->pose.pose.position.x;
    transformStamped.transform.translation.y = msg->pose.pose.position.y;
    transformStamped.transform.translation.z = msg->pose.pose.position.z;
    transformStamped.transform.rotation.x = msg->pose.pose.orientation.x;
    transformStamped.transform.rotation.y = msg->pose.pose.orientation.y;
    transformStamped.transform.rotation.z = msg->pose.pose.orientation.z;
    transformStamped.transform.rotation.w = msg->pose.pose.orientation.w;

    br.sendTransform(transformStamped);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "mpcc_tf2_broadcaster");

    ros::NodeHandle node;
    TFPublisher tf_publisher;

    ros::spin();
    return 0;
};