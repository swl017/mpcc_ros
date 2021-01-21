// Copyright 2019 Alexander Liniger

// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at

//     http://www.apache.org/licenses/LICENSE-2.0

// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
///////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////

#include "mpcc_controller_node.h"
using json = nlohmann::json;

namespace mpcc
{
MpccRos::MpccRos()
{
    ros::NodeHandle nh;

    ego_odom_sub_ = nh.subscribe("simulation/bodyOdom", 1, &MpccRos::stateCallback, this);
    control_pub_ = nh.advertise<ackermann_msgs::AckermannDriveStamped>("control", 1);

    mpcInit();
}
MpccRos::MpccRos(int n_sqp, int n_reset,double sqp_mixing, double Ts,const PathToJson path, json config):
mpc(n_sqp, n_reset, sqp_mixing, Ts, path),
json_paths(path),
jsonConfig(config)
{
    ros::NodeHandle nh;
    ros::NodeHandle nhp("~");
    // use_test_sim_ = false;
    Ts_ = jsonConfig["Ts"];

    nhp.param("use_test_sim", use_test_sim_, true);
    nhp.param<std::string>("ego_pose_topic", ego_pose_topic_, "sim_pose");

    ROS_INFO("use_sim: %s",use_test_sim_ ? "true" : "false");

    ego_odom_sub_ = nh.subscribe("/Odometry/ekf_estimated", 1, &MpccRos::stateCallback, this);
    reference_path_sub_ = nh.subscribe("/Path/LocalWaypoint/OnBody", 1, &MpccRos::referencePathCallback, this);
    track_boundary_left_sub_ = nh.subscribe("/mpc/left_boundary", 1, &MpccRos::trackBoundLeftCallback, this);
    track_boundary_right_sub_ = nh.subscribe("/mpc/right_boundary", 1, &MpccRos::trackBoundRightCallback, this);
    
    control_pub_ = nh.advertise<ackermann_msgs::AckermannDriveStamped>("mpc_control", 1);

    path_pub_ = nh.advertise<nav_msgs::Path>("mpc/mpc_center_line", 1);
    bound_in_pub_ = nh.advertise<nav_msgs::Path>("mpc/bound_in", 1);
    bound_out_pub_ = nh.advertise<nav_msgs::Path>("mpc/bound_out", 1);
    mpc_bound_in_pub_ = nh.advertise<nav_msgs::Path>("mpc/mpc_bound_in", 1);
    mpc_bound_out_pub_ = nh.advertise<nav_msgs::Path>("mpc/mpc_bound_out", 1);

    sol_trajectory_pub_ = nh.advertise<nav_msgs::Path>("mpc/mpc_solution_trajectory", 1);

    pose_pub = nh.advertise<nav_msgs::Odometry>(ego_pose_topic_.c_str(), 1);

    mpcInit();
}

MpccRos::~MpccRos()
{
    // Performance summary
    double mean_time = 0.0;
    double max_time = 0.0;
    for(MPCReturn log_i : log)
    {
        mean_time += log_i.time_total;
        if(log_i.time_total > max_time)
            max_time = log_i.time_total;
    }
    std::cout << "mean nmpc time " << mean_time/double(jsonConfig["n_sim"]) << std::endl;
    std::cout << "max nmpc time " << max_time << std::endl;
}

void MpccRos::mpcInit()
{
    ////////////////////////////////////////
    /////// MPC class initialization ///////
    ////////////////////////////////////////

    Track track = Track(json_paths.track_path);
    TrackPos track_xy = track.getTrack();
    // mpc.setTrack(track_xy.X,track_xy.Y);
    mpc.setTrack(track_xy.X,track_xy.Y,track_xy.X_inner,track_xy.Y_inner,track_xy.X_outer,track_xy.Y_outer);
    // phi_0 = std::atan2(track_xy.Y(304) - track_xy.Y(300),track_xy.X(304) - track_xy.X(300));
    // x0 = {track_xy.X(300),track_xy.Y(300),phi_0,jsonConfig["v0"],0,0,0,0.5,0,jsonConfig["v0"]};
    phi_0 = std::atan2(track_xy.Y(4) - track_xy.Y(0),track_xy.X(4) - track_xy.X(0));
    x0 = {track_xy.X(0),track_xy.Y(0),phi_0,jsonConfig["v0"],0,0,0,0.5,0,jsonConfig["v0"]};

    
    ////////////////////////////////////////////
    /////// End MPC class initialization ///////
    ////////////////////////////////////////////

    std::cout << "Center path length: " << track_xy.X.size() << std::endl;
    std::cout << "Bound in length: " << track_xy.X_inner.size() << std::endl;
    std::cout << "Bound out length: " << track_xy.X_outer.size() << std::endl;

    std::string frame_id = "odom";
    center_path_.header.frame_id = frame_id;
    bound_in_.header.frame_id    = frame_id;
    bound_out_.header.frame_id   = frame_id;
    mpc_bound_in_.header.frame_id   = frame_id;
    mpc_bound_out_.header.frame_id  = frame_id;
    sol_trajectory_.header.frame_id = frame_id;
    for(int i=0; i<track_xy.X.size(); i++)
    {
        geometry_msgs::PoseStamped pose;
        pose.pose.position.x = track_xy.X(i);
        pose.pose.position.y = track_xy.Y(i);

        center_path_.poses.push_back(pose);
    }
    for(int i=0; i<track_xy.X_inner.size(); i++)
    {
        geometry_msgs::PoseStamped pose;
        pose.pose.position.x = track_xy.X_inner(i);
        pose.pose.position.y = track_xy.Y_inner(i);

        bound_in_.poses.push_back(pose);
    }
    for(int i=0; i<track_xy.X_outer.size(); i++)
    {
        geometry_msgs::PoseStamped pose;
        pose.pose.position.x = track_xy.X_outer(i);
        pose.pose.position.y = track_xy.Y_outer(i);

        bound_out_.poses.push_back(pose);
    }
    if(use_test_sim_)
    {
        publishTrack();
        runTestSim();
    }

}

void MpccRos::referencePathCallback(const nav_msgs::PathConstPtr& msg)
{
    b_rcv_reference_path_ = true;
    center_path_.poses.clear();
    for(int i = 0; i < msg->poses.size(); i++)
    {
        geometry_msgs::PoseStamped pose;
        // pose.pose.position.x = msg->poses[i].pose.position.x;
        // pose.pose.position.y = msg->poses[i].pose.position.y;
        toGlobalPath(msg->poses[i].pose.position, pose.pose.position);
        center_path_.poses.push_back(pose);
    }
}

void MpccRos::trackBoundLeftCallback(const nav_msgs::PathConstPtr& msg)
{
    b_rcv_track_boundary_left_ = true;
    bound_in_.poses.clear();
    for(int i = 0; i < msg->poses.size(); i++)
    {
        geometry_msgs::PoseStamped pose;
        // pose.pose.position.x = msg->poses[i].pose.position.x;
        // pose.pose.position.y = msg->poses[i].pose.position.y;
        // toGlobalPath(bound_in.poses[i].pose.position, pose.pose.position);
        toGlobalPath(msg->poses[i].pose.position, pose.pose.position);
        bound_in_.poses.push_back(pose);
    }
}

void MpccRos::trackBoundRightCallback(const nav_msgs::PathConstPtr& msg)
{
    b_rcv_track_boundary_right_ = true;
    bound_out_.poses.clear();
    for(int i = 0; i < msg->poses.size(); i++)
    {
        geometry_msgs::PoseStamped pose;
        // pose.pose.position.x = msg->poses[i].pose.position.x;
        // pose.pose.position.y = msg->poses[i].pose.position.y;
        // toGlobalPath(bound_out.poses[i].pose.position, pose.pose.position);
        toGlobalPath(msg->poses[i].pose.position, pose.pose.position);
        bound_out_.poses.push_back(pose);
    }
}

void MpccRos::toGlobalPath(geometry_msgs::Point local_position, geometry_msgs::Point &global_position)
{
    /**
     * To global path:
     * [g_x, g_y, 1]^T = [Rot(yaw_o), Trans(x_o,y_o); 0(dim), 1] * [l_x, l_y, 1]^T
     * 
     * To local path:
     * [l_x, l_y, 1]^T = [Rot(yaw_o)^T, -Rot(yaw_o)^T*Trans(x_o,y_o); 0(dim), 1] * [g_x, g_y, 1]^T
     * 
     **/
    double l_x = local_position.x, l_y = local_position.y;
    double x_o = x_.X, y_o = x_.Y, yaw_o = x_.phi;

    double g_x = cos(yaw_o) * l_x - sin(yaw_o) * l_y + x_o;
    double g_y = sin(yaw_o) * l_x + cos(yaw_o) * l_y + y_o;

    global_position.x = g_x;
    global_position.y = g_y;
}

void MpccRos::filterBoundary(const nav_msgs::PathConstPtr &raw_path, nav_msgs::Path &filtered_path)
{
    int temp_index = 0;
    int path_length = raw_path->poses.size();
    if(path_length > 0)
    {
        double boundary_jump_thres = 2.0; // meter
        double last_x = raw_path->poses.at(0).pose.position.x;
        double last_y = raw_path->poses.at(0).pose.position.y;

        filtered_path.poses.clear();

        for (int i = 1; i < path_length; i++)
        {
            double curr_x = raw_path->poses.at(i).pose.position.x;
            double curr_y = raw_path->poses.at(i).pose.position.y;
            double dist = sqrt((curr_x - last_x) * (curr_x - last_x) + (curr_y - last_y) * (curr_y - last_y));
            if (dist < boundary_jump_thres || i - temp_index > 5 && i < 5)
            {
                geometry_msgs::PoseStamped pose;
                pose.pose.position.x = curr_x;
                pose.pose.position.y = curr_y;
                filtered_path.poses.push_back(pose);
                temp_index = i;
            }
            else
            {
            }
            last_x = curr_x;
            last_y = curr_y;
        }
    }
}

void MpccRos::stateCallback(const nav_msgs::OdometryConstPtr& msg)
{
    x_.X     = msg->pose.pose.position.x;
    x_.Y     = msg->pose.pose.position.y;
    EulerAngles angles;
    angles = ToEulerAngles(msg->pose.pose.orientation);
    x_.phi   = std::min(10., std::max(-10.,angles.yaw));
    x_.vx    = msg->twist.twist.linear.x;
    x_.vy    = msg->twist.twist.linear.y;
    x_.r     = std::min(8., std::max(-8., msg->twist.twist.angular.z));
    // TODO: Check if we can get vehicle steer/throttle state(position)
    x_.D     = std::min(1., std::max(-1., x_.D+u_.dD*Ts_));
    x_.delta = std::min(0.35, std::max(-0.35, x_.delta+u_.dDelta*Ts_));
    x_.vs    = sqrt(x_.vx*x_.vx + x_.vy*x_.vy); //u_.dVs;
    // x_.s     = 0.0;

    int raceline_length = center_path_.poses.size();
    int boundary_in_length = bound_in_.poses.size();
    int boundary_out_length = bound_out_.poses.size();
    // RCLCPP_INFO(rclcpp::get_logger("mpcc_controller"), "MPC Planner received: c %d, in %d, out %d",
    //         raceline_length, boundary_in_length, boundary_out_length);

    if(raceline_length > 20 && boundary_in_length > 20 && boundary_out_length  > 20
        && b_rcv_reference_path_ && b_rcv_track_boundary_left_ && b_rcv_track_boundary_right_)
    {
        Eigen::VectorXd raceline_X(raceline_length);
        Eigen::VectorXd raceline_Y(raceline_length);
        for (int i=0; i<raceline_length; i++)
        {
            geometry_msgs::Point global_point;
            toGlobalPath(center_path_.poses[i].pose.position, global_point);
            // raceline_X(i) = center_path_.poses[i].pose.position.x;
            // raceline_Y(i) = center_path_.poses[i].pose.position.y;
            raceline_X(i) = global_point.x;
            raceline_Y(i) = global_point.y;
        }
        std::cout << "raceline_X " << raceline_X.size() << " raceline_Y " << raceline_Y.size() << " / " << raceline_length << std::endl;

        Eigen::VectorXd boundary_in_X(boundary_in_length);
        Eigen::VectorXd boundary_in_Y(boundary_in_length);
        for (int i=0; i<boundary_in_length; i++)
        {
            geometry_msgs::Point global_point;
            toGlobalPath(bound_in_.poses[i].pose.position, global_point);
            // boundary_in_X(i) = bound_in_.poses[i].pose.position.x;
            // boundary_in_Y(i) = bound_in_.poses[i].pose.position.y;
            boundary_in_X(i) = global_point.x;
            boundary_in_Y(i) = global_point.y;
        }
        std::cout << "boundary_in_X " << boundary_in_X.size() << " boundary_in_X " << boundary_in_X.size() << " / " << boundary_in_length << std::endl;

        Eigen::VectorXd boundary_out_X(boundary_out_length);
        Eigen::VectorXd boundary_out_Y(boundary_out_length);
        for (int i=0; i<boundary_out_length; i++)
        {
            geometry_msgs::Point global_point;
            toGlobalPosition(ego_odom_.pose.pose, bound_in_.poses[i].pose.position, global_point);
            // boundary_out_X(i) = bound_out_.poses[i].pose.position.x;
            // boundary_out_Y(i) = bound_out_.poses[i].pose.position.y;
            boundary_out_X(i) = global_point.x;
            boundary_out_Y(i) = global_point.y;
        }
        std::cout << "boundary_out_X " << boundary_out_X.size() << " boundary_out_X " << boundary_out_X.size() << " / " << boundary_out_length << std::endl;
        // mpc.setTrack(raceline_X, raceline_Y);
        mpc.setTrack(raceline_X, raceline_Y, boundary_in_X, boundary_in_Y, boundary_out_X, boundary_out_Y);
    runControlLoop(x_);
    publishControl(u_);
    publishTrack();
    debugSolution();
    b_rcv_reference_path_ = false;
    b_rcv_track_boundary_left_ = false;
    b_rcv_track_boundary_right_ = false;
    }

}

void MpccRos::toGlobalPosition(geometry_msgs::Pose ego_pose, geometry_msgs::Point local_path_point, geometry_msgs::Point& global_path_point)
{
    /**
     * Path Transform
     * [global_path, 1]^T = [Rot, Trans; 0(dim), 1] * [local_path, 1]^T
     * [local_path,  1]^T = [Rot^T, -Rot^T*Trans; 0(dim), 1] * [global_path, 1]^T
     **/
    EulerAngles angles;
    angles = ToEulerAngles(ego_pose.orientation);
    double yaw = angles.yaw;

    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = cos(yaw)*local_path_point.x - sin(yaw)*local_path_point.y + ego_pose.position.x;
    pose.pose.position.y = sin(yaw)*local_path_point.y + cos(yaw)*local_path_point.y + ego_pose.position.y;

    global_path_point = pose.pose.position;
}

void MpccRos::runControlLoop(State x)
{
        std::cout << "DEBUG RUN" << std::endl;
    MPCReturn mpc_sol = mpc.runMPC(x);
    u_ = mpc_sol.u0;
    mpc_horizon_ = mpc_sol.mpc_horizon;
    log.push_back(mpc_sol);
}

void MpccRos::publishControl(Input u)
{
        std::cout << "DEBUG CON" << std::endl;
    u_sig_.dD     += u.dD;
    u_sig_.dDelta += u.dDelta;
    u_sig_.dVs    += u.dVs;

    ackermann_msgs::AckermannDriveStamped msg;
    msg.header.stamp = ros::Time::now();
    // TODO: Need debate
    msg.drive.steering_angle = x_.delta;
    msg.drive.steering_angle_velocity = u.dDelta;
    msg.drive.speed          = sqrt(pow(x_.vx, 2.0)+pow(x_.vy, 2.0));
    msg.drive.acceleration   = x_.D;
    msg.drive.jerk   = u.dD;
    control_pub_.publish(msg);

}

void MpccRos::debugSolution()
{
    mpc_bound_in_.poses.clear();
    mpc_bound_out_.poses.clear();
    if(mpc.pos_inner_.size() > 5)
    {
        for (int i = 0; i < N; i++)
        {
            geometry_msgs::PoseStamped pose_in;
            pose_in.pose.position.x = mpc.pos_inner_[i][0];
            pose_in.pose.position.y = mpc.pos_inner_[i][1];
            mpc_bound_in_.poses.push_back(pose_in);

            geometry_msgs::PoseStamped pose_out;
            pose_out.pose.position.x = mpc.pos_outer_[i][0];
            pose_out.pose.position.y = mpc.pos_outer_[i][1];
            mpc_bound_out_.poses.push_back(pose_out);
        }
    }
    mpc_bound_in_pub_.publish(mpc_bound_in_);
    mpc_bound_out_pub_.publish(mpc_bound_out_);
    sol_trajectory_.poses.clear();
    std::cout << "MPC horizon size: " << mpc_horizon_.size() << std::endl;
    for(int i=0; i<N; i++)
    {
        geometry_msgs::PoseStamped pose;
        pose.pose.position.x = mpc_horizon_.at(i).xk.X;
        pose.pose.position.y = mpc_horizon_.at(i).xk.Y;

        sol_trajectory_.poses.push_back(pose);
    }
    sol_trajectory_pub_.publish(sol_trajectory_);
}

void MpccRos::publishTrack()
{
    std::cout << "DEBUG Track" << std::endl;
    path_pub_.publish(center_path_);
    bound_in_pub_.publish(bound_in_);
    bound_out_pub_ .publish(bound_out_);
}

void MpccRos::runTestSim()
{
    Integrator integrator = Integrator(jsonConfig["Ts"],json_paths);
    int i = 0;
    // for(int i=0;i<jsonConfig["n_sim"];i++)
    while(ros::ok())
    {
        i++;
        if(ros::ok())
        {
            MPCReturn mpc_sol = mpc.runMPC(x0);
            u_ = mpc_sol.u0;
            mpc_horizon_ = mpc_sol.mpc_horizon;
            log.push_back(mpc_sol);
            x0 = integrator.simTimeStep(x0,mpc_sol.u0,jsonConfig["Ts"]);
            x_ = x0;

            nav_msgs::Odometry msg;
            msg.header.seq = i;
            msg.header.stamp = ros::Time::now();
            msg.header.frame_id = "odom";
            msg.child_frame_id = "base_link";
            msg.pose.pose.position.x = x0.X;
            msg.pose.pose.position.y = x0.Y;
            geometry_msgs::Quaternion q;
            q = ToQuaternion(x0.phi, 0., 0.);
            msg.pose.pose.orientation.x = q.x;
            msg.pose.pose.orientation.y = q.y;
            msg.pose.pose.orientation.z = q.z;
            msg.pose.pose.orientation.w = q.w;
            msg.twist.twist.linear.x = x_.vx;
            msg.twist.twist.linear.y = x_.vy;
            msg.twist.twist.angular.z = x_.r;
            pose_pub.publish(msg);

            publishControl(u_);
            publishTrack();
            debugSolution();
        }
    }
}
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "mpcc_controller_node");
    ros::NodeHandle nhp("~");
    using namespace mpcc;
    std::string config_path;
    nhp.param("config_path", config_path, std::string("/home/usrg/catkin_ws/src/mpcc_ros/src/mpcc_controller/mpcc_controller/Params/config.json"));
    std::cout << "Openning config at " << config_path << std::endl;
    std::ifstream iConfig(config_path.c_str());
    json jsonConfig;
    iConfig >> jsonConfig;

    PathToJson json_paths {jsonConfig["model_path"],
                           jsonConfig["cost_path"],
                           jsonConfig["bounds_path"],
                           jsonConfig["track_path"],
                           jsonConfig["normalization_path"]};

    // std::cout << testSpline() << std::endl;
    // std::cout << testArcLengthSpline(json_paths) << std::endl;

    // std::cout << testIntegrator(json_paths) << std::endl;
    // std::cout << testLinModel(json_paths) << std::endl;

    // std::cout << testAlphaConstraint(json_paths) << std::endl;
    // std::cout << testTireForceConstraint(json_paths) << std::endl;
    // std::cout << testTrackConstraint(json_paths) << std::endl;

    // std::cout << testCost(json_paths) << std::endl;

    Integrator integrator = Integrator(jsonConfig["Ts"],json_paths);
    //Plotting plotter = Plotting(jsonConfig["Ts"],json_paths);

    Track track = Track(json_paths.track_path);
    TrackPos track_xy = track.getTrack();

    MpccRos mpcc_ros(jsonConfig["n_sqp"],jsonConfig["n_reset"],jsonConfig["sqp_mixing"],jsonConfig["Ts"],json_paths, jsonConfig);

    // ros::Rate r(50);
    // while(ros::ok())
    // {
    //     State x = mpcc_ros_pub_sub.x_;
    //     MPCReturn mpc_sol = mpc.runMPC(x);
    //     x0 = integrator.simTimeStep(x,mpc_sol.u0,jsonConfig["Ts"]);
    //     log.push_back(mpc_sol);

    //     mpcc_ros_pub_sub.u_ = mpc_sol.u0;
    //     mpcc_ros_pub_sub.publishControl();
    //     ros::spinOnce();
    //     r.sleep();
    // }
    ros::spin();

    
    return 0;
}


