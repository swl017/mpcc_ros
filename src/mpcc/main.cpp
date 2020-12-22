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

#include "mpcc_ros.h"
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

    ego_odom_sub_ = nh.subscribe("simulation/bodyOdom", 1, &MpccRos::stateCallback, this);
    control_pub_ = nh.advertise<ackermann_msgs::AckermannDriveStamped>("control", 1);

    path_pub_      = nh.advertise<nav_msgs::Path>("path/center_line", 1);
    bound_in_pub_  = nh.advertise<nav_msgs::Path>("path/bound_in", 1);
    bound_out_pub_ = nh.advertise<nav_msgs::Path>("path/bound_out", 1);
    sol_trajectory_pub_ = nh.advertise<nav_msgs::Path>("path/solution_trajectory", 1);

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
    mpc.setTrack(track_xy.X,track_xy.Y);
    phi_0 = std::atan2(track_xy.Y(1) - track_xy.Y(0),track_xy.X(1) - track_xy.X(0));
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
    x_.D     = std::min(1., std::max(-0.1, x_.D+u_.dD));
    x_.delta = std::min(0.35, std::max(-0.35, x_.delta+u_.dDelta));
    x_.vs    = sqrt(x_.vx*x_.vx + x_.vy*x_.vy); //u_.dVs;

    runControlLoop(x_);
}

void MpccRos::runControlLoop(State x)
{
    MPCReturn mpc_sol = mpc.runMPC(x);
    u_ = mpc_sol.u0;
    mpc_horizon_ = mpc_sol.mpc_horizon;
    log.push_back(mpc_sol);
    publishControl(u_);
    publishTrack();
}

void MpccRos::publishControl(Input u)
{
    u_sig_.dD     += u.dD;
    u_sig_.dDelta += u.dDelta;
    u_sig_.dVs    += u.dVs;

    ackermann_msgs::AckermannDriveStamped msg;
    msg.drive.acceleration   = u.dD;//u_sig_.dD;
    msg.drive.steering_angle = u.dDelta;//u_sig_.dDelta;
    msg.drive.speed          = u.dVs;
    control_pub_.publish(msg);

    sol_trajectory_.poses.clear();
    std::cout << "MPC horizon size: " << mpc_horizon_.size() << std::endl;
    for(int i=0; i<mpc_horizon_.size(); i++)
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
    path_pub_.publish(center_path_);
    bound_in_pub_.publish(bound_in_);
    bound_out_pub_ .publish(bound_out_);
}
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "mpcc");
    ros::NodeHandle nhp("~");
    using namespace mpcc;
    std::ifstream iConfig("/home/sw/catkin_ws/src/mpcc_ros/src/mpcc/Params/config.json");
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


