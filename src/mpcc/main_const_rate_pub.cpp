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
}

MpccRos::~MpccRos()
{
}

void MpccRos::stateCallback(const nav_msgs::OdometryConstPtr& msg)
{
    x_.X     = msg->pose.pose.position.x;
    x_.Y     = msg->pose.pose.position.y;
    EulerAngles angles;
    angles = ToEulerAngles(msg->pose.pose.orientation);
    x_.phi   = angles.yaw;
    x_.vx    = msg->twist.twist.linear.x;
    x_.vy    = msg->twist.twist.linear.y;
    x_.r     = msg->twist.twist.angular.z;
    x_.D     += u_.dD;
    x_.delta += u_.dDelta;
    x_.vs    = sqrt(x_.vx*x_.vx + x_.vy*x_.vy); //u_.dVs;
}

void MpccRos::publishControl()
{
    ackermann_msgs::AckermannDriveStamped u;
    u.drive.acceleration   = x_.D;
    u.drive.steering_angle = x_.delta;
    u.drive.speed          = x_.vs;
    control_pub_.publish(u);
}
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "mpcc");
    ros::NodeHandle nhp("~");
    using namespace mpcc;

    MpccRos mpcc_ros_pub_sub;

    package_path_ = ros::package::getPath("mpcc_ros");
    // nhp.param("config_path", config_path_, package_path_+"/src/mpcc/Params/config.json");
    package_path_ = "/home/sw/catkin_ws/src/mpcc_ros/src/mpcc/Params/config.json";
    nhp.param("config_path", config_path_, package_path_);
    std::cout << "config_path = " << config_path_ << std::endl;
    std::cout << "l01" << std::endl;
    std::ifstream iConfig(config_path_);
    json jsonConfig;
    std::cout << "l03" << std::endl;
    iConfig >> jsonConfig;
    std::cout << "l04" << std::endl;

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

    Track track = Track(json_paths.track_path);
    TrackPos track_xy = track.getTrack();

    std::list<MPCReturn> log;
    MPC mpc(jsonConfig["n_sqp"],jsonConfig["n_reset"],jsonConfig["sqp_mixing"],jsonConfig["Ts"],json_paths);
    mpc.setTrack(track_xy.X,track_xy.Y);
    const double phi_0 = std::atan2(track_xy.Y(1) - track_xy.Y(0),track_xy.X(1) - track_xy.X(0));
    State x0 = {track_xy.X(0),track_xy.Y(0),phi_0,jsonConfig["v0"],0,0,0,0.5,0,jsonConfig["v0"]};
    mpcc_ros_pub_sub.x_ = x0;

    ros::Rate r(50);
    while(ros::ok())
    {
        State x = mpcc_ros_pub_sub.x_;
        MPCReturn mpc_sol = mpc.runMPC(x);
        x0 = integrator.simTimeStep(x,mpc_sol.u0,jsonConfig["Ts"]);
        log.push_back(mpc_sol);

        mpcc_ros_pub_sub.u_ = mpc_sol.u0;
        mpcc_ros_pub_sub.publishControl();
        ros::spinOnce();
        r.sleep();
    }

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
    
    return 0;
}

