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

#include <ros/ros.h>

#include "Tests/spline_test.h"
#include "Tests/model_integrator_test.h"
#include "Tests/constratins_test.h"
#include "Tests/cost_test.h"

#include "MPC/mpc.h"
#include "Model/integrator.h"
#include "Params/track.h"
//#include "Plotting/plotting.h"

#include <nlohmann/json.hpp>
using json = nlohmann::json;

int main() {

    using namespace mpcc;
    std::cout << "l01" << std::endl;
    std::ifstream iConfig("Params/config.json");
    // std::ifstream iConfig;
    // iConfig.open("src/mpcc/Params/config.json");
    // std::cout << "l02" << std::endl;
    // if(iConfig.is_open())    //파일이 열렸는지 확인
    // {
    //     std::cout << "l05" << std::endl;
    //     char arr[256];
    //     iConfig.getline(arr, 256);
    //     std::cout << "arr = " << arr << std::endl;
    // }
    // else{
    //     std::cout << "l06" << std::endl;
    // }
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
    //Plotting plotter = Plotting(jsonConfig["Ts"],json_paths);

    Track track = Track(json_paths.track_path);
    TrackPos track_xy = track.getTrack();

    std::list<MPCReturn> log;
    MPC mpc(jsonConfig["n_sqp"],jsonConfig["n_reset"],jsonConfig["sqp_mixing"],jsonConfig["Ts"],json_paths);
    mpc.setTrack(track_xy.X,track_xy.Y);
    const double phi_0 = std::atan2(track_xy.Y(1) - track_xy.Y(0),track_xy.X(1) - track_xy.X(0));
    State x0 = {track_xy.X(0),track_xy.Y(0),phi_0,jsonConfig["v0"],0,0,0,0.5,0,jsonConfig["v0"]};
    for(int i=0;i<jsonConfig["n_sim"];i++)
    {
        MPCReturn mpc_sol = mpc.runMPC(x0);
        x0 = integrator.simTimeStep(x0,mpc_sol.u0,jsonConfig["Ts"]);
        log.push_back(mpc_sol);
    }
    //plotter.plotRun(log,track_xy);
    //plotter.plotSim(log,track_xy);

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


