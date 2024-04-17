/**
 * @file main.cpp
 * @author Handian Yang
 * @copyright Released under the terms of the GPLv3.0 or later
 * @date 2024
 * 
 * @brief The executable for Model Predictive Control (MPC) implementation for marslite robots.
 * @note `main.cpp` is part of `marslite_simulation_ws`.
 * 
 * `marslite_simulation_ws` is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published
 *      by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
 * 
 * `marslite_simulation_ws` is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of
 *      MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License along with `marslite_simulation_ws`. If not, see <https://www.gnu.org/licenses/>. 
*/

#include <ros/ros.h>

#include "marslite_mpc/marslite_mpc.h"
using MPC = marslite::mpc::ModelPredictiveControl;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "mpc");

    std::shared_ptr<MPC> classPtr = std::make_shared<MPC>();
    ROS_ASSERT(classPtr->initializeQPSolver());
    ROS_ASSERT(classPtr->solveQP());

    return 0;
}