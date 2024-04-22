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
using marslite::mpc::ModelPredictiveControl;

#include "marslite_properties/Exceptions.h"
using marslite::ConstructorInitializationFailedException;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "mpc");

    ModelPredictiveControl::MPCClassPtr mpcClassPtr;
    try {
        mpcClassPtr = std::make_shared<ModelPredictiveControl>();
        ROS_ASSERT(mpcClassPtr->solveQP());
    } catch (const ConstructorInitializationFailedException& ex) {
        ROS_ERROR_STREAM(ex.what());
    }

    return 0;
}