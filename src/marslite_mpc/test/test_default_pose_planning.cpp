/**
 * @file testDefaultPosePlanning.cpp
 * @author Handian Yang
 * @copyright Released under the terms of the GPLv3.0 or later
 * @date 2024
 * 
 * @brief The executable for testing the default poses planning in Model Predictive Control (MPC) function for marslite robots.
 * @note `testDefaultPosePlanning.cpp` is part of `marslite_simulation_ws`.
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
using marslite::exceptions::ConstructorInitializationFailedException;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "test_default_pose_planning");

    ros::AsyncSpinner spinner(0);
    spinner.start();

    try {
        ModelPredictiveControl::MPCClassPtr mpcClassPtr = std::make_shared<ModelPredictiveControl>();

        mpcClassPtr->setTargetPose(marslite::poses::MARSLITE_POSE_HOME);
        ROS_ASSERT(mpcClassPtr->trajectoryPlanningQPSolver());
        
        mpcClassPtr->setTargetPose(marslite::poses::MARSLITE_POSE_DEFAULT1);
        ROS_ASSERT(mpcClassPtr->trajectoryPlanningQPSolver());

        mpcClassPtr->setTargetPose(marslite::poses::MARSLITE_POSE_DEFAULT2);
        ROS_ASSERT(mpcClassPtr->trajectoryPlanningQPSolver());

        mpcClassPtr->setTargetPose(marslite::poses::MARSLITE_POSE_DEFAULT3);
        ROS_ASSERT(mpcClassPtr->trajectoryPlanningQPSolver());

        mpcClassPtr->setTargetPose(marslite::poses::MARSLITE_POSE_DEFAULT4);
        ROS_ASSERT(mpcClassPtr->trajectoryPlanningQPSolver());

        mpcClassPtr->setTargetPose(marslite::poses::MARSLITE_POSE_HOME);
        ROS_ASSERT(mpcClassPtr->trajectoryPlanningQPSolver());

    } catch (const ConstructorInitializationFailedException& ex) {
        ROS_ERROR_STREAM(ex.what());
    }

    ros::shutdown();
    return 0;
}