/**
 * @file test_default_pose_planning.cpp
 * @author Handian Yang
 * @copyright Released under the terms of the GPLv3.0 or later
 * @date 2024
 * 
 * @brief The executable for testing the default poses planning in
 *        Model Predictive Control (MPC) function for marslite robots.
 * @note `test_default_pose_planning.cpp` is part of `marslite_simulation_ws`.
 * 
 * `marslite_simulation_ws` is free software: you can redistribute it and/or
 *  modify it under the terms of the GNU General Public License as published
 *  by the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 * 
 * `marslite_simulation_ws` is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 *  Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License along
 *  with `marslite_simulation_ws`. If not, see <https://www.gnu.org/licenses/>. 
*/

#include <ros/ros.h>

#include "marslite_control/marslite_control.h"
using marslite::control::MarsliteControlScheme;

#include "marslite_properties/Exception.h"
using marslite::exception::ConstructorInitializationFailedException;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "test_default_pose_planning");

    ros::AsyncSpinner spinner(0);
    spinner.start();

    try {
        MarsliteControlScheme::ControlClassPtr controlClassPtr = std::make_shared<MarsliteControlScheme>();

        // Test 1: planning to the home pose of marslite robots
        controlClassPtr->updateInitialStateFromRobotState();
        controlClassPtr->setTargetPose(marslite::pose::HOME);
        ROS_ASSERT(controlClassPtr->trajectoryPlanningQPSolver());
        
        // Test 2: planning to the default1 pose
        controlClassPtr->updateInitialStateFromRobotState();
        controlClassPtr->setTargetPose(marslite::pose::DEFAULT1);
        ROS_ASSERT(controlClassPtr->trajectoryPlanningQPSolver());

        // Test 3: planning to the default2 pose
        controlClassPtr->updateInitialStateFromRobotState();
        controlClassPtr->setTargetPose(marslite::pose::DEFAULT2);
        ROS_ASSERT(controlClassPtr->trajectoryPlanningQPSolver());

        // Test 4: planning to the default3 pose
        controlClassPtr->updateInitialStateFromRobotState();
        controlClassPtr->setTargetPose(marslite::pose::DEFAULT3);
        ROS_ASSERT(controlClassPtr->trajectoryPlanningQPSolver());

        // Test 5: planning to the default4 pose
        controlClassPtr->updateInitialStateFromRobotState();
        controlClassPtr->setTargetPose(marslite::pose::DEFAULT4);
        ROS_ASSERT(controlClassPtr->trajectoryPlanningQPSolver());

        // Test 6: planning to the home pose of marslite robots
        controlClassPtr->updateInitialStateFromRobotState();
        controlClassPtr->setTargetPose(marslite::pose::HOME);
        ROS_ASSERT(controlClassPtr->trajectoryPlanningQPSolver());

    } catch (const ConstructorInitializationFailedException& ex) {
        ROS_ERROR_STREAM(ex.what());
    }

    ros::shutdown();
    return 0;
}