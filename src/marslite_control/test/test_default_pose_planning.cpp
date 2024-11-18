/**
 * marslite_simulation_ws/test/test_default_pose_planning.cpp
 * 
 * Copyright (C) 2024 Handian Yang
 * 
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <https://www.gnu.org/licenses/>.
*/

#include <ros/ros.h>

#include "marslite_control/marslite_control.h"
using marslite::control::MarsliteControl;

#include "marslite_properties/Exception.h"
using marslite::exception::ConstructorInitializationFailedException;
using marslite::exception::TimeOutException;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "test_default_pose_planning");

  ros::AsyncSpinner spinner(0);
  spinner.start();

  try {
    MarsliteControl::ControlPtr control_ptr = std::make_shared<MarsliteControl>();

    // Test 1: planning to the home pose of marslite robots
    control_ptr->updateInitialPoseFromCurrent();
    control_ptr->setTargetPose(marslite::pose::HOME);
    ROS_ASSERT(control_ptr->planTrajectoryWithQPSolver(ros::Duration(10)));
    
    // Test 2: planning to the default1 pose
    control_ptr->updateInitialPoseFromCurrent();
    control_ptr->setTargetPose(marslite::pose::DEFAULT1);
    ROS_ASSERT(control_ptr->planTrajectoryWithQPSolver(ros::Duration(10)));

    // Test 3: planning to the default2 pose
    control_ptr->updateInitialPoseFromCurrent();
    control_ptr->setTargetPose(marslite::pose::DEFAULT2);
    ROS_ASSERT(control_ptr->planTrajectoryWithQPSolver(ros::Duration(10)));

    // Test 4: planning to the default3 pose
    control_ptr->updateInitialPoseFromCurrent();
    control_ptr->setTargetPose(marslite::pose::DEFAULT3);
    ROS_ASSERT(control_ptr->planTrajectoryWithQPSolver(ros::Duration(10)));

    // Test 5: planning to the default4 pose
    control_ptr->updateInitialPoseFromCurrent();
    control_ptr->setTargetPose(marslite::pose::DEFAULT4);
    ROS_ASSERT(control_ptr->planTrajectoryWithQPSolver(ros::Duration(10)));

    // Test 6: planning to the home pose of marslite robots
    control_ptr->updateInitialPoseFromCurrent();
    control_ptr->setTargetPose(marslite::pose::HOME);
    ROS_ASSERT(control_ptr->planTrajectoryWithQPSolver(ros::Duration(10)));

  } catch (const ConstructorInitializationFailedException& ex) {
    ROS_ERROR_STREAM(ex.what());
  } catch (const TimeOutException& ex) {
    ROS_ERROR_STREAM(ex.what());
  }

  ROS_INFO("All tests passed! Type Ctrl+C to exit Gazebo...");
  ros::shutdown();
  return 0;
}