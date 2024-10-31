/**
 * marslite_simulation_ws/test/test_gripper_moving.cpp
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
using marslite::exception::TransformNotFoundException;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "test_gripper_moving");

  ros::AsyncSpinner spinner(0);
  spinner.start();

  try {
    MarsliteControl::ControlPtr control_ptr = std::make_shared<MarsliteControl>();

    // Preparation: move the robot to the HOME pose
    ROS_ASSERT(control_ptr->updateInitialPoseFromCurrent());
    ROS_ASSERT(control_ptr->setTargetPose(marslite::pose::HOME));
    ROS_ASSERT(control_ptr->planTrajectoryWithQPSolver());
    ROS_INFO("Preparation done.");
    
    // Test 1: Move the gripper forward and backward by 0.1 meters
    ROS_ASSERT(control_ptr->moveGripperForward(0.2));
    ROS_ASSERT(control_ptr->moveGripperBackward(0.1));
    ROS_INFO("Test 1 passed.");

    // Test 2: Move the gripper left and right by 0.1 meters
    ROS_ASSERT(control_ptr->moveGripperLeft(0.1));
    ROS_ASSERT(control_ptr->moveGripperRight(0.1));
    ROS_ASSERT(control_ptr->moveGripperRight(0.1));
    ROS_ASSERT(control_ptr->moveGripperLeft(0.1));
    ROS_INFO("Test 2 passed.");

    // Test 3: Move the gripper up and down by 0.1 meters
    ROS_ASSERT(control_ptr->moveGripperUp(0.1));
    ROS_ASSERT(control_ptr->moveGripperDown(0.1));
    ROS_ASSERT(control_ptr->moveGripperDown(0.1));
    ROS_ASSERT(control_ptr->moveGripperUp(0.1));
    ROS_INFO("Test 3 passed.");

  } catch (const ConstructorInitializationFailedException& ex) {
    // Failed to initialize the control class
    ROS_ERROR_STREAM(ex.what());
  } catch (const TimeOutException& ex) {
    // Failed to subscribe to some topics or connect to the action server
    ROS_ERROR_STREAM(ex.what());
  } catch (const TransformNotFoundException& ex) {
    // Failed to look up the transform between two frames
    ROS_ERROR_STREAM(ex.what());
  }

  ROS_INFO("All tests passed! Type Ctrl+C to exit Gazebo...");
  ros::shutdown();
  return 0;
}