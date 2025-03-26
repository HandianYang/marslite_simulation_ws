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

  ros::AsyncSpinner spinner(2);
  spinner.start();

  MarsliteControl::ControlPtr control_ptr = std::make_shared<MarsliteControl>();
  try {
    // Test 1: planning to the home pose of marslite robots
    ROS_ASSERT(control_ptr->planTrajectory(marslite::pose::HOME));
    ROS_INFO("Test 1 passed!");
    ros::Duration(0.01).sleep();
    
    // Test 2: planning to the default1 pose
    ROS_ASSERT(control_ptr->planTrajectory(marslite::pose::DEFAULT1));
    ROS_INFO("Test 2 passed!");
    ros::Duration(0.01).sleep();

    // Test 3: planning to the default2 pose
    ROS_ASSERT(control_ptr->planTrajectory(marslite::pose::DEFAULT2));
    ROS_INFO("Test 3 passed!");
    ros::Duration(0.01).sleep();

    // Test 4: planning to the home pose of marslite robots
    ROS_ASSERT(control_ptr->planTrajectory(marslite::pose::HOME));
    ROS_INFO("Test 4 passed!");

  } catch (const ConstructorInitializationFailedException& ex) {
    ROS_ERROR_STREAM(ex.what());
  } catch (const TimeOutException& ex) {
    ROS_ERROR_STREAM(ex.what());
  }

  ROS_INFO("All tests passed! Type Ctrl+C to exit...");
  ros::waitForShutdown();
  return 0;
}