/**
 * marslite_simulation_ws/test/test_joystick_teleoperation.cpp
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

int main(int argc, char** argv)
{
  ros::init(argc, argv, "test_joystick_teleoperation");

  ros::AsyncSpinner spinner(0);
  spinner.start();

  try {
    MarsliteControl::ControlPtr control_ptr= std::make_shared<MarsliteControl>();
    control_ptr->updateInitialPoseFromCurrent();
    control_ptr->setTargetPose(marslite::pose::HOME);
    control_ptr->planTrajectoryWithQPSolver();
    ROS_INFO("Preparation done.");

    control_ptr->joystickTeleoperation();
  } catch (const ConstructorInitializationFailedException& ex) {
    ROS_ERROR_STREAM(ex.what());
  }
  
  ROS_INFO("All tests passed! Type Ctrl+C to exit Gazebo...");
  ros::waitForShutdown();
  return 0;
}