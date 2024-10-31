/**
 * marslite_simulation_ws/test/test_pose_planning.cpp
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
using marslite::exception::AssertionFailedException;

void addTestInDegree(const char* testcase, const double& Q1, const double& Q2, const double& Q3,
                     const double& Q4, const double& Q5, const double& Q6)
{
  // Drive the robot back to the HOME pose, and then drive it to the target pose
  try {
    MarsliteControl::ControlPtr control_ptr = std::make_shared<MarsliteControl>();

    control_ptr->updateInitialPoseFromCurrent();
    control_ptr->setTargetPose(marslite::pose::HOME);
    ROS_ASSERT_MSG(control_ptr->trajectoryPlanningQPSolver(),
        "[%s] Failed to move the robot to the HOME pose.", testcase);
    
    StateVector target_pose = (StateVector() <<
        deg2Rad(Q1), deg2Rad(Q2), deg2Rad(Q3),
        deg2Rad(Q4), deg2Rad(Q5), deg2Rad(Q6), 0, 0
    ).finished();
    control_ptr->updateInitialPoseFromCurrent();
    control_ptr->setTargetPose(target_pose);
    ROS_ASSERT_MSG(control_ptr->trajectoryPlanningQPSolver(),
        "[%s] Failed to move the robot to the target pose.", testcase);
  } catch (const AssertionFailedException& e) {
    throw e;        
  }
  ROS_INFO("[%s] Test passed.", testcase);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "test_pose_planning");

  ros::AsyncSpinner spinner(0);
  spinner.start();

  try {
    addTestInDegree("Test 1", -0.012, 2.145, 86.246, -178.373, 89.999, -89.971);
    addTestInDegree("Test 2", -0.012, 85.459, -86.246, -89.195, 89.999, -89.971);
    addTestInDegree("Test 3", -0.012, -35.656, 112.621, 13.054, -89.999, 90.029);
    addTestInDegree("Test 4", -0.012, 72.271, -112.621, 130.368, -89.999, 90.029);
    addTestInDegree("Test 5", 121.374, -72.273, 112.624, -130.361, 89.985, 148.643);
    addTestInDegree("Test 6", 121.374, 35.657, -112.624, -13.044, 89.985, 148.643);
    addTestInDegree("Test 7", 121.374, -85.457, 86.243, 89.203, -89.985, -31.357);
    addTestInDegree("Test 8", 121.374, -2.145, -86.243, 178.378, -89.985, -31.357);
  } catch (const ConstructorInitializationFailedException& ex) {
    ROS_ERROR_STREAM(ex.what());
  } catch (const TimeOutException& ex) {
    ROS_ERROR_STREAM(ex.what());
  }

  ros::shutdown();
  return 0;
}