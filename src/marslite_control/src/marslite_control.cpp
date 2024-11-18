/**
 * marslite_simulation_ws/marslite_control/src/marslite_control.cpp
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

#include "marslite_control/marslite_control.h"

/**
 * @namespace marslite operation namespace
*/
namespace marslite {

/**
 * @namespace control namespace for marslite robots
*/
namespace control {

MarsliteControl::MarsliteControl(const ros::NodeHandle& nh)
    : nh_(nh), loop_rate_(ros::Rate(25)), debug_msg_enabled_(true),
    timeout_(ros::Duration(30)), polling_sleep_duration_(ros::Duration(0.1)),
    is_hand_trigger_pressed_(false), is_index_trigger_pressed_(false)
{
  try {
    mpc_ptr_ = std::make_shared<ModelPredictiveControl>();
    kinematics_ptr_ = std::make_shared<TMKinematics>(TM5_700);
  } catch (const ConstructorInitializationFailedException& ex) {
    throw ex;
  }

  this->parseParameters();

  try {
    this->subscribeToJointState();
    this->connectJointTrajectoryActionServer();
    if (use_joystick_) {
      this->subscribeToLeftJoyPose();
      this->subscribeToLeftJoy();
    }
  } catch (const TimeOutException& ex) {
    ROS_ERROR("%s", ex.what());
    throw ConstructorInitializationFailedException();
  }

  // Set the joint names for the trajectory
  trajectory_goal_.trajectory.joint_names = {
    "tm_shoulder_1_joint",
    "tm_shoulder_2_joint",
    "tm_elbow_joint",
    "tm_wrist_1_joint",
    "tm_wrist_2_joint",
    "tm_wrist_3_joint"
  };
}

/* ********************************************** *
 *                Public members                  *
 * ********************************************** */

void MarsliteControl::joystickTeleoperation()
{
  tf::Transform previous_left_joy_pose_TF;
  tf::Transform current_left_joy_pose_TF;
  
  tf::Transform previous_gripper_TF;
  tf::Transform desired_gripper_TF;
  tf::Transform relative_gripper_TF;

  bool first_initialization = true;
  while (ros::ok()) {
    do {  // dummy do-while loop for the break statement
      if (is_hand_trigger_pressed_ && is_index_trigger_pressed_) {
        if (first_initialization) {
          previous_left_joy_pose_TF = current_left_joy_pose_TF = this->getLeftJoyPoseTF();
          previous_gripper_TF = desired_gripper_TF = this->getGripperTF();
          first_initialization = false;

          ROS_INFO_STREAM("Begin the teleoperation...");
          break;
        }

        current_left_joy_pose_TF = this->getLeftJoyPoseTF();
        if (this->isSamePose(previous_left_joy_pose_TF, current_left_joy_pose_TF)) {
          ROS_WARN_THROTTLE(2, "The left joystick is not moving. Skip the teleoperation...");
          break;
        }

        relative_gripper_TF = this->getScaledRelativeTF(
          previous_left_joy_pose_TF,
          current_left_joy_pose_TF
        );
        desired_gripper_TF = previous_gripper_TF * relative_gripper_TF * robotiq_to_flange_TF;
        if (!this->moveGripper(previous_gripper_TF, desired_gripper_TF)) {
          ROS_ERROR_STREAM("Failed to move the gripper. Ignore the command...");
          break;
        }

        previous_left_joy_pose_TF = current_left_joy_pose_TF;
        previous_gripper_TF = desired_gripper_TF * robotiq_to_flange_TF.inverse();
      } else {
        first_initialization = true;
      }
    } while (false);
    
    loop_rate_.sleep();
  }
}

bool MarsliteControl::moveGripper(const tf::Transform& previous_gripper_TF,
                                  const tf::Transform& desired_gripper_TF)
{
  TMKinematics::TransformationMatrix desired_gripper_TM =
      this->convertToTransformationMatrix(desired_gripper_TF);
  
  Eigen::MatrixXd IKSolution = kinematics_ptr_->inverseKinematics(desired_gripper_TM);
  if (IKSolution.rows() == 0) {
    ROS_WARN_STREAM("Cannot find any inverse kinematics solution. Ignore the command...");
    return false;
  }
  
  StateVector desired_joint_states = this->findClosestValidJointAngles(IKSolution);
  if (desired_joint_states == StateVector::Zero()) {
    ROS_WARN_STREAM("Cannot find any valid solution. Ignore the command...");
    return false;
  }

  if (this->isTargetJointTooFar(desired_joint_states) &&
      this->isTargetPositionTooFar(previous_gripper_TF, desired_gripper_TF)) {
    ROS_WARN_STREAM("Solution too far from the current joint states. Ignore the command...");
    return false;
  }

  if (!this->planTrajectory(desired_joint_states)) {
    ROS_WARN_STREAM("Cannot plan the trajectory. Ignore the command...");
    return false;
  }

  return true;
}

bool MarsliteControl::planTrajectoryWithQPSolver()
{  
  // Solve the trajectory planning problem
  if (!mpc_ptr_->solveQP(trajectory_goal_.trajectory.points))
    return false;

  // Send the trajectory goal to the server, and wait for the result
  trajectory_action_client_->sendGoal(trajectory_goal_);
  trajectory_action_client_->waitForResult(ros::Duration(loop_rate_));

  // Check if the action was successful
  ROS_INFO_STREAM_COND(debug_msg_enabled_, "  Planning result: " << trajectory_action_client_->getState().toString());
  if (trajectory_action_client_->getState() == actionlib::SimpleClientGoalState::LOST) {
    return false;
  }

  // Clear the trajectory goal before the next planning
  trajectory_goal_.trajectory.points.clear();
  return true;
}

bool MarsliteControl::planTrajectoryWithQPSolver(const ros::Duration& timeout)
{
  // Solve the trajectory planning problem
  if (!mpc_ptr_->solveQP(trajectory_goal_.trajectory.points))
    return false;

  // Send the trajectory goal to the server, and wait for the result
  trajectory_action_client_->sendGoal(trajectory_goal_);
  trajectory_action_client_->waitForResult(timeout);

  // Check if the action was successful
  ROS_INFO_STREAM_COND(debug_msg_enabled_, "  Planning result: " << trajectory_action_client_->getState().toString());
  if (trajectory_action_client_->getState() == actionlib::SimpleClientGoalState::LOST) {
    return false;
  }

  // Clear the trajectory goal before the next planning
  trajectory_goal_.trajectory.points.clear();
  return true;
}

TMKinematics::TransformationMatrix MarsliteControl::convertToTransformationMatrix(
    const tf::Transform& transform) const
{
  const double nx = transform.getBasis().getColumn(0).getX();
  const double ny = transform.getBasis().getColumn(0).getY();
  const double nz = transform.getBasis().getColumn(0).getZ();
  const double ox = transform.getBasis().getColumn(1).getX();
  const double oy = transform.getBasis().getColumn(1).getY();
  const double oz = transform.getBasis().getColumn(1).getZ();
  const double ax = transform.getBasis().getColumn(2).getX();
  const double ay = transform.getBasis().getColumn(2).getY();
  const double az = transform.getBasis().getColumn(2).getZ();
  const double px = transform.getOrigin().getX();
  const double py = transform.getOrigin().getY();
  const double pz = transform.getOrigin().getZ();

  TMKinematics::TransformationMatrix TM;
  TM << nx, ox, ax, px,
        ny, oy, ay, py,
        nz, oz, az, pz,
        0,  0,  0,  1;
  return TM;
}

/* ********************************************** *
 *               Private members                  *
 * ********************************************** */

void MarsliteControl::parseParameters()
{
  ros::NodeHandle pnh("~");
  pnh.param("debug_msg_enabled", debug_msg_enabled_, false);
  pnh.param("use_joystick", use_joystick_, true);
  pnh.param("position_scale", position_scale_, 1.0);
  pnh.param("orientation_scale", orientation_scale_, 1.0);
  pnh.param("position_difference_threshold", position_difference_threshold_, 0.01);
  pnh.param("orientation_difference_threshold", orientation_difference_threshold_, 0.01);
}

void MarsliteControl::connectJointTrajectoryActionServer()
{
  // Initialize the `FollowJointTrajectoryAction` action client
  trajectory_action_client_ = std::make_shared<JointTrajectoryAction>(
    "/arm_controller/follow_joint_trajectory", true
  );

  // Wait for the connection to the server
  ROS_INFO_STREAM("Connecting to FollowJointTrajectoryAction action server...");
  if (!trajectory_action_client_->waitForServer(timeout_))
    throw TimeOutException(timeout_);
  ROS_INFO_STREAM("FollowJointTrajectoryAction action server connected!");
}

tf::Transform MarsliteControl::lookUpTransformWithTimeout(
    const std::string& target_frame,
    const std::string& source_frame) const
{
  tf::TransformListener listener;
  tf::StampedTransform stampedTF;
  tf::Transform TF;

  try {
    // ROS_INFO_STREAM_COND(debug_msg_enabled_, "Obtain Transform from "
    //     << target_frame << " to " << source_frame << " ...");
    listener.waitForTransform(target_frame, source_frame, ros::Time(0),
        timeout_, polling_sleep_duration_);
    listener.lookupTransform(target_frame, source_frame, ros::Time(0), stampedTF);
    TF.setOrigin(stampedTF.getOrigin());
    TF.setRotation(stampedTF.getRotation());
    // ROS_INFO_STREAM_COND(debug_msg_enabled_, "Transform obtained!");
  } catch (tf::TransformException &ex) {
    throw TransformNotFoundException(target_frame, source_frame);
  }

  return TF;
}

tf::StampedTransform MarsliteControl::lookUpStampedTransformWithTimeout(
    const std::string& target_frame,
    const std::string& source_frame) const
{
  tf::TransformListener listener;
  tf::StampedTransform stampedTF;

  try {
    // ROS_INFO_STREAM_COND(debug_msg_enabled_, "Obtain StampedTransform from "
    //     << target_frame << " to " << source_frame << " ...");
    listener.waitForTransform(target_frame, source_frame, ros::Time(0),
        timeout_, polling_sleep_duration_);
    listener.lookupTransform(target_frame, source_frame, ros::Time(0), stampedTF);
    // ROS_INFO_STREAM_COND(debug_msg_enabled_, "StampedTransform obtained!");
  } catch (tf::TransformException &ex) {
    throw TransformNotFoundException(target_frame, source_frame);
  }

  return stampedTF;
}

tf::Transform MarsliteControl::getScaledRelativeTF(
    const tf::Transform& initial_TF,
    const tf::Transform& current_TF) const
{
  tf::Transform relative_TF = initial_TF.inverseTimes(current_TF);
  tf::Vector3 scaled_position = this->getScaledPosition(relative_TF);
  tf::Quaternion scaled_orientation = this->getScaledOrientation(relative_TF);
  relative_TF.setOrigin(scaled_position);
  relative_TF.setRotation(scaled_orientation);
  return relative_TF;
}

MarsliteControl::StateVector MarsliteControl::findClosestValidJointAngles(
    const Eigen::MatrixXd& IKSolution) const
{
  StateVector desired_joint_states = StateVector::Zero();
  double min_distance = std::numeric_limits<double>::max();
  double current_distance = 0;
  bool is_valid = true;

  for (uint8_t i = 0; i < IKSolution.rows(); ++i) {
    is_valid = true;
    for (uint8_t j = 0; j < 6; ++j) {
      if (IKSolution(i,j) > marslite::constraint::POSITION_MAX(j) || 
          IKSolution(i,j) < marslite::constraint::POSITION_MIN(j)) {
        is_valid = false;
        break;
      }
    }
    if (!is_valid) continue;

    current_distance = (IKSolution.row(i).transpose()-joint_states_.block(0, 0, 6, 1)).norm();
    if (current_distance < min_distance) {
      min_distance = current_distance;
      desired_joint_states.block(0, 0, 6, 1) = IKSolution.row(i).transpose();
    }
  }

  return desired_joint_states;
}


void MarsliteControl::jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg)
{
  joint_states_ << this->getShoulder1JointAngle(msg),
                   this->getShoulder2JointAngle(msg),
                   this->getElbowJointAngle(msg),
                   this->getWrist1JointAngle(msg),
                   this->getWrist2JointAngle(msg),
                   this->getWrist3JointAngle(msg),
                   0,  // mobile base position
                   0;  // mobile base orientation
}

void MarsliteControl::leftJoyCallback(const sensor_msgs::Joy::ConstPtr& msg)
{
  switch (msg->axes.size()) {
    case 4:
      // [3] primary hand trigger
      is_hand_trigger_pressed_ = (msg->axes[3] > kTriggerThreshold);
    case 3:
      // [2] primary index trigger
      is_index_trigger_pressed_ = (msg->axes[2] > kTriggerThreshold);
      break;
    case 0:
      ROS_WARN_ONCE("[leftJoyCallback] Failed to receive the joy topic.");
      ROS_WARN_ONCE("  Please check your joystick(s) setup or rosbridge connection.");
      break;
    default:
      ROS_WARN_ONCE("[leftJoyCallback] Mismatch number of left joystick axes (%lu).", msg->axes.size());
      ROS_WARN_ONCE("  Please check your joystick(s) setup or rosbridge connection.");
      break;
  }
}

void MarsliteControl::leftJoyPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
  left_joy_pose_ = *msg;
}

}  // namespace control

}  // namespace marslite