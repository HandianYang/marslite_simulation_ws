/**
 * marslite_simulation_ws/marslite_control/include/marslite_control.h
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

#ifndef MARSLITE_CONTROL_MARSLITE_CONTROL_H_
#define MARSLITE_CONTROL_MARSLITE_CONTROL_H_

#include <ros/ros.h>
#include <vector>
#include <mutex>

#include <Eigen/Dense>              // eigen
#include "OsqpEigen/OsqpEigen.h"    // osqp-eigen

#include <sensor_msgs/JointState.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/PoseStamped.h>

#include <actionlib/client/simple_action_client.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <trajectory_msgs/JointTrajectory.h>

#include <tf/transform_listener.h>

#include "model_predictive_control/model_predictive_control.h"
using marslite::control::ModelPredictiveControl;

#include "marslite_properties/Exception.h"
using marslite::exception::TimeOutException;
using marslite::exception::ConstructorInitializationFailedException;
using marslite::exception::TransformNotFoundException;

#include "marslite_properties/Time.h"
using marslite::time::subscribeTopicWithTimeout;

#include "tm_kinematics/tm_kinematics.h"
using marslite::control::TMKinematics;

/**
 * @namespace marslite operation namespace
*/
namespace marslite {

/**
 * @namespace control namespace for marslite robots
*/
namespace control {

static constexpr double kTriggerThreshold = 0.95;
static tf::Transform base_to_tool_rotation_TF(tf::Quaternion(0.5, 0.5, 0.5, 0.5), tf::Vector3(0, 0, 0));
static tf::Transform tool_to_base_rotation_TF = base_to_tool_rotation_TF.inverse();

/**
 * @brief The class for marslite control
*/
class MarsliteControl {
public:
  using ControlPtr = std::shared_ptr<MarsliteControl>;
  using JointTrajectoryAction = actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>;
  using StateVector = Eigen::Matrix<double, MPC_STATE_SIZE, 1>;
  using InputVector = Eigen::Matrix<double, MPC_INPUT_SIZE, 1>;
  using TransformationMatrix = Eigen::Matrix<double, 4, 4>;

  explicit MarsliteControl(const ros::NodeHandle& nh = ros::NodeHandle());
  
  /**
   * @brief The main function for joystick teleoperation.
   */
  void joystickTeleoperation();

  /**
   * @brief Drive the robotic gripper from the current pose to the desired pose.
   * @param previous_gripper_TF The previous pose of the gripper.
   * @param desired_gripper_TF The desired pose of the gripper.
   * @return True if the gripper moves successfully. Specifically, the function
   *        returns `FALSE` if:
   *        (1) the inverse kinematics solution is not found;
   *        (2) the solutions are all invalid;
   *        (3) the valid solution is too far from the current joint states;
   *        (4) the trajectory planning fails.
   */
  bool moveGripper(const tf::Transform& previous_gripper_TF,
                   const tf::Transform& desired_gripper_TF);

  /**
   * @brief The process of planning a trajectory for the marslite robot.
   * @param desired_joint_states The desired joint states.
   * @return True if the trajectory is successfully planned.
   * @note Make sure that `desired_joint_states` is valid.
   */
  inline bool planTrajectory(const StateVector& desired_joint_states)
  {
    if (!this->updateInitialPoseFromCurrent()) {
      ROS_ERROR_STREAM("  Failed to update the initial pose from the current pose.");
      return false;
    }
    if (!this->setTargetPose(desired_joint_states)) {
      ROS_ERROR_STREAM("  Failed to set the target pose.");
      return false;
    }
    if (!this->planTrajectoryWithQPSolver()) {
      ROS_ERROR_STREAM("  Failed to solve the trajectory planning problem.");
      return false;
    }

    return true;
  }

  /**
   * @brief Update the initial pose of the marslite robot from the current
   *        joint states.
   * @return True if `joint_states_` is not out of bound.
   */
  inline bool updateInitialPoseFromCurrent()
  {
    return this->setInitialPose(joint_states_);
  }

  /**
   * @brief Sets the initial pose for the marslite robot.
   * @param initial_pose The initial pose to set, with size `MPC_STATE_SIZE x 1`.
   *                    Defaults to `marslite::pose::INITIAL`.
   * @return True if `initial_pose` is not out of bound.
   */
  inline bool setInitialPose(const StateVector& initial_pose = marslite::pose::INITIAL)
  {
    return mpc_ptr_->setInitialPose(initial_pose);
  }

  /**
   * @brief Sets the target pose for the marslite robot.
   * @param target_pose The target pose vector, with size `MPC_STATE_SIZE x 1`.
   *                   Defaults to `marslite::pose::INITIAL`.
   * @return True if `target_pose` is not out of bound.
   */
  inline bool setTargetPose(const StateVector& target_pose = marslite::pose::INITIAL)
  {
    return mpc_ptr_->setTargetPose(target_pose);
  }

  /**
   * @brief Plan the trajectory of the robot using QP solver. The planner keeps waiting
   *        for the result until it is executed successfully.
   * @return True if the trajectory can be executed, and it is executed successfully.
  */
  bool planTrajectoryWithQPSolver();

  /**
   * @brief Plan the trajectory of the robot using QP solver with the given timeout.
   *        The planner waits for the result within the specified timeout duration.
   * @param timeout The timeout duration.
   * @return True if the trajectory can be executed, and it is executed successfully.
  */
  bool planTrajectoryWithQPSolver(const ros::Duration& timeout);

  /**
   * @brief Get the angle value of the first joint (i.e. `tm_shoulder_1_joint`)
   *        of the TM5 robotic arm.
   * @param joint_state joint states of the marslite robot.
   * @return The angle value in radians.
   */
  inline double getShoulder1JointAngle(const sensor_msgs::JointState::ConstPtr& joint_state) const
  {
    try {
      return use_sim_ ? joint_state->position.at(4) : joint_state->position.at(2);
    } catch (const std::out_of_range& e) {
      ROS_ERROR("Error: %s. Return 0 instead...", e.what());
    }
    return 0.0;
  }

  /**
   * @brief Get the angle value of the second joint (i.e. `tm_shoulder_2_joint`)
   *        of the TM5 robotic arm.
   * @param joint_state joint states of marslite robot.
   * @return The angle value in radians.
   */
  inline double getShoulder2JointAngle(const sensor_msgs::JointState::ConstPtr& joint_state) const
  {
    try {
      return use_sim_ ? joint_state->position.at(5) : joint_state->position.at(3);
    } catch (const std::out_of_range& e) {
      ROS_ERROR("Error: %s. Return 0 instead...", e.what());
    }
    return 0.0;
  }

  /**
   * @brief Get the angle value of the third joint (i.e. `tm_elbow_joint`)
   *        of the TM5 robotic arm.
   * @param joint_state joint states of marslite robot.
   * @return The angle value in radians.
   */
  inline double getElbowJointAngle(const sensor_msgs::JointState::ConstPtr& joint_state) const
  {
    try {
      return use_sim_ ? joint_state->position.at(3) : joint_state->position.at(4);
    } catch (const std::out_of_range& e) {
      ROS_ERROR("Error: %s. Return 0 instead...", e.what());
    }
    return 0.0;
  }

  /**
   * @brief Get the angle value of the fourth joint (i.e. `tm_wrist_1_joint`)
   *        of the TM5 robotic arm.
   * @param joint_state joint states of marslite robot.
   * @return The angle value in radians.
   */
  inline double getWrist1JointAngle(const sensor_msgs::JointState::ConstPtr& joint_state) const
  {
    try {
      return use_sim_ ? joint_state->position.at(6) : joint_state->position.at(5);
    } catch (const std::out_of_range& e) {
      ROS_ERROR("Error: %s. Return 0 instead...", e.what());
    }
    return 0.0;
  }

  /**
   * @brief Get the angle value of the fifth joint (i.e. `tm_wrist_2_joint`)
   *        of the TM5 robotic arm.
   * @param joint_state joint states of marslite robot.
   * @return The angle value in radians.
   */
  inline double getWrist2JointAngle(const sensor_msgs::JointState::ConstPtr& joint_state) const
  {
    try {
      return use_sim_ ? joint_state->position.at(7) : joint_state->position.at(6);
    } catch (const std::out_of_range& e) {
      ROS_ERROR("Error: %s. Return 0 instead...", e.what());
    }
    return 0.0;
  }

  /**
   * @brief Get the angle value of the sixth joint (i.e. `tm_wrist_3_joint`)
   *        of the TM5 robotic arm.
   * @param joint_state joint states of marslite robot.
   * @return The angle value in radians.
   */
  inline double getWrist3JointAngle(const sensor_msgs::JointState::ConstPtr& joint_state) const
  {
    try {
      return use_sim_ ? joint_state->position.at(8) : joint_state->position.at(7);
    } catch (const std::out_of_range& e) {
      ROS_ERROR("Error: %s. Return 0 instead...", e.what());
    }
    return 0.0;
  }

  inline tf::Transform getGripperTF() const
  {
    try {
      return use_sim_ ?
          this->lookUpTransformWithTimeout("/tm_base", "/robotiq_85_base_link") :
          this->lookUpTransformWithTimeout("/tm_base", "/tm_tip_link");
    } catch (const TransformNotFoundException& e) {
      ROS_ERROR("Error: %s. Return default constructor of tf::Transform instead...", e.what());
    }
    return tf::Transform();
  }

  inline tf::Transform getLeftJoyPoseTF() const
  {
    tf::Transform left_joy_pose_TF;
    tf::poseMsgToTF(left_joy_pose_.pose, left_joy_pose_TF);
    return left_joy_pose_TF;
  }
  
  /**
   * @brief Check if the target joint states are too far from the current joint states.
   * @param desired_joint_states The desired joint states.
   */
  inline bool isTargetJointTooFar(const StateVector& desired_joint_states) const
  {
    for (uint8_t i = 0; i < 6; ++i) {
      if (std::abs(desired_joint_states(i) - joint_states_(i)) > 0.1) {
        return true;
      }
    }
    return false;
  }

  /**
   * @brief Check if the target position is too far from the current position.
   * @param previous_gripper_TF The previous pose of the gripper.
   * @param desired_gripper_TF The desired pose of the gripper.
   */
  inline bool isTargetPositionTooFar(const tf::Transform& previous_gripper_TF,
                                     const tf::Transform& desired_gripper_TF) const
  {
    return (previous_gripper_TF.getOrigin() - desired_gripper_TF.getOrigin()).length() > 0.05;
  }

  /**
   * @brief Drive the robotic gripper forward by a distance.
   * @param distance The distance to move forward (in meters).
   * @return True if the gripper moves forward successfully.
   */
  inline bool moveGripperForward(const double& distance)
  {
    tf::Transform initial_TF = this->getGripperTF();
    tf::Transform relative_TF(tf::Quaternion(0, 0, 0, 1), tf::Vector3(distance, 0, 0));
    tf::Transform desired_TF = use_sim_ ?
        initial_TF * relative_TF * base_to_tool_rotation_TF : 
        initial_TF * tool_to_base_rotation_TF * relative_TF * base_to_tool_rotation_TF;
    return this->moveGripper(initial_TF, desired_TF);
  }

  /**
   * @brief Drive the robotic gripper backward by a distance.
   * @param distance The distance to move backward (in meters).
   * @return True if the gripper moves backward successfully.
   */
  inline bool moveGripperBackward(const double& distance)
  {
    tf::Transform initial_TF = this->getGripperTF();
    tf::Transform relative_TF(tf::Quaternion(0, 0, 0, 1), tf::Vector3(-distance, 0, 0));
    tf::Transform desired_TF = use_sim_ ?
        initial_TF * relative_TF * base_to_tool_rotation_TF : 
        initial_TF * tool_to_base_rotation_TF * relative_TF * base_to_tool_rotation_TF;
    return this->moveGripper(initial_TF, desired_TF);
  }

  /**
   * @brief Drive the robotic gripper left by a distance.
   * @param distance The distance to move left (in meters).
   * @return True if the gripper moves left successfully.
   */
  inline bool moveGripperLeft(const double& distance)
  {
    tf::Transform initial_TF = this->getGripperTF();
    tf::Transform relative_TF(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0, distance, 0));
    tf::Transform desired_TF = use_sim_ ?
        initial_TF * relative_TF * base_to_tool_rotation_TF : 
        initial_TF * tool_to_base_rotation_TF * relative_TF * base_to_tool_rotation_TF;
    return this->moveGripper(initial_TF, desired_TF);
  }

  /**
   * @brief Drive the robotic gripper right by a distance.
   * @param distance The distance to move right (in meters).
   * @return True if the gripper moves right successfully.
   */
  inline bool moveGripperRight(const double& distance)
  {
    tf::Transform initial_TF = this->getGripperTF();
    tf::Transform relative_TF(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0, -distance, 0));
    tf::Transform desired_TF = use_sim_ ?
        initial_TF * relative_TF * base_to_tool_rotation_TF : 
        initial_TF * tool_to_base_rotation_TF * relative_TF * base_to_tool_rotation_TF;
    return this->moveGripper(initial_TF, desired_TF);
  }

  /**
   * @brief Drive the robotic gripper up by a distance.
   * @param distance The distance to move up (in meters).
   * @return True if the gripper moves up successfully.
   */
  inline bool moveGripperUp(const double& distance)
  {
    tf::Transform initial_TF = this->getGripperTF();
    tf::Transform relative_TF(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0, 0, distance));
    tf::Transform desired_TF = use_sim_ ?
        initial_TF * relative_TF * base_to_tool_rotation_TF : 
        initial_TF * tool_to_base_rotation_TF * relative_TF * base_to_tool_rotation_TF;
    return this->moveGripper(initial_TF, desired_TF);
  }

  /**
   * @brief Drive the robotic gripper down by a distance.
   * @param distance The distance to move down (in meters).
   * @return True if the gripper moves down successfully.
   */
  inline bool moveGripperDown(const double& distance)
  {
    tf::Transform initial_TF = this->getGripperTF();
    tf::Transform relative_TF(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0, 0, -distance));
    tf::Transform desired_TF = use_sim_ ?
        initial_TF * relative_TF * base_to_tool_rotation_TF : 
        initial_TF * tool_to_base_rotation_TF * relative_TF * base_to_tool_rotation_TF;
    return this->moveGripper(initial_TF, desired_TF);
  }

  /**
   * @brief Convert a `tf::Transform` object to a `TransformationMatrix`
   *        object, which is `Eigen::Matrix<double, 4, 4>`.
   * @param transform The `tf::Transform` object.
   * @return The transformation matrix.
   */
  TMKinematics::TransformationMatrix convertToTransformationMatrix(
      const tf::Transform& transform
  ) const;


private:

  ModelPredictiveControl::MPCPtr mpc_ptr_;
  TMKinematics::TMKinematicsPtr kinematics_ptr_;

  ros::NodeHandle nh_;
  ros::Subscriber joint_state_subscriber_;
  ros::Subscriber left_joy_subscriber_;
  ros::Subscriber left_joy_pose_subscriber_;
  StateVector joint_states_;
  geometry_msgs::PoseStamped left_joy_pose_;
  tf::TransformListener tf_listener_;

  ros::Rate loop_rate_;
  ros::Duration timeout_;
  ros::Duration polling_sleep_duration_;

  std::shared_ptr<JointTrajectoryAction> trajectory_action_client_; 
  control_msgs::FollowJointTrajectoryGoal trajectory_goal_;
  
  bool use_sim_;
  bool debug_msg_enabled_;
  bool use_joystick_;
  bool is_hand_trigger_pressed_;
  bool is_index_trigger_pressed_;
  bool is_button_X_pressed_over_3s_;
  bool is_button_Y_pressed_;

  double position_scale_;
  double orientation_scale_;
  double position_difference_threshold_;
  double orientation_difference_threshold_;

private:
  /**
   * @brief Parse the parameters from the ROS parameter server.
   */
  void parseParameters();

  /**
   * @brief Connect to the `FollowJointTrajectoryAction` Server.
   * @throw `marslite::exceptions::TimeOutException` if the connection
   *        is not successful within the timeout.
   */
  void connectJointTrajectoryActionServer();

  /**
   * @brief Subscribe to the `joint_state` topic.
   * @throw `marslite::exceptions::TimeOutException` if the subscription
   *         is not successful within the timeout.
   */
  inline void subscribeToJointState() {
    try {
      subscribeTopicWithTimeout<MarsliteControl, sensor_msgs::JointState>(
        this, nh_, joint_state_subscriber_, "/joint_states",
        1, &MarsliteControl::jointStateCallback,
        debug_msg_enabled_, timeout_, polling_sleep_duration_
      );
    } catch (const TimeOutException& e) {
      throw e;
    }
  }

  /**
   * @brief Subscribe to `/unity/joy_pose/left` topic, which contains the
   *        pose information of the left joystick.
   * @throw `marslite::exceptions::TimeOutException` if the subscription
   *        is not successful within the timeout.
   */
  inline void subscribeToLeftJoyPose() {
    try {
      subscribeTopicWithTimeout<MarsliteControl, geometry_msgs::PoseStamped>(
        this, nh_, left_joy_pose_subscriber_, "/unity/joy_pose/left",
        1, &MarsliteControl::leftJoyPoseCallback,
        debug_msg_enabled_, timeout_, polling_sleep_duration_
      );
    } catch (const TimeOutException& e) {
      throw e;
    }
  }

  /**
   * @brief Subscribe to `/unity/joy/left` topic, which contains the
   *        joystick information of the left joystick.
   * @throw `marslite::exceptions::TimeOutException` if the subscription
   *        is not successful within the timeout.
   */
  inline void subscribeToLeftJoy() {
    try {
      subscribeTopicWithTimeout<MarsliteControl, sensor_msgs::Joy>(
        this, nh_, left_joy_subscriber_, "/unity/joy/left",
        1, &MarsliteControl::leftJoyCallback,
        debug_msg_enabled_, timeout_, polling_sleep_duration_
      );
    } catch (const TimeOutException& e) {
      throw e;
    }
  }
    
  /**
   * @brief Get `StampedTransform` within timeout between two frames by frame ID.
   * @param target_frame The target frame ID (in type `std::string`).
   * @param source_frame The source frame ID (in type `std::string`).
   * @return `StampedTransform` between the two frames.
   * @throw `marslite::exceptions::TransformNotFoundException` if the transform
   *        is not found within the timeout.
   */
  tf::StampedTransform lookUpStampedTransformWithTimeout(
      const std::string& target_frame,
      const std::string& source_frame
  ) const;

  /**
   * @brief Get `Transform` within timeout between two frames by frame ID.
   * @param target_frame The target frame ID (in type `std::string`).
   * @param source_frame The source frame ID (in type `std::string`).
   * @return `Transform` between the two frames.
   * @throw `marslite::exceptions::TransformNotFoundException` if the transform
   *        is not found within the timeout.
   */
  tf::Transform lookUpTransformWithTimeout(
      const std::string& target_frame,
      const std::string& source_frame
  ) const;

  /**
   * @brief Check if two `tf::Transform` objects are the same pose.
   * @param tf1 The first `tf::Transform` object.
   * @param tf2 The second `tf::Transform` object.
   */
  inline bool isSamePose(const tf::Transform& tf1, const tf::Transform& tf2) const
  {
    const double position_distance = this->getTFPositionDistance(tf1, tf2);
    const double orientation_distance = this->getTFOrientationDistance(tf1, tf2);
    return (position_distance < position_difference_threshold_)
        && (orientation_distance < orientation_difference_threshold_);
  }

  /**
   * @brief Get the desired grip pose based on the joystick pose.
   * @param initial_left_joy_pose_TF The initial pose of the left joystick.
   * @param current_left_joy_pose_TF The current pose of the left joystick.
   * @return The desired pose of the robotic grip.
   */
  tf::Transform getScaledRelativeTF(
      const tf::Transform& initial_left_joy_pose_TF,
      const tf::Transform& current_left_joy_pose_TF) const;
  
  /**
   * @brief Get the scaled position based on the relative `tf::Transform`.
   * @param relative_TF The relative `tf::Transform` object.
   * @return The scaled position.
   */
  inline tf::Vector3 getScaledPosition(const tf::Transform& relative_TF) const
  {
    return relative_TF.getOrigin() * position_scale_;
  }

  /**
   * @brief Get the scaled orientation based on the relative `tf::Transform`.
   * @param relative_TF The relative `tf::Transform` object.
   * @return The scaled orientation (in type `tf::Quaternion`).
   */
  inline tf::Quaternion getScaledOrientation(const tf::Transform& relative_TF) const
  {
    double roll, pitch, yaw;
    tf::Matrix3x3 m(relative_TF.getRotation());
    m.getRPY(roll, pitch, yaw);
    return tf::createQuaternionFromRPY(
        roll * orientation_scale_,
        pitch * orientation_scale_,
        yaw * orientation_scale_
    );
  }

  /**
   * @brief Get the position distance between two `tf::Transform` objects.
   * @param tf1 The first `tf::Transform` object.
   * @param tf2 The second `tf::Transform` object.
   * @return The position distance.
   */
  inline double getTFPositionDistance(const tf::Transform& tf1, const tf::Transform& tf2) const
  {
    return tf1.getOrigin().distance(tf2.getOrigin());
  }

  /**
   * @brief Get the orientation distance between two `tf::Transform` objects.
   * @param tf1 The first `tf::Transform` object.
   * @param tf2 The second `tf::Transform` object.
   * @return The orientation distance.
   */
  inline double getTFOrientationDistance(const tf::Transform& tf1, const tf::Transform& tf2) const
  {
    Eigen::Quaterniond q1(tf1.getRotation().getW(), tf1.getRotation().getX(),
        tf1.getRotation().getY(), tf1.getRotation().getZ());
    Eigen::Quaterniond q2(tf2.getRotation().getW(), tf2.getRotation().getX(),
        tf2.getRotation().getY(), tf2.getRotation().getZ());
    
    Eigen::Vector3d rpy1 = q1.toRotationMatrix().eulerAngles(0, 1, 2);
    Eigen::Vector3d rpy2 = q2.toRotationMatrix().eulerAngles(0, 1, 2);
    return (rpy1 - rpy2).norm();
  }

  /**
   * @brief Find the closest valid joint angles to the current joint states
   *        based on the inverse kinematics solution.
   * @param IKSolution The inverse kinematics solution. The size of the matrix
   *                    is `n x 6`, where `n` is the number of solutions.
   * @return The closest valid joint angles in type `StateVector` (8 x 1).
   */
  StateVector findClosestValidJointAngles(const Eigen::MatrixXd& IKSolution) const;

  inline bool isCloseToCurrent(const StateVector& desired_joint_states) const
  {
    for (uint8_t i = 0; i < 6; ++i) {
      if (std::abs(desired_joint_states(i) - joint_states_(i)) > 0.1) {
        return false;
      }
    }
    return true;
  }

  /**
   * @brief Callback function for receiving robot state messages.
   */
  void jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg);

  /**
   * @brief Callback function for receiving joy messages.
   */
  void leftJoyCallback(const sensor_msgs::Joy::ConstPtr& msg);

  /**
   * @brief Callback function for receiving joy pose messages.
   */
  void leftJoyPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
};

}  // namespace control

}  // namespace marslite

#endif  // MARSLITE_CONTROL_MARSLITE_CONTROL_H_