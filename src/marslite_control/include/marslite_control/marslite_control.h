/**
 * @file marslite_control.h
 * @author Handian Yang
 * @copyright Released under the terms of the GPLv3.0 or later
 * @date 2024
 * 
 * @brief The header file for the control scheme in
 *        Model Predictive Control (MPC) function for marslite robots.
 * 
 * @note `marslite_control.h` is part of `marslite_simulation_ws`.
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

#ifndef MARSLITE_CONTROL_MARSLITE_CONTROL_H_
#define MARSLITE_CONTROL_MARSLITE_CONTROL_H_

#include <ros/ros.h>

#include <Eigen/Dense>              // eigen
#include "OsqpEigen/OsqpEigen.h"    // osqp-eigen

#include <sensor_msgs/JointState.h>
#include <geometry_msgs/PoseStamped.h>

#include <actionlib/client/simple_action_client.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <trajectory_msgs/JointTrajectory.h>

#include "model_predictive_control/model_predictive_control.h"
using marslite::control::ModelPredictiveControl;

#include "marslite_properties/Exception.h"
using marslite::exception::TimeOutException;
using marslite::exception::ConstructorInitializationFailedException;

/**
 * @namespace marslite operation namespace
*/
namespace marslite {

/**
 * @namespace control namespace for marslite robots
*/
namespace control {

/**
 * @brief The class for marslite control
*/
class MarsliteControlScheme {
public:
    using ControlClassPtr = std::shared_ptr<MarsliteControlScheme>;
    
    using JointTrajectoryAction = actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>;
    using StateVector = Eigen::Matrix<double, MPC_STATE_SIZE, 1>;
    using InputVector = Eigen::Matrix<double, MPC_INPUT_SIZE, 1>;

    explicit MarsliteControlScheme(const ros::NodeHandle& nh = ros::NodeHandle());

    /**
     * @brief Sets the initial pose for the Marslite MPC.
     *
     * This function sets the initial pose for the Marslite Model Predictive Control (MPC) algorithm.
     * The initial pose is used as the starting point for the MPC optimization process.
     *
     * @param initialPose The initial pose to set. Defaults to `marslite::pose::INITIAL`.
     */
    void setInitialPose(const StateVector& initialPose = marslite::pose::INITIAL);

    /**
     * @brief Sets the target pose for the MPC controller.
     *
     * This function sets the target pose for the MPC controller. The target pose is represented
     * by a matrix of size MPC_STATE_SIZE x 1, where MPC_STATE_SIZE is the size of the state vector.
     *
     * @param targetPose The target pose vector. Defaults to `marslite::pose::INITIAL`.
     */
    void setTargetPose(const StateVector& targetPose = marslite::pose::INITIAL);

    /**
     * @brief Updates the initial state from the robot state.
     * 
     * This function is responsible for updating the initial state of the robot based on the current robot state.
     * It performs the necessary calculations and updates the internal state variables accordingly.
     * 
     * @note This function should be called before solving the MPC problem.
     * @throw `marslite::exceptions::TimeOutException` if the robot state is not received within the timeout.
     */
    void updateInitialStateFromRobotState();

    /**
     * @brief Plan the trajectory of the robot using QP solver.
     * @return True if the trajectory can be executed, and it is executed successfully.
     * @note ALWAYS remember to initialize the QP solver by invoking `QPSolverInitialization()`
     *  function before solving the problem.
    */
    bool trajectoryPlanningQPSolver();

private:
    ModelPredictiveControl::MPCClassPtr mpcClassPtr_;

    /* ************************************ *
     *              ROS-related             *
     * ************************************ */
    ros::NodeHandle nh_;
    ros::Subscriber robotStateSubscriber_;
    ros::Subscriber joyPoseLeftSubscriber_;

    ros::Rate loopRate_;
    ros::Duration maxTimeout_;

    /* ************************************ *
     *                Messages              *
     * ************************************ */
    sensor_msgs::JointState robotState_;
    geometry_msgs::PoseStamped joyPoseLeft_;

    /* ************************************ *
     *         Trajectory planning          *
     * ************************************ */
    std::shared_ptr<JointTrajectoryAction> actionClient_;   // `FollowJointTrajectoryAction` action client
    control_msgs::FollowJointTrajectoryGoal trajectoryGoal_;          // the goal to be sent by action clients

    /* ************************************ *
     *                 Mutex                *
     * ************************************ */
    std::mutex robotStateMutex_;
    std::mutex joyPoseLeftMutex_;

private:
    /* ********************************************** *
     *                 Initialization                 *
     * ********************************************** */

    /**
     * @brief Subscribes to the robot state topic.
     * 
     * This function sets up a subscription to the robot state topic, allowing the
     * program to receive updates on the current state of the robot.
     * 
     * @note Make sure to call this function before attempting to use the robot state.
     * @throw `marslite::exceptions::TimeOutException` if the subscription is not successful within the timeout.
     */
    void subscribeRobotState();

    /**
     * @brief Subscribes to the pose of the joysticks.
     * 
     * This function sets up a subscription to receive joy pose messages.
     * It allows the program to receive and process joy pose data.
     */
    void subscribeJoyPose();

    /**
     * @brief Connect to the `FollowJointTrajectoryAction` Server.
     * 
     * This function establishes a connection with the FollowJointTrajectoryAction Server.
     * 
     * @note Make sure to call this function before attempting to send trajectory goals.
     * @throw `marslite::exceptions::TimeOutException` if the connection is not successful within the timeout.
     */
    void connectJointTrajectoryActionServer();

    /* ********************************************** *
     *                Callback functions              *
     * ********************************************** */

    /**
     * @brief Callback function for receiving robot state messages.
     * 
     * This function is called whenever a new robot state message is received.
     * It takes a `sensor_msgs::JointState::ConstPtr` as input, which contains
     * the joint state information of the robot.
     * 
     * @param msg The received robot state message.
     * 
     * @note This function is intended to be used as a callback for subscribing
     * to robot state messages. Make sure to properly initialize the subscriber
     * and set the appropriate topic to receive the messages.
     */
    void robotStateCallback(const sensor_msgs::JointStateConstPtr& msg);

    /**
     * @brief Callback function for receiving joy pose messages.
     * 
     * This function is called whenever a new joy pose message is received.
     * It takes a `geometry_msgs::PoseStamped::ConstPtr` as input, which contains
     * the pose information of the joysticks.
     * 
     * @param msg The received joy pose message.
     * 
     * @note This function is intended to be used as a callback for subscribing
     * to joy pose messages. Make sure to properly initialize the subscriber
     * and set the appropriate topic to receive the messages.
     */
    void joyPoseLeftCallback(const geometry_msgs::PoseStampedConstPtr& msg);
};

}  // namespace control

}  // namespace marslite

#endif  // MARSLITE_CONTROL_MARSLITE_CONTROL_H_