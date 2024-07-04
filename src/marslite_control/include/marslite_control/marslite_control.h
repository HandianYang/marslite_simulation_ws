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

/**
 * @namespace marslite operation namespace
*/
namespace marslite {

/**
 * @namespace control namespace for marslite robots
*/
namespace control {

static const double TRIGGER_THRESHOLD = 0.95;

/**
 * @brief The class for marslite control
*/
class MarsliteControlScheme {
public:
    using ControlClassPtr = std::shared_ptr<MarsliteControlScheme>;
    
    using JointTrajectoryAction = actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>;
    using StateVector = Eigen::Matrix<double, MPC_STATE_SIZE, 1>;
    using InputVector = Eigen::Matrix<double, MPC_INPUT_SIZE, 1>;
    using TransformationMatrix = Eigen::Matrix<double, 4, 4>;

    explicit MarsliteControlScheme(const ros::NodeHandle& nh = ros::NodeHandle());

    /* ********************************************** *
     *                     Setters                    *
     * ********************************************** */

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
     * @brief Updates the initial state from the current robot state.
     * 
     * This function is responsible for updating the initial state of the robot based on the current robot state.
     * It performs the necessary calculations and updates the internal state variables accordingly.
     * 
     * @note This function should be called before solving the MPC problem.
     * @throw `marslite::exceptions::TimeOutException` if the robot state is not received within the timeout.
     */
    void updateInitialStateFromCurrent();

    /* ********************************************** *
     *                   Operations                   *
     * ********************************************** */

    /**
     * @brief The main function for joystick teleoperation.
     */
    void joystickTeleoperation();

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
    ros::Subscriber jointStateSubscriber_;
    ros::Subscriber leftJoySubscriber_;
    ros::Subscriber leftJoyPoseSubscriber_;

    ros::Rate loopRate_;
    ros::Duration timeout_;
    ros::Duration pollingSleepDuration_;

    /* ************************************ *
     *                Messages              *
     * ************************************ */
    sensor_msgs::JointState jointState_;        // JointState message of the robot
    sensor_msgs::Joy leftJoy_;                  // Joy message of the left joystick
    geometry_msgs::PoseStamped leftJoyPose_;    // PoseStamped message of the left joystick

    /* ************************************ *
     *         Trajectory planning          *
     * ************************************ */
    // `FollowJointTrajectoryAction` action client
    std::shared_ptr<JointTrajectoryAction> actionClient_;

    // the goal to be sent by action clients
    control_msgs::FollowJointTrajectoryGoal trajectoryGoal_;

    /* ************************************ *
     *                 Mutex                *
     * ************************************ */
    std::mutex jointStateMutex_;    // mutex for `jointState_`
    std::mutex leftJoyMutex_;       // mutex for `leftJoy_`
    std::mutex leftJoyPoseMutex_;   // mutex for `leftJoyPose_`
    

    /* ************************************ *
     *                 Flags                *
     * ************************************ */
    bool messageEnabled_;           // flag to enable/disable messages
    bool handTriggerIsPressed_;     // flag to indicate if the primary hand trigger is pressed
    bool indexTriggerIsPressed_;    // flag to indicate if the primary index trigger is pressed

private:
    /* ********************************************** *
     *                 Initialization                 *
     * ********************************************** */

    /**
     * @brief Connect to the `FollowJointTrajectoryAction` Server.
     * 
     * This function establishes a connection with the FollowJointTrajectoryAction Server.
     * 
     * @note Make sure to call this function before attempting to send trajectory goals.
     * @throw `marslite::exceptions::TimeOutException` if the connection is not successful within the timeout.
     */
    void connectJointTrajectoryActionServer();

    // TransformationMatrix poseToTransformationMatrix(const geometry_msgs::Pose& pose);

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
    void jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg);

    /**
     * @brief Callback function for receiving joy messages.
     * 
     * This function is called whenever a new joy message is received.
     * It takes a `sensor_msgs::Joy::ConstPtr` as input, which contains
     * the joystick information.
     * 
     * @param msg The received joy message.
     * @note This function is intended to be used as a callback for subscribing
     * to joy messages. Make sure to properly initialize the subscriber
     * and set the appropriate topic to receive the messages.
     */
    void leftJoyCallback(const sensor_msgs::Joy::ConstPtr& msg);

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
    void leftJoyPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
};

}  // namespace control

}  // namespace marslite

#endif  // MARSLITE_CONTROL_MARSLITE_CONTROL_H_