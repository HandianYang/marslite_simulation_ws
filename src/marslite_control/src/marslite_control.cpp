/**
 * @file marslite_control.cpp
 * @author Handian Yang
 * @copyright Released under the terms of the GPLv3.0 or later
 * @date 2024
 * 
 * @brief The executable file for the control scheme in Model Predictive Control (MPC) function for marslite robots.
 * @note `marslite_control.cpp` is part of `marslite_simulation_ws`.
 * 
 * `marslite_simulation_ws` is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published
 *      by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
 * 
 * `marslite_simulation_ws` is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of
 *      MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License along with `marslite_simulation_ws`. If not, see <https://www.gnu.org/licenses/>. 
*/

#include "marslite_control/marslite_control.h"

namespace marslite {

namespace control {

MarsliteControlScheme::MarsliteControlScheme(const ros::NodeHandle& nh)
    : nh_(nh), loopRate_(ros::Duration(25)), maxTimeout_(ros::Duration(30))
{
    try {
        // Initialize the MPC class pointer
        mpcClassPtr_ = std::make_shared<ModelPredictiveControl>();
    } catch (const ConstructorInitializationFailedException& ex) {
        throw ex;
    }

    try {
        // Subscribe the `/joint_states` topic
        this->subscribeRobotState();

        // Connect to the `FollowJointTrajectoryAction` action server
        this->connectJointTrajectoryActionServer();

    } catch (const TimeOutException& ex) {
        ROS_ERROR_STREAM(ex.what());
        throw ConstructorInitializationFailedException();
    }

    // Set the joint names for the trajectory
    trajectoryGoal_.trajectory.joint_names = {"tm_shoulder_1_joint", "tm_shoulder_2_joint", "tm_elbow_joint",
                                    "tm_wrist_1_joint", "tm_wrist_2_joint", "tm_wrist_3_joint"};
}

void MarsliteControlScheme::setInitialPose(const StateVector& initialPose)
{
    mpcClassPtr_->setInitialPose(initialPose);
}

void MarsliteControlScheme::setTargetPose(const StateVector& targetPose)
{
    mpcClassPtr_->setTargetPose(targetPose);
}

void MarsliteControlScheme::updateInitialStateFromRobotState()
{
    const ros::Duration timestep = ros::Duration(0.1);
    ros::Duration signalTimeoutTimer = ros::Duration(0);

    // Wait for the first valid message of the `/joint_states` topic
    while (ros::ok() && robotState_.position.empty() && signalTimeoutTimer < maxTimeout_) {
        // Check message validation every 0.1 seconds
        signalTimeoutTimer += timestep;
        timestep.sleep();
    }
    if (signalTimeoutTimer >= maxTimeout_) throw TimeOutException(maxTimeout_);

    std::unique_lock<std::mutex> lock(robotStateMutex_);
    {
        mpcClassPtr_->setInitialPose(
            (StateVector() <<   robotState_.position[4],    // tm_shoulder_1_joint
                                robotState_.position[5],    // tm_shoulder_2_joint
                                robotState_.position[3],    // tm_elbow_joint
                                robotState_.position[6],    // tm_wrist_1_joint
                                robotState_.position[7],    // tm_wrist_2_joint
                                robotState_.position[8],    // tm_wrist_3_joint
                                0,  // mobile base position
                                0   // mobile base orientation
            ).finished()
        );
    }
    lock.unlock();
}

bool MarsliteControlScheme::trajectoryPlanningQPSolver()
{  
    ROS_INFO_STREAM("MarsliteControlScheme::trajectoryPlanningQPSolver()");

    // Solve the trajectory planning problem
    if (!mpcClassPtr_->solveQP(trajectoryGoal_.trajectory.points)) return false;

    // Send the trajectory goal to the server, and wait for the result
    actionClient_->sendGoal(trajectoryGoal_);
    actionClient_->waitForResult(maxTimeout_);

    // Check if the action was successful
    ROS_INFO_STREAM("\tPlanning result: " << actionClient_->getState().toString());
    if (actionClient_->getState() != actionlib::SimpleClientGoalState::SUCCEEDED) {
        ROS_ERROR_STREAM("\tFailed to execute the trajectory.");
        return false;
    }

    // Clear the trajectory goal
    trajectoryGoal_.trajectory.points.clear();
    return true;
}

/* ********************************************** *
 *                 Initialization                 *
 * ********************************************** */

void MarsliteControlScheme::subscribeRobotState()
{
    const ros::Duration timestep = ros::Duration(0.1);
    ros::Duration signalTimeoutTimer = ros::Duration(0);

    // Subscribe the `/joint_states` topic
    robotStateSubscriber_ = nh_.subscribe("/joint_states", 1, &MarsliteControlScheme::robotStateCallback, this);

    // Wait for the publisher of the `/joint_states` topic
    ROS_INFO_STREAM("MarsliteControlScheme::subscribeRobotState()");
    ROS_INFO_STREAM("\tWaiting for subscribing the \"/joint_states\" topic...");
    while (ros::ok() && robotStateSubscriber_.getNumPublishers() == 0 && signalTimeoutTimer < maxTimeout_) {
        // Check subscription every 0.1 seconds
        signalTimeoutTimer += timestep;
        timestep.sleep();
    }
    if (signalTimeoutTimer >= maxTimeout_) throw TimeOutException(maxTimeout_);

    ROS_INFO_STREAM("\tSuccessfully subscribing the \"/joint_states\" topic!");
}

void MarsliteControlScheme::subscribeJoyPose()
{
    const ros::Duration timestep = ros::Duration(0.1);
    ros::Duration signalTimeoutTimer = ros::Duration(0);

    // Subscribe the `/joy_pose` topic
    joyPoseLeftSubscriber_ = nh_.subscribe("/unity/joy_pose/left", 1, &MarsliteControlScheme::joyPoseLeftCallback, this);

    // Wait for the publisher of the `/joy_pose` topic
    ROS_INFO_STREAM("MarsliteControlScheme::subscribeLeftJoyPose()");
    ROS_INFO_STREAM("\tWaiting for subscribing the \"/unity/joy_pose/left\" topic...");
    while (ros::ok() && joyPoseLeftSubscriber_.getNumPublishers() == 0 && signalTimeoutTimer < maxTimeout_) {
        // Check subscription every 0.1 seconds
        signalTimeoutTimer += timestep;
        timestep.sleep();
    }
    if (signalTimeoutTimer >= maxTimeout_) throw TimeOutException(maxTimeout_);

    ROS_INFO_STREAM("\tSuccessfully subscribing the \"/unity/joy_pose/left\" topic!");
}

void MarsliteControlScheme::connectJointTrajectoryActionServer()
{
    // Initialize the `FollowJointTrajectoryAction` action client
    actionClient_ = std::make_shared<JointTrajectoryAction>("/arm_controller/follow_joint_trajectory", true);

    // Wait for the connection to the server
    ROS_INFO_STREAM("MarsliteControlScheme::connectJointTrajectoryActionServer()");
    ROS_INFO_STREAM("\tWaiting for connecting to the FollowJointTrajectoryAction action server...");
    
    if (!actionClient_->waitForServer(maxTimeout_)) throw TimeOutException(maxTimeout_);

    ROS_INFO_STREAM("\tSuccessfully connecting to the FollowJointTrajectoryAction action server!");
}

/* ********************************************** *
 *                Callback functions              *
 * ********************************************** */

void MarsliteControlScheme::robotStateCallback(const sensor_msgs::JointStateConstPtr& msg)
{
    std::unique_lock<std::mutex> lock(robotStateMutex_);
    {
        robotState_ = *msg;
    }
}

void MarsliteControlScheme::joyPoseLeftCallback(const geometry_msgs::PoseStampedConstPtr& msg)
{
    std::unique_lock<std::mutex> lock(joyPoseLeftMutex_);
    {
        joyPoseLeft_ = *msg;
    }
}

}  // namespace control

}  // namespace marslite