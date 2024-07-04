/**
 * @file marslite_control.cpp
 * @author Handian Yang
 * @copyright Released under the terms of the GPLv3.0 or later
 * @date 2024
 * 
 * @brief The source file for the control scheme in
 *        Model Predictive Control (MPC) function for marslite robots.
 * @note `marslite_control.cpp` is part of `marslite_simulation_ws`.
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

#include "marslite_control/marslite_control.h"

/**
 * @namespace marslite operation namespace
*/
namespace marslite {

/**
 * @namespace control namespace for marslite robots
*/
namespace control {

MarsliteControlScheme::MarsliteControlScheme(const ros::NodeHandle& nh)
        : nh_(nh), loopRate_(ros::Duration(25)), messageEnabled_(true),
        timeout_(ros::Duration(30)), pollingSleepDuration_(ros::Duration(0.1)),
        handTriggerIsPressed_(false), indexTriggerIsPressed_(false)
{
    try {
        // Initialize the MPC class pointer
        mpcClassPtr_ = std::make_shared<ModelPredictiveControl>();
    } catch (const ConstructorInitializationFailedException& ex) {
        throw ex;
    }

    try {
        // Subscribe the `/joint_states` topic
        subscribeTopicWithTimeout<MarsliteControlScheme, sensor_msgs::JointState>(
            this, nh_, jointStateSubscriber_, "/joint_states",
            1, &MarsliteControlScheme::jointStateCallback,
            timeout_, pollingSleepDuration_
        );
        
        // Subscribe the `/unity/joy_pose/left` topic
        subscribeTopicWithTimeout<MarsliteControlScheme, geometry_msgs::PoseStamped>(
            this, nh_, leftJoyPoseSubscriber_, "/unity/joy_pose/left",
            1, &MarsliteControlScheme::leftJoyPoseCallback,
            timeout_, pollingSleepDuration_
        );

        // Subscribe the `/unity/joy/left` topic
        subscribeTopicWithTimeout<MarsliteControlScheme, sensor_msgs::Joy>(
            this, nh_, leftJoySubscriber_, "/unity/joy/left",
            1, &MarsliteControlScheme::leftJoyCallback,
            timeout_, pollingSleepDuration_
        );

        // Connect to the `FollowJointTrajectoryAction` action server
        this->connectJointTrajectoryActionServer();

    } catch (const TimeOutException& ex) {
        ROS_ERROR("%s", ex.what());
        throw ConstructorInitializationFailedException();
    }

    // Set the joint names for the trajectory
    trajectoryGoal_.trajectory.joint_names = {
        "tm_shoulder_1_joint",
        "tm_shoulder_2_joint",
        "tm_elbow_joint",
        "tm_wrist_1_joint",
        "tm_wrist_2_joint",
        "tm_wrist_3_joint"
    };
}

void MarsliteControlScheme::setInitialPose(const StateVector& initialPose)
{
    mpcClassPtr_->setInitialPose(initialPose);
}

void MarsliteControlScheme::setTargetPose(const StateVector& targetPose)
{
    mpcClassPtr_->setTargetPose(targetPose);
}

void MarsliteControlScheme::updateInitialStateFromCurrent()
{
    const ros::Duration timestep = ros::Duration(0.1);
    ros::Duration signalTimeoutTimer = ros::Duration(0);

    // Wait for the first valid message of the `/joint_states` topic
    while (ros::ok() && jointState_.position.empty()) {
        // Check message validation every 0.1 seconds
        signalTimeoutTimer += timestep;
        if (signalTimeoutTimer >= timeout_) throw TimeOutException(timeout_);

        timestep.sleep();
    }

    std::unique_lock<std::mutex> lock(jointStateMutex_);
    {
        mpcClassPtr_->setInitialPose(
            (StateVector() <<   jointState_.position[4],    // tm_shoulder_1_joint
                                jointState_.position[5],    // tm_shoulder_2_joint
                                jointState_.position[3],    // tm_elbow_joint
                                jointState_.position[6],    // tm_wrist_1_joint
                                jointState_.position[7],    // tm_wrist_2_joint
                                jointState_.position[8],    // tm_wrist_3_joint
                                0,  // mobile base position
                                0   // mobile base orientation
            ).finished()
        );
    }
    lock.unlock();
}

/* ********************************************** *
 *                   Operations                   *
 * ********************************************** */

void MarsliteControlScheme::joystickTeleoperation()
{
    this->updateInitialStateFromCurrent();

    tf::TransformListener listener;
    tf::StampedTransform initialRoboticGripPoseStampedTF;
    tf::StampedTransform desiredRoboticGripPoseStampedTF;
    tf::Transform initialRoboticGripPoseTF;
    tf::Transform desiredRoboticGripPoseTF;

    geometry_msgs::PoseStamped joyPoseLeftInitial;
    tf::Transform initialLeftJoyPoseTF;     // initial pose of the left joystick
    tf::Transform currentLeftJoyPoseTF;     // current pose of the left joystick
    tf::Transform relativeLeftJoyPoseTF;    // relative pose of the left joystick
    
    bool startTeleoperation = false;

    while (ros::ok()) {
        if (handTriggerIsPressed_ && indexTriggerIsPressed_) {
            // The safety buttons are pressed. Begin teleoperating the robot.
            if (!startTeleoperation) {
                // Obtain the transform from /odom to /robotiq_85_base_link
                try {
                    ROS_INFO_STREAM_COND(messageEnabled_, "  Waiting for the transform from /odom to /robotiq_85_base_link...");
                    
                    listener.waitForTransform("/odom", "/robotiq_85_base_link", ros::Time(0), timeout_, pollingSleepDuration_);
                    listener.lookupTransform("/odom", "/robotiq_85_base_link", ros::Time(0), initialRoboticGripPoseStampedTF);
                    initialRoboticGripPoseTF.setOrigin(initialRoboticGripPoseStampedTF.getOrigin());
                    initialRoboticGripPoseTF.setRotation(initialRoboticGripPoseStampedTF.getRotation());

                    ROS_INFO_STREAM_COND(messageEnabled_, "  Successfully found the transform from /odom to /robotiq_85_base_link!");

                } catch (tf::TransformException &ex) {
                    ROS_ERROR("%s", ex.what());
                    throw TransformNotFoundException("/odom", "/robotiq_85_base_link");
                }

                tf::poseMsgToTF(leftJoyPose_.pose, initialLeftJoyPoseTF);

                startTeleoperation = true;
            } else {
                // Calculate the desired pose of the robot regarding the joystick pose
                tf::poseMsgToTF(leftJoyPose_.pose, currentLeftJoyPoseTF);
                relativeLeftJoyPoseTF = initialLeftJoyPoseTF.inverseTimes(currentLeftJoyPoseTF);
                desiredRoboticGripPoseTF = initialRoboticGripPoseTF * relativeLeftJoyPoseTF;
            }
            
        } else {
            startTeleoperation = false;
        }

        loopRate_.sleep();
        break;
    }
}

bool MarsliteControlScheme::trajectoryPlanningQPSolver()
{  
    ROS_INFO_STREAM_COND(messageEnabled_, "MarsliteControlScheme::trajectoryPlanningQPSolver()");

    // Solve the trajectory planning problem
    if (!mpcClassPtr_->solveQP(trajectoryGoal_.trajectory.points)) return false;

    // Send the trajectory goal to the server, and wait for the result
    actionClient_->sendGoal(trajectoryGoal_);
    actionClient_->waitForResult(timeout_);

    // Check if the action was successful
    ROS_INFO_STREAM_COND(messageEnabled_, "  Planning result: " << actionClient_->getState().toString());
    if (actionClient_->getState() != actionlib::SimpleClientGoalState::SUCCEEDED) {
        ROS_ERROR_STREAM_COND(messageEnabled_, "  Failed to execute the trajectory.");
        return false;
    }

    // Clear the trajectory goal
    trajectoryGoal_.trajectory.points.clear();
    return true;
}

/* ********************************************** *
 *                 Initialization                 *
 * ********************************************** */

void MarsliteControlScheme::connectJointTrajectoryActionServer()
{
    // Initialize the `FollowJointTrajectoryAction` action client
    actionClient_ = std::make_shared<JointTrajectoryAction>("/arm_controller/follow_joint_trajectory", true);

    // Wait for the connection to the server
    ROS_INFO_STREAM("MarsliteControlScheme::connectJointTrajectoryActionServer()");
    ROS_INFO_STREAM("\tWaiting for connecting to the FollowJointTrajectoryAction action server...");
    
    if (!actionClient_->waitForServer(timeout_)) throw TimeOutException(timeout_);

    ROS_INFO_STREAM("\tSuccessfully connecting to the FollowJointTrajectoryAction action server!");
}

/* ********************************************** *
 *                Callback functions              *
 * ********************************************** */

void MarsliteControlScheme::jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg)
{
    std::unique_lock<std::mutex> lock(jointStateMutex_);
    {
        jointState_ = *msg;
    }
}

void MarsliteControlScheme::leftJoyCallback(const sensor_msgs::Joy::ConstPtr& msg)
{
    std::unique_lock<std::mutex> lock(leftJoyMutex_);
    {
        leftJoy_ = *msg;

        switch (leftJoy_.axes.size()) {
        case 4:
            // [3] primary hand trigger
            handTriggerIsPressed_ = (leftJoy_.axes[3] > TRIGGER_THRESHOLD);
        case 3:
            // [2] primary index trigger
            indexTriggerIsPressed_ = (leftJoy_.axes[2] > TRIGGER_THRESHOLD);
            break;
        default:
            ROS_WARN_ONCE("Failed to receive the joy topic");
            ROS_WARN_ONCE(" Please check your joystick(s) setup or rosbridge connection.");
            break;
        }
    }
}

void MarsliteControlScheme::leftJoyPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    std::unique_lock<std::mutex> lock(leftJoyPoseMutex_);
    {
        leftJoyPose_ = *msg;
    }
}

}  // namespace control

}  // namespace marslite