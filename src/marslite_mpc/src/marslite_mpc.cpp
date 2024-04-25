/**
 * @file marslite_mpc.cpp
 * @author Handian Yang
 * @copyright Released under the terms of the GPLv3.0 or later
 * @date 2024
 * 
 * @brief The source file for Model Predictive Control (MPC) function for marslite robots.
 * @note `marslite_mpc.cpp` is part of `marslite_simulation_ws`.
 * 
 * `marslite_simulation_ws` is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published
 *      by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
 * 
 * `marslite_simulation_ws` is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of
 *      MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License along with `marslite_simulation_ws`. If not, see <https://www.gnu.org/licenses/>. 
*/

#include "marslite_mpc/marslite_mpc.h"

namespace marslite {

namespace mpc {

ModelPredictiveControl::ModelPredictiveControl(const ros::NodeHandle& nh)
    : nh_(nh), loopRate_(MPC_SAMPLE_FREQ), maxTimeout_(60)
{
    // Set MPC characteristics
    this->setDynamicsMatrices();
    this->setInequalityConstraints();
    this->setWeightMatrices();
    // x0_ = marslite::poses::MARSLITE_POSE_INITIAL;
    // xRef_ = marslite::poses::MARSLITE_POSE_DEFAULT1;

    // Cast the MPC problem as QP problem
    this->castMPCToQPHessian();
    this->castMPCToQPGradient();
    this->castMPCToQPConstraintMatrix();
    this->castMPCToQPConstraintVectors();

    // Initialize the QP solver
    if (!this->initializeQPSolver()) {
        ROS_ERROR_STREAM("[MPC::MPC()] Error initializing QP solver.");
        throw ConstructorInitializationFailedException();
    }

    try {
        // Subscribe the `/joint_states` topic
        this->subscribeRobotState();

        // Update the initial state from the `/joint_states` topic
        this->updateInitialStateFromRobotState();

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

void ModelPredictiveControl::setTargetPose(const StateVector& targetPose)
{
    xRef_ = targetPose;
}

bool ModelPredictiveControl::initializeQPSolver()
{
    // Set the warm start
    solver_.settings()->setWarmStart(true);
    solver_.settings()->setVerbosity(false);

    // Set the initial data of the QP solver
    solver_.data()->setNumberOfVariables(QP_STATE_SIZE);
    solver_.data()->setNumberOfConstraints(QP_BOUND_SIZE);
    if(!solver_.data()->setHessianMatrix(hessianMatrix_))   return false;
    if(!solver_.data()->setGradient(gradient_))             return false;
    if(!solver_.data()->setLinearConstraintsMatrix(constraintMatrix_))  return false;
    if(!solver_.data()->setLowerBound(lowerBound_))   return false;
    if(!solver_.data()->setUpperBound(upperBound_))   return false;

    // Instantiate the solver
    if(!solver_.initSolver()) return false;

    return true;
}

bool ModelPredictiveControl::trajectoryPlanningQPSolver()
{
    ROS_INFO_STREAM("ModelPredictiveControl::trajectoryPlanningQPSolver()");

    // Update the initial state from the robot state
    try {
        this->updateInitialStateFromRobotState();
    } catch (const TimeOutException& ex) {
        ROS_ERROR_STREAM(ex.what());
        return false;
    }

    // Update the gradient and the constraint vectors
    if (!this->updateGradient(xRef_)) {
        ROS_ERROR_STREAM("Error updating the gradient.");
        return false;
    }
    if (!this->updateConstraintVectors(x0_)) {
        ROS_ERROR_STREAM("Error updating the constraint vectors.");
        return false;
    }

    // [Special case] Check if the robot is already at the target pose
    if ((x0_ - xRef_).norm() < MPC_ZERO) {
        ROS_INFO_STREAM("\tThe robot is already at the target pose. Skip the planning process...");
        return true;
    }

    Eigen::VectorXd QPSolution; // QPSolution vector
    InputVector u;              // current input
    StateVector x = x0_;        // current state
    // timespec startTime, finishTime;                     // time record
    size_t counter = 0;         // counter for the trajectory waypoints


    // clock_gettime(CLOCK_REALTIME, &startTime);
    while ((x - xRef_).norm() >= MPC_ZERO) {
        // Solve the QP problem
        if (solver_.solveProblem() != OsqpEigen::ErrorExitFlag::NoError) {
            ROS_ERROR_STREAM("Unable to solve the problem.");
            return false;
        }

        // Check if the solution is unfeasible
        if (solver_.getStatus() != OsqpEigen::Status::Solved) {
            ROS_ERROR_STREAM("The solution is unfeasible.");
            return false;
        }

        // Obtain the solution, and append the new waypoint to the trajectory  (`trajectoryGoal_`)
        QPSolution = solver_.getSolution();
        u = QPSolution.block(QP_DYNAMIC_SIZE, 0, MPC_INPUT_SIZE, 1);

        trajectory_msgs::JointTrajectoryPoint trajectoryWaypoint;
        trajectoryWaypoint.positions  = {x[0], x[1], x[2], x[3], x[4], x[5]};
        trajectoryWaypoint.velocities = {u[0], u[1], u[2], u[3], u[4], u[5]};
        trajectoryWaypoint.time_from_start = ros::Duration((counter++) * MPC_SAMPLE_TIME);
        trajectoryGoal_.trajectory.points.push_back(trajectoryWaypoint);
        
        // Propagate the model
        x = A_ * x + B_ * u;

        // Update the constraint vectors
        if (!this->updateConstraintVectors(x)) {
            ROS_ERROR_STREAM("Error updating the constraint vectors.");
            return false;
        }
    }
    // clock_gettime(CLOCK_REALTIME, &finishTime);
    // ROS_INFO_STREAM("[MPC::trajectoryPlanningQPSolver()] Time taken: " << marslite::time::getOperationTime(startTime, finishTime));

    // Send the trajectory goal to the server, and wait for the result
    actionClient_->sendGoal(trajectoryGoal_);
    actionClient_->waitForResult();

    // Check if the action was successful
    ROS_INFO_STREAM("\tPlanning result: " << actionClient_->getState().toString());
    if (actionClient_->getState() != actionlib::SimpleClientGoalState::SUCCEEDED) {
        ROS_ERROR_STREAM("[MPC::trajectoryPlanningQPSolver()] Failed to execute the trajectory.");
        return false;
    }

    // Clear the trajectory goal
    trajectoryGoal_.trajectory.points.clear();

    return true;
}

/* ********************************************** *
 *                 Initialization                 *
 * ********************************************** */

void ModelPredictiveControl::subscribeRobotState()
{
    const ros::Duration timestep = ros::Duration(0.1);
    ros::Duration signalTimeoutTimer = ros::Duration(0);

    // Subscribe the `/joint_states` topic
    robotStateSubscriber_ = nh_.subscribe("/joint_states", 1, &ModelPredictiveControl::robotStateCallback, this);

    // Wait for the publisher of the `/joint_states` topic
    ROS_INFO_STREAM("ModelPredictiveControl::subscribeRobotState()");
    ROS_INFO_STREAM("\tWaiting for subscribing the \"/joint_states\" topic...");
    while (ros::ok() && robotStateSubscriber_.getNumPublishers() == 0 && signalTimeoutTimer < maxTimeout_) {
        // Check subscription every 0.1 seconds
        signalTimeoutTimer += timestep;
        timestep.sleep();
    }
    if (signalTimeoutTimer >= maxTimeout_) throw TimeOutException(maxTimeout_);

    ROS_INFO_STREAM("\tSuccessfully subscribing the \"/joint_states\" topic!");
}

void ModelPredictiveControl::connectJointTrajectoryActionServer()
{
    // Initialize the `FollowJointTrajectoryAction` action client
    actionClient_ = std::make_shared<JointTrajectoryAction>("/arm_controller/follow_joint_trajectory", true);

    // Wait for the connection to the server
    ROS_INFO_STREAM("ModelPredictiveControl::connectJointTrajectoryActionServer()");
    ROS_INFO_STREAM("\tWaiting for connecting to the FollowJointTrajectoryAction action server...");
    
    if (!actionClient_->waitForServer(maxTimeout_)) throw TimeOutException(maxTimeout_);

    ROS_INFO_STREAM("\tSuccessfully connecting to the FollowJointTrajectoryAction action server!");
}

/* ********************************************** *
 *                MPC/QP functions                *
 * ********************************************** */

void ModelPredictiveControl::setDynamicsMatrices()
{
    A_ << 1., 0., 0., 0., 0., 0., 0., 0.,
          0., 1., 0., 0., 0., 0., 0., 0.,
          0., 0., 1., 0., 0., 0., 0., 0.,
          0., 0., 0., 1., 0., 0., 0., 0.,
          0., 0., 0., 0., 1., 0., 0., 0.,
          0., 0., 0., 0., 0., 1., 0., 0.,
          0., 0., 0., 0., 0., 0., 1., 0.,
          0., 0., 0., 0., 0., 0., 0., 1.;

    B_ << MPC_SAMPLE_TIME, 0., 0., 0., 0., 0., 0., 0.,
          0., MPC_SAMPLE_TIME, 0., 0., 0., 0., 0., 0.,
          0., 0., MPC_SAMPLE_TIME, 0., 0., 0., 0., 0.,
          0., 0., 0., MPC_SAMPLE_TIME, 0., 0., 0., 0.,
          0., 0., 0., 0., MPC_SAMPLE_TIME, 0., 0., 0.,
          0., 0., 0., 0., 0., MPC_SAMPLE_TIME, 0., 0.,
          0., 0., 0., 0., 0., 0., MPC_SAMPLE_TIME, 0.,
          0., 0., 0., 0., 0., 0., 0., MPC_SAMPLE_TIME;
}

void ModelPredictiveControl::setInequalityConstraints()
{
    xMax_ = marslite::constraints::MARSLITE_POSITION_LIMIT_MAX;
    xMin_ = marslite::constraints::MARSLITE_POSITION_LIMIT_MIN;
    uMax_ = marslite::constraints::MARSLITE_VELOCITY_LIMIT_MAX;
    uMin_ = marslite::constraints::MARSLITE_VELOCITY_LIMIT_MIN;
    aMax_ = marslite::constraints::MARSLITE_ACCELERATION_LIMIT_MAX;
    aMin_ = marslite::constraints::MARSLITE_ACCELERATION_LIMIT_MIN;
}

void ModelPredictiveControl::setWeightMatrices()
{
    Q_.diagonal() << 1., 1., 1., 1., 1., 1., 1., 1.; 
    R_.diagonal() << 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01;
}

void ModelPredictiveControl::castMPCToQPHessian()
{
    hessianMatrix_.resize(QP_STATE_SIZE, QP_STATE_SIZE);

    double value;
    for (int i = 0; i < QP_DYNAMIC_SIZE; ++i) {
        value = Q_.diagonal()[i % MPC_STATE_SIZE];
        if (value != 0)
            hessianMatrix_.insert(i,i) = value;
    }

    for (int i = QP_DYNAMIC_SIZE; i < QP_STATE_SIZE; ++i) {
        value = R_.diagonal()[i % MPC_INPUT_SIZE];
        if (value != 0)
            hessianMatrix_.insert(i,i) = value;
    }
}

void ModelPredictiveControl::castMPCToQPGradient()
{
    const StateVector Qx_ref = Q_ * (-xRef_);

    gradient_ = Eigen::VectorXd::Zero(QP_STATE_SIZE, 1);
    for (int i = 0; i < QP_DYNAMIC_SIZE; ++i){
        gradient_(i,0) = Qx_ref(i % MPC_STATE_SIZE, 0);
    }
}

void ModelPredictiveControl::castMPCToQPConstraintMatrix()
{
    constraintMatrix_.resize(QP_BOUND_SIZE, QP_STATE_SIZE);

    for (int i = 0; i < QP_DYNAMIC_SIZE; ++i){
        constraintMatrix_.insert(i,i) = -1;
    }

    double value;
    for (int i = 0; i < MPC_WINDOW_SIZE; ++i) {
        for (int j = 0; j < MPC_STATE_SIZE; ++j) {
            for (int k = 0; k < MPC_STATE_SIZE; ++k) {
                value = A_(j,k);
                if (value != 0)
                    constraintMatrix_.insert(MPC_STATE_SIZE * (i+1) + j, MPC_STATE_SIZE * i + k) = value;
            }
        }
    }
    
    for (int i = 0; i < MPC_WINDOW_SIZE; ++i) {
        for (int j = 0; j < MPC_STATE_SIZE; ++j) {
            for (int k = 0; k < MPC_INPUT_SIZE; ++k) {
                value = B_(j,k);
                if (value != 0)
                    constraintMatrix_.insert(MPC_STATE_SIZE * (i+1) + j, MPC_INPUT_SIZE * i + k + QP_DYNAMIC_SIZE) = value;
            }
        }
    }

    for (int i = 0; i < QP_STATE_SIZE; ++i) {
        constraintMatrix_.insert(i + QP_DYNAMIC_SIZE, i) = 1;
    }

    for (int i = 0; i < MPC_INPUT_SIZE * MPC_WINDOW_SIZE; ++i) {
        constraintMatrix_.insert(i + 2 * QP_DYNAMIC_SIZE + QP_CONTROL_SIZE, i + QP_DYNAMIC_SIZE) = MPC_SAMPLE_FREQ;
    }

    for (int i = 0; i < MPC_INPUT_SIZE * (MPC_WINDOW_SIZE-1); ++i) {
        constraintMatrix_.insert(i + 2 * QP_DYNAMIC_SIZE + QP_CONTROL_SIZE + MPC_INPUT_SIZE, i + QP_DYNAMIC_SIZE) = -MPC_SAMPLE_FREQ;
    }
}

void ModelPredictiveControl::castMPCToQPConstraintVectors()
{
    // Evaluate the lower and the upper inequality vectors
    Eigen::VectorXd lowerInequality = Eigen::MatrixXd::Zero(QP_INEQUALITY_SIZE, 1); // lower inequality vector (in size `SS*(W+1) + 2*IS*W` by `1`)
    Eigen::VectorXd upperInequality = Eigen::MatrixXd::Zero(QP_INEQUALITY_SIZE, 1); // upper inequality vector (in size `SS*(W+1) + 2*IS*W` by `1`)
    for (int i = 0; i < MPC_WINDOW_SIZE+1; ++i) {
        lowerInequality.block(MPC_STATE_SIZE * i, 0, MPC_STATE_SIZE, 1) = xMin_;
        upperInequality.block(MPC_STATE_SIZE * i, 0, MPC_STATE_SIZE, 1) = xMax_;
    }
    for (int i = 0; i < MPC_WINDOW_SIZE; ++i) {
        lowerInequality.block(MPC_INPUT_SIZE * i + QP_DYNAMIC_SIZE, 0, MPC_INPUT_SIZE, 1) = uMin_;
        upperInequality.block(MPC_INPUT_SIZE * i + QP_DYNAMIC_SIZE, 0, MPC_INPUT_SIZE, 1) = uMax_;
    }
    for (int i = 0; i < MPC_WINDOW_SIZE; ++i) {
        lowerInequality.block(MPC_INPUT_SIZE * i + QP_STATE_SIZE, 0, MPC_INPUT_SIZE, 1) = aMin_;
        upperInequality.block(MPC_INPUT_SIZE * i + QP_STATE_SIZE, 0, MPC_INPUT_SIZE, 1) = aMax_;
    }

    // Evaluate the lower and the upper equality vectors
    Eigen::VectorXd lowerEquality = Eigen::MatrixXd::Zero(QP_EQUALITY_SIZE, 1); // lower equality vector (in size `SS*(W+1)` by `1`)
    Eigen::VectorXd upperEquality = Eigen::MatrixXd::Zero(QP_EQUALITY_SIZE, 1); // upper equality vector (in size `SS*(W+1)` by `1`)
    lowerEquality.block(0, 0, MPC_STATE_SIZE, 1) = -x0_;
    upperEquality = lowerEquality;
    
    // Merge inequality and equality vectors into constraint vectors
    lowerBound_ = Eigen::MatrixXd::Zero(QP_BOUND_SIZE, 1);
    lowerBound_ << lowerEquality, lowerInequality;

    upperBound_ = Eigen::MatrixXd::Zero(QP_BOUND_SIZE, 1);
    upperBound_ << upperEquality, upperInequality;
}

void ModelPredictiveControl::updateInitialStateFromRobotState()
{
    ros::Duration signalTimeoutTimer = ros::Duration(0);
    const ros::Duration timestep = ros::Duration(0.1);

    // Wait for the first valid message of the `/joint_states` topic
    while (ros::ok() && robotState_.position.empty() && signalTimeoutTimer < maxTimeout_) {
        // Check message validation every 0.1 seconds
        signalTimeoutTimer += timestep;
        timestep.sleep();
    }
    if (signalTimeoutTimer >= maxTimeout_) throw TimeOutException(maxTimeout_);

    std::unique_lock<std::mutex> lock(robotStateMutex_);
    {
        x0_ = (
            StateVector() << robotState_.position[4],    // tm_shoulder_1_joint
                            robotState_.position[5],    // tm_shoulder_2_joint
                            robotState_.position[3],    // tm_elbow_joint
                            robotState_.position[6],    // tm_wrist_1_joint
                            robotState_.position[7],    // tm_wrist_2_joint
                            robotState_.position[8],    // tm_wrist_3_joint
                            0,  // mobile base position
                            0   // mobile base orientation
        ).finished();
    }
    lock.unlock();
}

bool ModelPredictiveControl::updateGradient(const StateVector& xRef)
{
    const StateVector Qx_ref = Q_ * (-xRef);
    for (int i = 0; i < QP_DYNAMIC_SIZE; ++i)
        gradient_(i, 0) = Qx_ref(i % MPC_STATE_SIZE, 0);

    return solver_.updateGradient(gradient_);
}

bool ModelPredictiveControl::updateConstraintVectors(const StateVector& x)
{
    lowerBound_.block(0, 0, MPC_STATE_SIZE, 1) = -x;
    upperBound_.block(0, 0, MPC_STATE_SIZE, 1) = -x;

    return solver_.updateBounds(lowerBound_, upperBound_);
}

/* ********************************************** *
 *                Callback functions              *
 * ********************************************** */

void ModelPredictiveControl::robotStateCallback(const sensor_msgs::JointStateConstPtr& msg)
{
    std::unique_lock<std::mutex> lock(robotStateMutex_);
    {
        robotState_ = *msg;
    }
}

/* ********************************************** *
 *                 Debug functions                *
 * ********************************************** */

void ModelPredictiveControl::printStateVector(const StateVector& stateVector, const std::string& name)
{
    std::stringstream ss;
    ss << std::fixed << std::setprecision(2);
    ss << "\t" << name << ": ";
    for (int i = 0; i < stateVector.size(); ++i) {
        ss << std::round(stateVector[i] * 100) / 100;
        if (i != stateVector.size() - 1) {
            ss << ", ";
        }
    }
    ROS_INFO_STREAM(ss.str());
}

} // namespace mpc

} // namespace marslite