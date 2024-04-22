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
    // Subscribe the `/joint_states` topic
    if (!this->subscribeRobotState()) {
        ROS_ERROR_STREAM("[MPC::MPC()] Error subscribing the `/joint_states` topic.");
        throw marslite::ConstructorInitializationFailedException();
    }

    // Connect to the `FollowJointTrajectoryAction` action server
    if (!this->connectJointTrajectoryActionServer()) {
        ROS_ERROR_STREAM("[MPC::MPC()] Error connecting to the FollowJointTrajectoryAction action server.");
        throw marslite::ConstructorInitializationFailedException();
    }
    trajectoryGoal_.trajectory.joint_names = {"tm_shoulder_1_joint", "tm_shoulder_2_joint", "tm_elbow_joint",
                                    "tm_wrist_1_joint", "tm_wrist_2_joint", "tm_wrist_3_joint"};

    // Set MPC characteristics
    this->setDynamicsMatrices();
    this->setInequalityConstraints();
    this->setWeightMatrices();
    this->setInitialStateSpace(MARSLITE_POSE_INITIAL);
    this->setReferenceStateSpace(MARSLITE_POSE_DEFAULT1);

    // Cast the MPC problem as QP problem
    this->castMPCToQPHessian();
    this->castMPCToQPGradient();
    this->castMPCToQPConstraintMatrix();
    this->castMPCToQPConstraintVectors();

    // Initialize the QP solver
    if (!this->initializeQPSolver()) {
        ROS_ERROR_STREAM("[MPC::MPC()] Error initializing QP solver.");
        throw marslite::ConstructorInitializationFailedException();
    }
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

bool ModelPredictiveControl::solveQP()
{
    Eigen::VectorXd QPSolution;     // QPSolution vector
    Eigen::VectorXd controlInput;   // control input vector
    timespec startTime, finishTime; // time record
    size_t counter = 0;

    clock_gettime(CLOCK_REALTIME, &startTime);
    while (ros::ok()) {
        // Solve the QP problem
        if (solver_.solveProblem() != OsqpEigen::ErrorExitFlag::NoError) {
            ROS_ERROR_STREAM("[MPC::solveQP()] Unable to solve the problem.");
            return false;
        }

        // Check if the solution is unfeasible
        if (solver_.getStatus() != OsqpEigen::Status::Solved) {
            ROS_ERROR_STREAM("[MPC::solveQP()] The solution is unfeasible.");
            return false;
        }

        // Obtain the solution, and append the new waypoint to the trajectory  (`trajectoryGoal_`)
        QPSolution = solver_.getSolution();
        controlInput = QPSolution.block(QP_DYNAMIC_SIZE, 0, MPC_INPUT_SIZE, 1);

        trajectoryWaypoint_.positions = {x0_[0], x0_[1], x0_[2], x0_[3], x0_[4], x0_[5]};
        trajectoryWaypoint_.velocities = {controlInput[0], controlInput[1], controlInput[2],
                                            controlInput[3], controlInput[4], controlInput[5]};
        trajectoryWaypoint_.time_from_start = ros::Duration((counter++) * MPC_SAMPLE_TIME);
        trajectoryGoal_.trajectory.points.push_back(trajectoryWaypoint_);
        
        // Propagate the model
        x0_ = A_ * x0_ + B_ * controlInput;

        // for (size_t i = 0; i < MPC_STATE_SIZE; ++i)
        //     std::cout << x0_[i] << ' ';
        // std::cout << std::endl;

        // Check if we reach the desired pose. If not, then continue solving the problem
        if ((x0_ - xRef_).norm() < MPC_ZERO) {
            ROS_INFO_STREAM("[MPC::solveQP()] Reach to the desired point!");
            break;
        }

        // Update the constraint vectors
        if (!this->updateConstraintVectors()) {
            ROS_ERROR_STREAM("[MPC::solveQP()] Error updating the constraint vectors.");
            return false;
        }

        // ros::spinOnce();
        // loopRate_.sleep();
    }
    clock_gettime(CLOCK_REALTIME, &finishTime);
    ROS_INFO_STREAM("[MPC::solveQP()] Time taken: " << marslite::time::getOperationTime(startTime, finishTime));

    // Send the trajectory goal to the server, and wait for the result
    actionClient_->sendGoal(trajectoryGoal_);
    actionClient_->waitForResult();

    // Check if the action was successful
    ROS_INFO_STREAM(actionClient_->getState().toString());
    if (actionClient_->getState() != actionlib::SimpleClientGoalState::SUCCEEDED) {
        ROS_ERROR_STREAM("[MPC::solveQP()] Failed to execute the trajectory.");
        return false;
    }

    ROS_INFO_STREAM("[MPC::solveQP()] Successfully Execute the trajectory!");
    return true;
}

// bool ModelPredictiveControl::getStopNodeFlag() const noexcept
// {
//     return stopNode_;
// }

/* ********************************************** *
 *                 Initialization                 *
 * ********************************************** */

bool ModelPredictiveControl::subscribeRobotState()
{
    // Subscribe the `/joint_states` topic
    robotStateSubscriber_ = nh_.subscribe("/joint_states", 1, &ModelPredictiveControl::robotStateCallback, this);

    // Wait for the publisher of the `/joint_states` topic
    const ros::Duration timestep = ros::Duration(0.1);
    signalTimeoutTimer_ = ros::Duration(0);
    ROS_INFO_STREAM("[MPC::subscribeRobotState()] Waiting for subscribing the \"/joint_states\" topic...");
    while (ros::ok() && robotStateSubscriber_.getNumPublishers() == 0 && signalTimeoutTimer_ < maxTimeout_) {
        // Check subscription every 0.1 seconds
        signalTimeoutTimer_ += timestep;
        timestep.sleep();
    }

    if (signalTimeoutTimer_ >= maxTimeout_) {
        // [Error] Reach the timeout
        ROS_ERROR_STREAM("[MPC::subscribeRobotState()] Timeout (" << maxTimeout_ << " seconds) reached while"
                        << " waiting for subscribing the \"/joint_states\" topic.");
        return false;
    }

    ROS_INFO_STREAM("[MPC::subscribeRobotState()] Successfully subscribing the \"/joint_states\" topic!");
    return true;
}

bool ModelPredictiveControl::connectJointTrajectoryActionServer()
{
    // Initialize the `FollowJointTrajectoryAction` action client
    actionClient_ = std::make_shared<JointTrajectoryAction>("/arm_controller/follow_joint_trajectory", true);

    // Wait for the connection to the server
    ROS_INFO_STREAM("[MPC::connectJointTrajectoryActionServer()] Waiting for"
                        << " connecting the FollowJointTrajectoryAction action server...");
    if (!actionClient_->waitForServer(maxTimeout_)) {
        ROS_ERROR_STREAM("[MPC::connectJointTrajectoryActionServer()] Timeout (" << maxTimeout_ << " seconds) reached while"
                        << " waiting for FollowJointTrajectoryAction action server.");
        return false;
    }

    ROS_INFO_STREAM("[MPC::connectJointTrajectoryActionServer()] Successfully connecting the"
                        << " FollowJointTrajectoryAction action server!");
    return true;
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
    xMax_ = MPC_LIMIT_MARSLITE_POSITION_MAX;
    xMin_ = MPC_LIMIT_MARSLITE_POSITION_MIN;
    uMax_ = MPC_LIMIT_MARSLITE_VELOCITY_MAX;
    uMin_ = MPC_LIMIT_MARSLITE_VELOCITY_MIN;
    aMax_ = MPC_LIMIT_MARSLITE_ACCELERATION_MAX;
    aMin_ = MPC_LIMIT_MARSLITE_ACCELERATION_MIN;
}

void ModelPredictiveControl::setWeightMatrices()
{
    Q_.diagonal() << 1., 1., 1., 1., 1., 1., 1., 1.; 
    R_.diagonal() << 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01;
}

void ModelPredictiveControl::setInitialStateSpace(const StateVector& x0)
{
    x0_ = x0;
}

void ModelPredictiveControl::setReferenceStateSpace(const StateVector& xRef)
{
    xRef_ = xRef;
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
    Eigen::Matrix<double, MPC_STATE_SIZE, 1> Qx_ref = Q_ * (-xRef_);

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

bool ModelPredictiveControl::updateConstraintVectors()
{
    lowerBound_.block(0, 0, MPC_STATE_SIZE, 1) = -x0_;
    upperBound_.block(0, 0, MPC_STATE_SIZE, 1) = -x0_;
    if (!solver_.updateBounds(lowerBound_, upperBound_)) return false;

    return true;
}

/* ********************************************** *
 *                Callback functions              *
 * ********************************************** */

void ModelPredictiveControl::robotStateCallback(const sensor_msgs::JointState::ConstPtr& msg)
{
    robotState_ = *msg;
}

} // namespace mpc

} // namespace marslite