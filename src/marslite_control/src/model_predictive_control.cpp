/**
 * marslite_simulation_ws/marslite_control/src/model_predictive_control.cpp
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

#include "model_predictive_control/model_predictive_control.h"

namespace marslite {

namespace control {

ModelPredictiveControl::ModelPredictiveControl()
{
  // Set MPC characteristics
  this->setDynamicsMatrices();
  this->setInequalityConstraints();
  this->setWeightMatrices();

  // Cast the MPC problem as QP problem
  this->castMPCToQPHessian();
  this->castMPCToQPGradient();
  this->castMPCToQPConstraintMatrix();
  this->castMPCToQPConstraintVectors();

  // Initialize the QP solver
  if (!this->initializeQPSolver()) {
    ROS_ERROR_STREAM("Error initializing QP solver.");
    throw ConstructorInitializationFailedException();
  }
}

bool ModelPredictiveControl::initializeQPSolver()
{
  // Set the warm start
  solver_.settings()->setWarmStart(true);
  solver_.settings()->setVerbosity(false);
  solver_.settings()->setMaxIteration(1000);

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

bool ModelPredictiveControl::solveQP(std::vector<trajectory_msgs::JointTrajectoryPoint>& trajectoryWaypoints)
{
  if (this->isOutOfBound(x0_)) {
    ROS_ERROR_STREAM("Initial pose is out of bound.");
    return false;
  }

  if (this->isOutOfBound(xRef_)) {
    ROS_ERROR_STREAM("Target pose is out of bound.");
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
    ROS_INFO_STREAM("The target pose is almost at the initial pose. Skip the planning process...");
    return true;
  }

  Eigen::VectorXd QPSolution; // QPSolution vector
  InputVector u;              // current input
  StateVector x = x0_;        // current state
  size_t counter = 0;         // counter for the trajectory waypoints

  while ((x - xRef_).norm() >= MPC_ZERO) {
    // Solve the QP problem
    if (solver_.solveProblem() != OsqpEigen::ErrorExitFlag::NoError) {
      ROS_ERROR_STREAM("\tUnable to solve the problem.");
      return false;
    }

    // Check if the solution is unfeasible
    if (solver_.getStatus() != OsqpEigen::Status::Solved) {
      ROS_ERROR_STREAM("\tThe solution is unfeasible.");
      return false;
    }

    // Obtain the solution, and append the new waypoint to the trajectory  (`trajectoryGoal_`)
    QPSolution = solver_.getSolution();
    u = QPSolution.block(QP_DYNAMIC_SIZE, 0, MPC_INPUT_SIZE, 1);

    trajectory_msgs::JointTrajectoryPoint trajectoryWaypoint;
    trajectoryWaypoint.positions  = {x[0], x[1], x[2], x[3], x[4], x[5]};
    trajectoryWaypoint.velocities = {u[0], u[1], u[2], u[3], u[4], u[5]};
    trajectoryWaypoint.time_from_start = ros::Duration((counter++) * MPC_SAMPLE_TIME);
    trajectoryWaypoints.push_back(trajectoryWaypoint);
    ROS_INFO_STREAM_COND(counter % 100 == 0, x.transpose()*180/M_PI);
    
    // Propagate the model
    x = A_ * x + B_ * u;

    // Update the constraint vectors
    if (!this->updateConstraintVectors(x)) {
      ROS_ERROR_STREAM("\tError updating the constraint vectors.");
      return false;
    }
  }

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
  xMax_ = marslite::constraint::POSITION_MAX;
  xMin_ = marslite::constraint::POSITION_MIN;
  uMax_ = marslite::constraint::VELOCITY_MAX;
  uMin_ = marslite::constraint::VELOCITY_MIN;
  aMax_ = marslite::constraint::ACCELERATION_MAX;
  aMin_ = marslite::constraint::ACCELERATION_MIN;
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

} // namespace control

} // namespace marslite