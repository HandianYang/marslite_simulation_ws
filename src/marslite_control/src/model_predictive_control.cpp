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
  this->setDynamicsMatrices();
  this->setInequalityConstraints();
  this->setWeightMatrices();

  this->castMPCToQPHessian();
  this->castMPCToQPGradient();
  this->castMPCToQPConstraintMatrix();
  this->castMPCToQPConstraintVectors();

  if (!this->initializeQPSolver()) {
    ROS_ERROR_STREAM("Error initializing QP solver.");
    throw ConstructorInitializationFailedException();
  }
}

bool ModelPredictiveControl::initializeQPSolver()
{
  solver_.settings()->setWarmStart(true);
  solver_.settings()->setVerbosity(false);
  solver_.settings()->setMaxIteration(10000);

  solver_.data()->setNumberOfVariables(QP_STATE_SIZE);
  solver_.data()->setNumberOfConstraints(QP_BOUND_SIZE);
  if(!solver_.data()->setHessianMatrix(hessian_matrix_))   return false;
  if(!solver_.data()->setGradient(gradient_))             return false;
  if(!solver_.data()->setLinearConstraintsMatrix(constraint_matrix_))  return false;
  if(!solver_.data()->setLowerBound(lower_bound_))   return false;
  if(!solver_.data()->setUpperBound(upper_bound_))   return false;
  if(!solver_.initSolver()) return false;
  
  return true;
}

bool ModelPredictiveControl::solveQP(std::vector<trajectory_msgs::JointTrajectoryPoint>& trajectory_waypoints)
{
  if (this->isOutOfBound(x0_)) {
    ROS_ERROR_STREAM("  Initial pose is out of bound.");
    return false;
  }

  if (this->isOutOfBound(xRef_)) {
    ROS_ERROR_STREAM("  Target pose is out of bound.");
    return false;
  }

  if (!this->updateGradient(xRef_)) {
    ROS_ERROR_STREAM("  Error updating the gradient.");
    return false;
  }
  if (!this->updateConstraintVectors(x0_)) {
    ROS_ERROR_STREAM("  Error updating the constraint vectors.");
    return false;
  }

  // [Special case] Check if the robot is already at the target pose
  if ((x0_ - xRef_).norm() < MPC_ZERO) {
    ROS_INFO_STREAM("The target pose is almost at the initial pose. Skip the planning process...");
    return true;
  }

  Eigen::VectorXd qp_solution;  // QP solution vector
  InputVector u;                // current input
  StateVector x = x0_;          // current state
  size_t counter = 0;           // counter for the trajectory waypoints
  trajectory_msgs::JointTrajectoryPoint trajectory_waypoint;  // trajectory waypoint

  // (Crucial) Add the initial pose with zero velocity
  trajectory_waypoint.velocities = std::vector<double>(6, 0.0);
  trajectory_waypoint.positions  = {x[0], x[1], x[2], x[3], x[4], x[5]};
  trajectory_waypoint.time_from_start = ros::Duration((counter++) * MPC_SAMPLE_TIME);
  trajectory_waypoints.push_back(trajectory_waypoint);

  while ((x - xRef_).norm() >= MPC_ZERO) {
    if (solver_.solveProblem() != OsqpEigen::ErrorExitFlag::NoError) {
      ROS_ERROR_STREAM("\tUnable to solve the problem.");
      return false;
    }

    if (solver_.getStatus() != OsqpEigen::Status::Solved) {
      ROS_ERROR_STREAM("\tThe solution is unfeasible.");
      return false;
    }

    qp_solution = solver_.getSolution();
    u = qp_solution.block(QP_DYNAMIC_SIZE, 0, MPC_INPUT_SIZE, 1);
    x = A_ * x + B_ * u;
    
    trajectory_waypoint.velocities = {u[0], u[1], u[2], u[3], u[4], u[5]};
    trajectory_waypoint.positions  = {x[0], x[1], x[2], x[3], x[4], x[5]};
    trajectory_waypoint.time_from_start = ros::Duration((counter++) * MPC_SAMPLE_TIME);
    trajectory_waypoints.push_back(trajectory_waypoint);
     
    if (!this->updateConstraintVectors(x)) {
      ROS_ERROR_STREAM("\tError updating the constraint vectors.");
      return false;
    }
  }

  // (Crucial) Add the target pose with zero velocity
  trajectory_waypoint.velocities = std::vector<double>(6, 0.0);
  trajectory_waypoint.positions  = {xRef_[0], xRef_[1], xRef_[2], xRef_[3], xRef_[4], xRef_[5]};
  trajectory_waypoint.time_from_start = ros::Duration((counter++) * MPC_SAMPLE_TIME);
  trajectory_waypoints.push_back(trajectory_waypoint);

  // Downsample to 6 waypoints, including the initial and target poses
  // TODO: Modify the acceleration and the number of downsampled waypoints
  trajectory_waypoints = {
    trajectory_waypoints.front(),
    trajectory_waypoints[1],
    trajectory_waypoints[2],
    trajectory_waypoints[3],
    trajectory_waypoints[trajectory_waypoints.size() / 2],
    trajectory_waypoints.back()
  };

  return true;
}

bool ModelPredictiveControl::solveWithoutQP(std::vector<trajectory_msgs::JointTrajectoryPoint>& trajectory_waypoints)
{
  trajectory_msgs::JointTrajectoryPoint trajectory_waypoint;
  trajectory_waypoint.positions = {x0_[0], x0_[1], x0_[2], x0_[3], x0_[4], x0_[5]};
  trajectory_waypoint.velocities = {0, 0, 0, 0, 0, 0};
  trajectory_waypoint.time_from_start = ros::Duration(0.0);
  trajectory_waypoints.push_back(trajectory_waypoint);

  trajectory_waypoint.positions = {xRef_[0], xRef_[1], xRef_[2], xRef_[3], xRef_[4], xRef_[5]};
  trajectory_waypoint.velocities = {0, 0, 0, 0, 0, 0};
  trajectory_waypoint.time_from_start = ros::Duration(1.0); // TODO: Modify the duration
  trajectory_waypoints.push_back(trajectory_waypoint);

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
  hessian_matrix_.resize(QP_STATE_SIZE, QP_STATE_SIZE);

  double value;
  for (int i = 0; i < QP_DYNAMIC_SIZE; ++i) {
    value = Q_.diagonal()[i % MPC_STATE_SIZE];
    if (value != 0)
      hessian_matrix_.insert(i,i) = value;
  }

  for (int i = QP_DYNAMIC_SIZE; i < QP_STATE_SIZE; ++i) {
    value = R_.diagonal()[i % MPC_INPUT_SIZE];
    if (value != 0)
      hessian_matrix_.insert(i,i) = value;
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
  constraint_matrix_.resize(QP_BOUND_SIZE, QP_STATE_SIZE);

  for (int i = 0; i < QP_DYNAMIC_SIZE; ++i){
    constraint_matrix_.insert(i,i) = -1;
  }

  double value;
  for (int i = 0; i < MPC_WINDOW_SIZE; ++i) {
    for (int j = 0; j < MPC_STATE_SIZE; ++j) {
      for (int k = 0; k < MPC_STATE_SIZE; ++k) {
        value = A_(j,k);
        if (value != 0)
          constraint_matrix_.insert(MPC_STATE_SIZE * (i+1) + j, MPC_STATE_SIZE * i + k) = value;
      }
    }
  }
  
  for (int i = 0; i < MPC_WINDOW_SIZE; ++i) {
    for (int j = 0; j < MPC_STATE_SIZE; ++j) {
      for (int k = 0; k < MPC_INPUT_SIZE; ++k) {
        value = B_(j,k);
        if (value != 0)
          constraint_matrix_.insert(MPC_STATE_SIZE * (i+1) + j, MPC_INPUT_SIZE * i + k + QP_DYNAMIC_SIZE) = value;
      }
    }
  }

  for (int i = 0; i < QP_STATE_SIZE; ++i) {
    constraint_matrix_.insert(i + QP_DYNAMIC_SIZE, i) = 1;
  }

  for (int i = 0; i < MPC_INPUT_SIZE * MPC_WINDOW_SIZE; ++i) {
    constraint_matrix_.insert(i + 2 * QP_DYNAMIC_SIZE + QP_CONTROL_SIZE, i + QP_DYNAMIC_SIZE) = MPC_SAMPLE_FREQ;
  }

  for (int i = 0; i < MPC_INPUT_SIZE * (MPC_WINDOW_SIZE-1); ++i) {
    constraint_matrix_.insert(i + 2 * QP_DYNAMIC_SIZE + QP_CONTROL_SIZE + MPC_INPUT_SIZE, i + QP_DYNAMIC_SIZE) = -MPC_SAMPLE_FREQ;
  }
}

void ModelPredictiveControl::castMPCToQPConstraintVectors()
{
  // lower inequality vector (in size `SS*(W+1) + 2*IS*W` by `1`)
  Eigen::VectorXd lower_inequality = Eigen::MatrixXd::Zero(QP_INEQUALITY_SIZE, 1);
  // upper inequality vector (in size `SS*(W+1) + 2*IS*W` by `1`)
  Eigen::VectorXd upper_inequality = Eigen::MatrixXd::Zero(QP_INEQUALITY_SIZE, 1);
  for (int i = 0; i < MPC_WINDOW_SIZE+1; ++i) {
    lower_inequality.block(MPC_STATE_SIZE * i, 0, MPC_STATE_SIZE, 1) = xMin_;
    upper_inequality.block(MPC_STATE_SIZE * i, 0, MPC_STATE_SIZE, 1) = xMax_;
  }
  for (int i = 0; i < MPC_WINDOW_SIZE; ++i) {
    lower_inequality.block(MPC_INPUT_SIZE * i + QP_DYNAMIC_SIZE, 0, MPC_INPUT_SIZE, 1) = uMin_;
    upper_inequality.block(MPC_INPUT_SIZE * i + QP_DYNAMIC_SIZE, 0, MPC_INPUT_SIZE, 1) = uMax_;
  }
  for (int i = 0; i < MPC_WINDOW_SIZE; ++i) {
    lower_inequality.block(MPC_INPUT_SIZE * i + QP_STATE_SIZE, 0, MPC_INPUT_SIZE, 1) = aMin_;
    upper_inequality.block(MPC_INPUT_SIZE * i + QP_STATE_SIZE, 0, MPC_INPUT_SIZE, 1) = aMax_;
  }

  // lower equality vector (in size `SS*(W+1)` by `1`)
  Eigen::VectorXd lower_equality = Eigen::MatrixXd::Zero(QP_EQUALITY_SIZE, 1);
  // upper equality vector (in size `SS*(W+1)` by `1`)
  Eigen::VectorXd upper_equality = Eigen::MatrixXd::Zero(QP_EQUALITY_SIZE, 1);
  lower_equality.block(0, 0, MPC_STATE_SIZE, 1) = -x0_;
  upper_equality = lower_equality;
  
  // Merge inequality and equality vectors into constraint vectors
  lower_bound_ = Eigen::MatrixXd::Zero(QP_BOUND_SIZE, 1);
  lower_bound_ << lower_equality, lower_inequality;

  upper_bound_ = Eigen::MatrixXd::Zero(QP_BOUND_SIZE, 1);
  upper_bound_ << upper_equality, upper_inequality;
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
  lower_bound_.block(0, 0, MPC_STATE_SIZE, 1) = -x;
  upper_bound_.block(0, 0, MPC_STATE_SIZE, 1) = -x;

  return solver_.updateBounds(lower_bound_, upper_bound_);
}

/* ********************************************** *
 *                 Debug functions                *
 * ********************************************** */

void ModelPredictiveControl::printStateVector(const StateVector& state_vector, const std::string& name)
{
  std::stringstream ss;
  ss << std::fixed << std::setprecision(2);
  ss << "\t" << name << ": ";
  for (int i = 0; i < state_vector.size(); ++i) {
    ss << std::round(state_vector[i] * 100) / 100;
    if (i != state_vector.size() - 1) {
      ss << ", ";
    }
  }
  ROS_INFO_STREAM(ss.str());
}

} // namespace control

} // namespace marslite