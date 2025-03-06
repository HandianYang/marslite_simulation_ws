/**
 * marslite_simulation_ws/marslite_control/include/Constraint.h
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

#ifndef MARSLITE_CONTROL_MPC_CONSTRAINT_H_
#define MARSLITE_CONTROL_MPC_CONSTRAINT_H_

#include <Eigen/Dense>
#include "OsqpEigen/OsqpEigen.h"

#include "model_predictive_control/Constant.h"
using marslite::control::MPC_STATE_SIZE;
using marslite::control::MPC_INPUT_SIZE;
using StateVector = Eigen::Matrix<double, MPC_STATE_SIZE, 1>;
using InputVector = Eigen::Matrix<double, MPC_INPUT_SIZE, 1>;

/**
 * @namespace marslite operation namespace
*/
namespace marslite {

/**
 * @namespace control namespace for marslite robots
*/
namespace constraint {

// state upper constraint vector with no limits
static const StateVector STATE_FREE_MAX = ( StateVector()
    << OsqpEigen::INFTY, 
       OsqpEigen::INFTY,
       OsqpEigen::INFTY,
       OsqpEigen::INFTY,
       OsqpEigen::INFTY,
       OsqpEigen::INFTY,
       OsqpEigen::INFTY,
       OsqpEigen::INFTY
).finished();

// state lower constraint vector with no limits
static const StateVector STATE_FREE_MIN = ( StateVector()
    << -OsqpEigen::INFTY, 
       -OsqpEigen::INFTY,
       -OsqpEigen::INFTY,
       -OsqpEigen::INFTY,
       -OsqpEigen::INFTY,
       -OsqpEigen::INFTY,
       -OsqpEigen::INFTY,
       -OsqpEigen::INFTY
).finished();

// input upper constraint vector with no limits
static const InputVector INPUT_FREE_MAX = ( InputVector()
    << OsqpEigen::INFTY, 
       OsqpEigen::INFTY,
       OsqpEigen::INFTY,
       OsqpEigen::INFTY,
       OsqpEigen::INFTY,
       OsqpEigen::INFTY,
       OsqpEigen::INFTY,
       OsqpEigen::INFTY
).finished();

// input lower constraint vector with no limits
static const InputVector INPUT_FREE_MIN = ( InputVector()
    << -OsqpEigen::INFTY, 
       -OsqpEigen::INFTY,
       -OsqpEigen::INFTY,
       -OsqpEigen::INFTY,
       -OsqpEigen::INFTY,
       -OsqpEigen::INFTY,
       -OsqpEigen::INFTY,
       -OsqpEigen::INFTY
).finished();

// upper constraint vector of marslite's joint position
// [130°, 125°, 153°, 190°, 178°, 180°, inf, 180°]
static const StateVector POSITION_MAX = ( StateVector()
    << 2.26893,
       2.18166,
       2.67035,
       3.31612,
       3.10669,
       3.14159,
       OsqpEigen::INFTY,
       3.14159
).finished();

// lower constraint vector of marslite's joint position
// TODO: Modify the second joint angle from -0.75049 (-43°) to -0.73304(-42°)
// [-115°, -42°, -153°, -190°, -178°, -180°, -inf, -180°]
static const StateVector POSITION_MIN = ( StateVector()
    << -2.00713,
       -0.75049,
       -2.67035,
       -3.31612,
       -3.10669,
       -3.14159,
       -OsqpEigen::INFTY,
       -3.14159
).finished();

// upper constraint vector of marslite's joint velocity
// [180°/s, 180°/s, 180°/s, 183°/s, 183°/s, 183°/s, 0.7m/s, 180°/s]
static const InputVector VELOCITY_MAX = ( InputVector()
    << 3.15,
       3.15,
       3.15,
       3.2,
       3.2,
       3.2,
       0.7,
       3.15
).finished();

// lower constraint vector of marslite's joint velocity
// [-180°/s, -180°/s, -180°/s, -183°/s, -183°/s, -183°/s, -0.7m/s, -180°/s]
static const InputVector VELOCITY_MIN = ( InputVector()
    << -3.15,
       -3.15,
       -3.15,
       -3.2,
       -3.2,
       -3.2,
       -0.7,
       -3.15
).finished();

// upper constraint vector of marslite's joint acceleration
static const InputVector ACCELERATION_MAX = ( InputVector()
    << OsqpEigen::INFTY, 
       OsqpEigen::INFTY,
       OsqpEigen::INFTY,
       OsqpEigen::INFTY,
       OsqpEigen::INFTY,
       OsqpEigen::INFTY,
       OsqpEigen::INFTY,
       OsqpEigen::INFTY
).finished();

// lower constraint vector of marslite's joint acceleration
static const InputVector ACCELERATION_MIN = ( InputVector()
    << -OsqpEigen::INFTY, 
       -OsqpEigen::INFTY,
       -OsqpEigen::INFTY,
       -OsqpEigen::INFTY,
       -OsqpEigen::INFTY,
       -OsqpEigen::INFTY,
       -OsqpEigen::INFTY,
       -OsqpEigen::INFTY
).finished();

} // namespace constraint

} // namespace marslite

#endif  // #ifndef MARSLITE_CONTROL_MPC_CONSTRAINT_H_