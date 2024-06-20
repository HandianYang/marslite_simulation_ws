/**
 * @file Constraint.h
 * @author Handian Yang
 * @copyright Released under the terms of the GPLv3.0 or later
 * @date 2024
 * 
 * @brief The header file for defining constraints in
 *          Model Predictive Control (MPC) function for marslite robots.
 * @note `Constraints.h` is part of `marslite_simulation_ws`.
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

#ifndef MARSLITE_CONTROL_MPC_CONSTRAINT_H_
#define MARSLITE_CONTROL_MPC_CONSTRAINT_H_

#include <Eigen/Dense>
#include "OsqpEigen/OsqpEigen.h"

#include "model_predictive_control/Constant.h"
using marslite::control::MPC_STATE_SIZE;
using marslite::control::MPC_INPUT_SIZE;
using StateVector = Eigen::Matrix<double, MPC_STATE_SIZE, 1>;
using InputVector = Eigen::Matrix<double, MPC_INPUT_SIZE, 1>;

#include "marslite_properties/Arithmetic.h"
using marslite::math::deg2Rad;

/**
 * @namespace marslite operation namespace
*/
namespace marslite {

/**
 * @namespace control namespace for marslite robots
*/
namespace constraint {

// state upper constraint vector with no limits
static const StateVector STATE_FREE_MAX = (
    StateVector() << OsqpEigen::INFTY, 
                     OsqpEigen::INFTY,
                     OsqpEigen::INFTY,
                     OsqpEigen::INFTY,
                     OsqpEigen::INFTY,
                     OsqpEigen::INFTY,
                     OsqpEigen::INFTY,
                     OsqpEigen::INFTY
).finished();

// state lower constraint vector with no limits
static const StateVector STATE_FREE_MIN = (
    StateVector() << -OsqpEigen::INFTY, 
                     -OsqpEigen::INFTY,
                     -OsqpEigen::INFTY,
                     -OsqpEigen::INFTY,
                     -OsqpEigen::INFTY,
                     -OsqpEigen::INFTY,
                     -OsqpEigen::INFTY,
                     -OsqpEigen::INFTY
).finished();

// input upper constraint vector with no limits
static const InputVector INPUT_FREE_MAX = (
    InputVector() << OsqpEigen::INFTY, 
                     OsqpEigen::INFTY,
                     OsqpEigen::INFTY,
                     OsqpEigen::INFTY,
                     OsqpEigen::INFTY,
                     OsqpEigen::INFTY,
                     OsqpEigen::INFTY,
                     OsqpEigen::INFTY
).finished();

// input lower constraint vector with no limits
static const InputVector INPUT_FREE_MIN = (
    InputVector() << -OsqpEigen::INFTY, 
                     -OsqpEigen::INFTY,
                     -OsqpEigen::INFTY,
                     -OsqpEigen::INFTY,
                     -OsqpEigen::INFTY,
                     -OsqpEigen::INFTY,
                     -OsqpEigen::INFTY,
                     -OsqpEigen::INFTY
).finished();

// upper constraint vector of marslite's joint position 
static const StateVector POSITION_MAX = (
    StateVector() << deg2Rad(130),
                     deg2Rad(125),
                     deg2Rad(153),
                     deg2Rad(190),
                     deg2Rad(178),
                     deg2Rad(180),
                     OsqpEigen::INFTY,
                     deg2Rad(180)
).finished();

// lower constraint vector of marslite's joint position 
static const StateVector POSITION_MIN = (
    StateVector() << deg2Rad(-115),
                     deg2Rad(-43),
                     deg2Rad(-153),
                     deg2Rad(-190),
                     deg2Rad(-178),
                     deg2Rad(-180),
                     -OsqpEigen::INFTY,
                     deg2Rad(-180)
).finished();

// upper constraint vector of marslite's joint velocity
static const InputVector VELOCITY_MAX = (
    InputVector() << deg2Rad(180),
                     deg2Rad(180),
                     deg2Rad(180),
                     deg2Rad(183),
                     deg2Rad(183),
                     deg2Rad(183),
                     0.7,
                     deg2Rad(180)
).finished();

// lower constraint vector of marslite's joint velocity
static const InputVector VELOCITY_MIN = (
    InputVector() << deg2Rad(-180),
                     deg2Rad(-180),
                     deg2Rad(-180),
                     deg2Rad(-183),
                     deg2Rad(-183),
                     deg2Rad(-183),
                     -0.7,
                     deg2Rad(-180)
).finished();

// upper constraint vector of marslite's joint acceleration
static const InputVector ACCELERATION_MAX = (
    InputVector() << OsqpEigen::INFTY, 
                     OsqpEigen::INFTY,
                     OsqpEigen::INFTY,
                     OsqpEigen::INFTY,
                     OsqpEigen::INFTY,
                     OsqpEigen::INFTY,
                     OsqpEigen::INFTY,
                     OsqpEigen::INFTY
).finished();

// lower constraint vector of marslite's joint acceleration
static const InputVector ACCELERATION_MIN = (
    InputVector() << -OsqpEigen::INFTY, 
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