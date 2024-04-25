/**
 * @file Constraints.h
 * @author Handian Yang
 * @copyright Released under the terms of the GPLv3.0 or later
 * @date 2024
 * 
 * @brief The header file for defining constraints in Model Predictive Control (MPC) function for marslite robots.
 * @note `Constraints.h` is part of `marslite_simulation_ws`.
 * 
 * `marslite_simulation_ws` is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published
 *      by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
 * 
 * `marslite_simulation_ws` is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of
 *      MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License along with `marslite_simulation_ws`. If not, see <https://www.gnu.org/licenses/>. 
*/

#ifndef MARSLITE_MPC_CONSTRAINTS_H_
#define MARSLITE_MPC_CONSTRAINTS_H_

#include <Eigen/Dense>
#include "OsqpEigen/OsqpEigen.h"

#include "marslite_mpc/Constants.h"
using marslite::mpc::MPC_STATE_SIZE;
using marslite::mpc::MPC_INPUT_SIZE;
using StateVector = Eigen::Matrix<double, MPC_STATE_SIZE, 1>;
using InputVector = Eigen::Matrix<double, MPC_INPUT_SIZE, 1>;

#include "marslite_properties/Arithmetics.h"
using marslite::math::deg2Rad;

/**
 * @namespace marslite operation namespace
*/
namespace marslite {

/**
 * @namespace MPC namespace for marslite robots. Relationship: `marslite`::`MPC`
*/
namespace constraints {

// state upper constraint vector with no limits
static const StateVector STATE_FREE_LIMIT_MAX = (
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
static const StateVector STATE_FREE_LIMIT_MIN = (
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
static const InputVector INPUT_FREE_LIMIT_MAX = (
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
static const InputVector INPUT_FREE_LIMIT_MIN = (
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
static const StateVector MARSLITE_POSITION_LIMIT_MAX = (
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
static const StateVector MARSLITE_POSITION_LIMIT_MIN = (
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
static const InputVector MARSLITE_VELOCITY_LIMIT_MAX = (
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
static const InputVector MARSLITE_VELOCITY_LIMIT_MIN = (
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
static const InputVector MARSLITE_ACCELERATION_LIMIT_MAX = (
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
static const InputVector MARSLITE_ACCELERATION_LIMIT_MIN = (
    InputVector() << -OsqpEigen::INFTY, 
                     -OsqpEigen::INFTY,
                     -OsqpEigen::INFTY,
                     -OsqpEigen::INFTY,
                     -OsqpEigen::INFTY,
                     -OsqpEigen::INFTY,
                     -OsqpEigen::INFTY,
                     -OsqpEigen::INFTY
).finished();

} // namespace mpc

} // namespace marslite

#endif  // #ifndef MARSLITE_MPC_CONSTRAINTS_H_