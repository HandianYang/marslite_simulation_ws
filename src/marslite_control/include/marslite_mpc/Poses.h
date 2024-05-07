/**
 * @file Poses.h
 * @author Handian Yang
 * @copyright Released under the terms of the GPLv3.0 or later
 * @date 2024
 * 
 * @brief The header file for defining some poses for marslite robots.
 * @note `Poses.h` is part of `marslite_simulation_ws`.
 * 
 * `marslite_simulation_ws` is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published
 *      by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
 * 
 * `marslite_simulation_ws` is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of
 *      MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License along with `marslite_simulation_ws`. If not, see <https://www.gnu.org/licenses/>. 
*/

#ifndef MARSLITE_MPC_POSES_H_
#define MARSLITE_MPC_POSES_H_

#include <Eigen/Dense>

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
 * @namespace poses namespace for poses in Model Predictive Control (MPC) function
*/
namespace poses {

static const StateVector MARSLITE_POSE_INITIAL = (
    StateVector() << 0., 0., 0., 0., 0., 0., 0., 0.
).finished();

static const StateVector MARSLITE_POSE_HOME = (
    StateVector() << deg2Rad(0),
                     deg2Rad(-42),
                     deg2Rad(113),
                     deg2Rad(-71),
                     deg2Rad(90),
                     deg2Rad(0),
                     0,
                     deg2Rad(0)
).finished();

static const StateVector MARSLITE_POSE_DEFAULT1 = (
    StateVector() << deg2Rad(0),
                     deg2Rad(0),
                     deg2Rad(90),
                     deg2Rad(0),
                     deg2Rad(90),
                     deg2Rad(0),
                     0,
                     deg2Rad(0)
).finished();

static const StateVector MARSLITE_POSE_DEFAULT2 = (
    StateVector() << deg2Rad(0),
                     deg2Rad(0),
                     deg2Rad(90),
                     deg2Rad(-90),
                     deg2Rad(90),
                     deg2Rad(0),
                     0,
                     deg2Rad(0)
).finished();

static const StateVector MARSLITE_POSE_DEFAULT3 = (
    StateVector() << deg2Rad(0),
                     deg2Rad(0),
                     deg2Rad(90),
                     deg2Rad(90),
                     deg2Rad(-90),
                     deg2Rad(0),
                     0,
                     deg2Rad(0)
).finished();

static const StateVector MARSLITE_POSE_DEFAULT4 = (
    StateVector() << deg2Rad(0),
                     deg2Rad(0),
                     deg2Rad(90),
                     deg2Rad(0),
                     deg2Rad(-90),
                     deg2Rad(0),
                     0,
                     deg2Rad(0)
).finished();

} // namespace poses

} // namespace marslite

#endif  // #ifndef MARSLITE_MPC_POSES_H_