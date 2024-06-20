/**
 * @file Pose.h
 * @author Handian Yang
 * @copyright Released under the terms of the GPLv3.0 or later
 * @date 2024
 * 
 * @brief The header file for defining some poses for marslite robots.
 * @note `Poses.h` is part of `marslite_simulation_ws`.
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

#ifndef MARSLITE_CONTROL_MPC_POSE_H_
#define MARSLITE_CONTROL_MPC_POSE_H_

#include <Eigen/Dense>

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
 * @namespace pose namespace for marslite robots
 */
namespace pose {

static const StateVector INITIAL = (
    StateVector() << 0., 0., 0., 0., 0., 0., 0., 0.
).finished();

static const StateVector HOME = (
    StateVector() << deg2Rad(0),
                     deg2Rad(-42),
                     deg2Rad(113),
                     deg2Rad(-71),
                     deg2Rad(90),
                     deg2Rad(0),
                     0,
                     deg2Rad(0)
).finished();

static const StateVector DEFAULT1 = (
    StateVector() << deg2Rad(0),
                     deg2Rad(0),
                     deg2Rad(90),
                     deg2Rad(0),
                     deg2Rad(90),
                     deg2Rad(0),
                     0,
                     deg2Rad(0)
).finished();

static const StateVector DEFAULT2 = (
    StateVector() << deg2Rad(0),
                     deg2Rad(0),
                     deg2Rad(90),
                     deg2Rad(-90),
                     deg2Rad(90),
                     deg2Rad(0),
                     0,
                     deg2Rad(0)
).finished();

static const StateVector DEFAULT3 = (
    StateVector() << deg2Rad(0),
                     deg2Rad(0),
                     deg2Rad(90),
                     deg2Rad(90),
                     deg2Rad(-90),
                     deg2Rad(0),
                     0,
                     deg2Rad(0)
).finished();

static const StateVector DEFAULT4 = (
    StateVector() << deg2Rad(0),
                     deg2Rad(0),
                     deg2Rad(90),
                     deg2Rad(0),
                     deg2Rad(-90),
                     deg2Rad(0),
                     0,
                     deg2Rad(0)
).finished();

} // namespace pose

} // namespace marslite

#endif  // #ifndef MARSLITE_CONTROL_MPC_POSE_H_