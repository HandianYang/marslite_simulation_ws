/**
 * @file Constants.h
 * @author Handian Yang
 * @copyright Released under the terms of the GPLv3.0 or later
 * @date 2024
 * 
 * @brief The header file for defining some useful constants in Model Predictive Control (MPC) function for marslite robots.
 * @note `Constants.h` is part of `marslite_simulation_ws`.
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

#ifndef MARSLITE_MPC_CONSTANTS_H_
#define MARSLITE_MPC_CONSTANTS_H_

#include <stdint.h>

/**
 * @namespace marslite operation namespace
*/
namespace marslite {

/**
 * @namespace MPC namespace for marslite robots. Relationship: `marslite`::`mpc`
*/
namespace mpc {

static const double MPC_ZERO = 1e-04;    // distance that below this value would be treated as 0
static const double MPC_SAMPLE_FREQ = 25;
static const double MPC_SAMPLE_TIME = 1./MPC_SAMPLE_FREQ;

enum MPCConstants : uint64_t {
    MPC_STATE_SIZE  = 8,    // size of the state vector in MPC (or `SS` for short)
    MPC_INPUT_SIZE  = 8,    // size of the input vector in MPC (or `IS` for short)
    MPC_WINDOW_SIZE = 6     // size of the preview window (or `W` for short)
};

enum QPConstants : uint64_t {
    QP_DYNAMIC_SIZE     = MPC_STATE_SIZE*(MPC_WINDOW_SIZE+1),       // `SS` * (`W` + 1)
    QP_CONTROL_SIZE     = MPC_INPUT_SIZE*MPC_WINDOW_SIZE,           // `IS` * `W`

    QP_STATE_SIZE       = QP_DYNAMIC_SIZE   + QP_CONTROL_SIZE,      // `SS` * (`W` + 1) + `IS` * `W`
    QP_INEQUALITY_SIZE  = QP_DYNAMIC_SIZE   + QP_CONTROL_SIZE*2,    // `SS` * (`W` + 1) + 2 * `IS` * `W`
    QP_EQUALITY_SIZE    = QP_DYNAMIC_SIZE,                          // `SS` * (`W` + 1)
    QP_BOUND_SIZE       = QP_DYNAMIC_SIZE*2 + QP_CONTROL_SIZE*2     // 2 * `SS` * (`W` + 1) + 2 * `IS` * `W`
};

} // namespace mpc

} // namespace marslite

#endif  // #ifndef MARSLITE_MPC_CONSTANTS_H_