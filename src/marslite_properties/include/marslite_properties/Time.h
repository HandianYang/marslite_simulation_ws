/**
 * @file Time.h
 * @author Handian Yang
 * @copyright Released under the terms of the GPLv3.0 or later
 * @date 2024
 * 
 * @brief The header file for all time functions for marslite robots.
 * @note `Time.h` is part of `marslite_simulation_ws`.
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

#ifndef MARSLITE_TIME_H_
#define MARSLITE_TIME_H_


#include <time.h>

/**
 * @namespace marslite operation namespace
*/
namespace marslite {

/**
 * @namespace Time properties/operations for marslite. Relationship: `marslite:time`
*/
namespace time {

/**
 * @brief Calculate the time between the given starting time instance and finishing time instance.
 * @param start the given starting time instance (in type `struct timespec`)
 * @param finish the given finishing time instance (in type `struct timespec`)
 * @return the time interval in seconds
*/
const static inline double getOperationTime(const timespec& start, const timespec& finish)
{
    return (finish.tv_sec - start.tv_sec) + (finish.tv_nsec - start.tv_nsec) / 1e09;
}

} // namespace time

} // namespace marslite



#endif // MARSLITE_TIME_H_