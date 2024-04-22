/**
 * @file Arithmetics.h
 * @author Handian Yang
 * @copyright Released under the terms of the GPLv3.0 or later
 * @date 2024
 * 
 * @brief The header file for all arithmetic functions for marslite robots.
 * @note `Arithmetics.h` is part of `marslite_simulation_ws`.
 * 
 * `marslite_simulation_ws` is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published
 *      by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
 * 
 * `marslite_simulation_ws` is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of
 *      MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License along with `marslite_simulation_ws`. If not, see <https://www.gnu.org/licenses/>. 
*/

#ifndef MARSLITE_ARITHMETICS_H_
#define MARSLITE_ARITHMETICS_H_

#include <vector>
#include <stdint.h>
#include <iostream>
#include <time.h>

#include "marslite_properties/Exceptions.h"
using marslite::math::DataNumberLessThan2Exception;
using marslite::math::MismatchSizeException;

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


/**
 * @namespace Mathematic operations for marslite. Relationship: `marslite:math`
*/
namespace math {

/**
 * @brief Generate a vector of evenly spaced points between `begin` and `end`.
 * @tparam dataT the type of the data
 * @param begin the first element of vector
 * @param end the last element of vector
 * @param number the number of the elements
 * @return the vector (in `std::vector<float>` type)
*/
template <class dataT>
static std::vector<dataT> linspace(const dataT& begin, const dataT& end, const size_t& dataNum)
{
    if (dataNum < 2)
        throw DataNumberLessThan2Exception(dataNum);

    std::vector<dataT> ret(dataNum, 0);
    ret[0] = begin;

    const dataT space = (end - begin) / (dataNum - 1);
    for (size_t i = 1; i < dataNum; ++i)
        ret[i] = ret[i-1] + space;

    return ret;
}

/**
 * @brief Calculate the definite integral of func(x)
 * @tparam dataT the type of the data
 * @param x the domain of the function
 * @param func the codomain of the function
 * @return the definite integral result
*/
template <class dataT>
static dataT integral(const std::vector<dataT>& x, const std::vector<dataT>& func)
{
    if (x.size() != func.size())
        throw MismatchSizeException(x.size(), func.size());
    if (x.size() < 2)
        throw DataNumberLessThan2Exception(x.size());
    
    dataT result = 0;
    const dataT space = x[1] - x[0];
    const size_t dataNum = x.size();
    for (size_t i = 1; i < dataNum; ++i)
        result += (func[i] - func[i-1]) / space;

    return result;
}

/**
 * @brief Determine whether the floating number should be treated as 0.
 * @param num the number to be determined
 * @return `TRUE` if the number is close to 0
*/
static inline bool reachZero(const double& num) {
    return (num >= -1e-06) && (num <= 1e-06);
}

/**
 * @brief Convert from degree to radian
 * @tparam dataT the data type of the number
 * @param deg angle in degree
 * @return angle in radian
*/
template <class dataT>
static inline double deg2Rad(const dataT& deg) {
    return deg / 180. * M_PI;
}

/**
 * @brief Convert from radian to degree
 * @tparam dataT the data type of the number
 * @param rad angle in radian
 * @return angle in degree
*/
template <class dataT>
static inline double rad2Deg(const dataT& rad) {
    return rad * 180. / M_PI;
}

} // namespace math

} // namespace marslite

#endif  // #ifndef MARSLITE_ARITHMETICS_H_