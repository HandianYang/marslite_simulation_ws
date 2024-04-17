/**
 * @file Exceptions.h
 * @author Handian Yang
 * @copyright Released under the terms of the GPLv3.0 or later
 * @date 2024
 * 
 * @brief The header file for all exceptions that may occur at marslite robots' operations.
 * @note `Exceptions.h` is part of `marslite_simulation_ws`.
 * 
 * `marslite_simulation_ws` is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published
 *      by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
 * 
 * `marslite_simulation_ws` is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of
 *      MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License along with `marslite_simulation_ws`. If not, see <https://www.gnu.org/licenses/>. 
*/

#ifndef MARSLITE_EXCEPTIONS_H_
#define MARSLITE_EXCEPTIONS_H_

#include <string>
#include <exception>

/**
 * @namespace marslite operation namespace
*/
namespace marslite {

/**
 * @namespace Mathematic operations for marslite. Relationship: `marslite:math`
*/
namespace math {

/**
 * @brief Exception for vectors with size less than 2.
*/
class DataNumberLessThan2Exception : public std::exception {
public:
    explicit DataNumberLessThan2Exception(const long& dataNum = -1) : dataNum_(dataNum) {}

    const char* what() const noexcept override {
        const std::string msg = "[Error] number of data (" + std::to_string(dataNum_) + " was given) less than 2";
        return msg.c_str();
    }

private:
    long dataNum_;
};

/**
 * @brief Exception for mismatch size between two vectors.
*/
class MismatchSizeException : public std::exception {
public:
    explicit MismatchSizeException(const long& aSize = -1, const long& bSize = -1) : aSize_(aSize), bSize_(bSize) {}

    const char* what() const noexcept override {
        const std::string msg = "[Error] mismatch size of two vectors (" + std::to_string(aSize_)
            + " and " + std::to_string(bSize_) + " were given)";
        return msg.c_str();
    }

private:
    long aSize_, bSize_;
};

} // namespace math

} // namespace marslite



#endif  // #ifndef MARSLITE_EXCEPTIONS_H_