/**
 * @file Exceptions.h
 * @author Handian Yang
 * @copyright Released under the terms of the GPLv3.0 or later
 * @date 2024
 * 
 * @brief The header file for all exceptions that may occur at marslite robots' operations.
 * @note `Exceptions.h` is part of `marslite_simulation_ws`.
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

#ifndef MARSLITE_PROPERTIES_EXCEPTION_H_
#define MARSLITE_PROPERTIES_EXCEPTION_H_

#include <string>
#include <exception>
#include <ros/duration.h>

/**
 * @namespace marslite operation namespace
*/
namespace marslite {

/**
 * @namespace Exceptions for marslite operations.
*/
namespace exception {

/**
 * @brief Exception for failed class initialization.
 */
class ConstructorInitializationFailedException : public std::exception {
public:
    const char* what() const noexcept override {
        return "Failed to initialize the class constructor. Aborting...";
    }
};

class AssertionFailedException : public std::exception {
public:
    const char* what() const noexcept override {
        return "Assertion failed. Aborting...";
    }
};

/**
 * @brief Exception for reaching the maximum timeout.
 */
class TimeOutException : public std::exception {
public:
    explicit TimeOutException(const ros::Duration& maxTimeout)
        : maxTimeout_(maxTimeout) {}

    const char* what() const noexcept override {
        const std::string msg = "Timeout (" + std::to_string(maxTimeout_.toSec())
            + " seconds) reached. Aborting...";
        return msg.c_str();
    }
private:
    ros::Duration maxTimeout_;
};

/**
 * @brief Exception for failing to find the transform between two frames.
 */
class TransformNotFoundException : public std::exception {
public:
    explicit TransformNotFoundException(const std::string& fromFrame, const std::string& toFrame)
        : fromFrame_(fromFrame), toFrame_(toFrame) {}

    const char* what() const noexcept override {
        const std::string msg = "Failed to find the transform from frame " + fromFrame_
            + " to frame " + toFrame_ + ". Aborting...";
        return msg.c_str();
    }
private:
    std::string fromFrame_, toFrame_;
};


/**
 * @brief Exception for vectors with size less than 2.
*/
class DataNumberLessThan2Exception : public std::exception {
public:
    explicit DataNumberLessThan2Exception(const long& dataNum = -1)
        : dataNum_(dataNum) {}

    const char* what() const noexcept override {
        const std::string msg = "Number of data (" + std::to_string(dataNum_)
            + " was given) less than 2. Aborting...";
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
    explicit MismatchSizeException(const long& aSize = -1, const long& bSize = -1)
        : aSize_(aSize), bSize_(bSize) {}

    const char* what() const noexcept override {
        const std::string msg = "Mismatch size of two vectors (" + std::to_string(aSize_)
            + " and " + std::to_string(bSize_) + " were given)";
        return msg.c_str();
    }

private:
    long aSize_, bSize_;
};

} // namespace exception

} // namespace marslite


#endif  // #ifndef MARSLITE_PROPERTIES_EXCEPTION_H_