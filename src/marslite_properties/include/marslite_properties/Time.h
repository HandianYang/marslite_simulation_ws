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
#include <ros/ros.h>
#include "marslite_properties/Exception.h"
using marslite::exception::TimeOutException;

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
static inline double getOperationTime(const timespec& start, const timespec& finish)
{
    return (finish.tv_sec - start.tv_sec) + (finish.tv_nsec - start.tv_nsec) / 1e09;
}

/**
 * @brief Subscribes to a ROS topic with a timeout.
 *
 * This function subscribes to the specified ROS topic and waits for the
 *  publisher to become available. If the publisher does not become available
 *  within the specified timeout duration, a `TimeOutException` is thrown.
 * 
 * @tparam ClassType The class type of the class instance.
 * @tparam MesssageType The message type of the ROS topic.
 *
 * @param classPtr A pointer to the class instance that contains the callback function.
 * @param nh The ROS NodeHandle.
 * @param subscriber The ROS Subscriber object.
 * @param topic The name of the ROS topic to subscribe to.
 * @param queueSize The maximum number of messages to queue up for processing.
 * @param callback The callback function to be called when a message is received.
 * @param timeout The maximum duration to wait for the publisher to become
 *                  available. Default is 10 seconds.
 * @param pollingSleepDuration The duration to sleep between checking for the
 *                              publisher. Default is 0.1 seconds.
 *
 * @throws `TimeOutException` if the publisher does not become available within
 *          the specified timeout duration.
 */
template <typename ClassType, typename MesssageType>
static void subscribeTopicWithTimeout(ClassType* classPtr,
        ros::NodeHandle& nh, ros::Subscriber& subscriber,
        const char* topic, const uint32_t& queueSize,
        void (ClassType::*callback)(const typename MesssageType::ConstPtr&),
        const bool& messageEnabled = false,
        const ros::Duration& timeout = ros::Duration(10),
        const ros::Duration& pollingSleepDuration = ros::Duration(0.1))
{
    ros::Duration signalTimeoutTimer = ros::Duration(0);

    // Subscribe the given ROS topic
    subscriber = nh.subscribe<MesssageType>(topic, queueSize, callback, classPtr);

    // Wait for the publisher of the given ROS topic
    ROS_INFO_STREAM_COND(messageEnabled, "Subscribing the \""
            << topic << "\" topic...");
    while (ros::ok() && subscriber.getNumPublishers() == 0) {
        // Check subscription every 0.1 seconds
        signalTimeoutTimer += pollingSleepDuration;
        if (signalTimeoutTimer >= timeout) throw TimeOutException(timeout);
        
        pollingSleepDuration.sleep();
    }
    ROS_INFO_STREAM_COND(messageEnabled, "\"" << topic << "\" subscribed!");
}

} // namespace time

} // namespace marslite



#endif // MARSLITE_TIME_H_