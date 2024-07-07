/**
 * @file test_kinematics.cpp
 * @author Handian Yang
 * @copyright Released under the terms of the GPLv3.0 or later
 * @date 2024
 * 
 * @brief The executable for testing the kinematics function for TM5 robotic arms.
 * 
 * @note `test_kinematics.cpp` is part of `marslite_simulation_ws`.
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

#include <ros/ros.h>
// #include <vector>
#include <Eigen/Dense>

#include "tm_driver_t/tm_kin.h"
#include "tm_kinematics/tm_kinematics.h"
using marslite::control::TMKinematics;

#include "marslite_properties/Arithmetic.h"
using marslite::math::rad2Deg;
using marslite::math::deg2Rad;

#include "marslite_properties/Exception.h"
using marslite::exception::ConstructorInitializationFailedException;
using marslite::exception::AssertionFailedException;

// tolerance for comparing two angles
static constexpr double kAngleTolerance = 1e-03;

/**
 * @brief Print the transformation matrix provided from `tm_kin.h`.
 * @param T The transformation matrix of type `double*` in size 16.
 * @param name The name of the transformation matrix. Default is an empty string.
 */
void printTransformationMatrix(const double* T, const char* name = "")
{
    ROS_INFO("[Transformation matrix] %s", name);
    ROS_INFO("%6.3f  %6.3f  %6.3f  %6.3f", T[0], T[1], T[2], T[3]);
    ROS_INFO("%6.3f  %6.3f  %6.3f  %6.3f", T[4], T[5], T[6], T[7]);
    ROS_INFO("%6.3f  %6.3f  %6.3f  %6.3f", T[8], T[9], T[10], T[11]);
    ROS_INFO("%6.3f  %6.3f  %6.3f  %6.3f", T[12], T[13], T[14], T[15]);
    ROS_INFO("--------------------------------------------------");
}

/**
 * @brief Print the transformation matrix provided from `tm_kinematics.h`.
 * @param T The transformation matrix of type `Eigen::MatrixXd` in size 4x4.
 * @param name The name of the transformation matrix. Default is an empty string.
 */
void printTransformationMatrix(const TMKinematics::TransformationMatrix& T, const char* name = "")
{
    ROS_INFO("[Transformation matrix] %s", name);
    ROS_INFO("%6.3f  %6.3f  %6.3f  %6.3f", T(0,0), T(0,1), T(0,2), T(0,3));
    ROS_INFO("%6.3f  %6.3f  %6.3f  %6.3f", T(1,0), T(1,1), T(1,2), T(1,3));
    ROS_INFO("%6.3f  %6.3f  %6.3f  %6.3f", T(2,0), T(2,1), T(2,2), T(2,3));
    ROS_INFO("%6.3f  %6.3f  %6.3f  %6.3f", T(3,0), T(3,1), T(3,2), T(3,3));
    ROS_INFO("--------------------------------------------------");
}

/**
 * @brief Print the inverse kinematics solutions provided from `tm_kin.h`
 *        in radians.
 * @param Q The joint angles of type `double*` in radians. This pointer points
 *          to an array with size Nx6, where N is the number of solutions.
 * @param numSolution The number of solutions.
 * @param name The name of the solutions. Default is an empty string.
 */
void printIKSolution(const double* Q, const uint8_t& numSolution, const char* name = "")
{
    ROS_INFO("[Inverse kinematics solutions] %s", name);
    for (uint8_t i = 0; i < numSolution; i++) {
        ROS_INFO("(%d) %8.3f, %8.3f, %8.3f, %8.3f, %8.3f, %8.3f", i,
            rad2Deg(Q[i*6]), rad2Deg(Q[i*6+1]), rad2Deg(Q[i*6+2]),
            rad2Deg(Q[i*6+3]), rad2Deg(Q[i*6+4]), rad2Deg(Q[i*6+5]));
    }
    ROS_INFO("--------------------------------------------------");
}

/**
 * @brief Print the inverse kinematics solutions provided from `tm_kinematics.h`
 *        in radians.
 * @param Q The joint angles of type `Eigen::MatrixXd` in radians. The matrix
 *          is in size Nx6, where N is the number of solutions.
 * @param name The name of the solutions. Default is an empty string.
 */
void printIKSolution(const Eigen::MatrixXd& Q, const char* name = "")
{
    ROS_INFO("[Inverse kinematics solutions] %s", name);
    for (uint8_t i = 0; i < Q.rows(); i++) {
        ROS_INFO("(%d) %8.3f, %8.3f, %8.3f, %8.3f, %8.3f, %8.3f", i,
            rad2Deg(Q(i, 0)), rad2Deg(Q(i, 1)), rad2Deg(Q(i, 2)),
            rad2Deg(Q(i, 3)), rad2Deg(Q(i, 4)), rad2Deg(Q(i, 5)));
    }
    ROS_INFO("--------------------------------------------------");
}

/**
 * @brief Test the forward kinematics function by comparing the results with the
 *        standard solutions provided by `tm_kin.h`. The result will be
 *        considered as valid if the two transformation matrices are the same.
 * @param Q The joint angles in radians in size 6.
 * @return `true` if the test passes, `false` otherwise.
 */
bool testForwardKinematics(const double* Q)
{
    // Calculate the forward kinematics using `tm_kin.h` methods
    double Q_old[6] = { Q[0], Q[1], Q[2], Q[3], Q[4], Q[5] };
    double T_old[16];
    tm_kinematics::forward(Q_old, T_old);

    // Calculate the forward kinematics using `tm_kinematics.h` methods
    TMKinematics::TMKinematicsPtr classPtr = std::make_shared<TMKinematics>(marslite::control::TM5_700);
    TMKinematics::JointVector Q_new;
    Q_new << Q[0], Q[1], Q[2], Q[3], Q[4], Q[5];
    TMKinematics::TransformationMatrix T_new = classPtr->forwardKinematics(Q_new);

    // Compare two results
    for (uint8_t i = 0; i < 16; i++) {
        if (std::abs(T_old[i] - T_new(i/4, i%4)) > kAngleTolerance) {
            ROS_ERROR("Forward kinematics test failed.");
            printTransformationMatrix(T_old, "Standard solution");
            printTransformationMatrix(T_new, "My solution");
            return false;
        }
    }
    
    ROS_INFO("Forward kinematics test passed!");
    printTransformationMatrix(T_new, "My solution");
    return true;
}

/**
 * @brief Test the inverse kinematics function by comparing the results with the
 *        standard solutions provided by `tm_kin.h`. The result will be
 *        considered as valid only if all solutions are the same as the standard
 *        solutions.
 * @param T The transformation matrix in size 4x4 (=16)
 * @return `true` if the test passes, `false` otherwise.
 */
bool testInverseKinematics(const double* T)
{
    // Calculate the inverse kinematics using `tm_kin.h` methods
    double Q_sols_old[8*6];
    double T_old[16] = { T[0], T[1], T[2], T[3],
                         T[4], T[5], T[6], T[7], 
                         T[8], T[9], T[10], T[11], 
                         T[12], T[13], T[14], T[15] };
    const int num_sols_old = tm_kinematics::inverse(T_old, Q_sols_old, 0.0);

    // Calculate the inverse kinematics using `tm_kinematics.h` methods
    TMKinematics::TMKinematicsPtr classPtr = std::make_shared<TMKinematics>(marslite::control::TM5_700);
    TMKinematics::TransformationMatrix T_new;
    T_new << T[0], T[1], T[2], T[3],
             T[4], T[5], T[6], T[7],
             T[8], T[9], T[10], T[11],
             T[12], T[13], T[14], T[15];
    Eigen::MatrixXd Q_sols_new = classPtr->inverseKinematics(T_new);

    // Compare the number of solutions of two results
    if (num_sols_old != Q_sols_new.rows()) {
        ROS_ERROR("Inverse kinematics test failed.");
        ROS_ERROR("Number of solutions: %d (Standard) vs %ld (My solution)", num_sols_old, Q_sols_new.rows());
        return false;
    }

    // Compare two results
    bool isValidSolution = false;
    bool isVisited[8] = { false };
    for (uint8_t i = 0; i < num_sols_old; i++) {
        for (uint8_t j = 0; j < num_sols_old; ++j) {
            if (isVisited[j]) continue;
            
            // Find the solution that is closest to one of the standard solutions
            isValidSolution = true;
            for (uint8_t k = 0; k < 6; k++) {
                const double diff = std::abs(Q_sols_old[i*6+k] - Q_sols_new(j, k));
                if (diff > kAngleTolerance && diff < 2*M_PI - kAngleTolerance) {
                    isValidSolution = false;
                    break;
                }
            }
            
            // Mark the solution as visited if it is valid
            if (isValidSolution) {
                isVisited[j] = true;
                break;
            }
        }

        if (!isValidSolution) {
            // Fail to find the same solution
            ROS_ERROR("Inverse kinematics test failed.");
            printIKSolution(Q_sols_old, num_sols_old, "Standard solution");
            printIKSolution(Q_sols_new, "My solution");
            return false;
        }
    }

    ROS_INFO("Inverse kinematics test passed!");
    printIKSolution(Q_sols_new, "My solution");
    return true;
}

/**
 * @brief Throw an exception when the assertion fails. This function is used
 *        within the `ROS_ASSERT_CMD` macro to make the code more readable.
 *        The function is called only when the assertion fails, and throws
 *        an `AssertionFailedException`.
 * @throw `AssertionFailedException`
 */
inline void throwAssertionFailedException()
{
    throw AssertionFailedException();
}

/**
 * @brief Add a test case for the kinematics function with the given joint
 *        angles in degrees. The function will test the forward kinematics
 *        and inverse kinematics, and compare the results with the standard
 *        solutions (provided from `tm_kin.h`).
 * @param Q1 The joint angle of the 1st joint in degrees.
 * @param Q2 The joint angle of the 2nd joint in degrees.
 * @param Q3 The joint angle of the 3rd joint in degrees.
 * @param Q4 The joint angle of the 4th joint in degrees.
 * @param Q5 The joint angle of the 5th joint in degrees.
 * @param Q6 The joint angle of the 6th joint in degrees.
 * @throw `AssertionFailedException` if the test fails.
 */
void addTestInDegree(const double& Q1, const double& Q2, const double& Q3,
                     const double& Q4, const double& Q5, const double& Q6)
{
    try {
        // [Forward kinematics]
        double Q[6] = { deg2Rad(Q1), deg2Rad(Q2), deg2Rad(Q3),
                        deg2Rad(Q4), deg2Rad(Q5), deg2Rad(Q6) };
        ROS_ASSERT_CMD(testForwardKinematics(Q), throwAssertionFailedException());
        
        // [Inverse kinematics]
        double T[16];
        tm_kinematics::forward(Q, T);
        ROS_ASSERT_CMD(testInverseKinematics(T), throwAssertionFailedException());
    } catch (const AssertionFailedException& e) {
        throw e;        
    }
}

/**
 * @brief Add a test case for the kinematics function with the given joint
 *        angles in radians. The function will test the forward kinematics
 *        and inverse kinematics, and compare the results with the standard
 *        solutions (provided from `tm_kin.h`).
 * @param Q1 The joint angle of the 1st joint in radians.
 * @param Q2 The joint angle of the 2nd joint in radians.
 * @param Q3 The joint angle of the 3rd joint in radians.
 * @param Q4 The joint angle of the 4th joint in radians.
 * @param Q5 The joint angle of the 5th joint in radians.
 * @param Q6 The joint angle of the 6th joint in radians.
 * @throw `AssertionFailedException` if the test fails.
 */
void addTestInRadian(const double& Q1, const double& Q2, const double& Q3,
                     const double& Q4, const double& Q5, const double& Q6)
{
    try {
        // [Forward kinematics]
        double Q[6] = { Q1, Q2, Q3, Q4, Q5, Q6 };
        ROS_ASSERT_CMD(testForwardKinematics(Q), throwAssertionFailedException());
        
        // [Inverse kinematics]
        double T[16];
        tm_kinematics::forward(Q, T);
        ROS_ASSERT_CMD(testInverseKinematics(T), throwAssertionFailedException());
    } catch (const AssertionFailedException& e) {
        throw e;        
    }
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "test_kinematics");

    try {
        ROS_INFO("--------------------------------------------------");

        // Test 1: HOME pose
        addTestInDegree(0, -42, 113, -71, 90, 0);

        // Test 2: DEFAULT1 pose
        addTestInDegree(0, 0, 90, 0, 90, 0);

        // Test 3: DEFAULT2 pose
        addTestInDegree(0, 0, 90, -90, 90, 0);

        // Test 4: DEFAULT3 pose
        addTestInDegree(0, 0, 90, 90, -90, 0);

        // Test 5: customized pose
        addTestInDegree(20, -30, 100, -60, 80, 10);

        // Test 6: customized pose
        addTestInDegree(-10, 20, 80, -40, 70, 5);

        // Test 7: customized pose
        addTestInDegree(30, -40, 120, -80, 100, 15);

        // Test 8: customized pose
        addTestInDegree(-20, 30, 100, -60, 0, 10);

        ROS_INFO("All tests passed!");

    } catch (const ConstructorInitializationFailedException& e) {
        ROS_ERROR("%s", e.what());
    } catch (const AssertionFailedException& e) {
        ROS_ERROR("%s", e.what());
    }

    ros::shutdown();
    return 0;
}