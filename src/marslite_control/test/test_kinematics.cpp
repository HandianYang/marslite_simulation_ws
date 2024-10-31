/**
 * marslite_simulation_ws/test/test_kinematics.cpp
 * 
 * Copyright (C) 2024 Handian Yang
 * 
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <https://www.gnu.org/licenses/>.
*/

#include <ros/ros.h>
#include <Eigen/Dense>

#include "tm_driver_t/tm_kin.h"           // Standard library
#include "tm_kinematics/tm_kinematics.h"  // My library
using marslite::control::TMKinematics;

#include "marslite_properties/Arithmetic.h"
using marslite::math::rad2Deg;
using marslite::math::deg2Rad;

#include "marslite_properties/Exception.h"
using marslite::exception::ConstructorInitializationFailedException;
using marslite::exception::AssertionFailedException;

#include "model_predictive_control/Pose.h"

// tolerance for comparing two angles
static constexpr double kTolerance = 1e-03;

/**
 * @brief Print the transformation matrix provided from `tm_kin.h`.
 * @param T The transformation matrix of type `double*` in size 16.
 * @param name The name of the transformation matrix. Default is an empty string.
 */
void printTransformationMatrix(const double* T, const char* name = "")
{
  ROS_INFO("[Transformation matrix] %s", name);
  ROS_INFO("%6.3f  %6.3f  %6.3f  %6.3f",  T[0],  T[1],  T[2],  T[3]);
  ROS_INFO("%6.3f  %6.3f  %6.3f  %6.3f",  T[4],  T[5],  T[6],  T[7]);
  ROS_INFO("%6.3f  %6.3f  %6.3f  %6.3f",  T[8],  T[9], T[10], T[11]);
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
        rad2Deg(Q[i*6]),    rad2Deg(Q[i*6+1]),  rad2Deg(Q[i*6+2]),
        rad2Deg(Q[i*6+3]),  rad2Deg(Q[i*6+4]),  rad2Deg(Q[i*6+5]));
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
    if (std::abs(T_old[i] - T_new(i/4, i%4)) > kTolerance) {
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
 * @brief Check if two transformation matrices are the same.
 * @param T1 The first transformation matrix in size 16.
 * @param T2 The second transformation matrix in size 16.
 * @return `true` if the two transformation matrices are the same, `false` otherwise.
 */
bool isSameTransformationMatrix(const double* T1, const double* T2)
{
  for (uint8_t i = 0; i < 16; i++) {
    if (std::abs(T1[i] - T2[i]) > kTolerance) {
      return false;
    }
  }
  return true;
}

/**
 * @brief Test the standard inverse kinematics function by comparing the
 *        transformation matrix obtained from the solution with the input
 *        transformation matrix. The result will be considered as valid only
 *        if two transformation matrices are the same.
 * @param T The transformation matrix in size 16.
 * @return `true` if the test passes, `false` otherwise.
 */
bool testStandardInverseKinematics(const double* T)
{
  // Calculate the inverse kinematics using `tm_kin.h` methods
  double Q_sols[8*6];
  const int num_sols_old = tm_kinematics::inverse(T, Q_sols, 0.0);

  double T_test[16];
  for (uint8_t i = 0; i < num_sols_old; i++) {
    double Q_test[6] = { Q_sols[i*6], Q_sols[i*6+1], Q_sols[i*6+2],
                         Q_sols[i*6+3], Q_sols[i*6+4], Q_sols[i*6+5] };
    tm_kinematics::forward(Q_test, T_test);
    if (!isSameTransformationMatrix(T, T_test)) {
      ROS_INFO("Standard inverse kinematics test failed! The solution %d is not valid.", i);
      printIKSolution(Q_sols, num_sols_old, "Standard solution");
      return false;
    }
  }

  ROS_INFO("Standard inverse kinematics test passed!");
  printIKSolution(Q_sols, num_sols_old, "Standard solution");
  return true;
}

/**
 * @brief Test my inverse kinematics function by comparing the
 *        transformation matrix obtained from the solution with the input
 *        transformation matrix. The result will be considered as valid only
 *        if two transformation matrices are the same.
 * @param T The transformation matrix in size 16.
 * @return `true` if the test passes, `false` otherwise.
 */
bool testMyInverseKinematics(const double* T)
{
  // Calculate the inverse kinematics using `tm_kinematics.h` methods
  TMKinematics::TMKinematicsPtr classPtr = std::make_shared<TMKinematics>(marslite::control::TM5_700);
  TMKinematics::TransformationMatrix T6;
  T6 << T[0], T[1], T[2], T[3],
        T[4], T[5], T[6], T[7],
        T[8], T[9], T[10], T[11],
        T[12], T[13], T[14], T[15];
  Eigen::MatrixXd Q_sols = classPtr->inverseKinematics(T6);

  TMKinematics::TransformationMatrix T_test;
  for (uint8_t i = 0; i < Q_sols.rows(); i++) {
    TMKinematics::JointVector Q_test;
    Q_test << Q_sols(i, 0), Q_sols(i, 1), Q_sols(i, 2),
              Q_sols(i, 3), Q_sols(i, 4), Q_sols(i, 5);
    T_test = classPtr->forwardKinematics(Q_test);
    if (!isSameTransformationMatrix(T6.data(), T_test.data())) {
      ROS_INFO("My inverse kinematics test failed! The solution %d is not valid.", i);
      printIKSolution(Q_sols, "My solution");
      return false;
    }
  }

  ROS_INFO("My inverse kinematics test passed!");
  printIKSolution(Q_sols, "My solution");
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
    ROS_ASSERT_CMD(testStandardInverseKinematics(T), throwAssertionFailedException());
    ROS_ASSERT_CMD(testMyInverseKinematics(T), throwAssertionFailedException());
  } catch (const AssertionFailedException& e) {
    throw e;        
  }
}

/**
 * @brief Add a test case for the kinematics function with the given joint
 *        angles in radians. The function will test the forward kinematics
 *        and inverse kinematics, and compare the results with the standard
 *        solutions (provided from `tm_kin.h`).
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
    ROS_ASSERT_CMD(testStandardInverseKinematics(T), throwAssertionFailedException());
    ROS_ASSERT_CMD(testMyInverseKinematics(T), throwAssertionFailedException());
  } catch (const AssertionFailedException& e) {
    throw e;
  }
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "test_kinematics");

  try {
    ROS_INFO("--------------------------------------------------");

    TMKinematics::TMKinematicsPtr kinematics_ptr = std::make_shared<TMKinematics>(marslite::control::TM5_700);

    TMKinematics::TransformationMatrix home = kinematics_ptr->forwardKinematics(marslite::pose::HOME.head(6));
    kinematics_ptr->printTransformationMatrix(home, "Home pose");
    kinematics_ptr->printIKSolutionInDegree(kinematics_ptr->inverseKinematics(home), "Home pose");
    
    home(2, 3) += 0.03;
    kinematics_ptr->printTransformationMatrix(home, "Home pose (moved forward by 3 cm)");
    kinematics_ptr->printIKSolutionInDegree(kinematics_ptr->inverseKinematics(home), "Home pose (moved forward by 3 cm)");

    home(2, 3) -= 0.06;
    kinematics_ptr->printTransformationMatrix(home, "Home pose (moved backward by 3 cm)");
    kinematics_ptr->printIKSolutionInDegree(kinematics_ptr->inverseKinematics(home), "Home pose (moved backward by 3 cm)");



    ROS_INFO("All tests passed!");

  } catch (const ConstructorInitializationFailedException& e) {
    ROS_ERROR("%s", e.what());
  } catch (const AssertionFailedException& e) {
    ROS_ERROR("%s", e.what());
  }

  ros::shutdown();
  return 0;
}