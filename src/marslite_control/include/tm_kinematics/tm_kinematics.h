/**
 * marslite_simulation_ws/marslite_control/include/tm_kinematics.h
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

#ifndef MARSLITE_CONTROL_TM_KINEAMTICS_H_
#define MARSLITE_CONTROL_TM_KINEAMTICS_H_

#include <Eigen/Dense>
#include <memory>
#include <cmath>
#include <ros/console.h>

#include "marslite_properties/Exception.h"
using marslite::exception::ConstructorInitializationFailedException;

#include "marslite_properties/Arithmetic.h"
using marslite::math::rad2Deg;

/**
 * @namespace marslite operation namespace
 */
namespace marslite {

/**
 * @namespace Control operations for marslite. Relationship: `marslite:control`
 */
namespace control {

/**
 * @enum RobotType
 * @brief The type of the TM5 robotic arm.
 */
enum RobotType {
  TM5_700,
  TM5_900,
  TM5_1100,
  TM5_1300
};

class TMKinematics {
public:
  using TMKinematicsPtr = std::shared_ptr<TMKinematics>;

  using TransformationMatrix = Eigen::Matrix<double, 4, 4>;
  using JointVector = Eigen::Matrix<double, 6, 1>;

  explicit TMKinematics(const RobotType& type = TM5_700);
    
  /**
   * @brief Calculate the forward kinematics of the TM5 robotic arm.
   * @param theta The joint angles (6-axis) of the TM5 robotic arm.
   * @return The transformation matrix of the end-effector in homogeneous
   *         coordinates (4x4).
   */
  inline TransformationMatrix forwardKinematics(const JointVector& theta) const
  {
    return A1(theta(0)) * A2(theta(1)) * A3(theta(2))
         * A4(theta(3)) * A5(theta(4)) * A6(theta(5));
  }

  /**
   * @brief Calculate the inverse kinematics of the TM5 robotic arm.
   * @param T The transformation matrix of the end-effector in homogeneous
   *          coordinates (4x4).
   * @return The joint angles (6-axis) of the TM5 robotic arm. The number
   *          of the solutions depends on the return value of each subroutine.
   * @note The number of the solutions is determined as:
   *      `solveQ1() * solveQ56P() * solveQ234()`, which is expected to be 8
   *      solutions (2 * 2 * 2) in normal cases. For special cases (such as
   *      singularities), please refer to the subroutines for more details.
   */
  Eigen::MatrixXd inverseKinematics(const TransformationMatrix& T) const;

  void printTransformationMatrix(const TransformationMatrix& T, const char* name = "") const;

  void printIKSolutionInDegree(const Eigen::MatrixXd& Q, const char* name = "") const;

private:
  RobotType type_;    
  double d1_;
  double a2_;
  double a3_;
  double d4_;
  double d5_;
  double d6_;

  const double kDistanceTolerance_ = 1e-5;

private:
  /* ***************************************** *
   *             Helper Functions              *
   * ***************************************** */

  /**
   * @brief Check if the value is close to the target.
   * @param value The value to be checked.
   * @param target The target value.
   * @return True if the value is close to the target, false otherwise.
   */
  inline const bool distanceCloseToTarget(const double& value, const double& target = 0) const {
    return std::abs(value - target) < kDistanceTolerance_;
  }

  /**
   * @brief Restrict the angle within the range of [-PI, PI].
   * @param theta The angle to be restricted.
   * @return The angle within the range of [-PI, PI].
   */
  inline double restrictAngleWithinPI(const double& theta) const {
    return std::fmod(theta + M_PI, 2 * M_PI) - M_PI;
  }
    
  /* ***************************************** *
   *             Forward Kinematics            *
   * ***************************************** */

  /**
   * @brief Get the transformation matrix of the i-th joint.
   * @param theta The angle of the rotation around the Z_(i-1) axis.
   * @param alpha The angle of the rotation around the X_i axis.
   * @param a The length of the translation along the X_i axis.
   * @param d The length of the translation between the i-th and (i-1)-th
   *          links along the Z_(i-1) axis.
   * @param offset The offset of `theta`
   * @return The transformation matrix of the i-th joint in homogeneous
   *         coordinates (4x4)
   */
  inline TransformationMatrix getTransformationMatrix(
      const double& theta,
      const double& alpha,
      const double& a,
      const double& d,
      const double& offset = 0
  ) const {
    TransformationMatrix T;
    T << cos(theta+offset), -sin(theta+offset)*cos(alpha),  sin(theta+offset)*sin(alpha), a*cos(theta+offset),
         sin(theta+offset),  cos(theta+offset)*cos(alpha), -cos(theta+offset)*sin(alpha), a*sin(theta+offset),
         0, sin(alpha), cos(alpha), d,
         0, 0, 0, 1;
    return T;
  }

  /**
   * @brief Get the inverse of the transformation matrix. Typically, the 
   *      rotation matrix of the inverse of the transformation matrix is
   *      the transpose of the original rotation matrix, and the translation
   *      vector is the inner product of the original vector and the normal, 
   *      orientation, and the approach vectors, respectively. That is,
   * 
   *             / nx ny nz -p*n \ 
   *      Tinv = | ox oy oz -p*o |
   *             | ax ay az -p*a |
   *             \  0  0  0    1 /
   *      
   *      where "*" denotes the inner product.
   * @param T The transformation matrix to be inverted.
   * @return The inverse of the transformation matrix in homogeneous
   *          coordinates (4x4).
   */
  inline TransformationMatrix getTransformationMatrixInverse(
        const TransformationMatrix& T
  ) const {
      TransformationMatrix Tinv;
      Tinv << T(0,0), T(1,0), T(2,0), -T(0,0)*T(0,3) - T(1,0)*T(1,3) - T(2,0)*T(2,3),
              T(0,1), T(1,1), T(2,1), -T(0,1)*T(0,3) - T(1,1)*T(1,3) - T(2,1)*T(2,3),
              T(0,2), T(1,2), T(2,2), -T(0,2)*T(0,3) - T(1,2)*T(1,3) - T(2,2)*T(2,3),
              0, 0, 0, 1;
      return Tinv;
  }

  inline TransformationMatrix A1(const double& theta) const {
    return getTransformationMatrix(theta, -M_PI_2, 0, d1_);
  }

  inline TransformationMatrix A2(const double& theta) const {
    return getTransformationMatrix(theta, 0, a2_, 0, -M_PI_2);
  }

  inline TransformationMatrix A3(const double& theta) const {
    return getTransformationMatrix(theta, 0, a3_, 0);
  }

  inline TransformationMatrix A4(const double& theta) const {
    return getTransformationMatrix(theta, M_PI_2, 0, d4_, M_PI_2);
  }

  inline TransformationMatrix A5(const double& theta) const {
    return getTransformationMatrix(theta, M_PI_2, 0, d5_);
  }

  inline TransformationMatrix A6(const double& theta) const {
    return getTransformationMatrix(theta, 0, 0, d6_);
  }

  /* ***************************************** *
   *             Inverse Kinematics            *
   * ***************************************** */

  /**
   * @brief The subroutine of `inverseKinematics` for solving the 1st joint.
   * @param T The transformation matrix of the end-effector.
   * @return The solutions of the first joint. The number of the solutions
   *          depends on the value of `r`, which was defined in the function.
   * @note The number of the solutions is determined as follows:
   * @note - [`r >= -d4`] 2 solutions
   * @note - [`r < -d4`]  0 solutions
   */
  Eigen::MatrixXd solveQ1(const TransformationMatrix& T) const;

  /**
   * @brief The subroutine of `inverseKinematics` for solving the 5th, 6th, and
   *        the position joint (QP = Q2 + Q3 + Q4).
   * @param T The transformation matrix of the end-effector.
   * @param Q1 The solution of the 1st joint.
   * @param Q6ref The reference value of the 6th joint. Default is 0.
   * @return The solutions of the 5th, 6th, and the position joint. The number
   *          of the solutions depends on the value of `sin(Q5)`.
   * @note The number of the solutions is determined as follows:
   * @note - [`sin(Q5) > 0`] 2 solution sets for {Q5, Q6, QP}
   * @note - [`sin(Q5) = 0`] 1 solution set for {Q5, Q6, QP}
   */
  Eigen::MatrixXd solveQ56P(const TransformationMatrix& T,
                            const double& Q1,
                            const double& Q6ref = 0) const;
    
  /**
   * @brief The subroutine of `inverseKinematics` for solving the 2nd, 3rd, and
   *       the 4th joint.
   * @param T The transformation matrix of the end-effector.
   * @param Q1 The solution of the 1st joint.
   * @param QP The solution of the position joint (QP = Q2 + Q3 + Q4).
   * @return The solutions of the 2nd, 3rd, and the 4th joint. The number of
   *         the solutions depends on the value of `s` and `t`, which were
   *         defined in the function.
   * @note The number of the solutions is determined as follows:
   * @note - [`s > 0` and `t > 0`] 2 solution sets for {Q2, Q3, Q4}
   * @note - [`s = 0` or `t = 0`]  1 solution set for {Q2, Q3, Q4}
   * @note - [`s < 0` or `t < 0`]  0 solution sets for {Q2, Q3, Q4}
   */
  Eigen::MatrixXd solveQ234(const TransformationMatrix& T,
                            const double& Q1,
                            const double& QP) const;
};

} // namespace control

} // namespace marslite


#endif  // #ifndef MARSLITE_CONTROL_TM_KINEAMTICS_H_