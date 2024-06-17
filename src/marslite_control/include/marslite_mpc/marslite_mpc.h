/**
 * @file marslite_mpc.h
 * @author Handian Yang
 * @copyright Released under the terms of the GPLv3.0 or later
 * @date 2024
 * 
 * @brief The header file for Model Predictive Control (MPC) function for marslite robots.
 * @note `marslite_mpc.h` is part of `marslite_simulation_ws`.
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

#ifndef MARSLITE_MPC_H_
#define MARSLITE_MPC_H_

#include <ros/ros.h>

#include <Eigen/Dense>              // eigen
#include "OsqpEigen/OsqpEigen.h"    // osqp-eigen


#include <trajectory_msgs/JointTrajectory.h>

#include "marslite_properties/Arithmetics.h"
#include "marslite_properties/Exceptions.h"
#include "marslite_mpc/Constants.h"
#include "marslite_mpc/Constraints.h"
#include "marslite_mpc/Poses.h"

/**
 * @namespace marslite operation namespace
*/
namespace marslite {

using exceptions::TimeOutException;
using exceptions::ConstructorInitializationFailedException;

/**
 * @namespace MPC namespace for marslite robots. Relationship: `marslite`::`mpc`
*/
namespace mpc {

class ModelPredictiveControl {
public:
    using MPCClassPtr = std::shared_ptr<ModelPredictiveControl>;

    using StateVector = Eigen::Matrix<double, MPC_STATE_SIZE, 1>;
    using InputVector = Eigen::Matrix<double, MPC_INPUT_SIZE, 1>;
    
    /**
     * @brief Constructor for the MPC controller.
     * 
     * This constructor initializes the MPC controller with the given `ros::NodeHandle`.
     *
     * @param nh The `ros::NodeHandle` to be used by the MPC controller.
     * @throw `marslite::exceptions::ConstructorInitializationFailedException` if the constructor fails to initialize.
     */
    explicit ModelPredictiveControl();

    /**
     * @brief Sets the initial pose for the Marslite MPC.
     *
     * This function sets the initial pose for the Marslite Model Predictive Control (MPC) algorithm.
     * The initial pose is used as the starting point for the MPC optimization process.
     *
     * @param initialPose The initial pose to set. Defaults to `marslite::poses::MARSLITE_POSE_INITIAL`.
     */
    void setInitialPose(const StateVector& initialPose = marslite::poses::MARSLITE_POSE_INITIAL);

    /**
     * @brief Sets the target pose for the MPC controller.
     *
     * This function sets the target pose for the MPC controller. The target pose is represented
     * by a matrix of size MPC_STATE_SIZE x 1, where MPC_STATE_SIZE is the size of the state vector.
     *
     * @param targetPose The target pose vector. Defaults to `marslite::poses::MARSLITE_POSE_INITIAL`.
     */
    void setTargetPose(const StateVector& targetPose = marslite::poses::MARSLITE_POSE_INITIAL);

    /**
     * @brief Initialize the QP solver.
     * @return True if the QP solver is successfully initialized.
     * @note (1) The function is activated by the constructor. Therefore, it is not necessary to call this function.
     * @note (2) DO NOT try to use QP solver if this function returns `FALSE`.
    */
    bool initializeQPSolver();

    /**
     * @brief Plan the trajectory of the robot using QP solver.
     * @return True if the trajectory can be executed, and it is executed successfully.
     * @note ALWAYS remember to initialize the QP solver by invoking `QPSolverInitialization()`
     *  function before solving the problem.
    */
    bool solveQP(std::vector<trajectory_msgs::JointTrajectoryPoint>& trajectoryWaypoints);

private:
    /* ************************************ *
     *            MPC parameters            *
     * ************************************ */
    Eigen::Matrix<double, MPC_STATE_SIZE, MPC_STATE_SIZE> A_;   // dynamic matrix
    Eigen::Matrix<double, MPC_STATE_SIZE, MPC_INPUT_SIZE> B_;   // control matrix
    StateVector xMax_, xMin_;      // state inequality constraints
    InputVector uMax_, uMin_;      // input inequality constraints
    InputVector aMax_, aMin_;      // input inequality constraints
    Eigen::DiagonalMatrix<double, MPC_STATE_SIZE> Q_;           // weight matrix for state vectors
    Eigen::DiagonalMatrix<double, MPC_INPUT_SIZE> R_;           // weight matrix for input vectors
    StateVector x0_;               // initial state space
    StateVector xRef_;             // reference state space
    
    /* ************************************ *
     *            QP parameters             *
     * ************************************ */
    Eigen::SparseMatrix<double> hessianMatrix_;     // hessian matrix (in size `SS*(W+1) + IS*W` by `SS*(W+1) + IS*W`)
    Eigen::SparseMatrix<double> constraintMatrix_;  // constraint matrix (in size `2*SS*(W+1) + 2*IS*W` by `SS*(W+1) + IS*W`)
    Eigen::VectorXd gradient_;      // gradient vector (in size `SS*(W+1) + IS*W` by `1`)
    Eigen::VectorXd lowerBound_;    // lower inequaltiy vector (in size `2*SS*(W+1) + 2*IS*W` by `1`) 
    Eigen::VectorXd upperBound_;    // upper inequaltiy vector (in size `2*SS*(W+1) + 2*IS*W` by `1`)

    OsqpEigen::Solver solver_;      // QP solver

private:

    /* ********************************************** *
     *                MPC/QP functions                *
     * ********************************************** */

    /**
     * @brief Set dynamic matrices `A_` and `B_` for MPC problems.
     * 
     * The size of the dynamic matrix `A_` is `MPC_STATE_SIZE` by `MPC_STATE_SIZE`, and
     *  the size of the control matrix `B_` is `MPC_STATE_SIZE` by `MPC_INPUT_SIZE`.
     * 
     * @note The function requires `A_` and `B_` to be `Eigen::Matrix` in `double` type.
    */
    void setDynamicsMatrices();

    /**
     * @brief Set state and input inequality constraints for MPC problems.
     * 
     * The size of the state inequality vectors `xMax_` and `xMin_` are `MPC_STATE_SIZE` by 1, and
     *  the size of the input inequality vectors `uMax_`, `uMin_`, `aMax_`, and `aMin_` are
     *  `MPC_INPUT_SIZE` by 1.
     * 
     * @note The function requires all inequality vectors to be `Eigen::Matrix` in `double` type.
    */
    void setInequalityConstraints();

    /**
     * @brief Set weight matrices `Q_` and `R_` for MPC problems.
     * 
     * The two weight matrices `Q_` and `R_` are symmetric positive definite matrices with size
     *  `MPC_STATE_SIZE` by `MPC_STATE_SIZE` and `MPC_INPUT_SIZE` by `MPC_INPUT_SIZE`, respectively.
     * 
     * @note The function requires `Q_` and `R_` to be `Eigen::Matrix` in `double` type.
    */
    void setWeightMatrices();

    /**
     * @brief Set hessian matrix from weight matrices `Q_` and `R_` for QP problems.
     * 
     * The size of the hessian matrix is `SS*(W+1) + IS*W` by `SS*(W+1) + IS*W`. The hessian matrix
     *  will typically be formed as `diag(Q, ..., Q, R, ..., R)`.
     * 
     * @note The function requires `Q_` and `R_` to be properly initialized and assigned values.
     *  Make sure to invoke the `setWeightMatrices()` function beforehand.
    */
    void castMPCToQPHessian();

    /**
     * @brief Set gradient vector from weight matrix `Q_` and reference state space `xRef_`
     *  for QP problems.
     * 
     * The size of the gradient vector is `SS*(W+1) + IS*W` by `1`. The gradient vector
     *  will typically be formed as `[-Q*xRef, ..., -Q*xRef]`.
     * 
     * @note The function requires `Q_` and `xRef_` to be properly initialized and assigned values.
     *  Make sure to invoke the `setWeightMatrices()` and `setReferenceStateSpace()` functions beforehand.
    */
    void castMPCToQPGradient();

    /**
     * @brief Set constraint matrix from dynamic matrices `A_` and `B_` for QP problems.
     * 
     * The size of the constraint matrix is `2*SS*(W+1) + 2*IS*W` by `SS*(W+1) + IS*W`. 
     * 
     * @note The function requires `A_` and `B_` to be properly initialized and assigned values.
     *  Make sure to invoke the `setDynamicsMatrices()` function beforehand.
    */
    void castMPCToQPConstraintMatrix();

    /**
     * @brief Set constraint vectors from state inequality constraints `xMax_` and `xMin_`, and from
     *  input inequality constraints `uMax_`, `uMin_`, `aMax_`, and `aMin_` for QP problems.
     * 
     * The size of the constraint vectors is `2*SS*(W+1) + 2*IS*W` by `1`. 
     * 
     * @note The function requires all inequality constraints to be properly initialized and
     *  assigned values. Make sure to invoke the `setInequalityConstraints()` function beforehand.
    */
    void castMPCToQPConstraintVectors();

    /**
     * @brief Updates the gradient.
     *
     * This function updates the gradient using the provided reference state vector.
     * If no reference state vector is provided, it uses a zero vector as the reference.
     *
     * @param xRef The reference state vector (default: zero vector).
     * @return True if the gradient was successfully updated, false otherwise.
     */
    bool updateGradient(const StateVector& xRef = StateVector::Zero());


    /**
     * @brief Updates the constraint vectors.
     *
     * This function updates the constraint vectors based on the given state vector.
     *
     * @param x The state vector (default: zero vector).
     * @
     * @return True if the constraint vectors were successfully updated, false otherwise.
     */
    bool updateConstraintVectors(const StateVector& x = StateVector::Zero());

    /* ********************************************** *
     *                 Debug functions                *
     * ********************************************** */

    /**
     * @brief Prints the given state vector.
     *
     * This function prints the given state vector to the console.
     *
     * @param stateVector The state vector to be printed.
     * @param name The name of the state vector (optional).
     */
    void printStateVector(const StateVector& stateVector, const std::string& name = "State Vector");
};

} // namespace mpc

} // namespace marslite

#endif  // #ifndef MARSLITE_MPC_H_