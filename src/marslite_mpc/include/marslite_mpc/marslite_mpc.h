/**
 * @file marslite_mpc.h
 * @author Handian Yang
 * @copyright Released under the terms of the GPLv3.0 or later
 * @date 2024
 * 
 * @brief The header file for Model Predictive Control (MPC) function for marslite robots.
 * @note `marslite_mpc.h` is part of `marslite_simulation_ws`.
 * 
 * `marslite_simulation_ws` is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published
 *      by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
 * 
 * `marslite_simulation_ws` is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of
 *      MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License along with `marslite_simulation_ws`. If not, see <https://www.gnu.org/licenses/>. 
*/

#ifndef MARSLITE_MPC_H_
#define MARSLITE_MPC_H_

#include <ros/ros.h>
#include <cmath>
#include <time.h>

#include <sensor_msgs/JointState.h>

#include <Eigen/Dense>              // eigen
#include "OsqpEigen/OsqpEigen.h"    // osqp-eigen

#include <actionlib/client/simple_action_client.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <trajectory_msgs/JointTrajectory.h>
using JointTrajectoryAction = actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>;

#include <experimental/optional>

#include "marslite_properties/Arithmetics.h"
#include "marslite_properties/Exceptions.h"
#include "marslite_mpc/Constants.h"
#include "marslite_mpc/Constraints.h"
#include "marslite_mpc/Poses.h"

/**
 * @namespace marslite operation namespace
*/
namespace marslite {

/**
 * @namespace MPC namespace for marslite robots. Relationship: `marslite`::`mpc`
*/
namespace mpc {

using StateVector = Eigen::Matrix<double, MPC_STATE_SIZE, 1>;
using InputVector = Eigen::Matrix<double, MPC_INPUT_SIZE, 1>;

class ModelPredictiveControl {
public:
    using MPCClassPtr = std::shared_ptr<ModelPredictiveControl>;
    
    explicit ModelPredictiveControl(const ros::NodeHandle& nh = ros::NodeHandle());

    /**
     * @brief Initialize the QP solver.
     * @return `TRUE` if the QP solver is successfully initialized.
     * @note DO NOT try to use QP solver if this function returns `FALSE`.
    */
    bool initializeQPSolver();

    /**
     * @brief Solve the given QP problem.
     * @return `TRUE` if the solution of the QP problem is found.
     * @note ALWAYS remember to initialize the QP solver by invoking `QPSolverInitialization()`
     *  function before solving the problem.
    */
    bool solveQP();

    /**
     * @brief Return value of the `stopNode_` member.
     * @return `FALSE` if everything is okay.
    */
    // bool getStopNodeFlag() const noexcept;

private:
    /* ************************************ *
     *              ROS-related             *
     * ************************************ */
    ros::NodeHandle nh_;
    ros::Rate loopRate_;
    ros::Duration signalTimeoutTimer_;      // timer that is activated when waiting some signals
    ros::Duration maxTimeout_;              // maximum timeout before blocking
    
    ros::Subscriber robotStateSubscriber_;

    /* ************************************ *
     *               Messages               *
     * ************************************ */
    sensor_msgs::JointState robotState_;    // joint states of the mobile robot

    /* ************************************ *
     *            MPC parameters            *
     * ************************************ */
    Eigen::Matrix<double, MPC_STATE_SIZE, MPC_STATE_SIZE> A_;   // dynamic matrix
    Eigen::Matrix<double, MPC_STATE_SIZE, MPC_INPUT_SIZE> B_;   // control matrix
    Eigen::Matrix<double, MPC_STATE_SIZE, 1> xMax_, xMin_;      // state inequality constraints
    Eigen::Matrix<double, MPC_INPUT_SIZE, 1> uMax_, uMin_;      // input inequality constraints
    Eigen::Matrix<double, MPC_INPUT_SIZE, 1> aMax_, aMin_;      // input inequality constraints
    Eigen::DiagonalMatrix<double, MPC_STATE_SIZE> Q_;           // weight matrix for state vectors
    Eigen::DiagonalMatrix<double, MPC_INPUT_SIZE> R_;           // weight matrix for input vectors
    Eigen::Matrix<double, MPC_STATE_SIZE, 1> x0_;               // initial state space
    Eigen::Matrix<double, MPC_STATE_SIZE, 1> xRef_;             // reference state space
    
    /* ************************************ *
     *            QP parameters             *
     * ************************************ */
    Eigen::SparseMatrix<double> hessianMatrix_;     // hessian matrix (in size `SS*(W+1) + IS*W` by `SS*(W+1) + IS*W`)
    Eigen::SparseMatrix<double> constraintMatrix_;  // constraint matrix (in size `2*SS*(W+1) + 2*IS*W` by `SS*(W+1) + IS*W`)
    Eigen::VectorXd gradient_;      // gradient vector (in size `SS*(W+1) + IS*W` by `1`)
    Eigen::VectorXd lowerBound_;    // lower inequaltiy vector (in size `2*SS*(W+1) + 2*IS*W` by `1`) 
    Eigen::VectorXd upperBound_;    // upper inequaltiy vector (in size `2*SS*(W+1) + 2*IS*W` by `1`)

    OsqpEigen::Solver solver_;      // QP solver

    /* ************************************ *
     *         Trajectory planning          *
     * ************************************ */
    std::shared_ptr<JointTrajectoryAction> actionClient_;   // `FollowJointTrajectoryAction` action client
    control_msgs::FollowJointTrajectoryGoal trajectoryGoal_;          // the goal to be sent by action clients
    trajectory_msgs::JointTrajectoryPoint trajectoryWaypoint_;

    /* ************************************ *
     *                 Flags                *
     * ************************************ */
    // bool stopNode_;



private:
    /* ********************************************** *
     *                 Initialization                 *
     * ********************************************** */

    /**
     * @brief Subscribe to the robot state.
     *
     * This function subscribes to the robot state topic and receives updates
     * whenever the robot state changes.
     *
     * @return True if the subscription was successful, false otherwise.
     */
    bool subscribeRobotState();

    /**
     * @brief Connect to the `FollowJointTrajectoryAction` Server.
     * 
     * This function establishes a connection with the FollowJointTrajectoryAction Server.
     * 
     * @return True if the connection is successful, false otherwise.
     */
    bool connectJointTrajectoryActionServer();


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
     * @brief Set initial state space `x0_` for MPC problems.
     * 
     * The size of the initial state spaces is `MPC_STATE_SIZE` by `1`.
     * 
     * @param x0 The initial state space to be assigned (in type `Eigen::Matrix<double, MPC_STATE_SIZE, 1>`)
     * 
     * @note The function requires `x0_` to be `Eigen::DiagonalMatrix` in `double` type.
    */
    void setInitialStateSpace(const StateVector& x0 = ZERO_POSE);

    /**
     * @brief Set reference state space `xRef_` for MPC problems.
     * 
     * The size of the reference state spaces is `MPC_STATE_SIZE` by `1`.
     * 
     * @param xRef The reference state space to be assigned (in type `Eigen::Matrix<double, MPC_STATE_SIZE, 1>`)
     * 
     * @note The function requires `xRef_` to be `Eigen::DiagonalMatrix` in `double` type.
    */
    void setReferenceStateSpace(const StateVector& xRef = ZERO_POSE);

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
     * @brief Update constraint vectors after each iteration.
     * 
     * @note The function requires constraint vectors `lowerBound_` and `upperBound_` to be
     *  properly initialized and assigned values. Make sure to invoke the
     *  `castMPCToQPConstraintVectors()` function beforehand.
    */
    bool updateConstraintVectors();

    /* ********************************************** *
     *                Callback functions              *
     * ********************************************** */

    /**
     * @brief Callback function for receiving robot state messages.
     * 
     * This function is called whenever a new robot state message is received.
     * It takes a `sensor_msgs::JointState::ConstPtr` as input, which contains
     * the joint state information of the robot.
     * 
     * @param msg The received robot state message.
     * 
     * @note This function is intended to be used as a callback for subscribing
     * to robot state messages. Make sure to properly initialize the subscriber
     * and set the appropriate topic to receive the messages.
     */
    void robotStateCallback(const sensor_msgs::JointState::ConstPtr& msg);
};

} // namespace mpc

} // namespace marslite

#endif  // #ifndef MARSLITE_MPC_H_