/**
 * file MPCExample.cpp
 * @author Giulio Romualdi
 * @copyright Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 * @date 2018
 */


// osqp-eigen
#include "OsqpEigen/OsqpEigen.h"

// eigen
#include <Eigen/Dense>

#include <iostream>
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64MultiArray.h>

#include <tm_kinematics/tm_kin.h>

#define M_PI 3.14159265358979323846

sensor_msgs::JointState current_joint_state;

void setDynamicsMatrices(Eigen::Matrix<double, 6, 6> &a, Eigen::Matrix<double, 6, 6> &b)
{
    a << 1., 0., 0., 0., 0., 0.,
         0., 1., 0., 0., 0., 0.,
         0., 0., 1., 0., 0., 0.,
         0., 0., 0., 1., 0., 0.,
         0., 0., 0., 0., 1., 0.,
         0., 0., 0., 0., 0., 1.;

    int t_in=1;

    b << t_in, 0., 0., 0., 0., 0.,
         0., t_in, 0., 0., 0., 0.,
         0., 0., t_in, 0., 0., 0.,
         0., 0., 0., t_in, 0., 0.,
         0., 0., 0., 0., t_in, 0.,
         0., 0., 0., 0., 0., t_in;
}


void setInequalityConstraints(Eigen::Matrix<double, 6, 1> &xMax, Eigen::Matrix<double, 6, 1> &xMin,
                              Eigen::Matrix<double, 6, 1> &uMax, Eigen::Matrix<double, 6, 1> &uMin)
{
    // input inequality constraints
    uMin << (-10.)/180*M_PI,
            (-10.)/180*M_PI,
            (-10.)/180*M_PI,
            (-10.)/180*M_PI,
            (-10.)/180*M_PI,
            (-10.)/180*M_PI;

    uMax << 10./180*M_PI,
            10./180*M_PI,
            10./180*M_PI,
            10./180*M_PI,
            10./180*M_PI,
            10./180*M_PI;


    // state inequality constraints
    xMin << (-270.)/180*M_PI,
            (-180.)/180*M_PI,
            (-155.)/180*M_PI,
            (-180.)/180*M_PI,
            (-180.)/180*M_PI,
            (-270.)/180*M_PI;

    xMax << 270./180*M_PI,
            180./180*M_PI,
            155./180*M_PI,
            180./180*M_PI,
            180./180*M_PI,
            270./180*M_PI;
}

void setWeightMatrices(Eigen::DiagonalMatrix<double, 6> &Q, Eigen::DiagonalMatrix<double, 6> &R)
{
    Q.diagonal() << 1., 1., 1., 1., 1., 1.;
    R.diagonal() << 0.005, 0.005, 0.005, 0.005, 0.005, 0.005;
}

void castMPCToQPHessian(const Eigen::DiagonalMatrix<double, 6> &Q, const Eigen::DiagonalMatrix<double, 6> &R, int mpcWindow,
                        Eigen::SparseMatrix<double> &hessianMatrix)
{

    hessianMatrix.resize(6*(mpcWindow+1) + 6 * mpcWindow, 6*(mpcWindow+1) + 6 * mpcWindow);

    //populate hessian matrix
    for(int i = 0; i<6*(mpcWindow+1) + 6 * mpcWindow; i++){
        if(i < 6*(mpcWindow+1)){
            int posQ=i%6;
            float value = Q.diagonal()[posQ];
            if(value != 0)
                hessianMatrix.insert(i,i) = value;
        }
        else{
            int posR=i%6;
            float value = R.diagonal()[posR];
            if(value != 0)
                hessianMatrix.insert(i,i) = value;
        }
    }
}

void castMPCToQPGradient(const Eigen::DiagonalMatrix<double, 6> &Q, const Eigen::Matrix<double, 6, 1> &x0, const Eigen::Matrix<double, 6, 1> &xRef, int mpcWindow,
                         Eigen::VectorXd &gradient)
{

    Eigen::Matrix<double,6,1> Qx_ref;

    // populate the gradient vector
    gradient = Eigen::VectorXd::Zero(6*(mpcWindow+1) +  6*mpcWindow, 1);
    for(int i = 0; i<6*(mpcWindow+1); i++){
        int posQ=i%6;

	if(posQ == 0)
	{
	    Qx_ref = Q * (-1) * (x0 + (xRef - x0) / mpcWindow * (i / 6));
	}

        float value = Qx_ref(posQ,0);
        gradient(i,0) = value;
    }
}

void castMPCToQPConstraintMatrix(const Eigen::Matrix<double, 6, 6> &dynamicMatrix, const Eigen::Matrix<double, 6, 6> &controlMatrix,
                                 int mpcWindow, Eigen::SparseMatrix<double> &constraintMatrix)
{
    constraintMatrix.resize(6*(mpcWindow+1)  + 6*(mpcWindow+1) + 6 * mpcWindow, 6*(mpcWindow+1) + 6 * mpcWindow);

    // populate linear constraint matrix
    for(int i = 0; i<6*(mpcWindow+1); i++){
        constraintMatrix.insert(i,i) = -1;
    }

    for(int i = 0; i < mpcWindow; i++)
        for(int j = 0; j<6; j++)
            for(int k = 0; k<6; k++){
                float value = dynamicMatrix(j,k);
                if(value != 0){
                    constraintMatrix.insert(6 * (i+1) + j, 6 * i + k) = value;
                }
            }

    for(int i = 0; i < mpcWindow; i++)
        for(int j = 0; j < 6; j++)
            for(int k = 0; k < 6; k++){
                float value = controlMatrix(j,k);
                if(value != 0){
                    constraintMatrix.insert(6*(i+1)+j, 6*i+k+6*(mpcWindow + 1)) = value;
                }
            }

    for(int i = 0; i<6*(mpcWindow+1) + 6*mpcWindow; i++){
        constraintMatrix.insert(i+(mpcWindow+1)*6,i) = 1;
    }
}

void castMPCToQPConstraintVectors(const Eigen::Matrix<double, 6, 1> &xMax, const Eigen::Matrix<double, 6, 1> &xMin,
                                   const Eigen::Matrix<double, 6, 1> &uMax, const Eigen::Matrix<double, 6, 1> &uMin,
                                   const Eigen::Matrix<double, 6, 1> &x0,
                                   int mpcWindow, Eigen::VectorXd &lowerBound, Eigen::VectorXd &upperBound)
{
    // evaluate the lower and the upper inequality vectors
    Eigen::VectorXd lowerInequality = Eigen::MatrixXd::Zero(6*(mpcWindow+1) +  6 * mpcWindow, 1);
    Eigen::VectorXd upperInequality = Eigen::MatrixXd::Zero(6*(mpcWindow+1) +  6 * mpcWindow, 1);
    for(int i=0; i<mpcWindow+1; i++){
        lowerInequality.block(6*i,0,6,1) = xMin;
        upperInequality.block(6*i,0,6,1) = xMax;
    }
    for(int i=0; i<mpcWindow; i++){
        lowerInequality.block(6 * i + 6 * (mpcWindow + 1), 0, 6, 1) = uMin;
        upperInequality.block(6 * i + 6 * (mpcWindow + 1), 0, 6, 1) = uMax;
    }

    // evaluate the lower and the upper equality vectors
    Eigen::VectorXd lowerEquality = Eigen::MatrixXd::Zero(6*(mpcWindow+1),1 );
    Eigen::VectorXd upperEquality;
    lowerEquality.block(0,0,6,1) = -x0;
    upperEquality = lowerEquality;
    lowerEquality = lowerEquality;

    // merge inequality and equality vectors
    lowerBound = Eigen::MatrixXd::Zero(2*6*(mpcWindow+1) +  6*mpcWindow,1 );
    lowerBound << lowerEquality,
        lowerInequality;

    upperBound = Eigen::MatrixXd::Zero(2*6*(mpcWindow+1) +  6*mpcWindow,1 );
    upperBound << upperEquality,
        upperInequality;
}


void updateConstraintVectors(const Eigen::Matrix<double, 6, 1> &x0,
                             Eigen::VectorXd &lowerBound, Eigen::VectorXd &upperBound)
{
    lowerBound.block(0,0,6,1) = -x0;
    upperBound.block(0,0,6,1) = -x0;
}


double getErrorNorm(const Eigen::Matrix<double, 6, 1> &x,
                    const Eigen::Matrix<double, 6, 1> &xRef)
{
    // evaluate the error
    Eigen::Matrix<double, 6, 1> error = x - xRef;

    // return the norm
    return error.norm();
}

void callback(const sensor_msgs::JointState& joint_states)
{
    for(int i=0; i<6; i++)
    {
	std::cout << "123" <<std::endl;
        current_joint_state.position[i] = joint_states.position[i];
	std::cout << current_joint_state.position[i] << std::endl;
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "mpc");
    ros::NodeHandle nh;

    ros::Publisher pub = nh.advertise<std_msgs::Float64MultiArray>("/velocity_cmd", 1);
    ros::Subscriber sub = nh.subscribe("/tm_joint_states", 1, callback);
    ros::Rate loop_rate(25);

    tf::TransformListener listener;
    tf::StampedTransform transform;

    // set the preview window
    int mpcWindow = 6;

    // allocate the dynamics matrices
    Eigen::Matrix<double, 6, 6> a;
    Eigen::Matrix<double, 6, 6> b;

    // allocate the constraints vector
    Eigen::Matrix<double, 6, 1> xMax;
    Eigen::Matrix<double, 6, 1> xMin;
    Eigen::Matrix<double, 6, 1> uMax;
    Eigen::Matrix<double, 6, 1> uMin;

    // allocate the weight matrices
    Eigen::DiagonalMatrix<double, 6> Q;
    Eigen::DiagonalMatrix<double, 6> R;

    // allocate the initial and the reference state space
    Eigen::Matrix<double, 6, 1> x0;
    Eigen::Matrix<double, 6, 1> xRef;

    // allocate QP problem matrices and vectores
    Eigen::SparseMatrix<double> hessian;
    Eigen::VectorXd gradient;
    Eigen::SparseMatrix<double> linearMatrix;
    Eigen::VectorXd lowerBound;
    Eigen::VectorXd upperBound;

    std_msgs::Float64MultiArray v_cmd;
    v_cmd.data.resize(6);

    current_joint_state.position.resize(6);

    ros::Duration(0.5).sleep();
    ros::spinOnce();

    // set the initial states
    x0 << current_joint_state.position[0], current_joint_state.position[1], current_joint_state.position[2], current_joint_state.position[3], current_joint_state.position[4], current_joint_state.position[5];

    // set the desired states
    try
    {
        listener.lookupTransform("/tm_base_link", "/tm_tip_link", ros::Time(0), transform);
    }
    catch (tf::TransformException &ex) 
    {
        ROS_ERROR("%s",ex.what());
        ros::Duration(1.0).sleep();
    }

    std::cout << transform.getOrigin().x() << std::endl << transform.getOrigin().y() << std::endl << transform.getOrigin().z() << std::endl;

    double T[16], ik_sols[48];

    T[0]=transform.getBasis()[0][0];
    T[1]=transform.getBasis()[0][1];
    T[2]=transform.getBasis()[0][2];
    T[3]=transform.getOrigin().x();
    T[4]=transform.getBasis()[1][0];
    T[5]=transform.getBasis()[1][1];
    T[6]=transform.getBasis()[1][2];
    T[7]=transform.getOrigin().y();
    T[8]=transform.getBasis()[2][0];
    T[9]=transform.getBasis()[2][1];
    T[10]=transform.getBasis()[2][2];
    //T[11]=transform.getOrigin().z();
    T[11]=0.5;
    T[12]=0;
    T[13]=0;
    T[14]=0;
    T[15]=1;

    std::cout << tm_kinematics::inverse(T, ik_sols, 0.0) << std::endl;
    for (const auto& e : ik_sols) {
        std::cout << e << std::endl;
    }

    if(tm_kinematics::inverse(T, ik_sols, 0.0)>0)
    {
        xRef <<  ik_sols[0], ik_sols[1], ik_sols[2], ik_sols[3], ik_sols[4], ik_sols[5];
    }
    else
    {
        return 0;
    }
    // set MPC problem quantities
    setDynamicsMatrices(a, b);
    setInequalityConstraints(xMax, xMin, uMax, uMin);
    setWeightMatrices(Q, R);

    // cast the MPC problem as QP problem
    castMPCToQPHessian(Q, R, mpcWindow, hessian);
    castMPCToQPGradient(Q, x0, xRef, mpcWindow, gradient);
    castMPCToQPConstraintMatrix(a, b, mpcWindow, linearMatrix);
    castMPCToQPConstraintVectors(xMax, xMin, uMax, uMin, x0, mpcWindow, lowerBound, upperBound);

    // instantiate the solver
    OsqpEigen::Solver solver;

    // settings
    //solver.settings()->setVerbosity(false);
    solver.settings()->setWarmStart(true);

    // set the initial data of the QP solver
    solver.data()->setNumberOfVariables(6 * (mpcWindow + 1) + 6 * mpcWindow);
    solver.data()->setNumberOfConstraints(2 * 6 * (mpcWindow + 1) +  6 * mpcWindow);
    if(!solver.data()->setHessianMatrix(hessian)) return 1;
    if(!solver.data()->setGradient(gradient)) return 1;
    if(!solver.data()->setLinearConstraintsMatrix(linearMatrix)) return 1;
    if(!solver.data()->setLowerBound(lowerBound)) return 1;
    if(!solver.data()->setUpperBound(upperBound)) return 1;

    // instantiate the solver
    if(!solver.initSolver()) return 1;    // controller input and QPSolution vector
    Eigen::VectorXd ctr;
    Eigen::VectorXd QPSolution;

    while(ros::ok())
    {
        // solve the QP problem
        if(!solver.solve()) return 1;

        // get the controller input
        QPSolution = solver.getSolution();
        ctr = QPSolution.block(6 * (mpcWindow + 1), 0, 6, 1);

	for(int i=0; i<6; i++)
	{
	    v_cmd.data[i] = ctr(i);
	}

	pub.publish(v_cmd);

        // save data into file
        auto x0Data = x0.data();

        // propagate the model
	ros::spinOnce();

	x0 << current_joint_state.position[0], current_joint_state.position[1], current_joint_state.position[2], current_joint_state.position[3], current_joint_state.position[4], current_joint_state.position[5];

	std::cout <<std::endl << ctr <<std::endl;
	std::cout << std::endl << x0 << std::endl;

	castMPCToQPGradient(Q, x0, xRef, mpcWindow, gradient);
	if(!solver.updateGradient(gradient)) return 1;

        // update the constraint bound
        updateConstraintVectors(x0, lowerBound, upperBound);
        if(!solver.updateBounds(lowerBound, upperBound)) return 1;

	loop_rate.sleep();
    }

    return 0;
}
