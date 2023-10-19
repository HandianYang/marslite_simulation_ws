/**
 * @file MPCExample.cpp
 * @author Giulio Romualdi
 * @copyright Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 * @date 2018
 */

#include <mpc/mpc.h>
#include <tm_kinematics/tm_kin.h>
//#include <librealsense2/rsutil.h> 

#include <std_msgs/Bool.h>
#include <geometry_msgs/Twist.h>

#include <iostream>
#include <cmath>

ModelPredictiveControl::ModelPredictiveControl(ros::NodeHandle& nh) : loop_rate(25)
{
    distance_warning_field = 2;
    distance_protective_field = 1.5;
    dec_factor = 1;
    dec_factor_previous = 1;
    //shift_factor = 1;
    //shift_factor_previous = 1;

    gripper_pub = nh.advertise<std_msgs::Bool>("/gripper/cmd_gripper", 1);
    joint_velocity_pub = nh.advertise<std_msgs::Float64MultiArray>("/velocity_cmd", 1);
    mobile_platform_velocity_pub = nh.advertise<geometry_msgs::Twist>("/mob_plat/cmd_vel", 1);
    apriltag_detection_cmd_pub = nh.advertise<std_msgs::Bool>("/apriltag_detection_enable", 1);
    des_ee_tra_pub = nh.advertise<nav_msgs::Path>("/des_ee_tra", 1);
    ee_tra_pub = nh.advertise<nav_msgs::Path>("/ee_tra", 1);
    des_ee_state_pub = nh.advertise<std_msgs::Float64MultiArray>("/des_ee_state", 1);
    ee_state_pub = nh.advertise<std_msgs::Float64MultiArray>("/ee_state", 1);
    robot_state_pub = nh.advertise<std_msgs::Float64MultiArray>("/robot_state", 1);
    robot_vel_pub = nh.advertise<std_msgs::Float64MultiArray>("/robot_vel", 1);
    joint_state_sub = nh.subscribe("/tm_joint_states", 1, &ModelPredictiveControl::joint_state_callback, this);
    mobile_platform_velocity_sub = nh.subscribe("/mob_plat/curr_vel", 1, &ModelPredictiveControl::mobile_platform_velocity_callback, this);
    apriltag_detection_sub = nh.subscribe("/tag_detections", 1, &ModelPredictiveControl::apriltag_detection_callback, this);   
    obstacles_detection_sub = nh.subscribe("/obs_det_output", 1, &ModelPredictiveControl::obstacles_detection_callback, this);

    current_joint_state.position.resize(6);
    current_joint_state.velocity.resize(6);

    apriltag_detected = false;
    obstacles_detection_enable = true;

    ros::Duration(0.5).sleep();

    callback_order = 0;
    ros::spinOnce();

    //realsense_intrinsic_matrix_sub.shutdown();

    while(1)
    {
        try
        {
            listener.lookupTransform("/odom", "/base_footprint", ros::Time(0), initial_mobile_platform_pose);
            break;
        }
        catch (tf::TransformException &ex)
        {
            ROS_ERROR("%s",ex.what());
            ros::Duration(0.05).sleep();
        }
    }

    time_step = 0.04;
    mpcWindow = 6;

    //set the weight matrices
    Q.diagonal() << 1., 1., 1., 1., 1., 1., 1.; 
    R.diagonal() << 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01;

    //set the dynamics matrices
    A << 1., 0., 0., 0., 0., 0., 0., 
         0., 1., 0., 0., 0., 0., 0., 
         0., 0., 1., 0., 0., 0., 0., 
         0., 0., 0., 1., 0., 0., 0., 
         0., 0., 0., 0., 1., 0., 0., 
         0., 0., 0., 0., 0., 1., 0., 
         0., 0., 0., 0., 0., 0., 1.; 
    
    B << time_step, 0., 0., 0., 0., 0., 0., 
         0., time_step, 0., 0., 0., 0., 0., 
         0., 0., time_step, 0., 0., 0., 0., 
         0., 0., 0., time_step, 0., 0., 0., 
         0., 0., 0., 0., time_step, 0., 0., 
         0., 0., 0., 0., 0., time_step, 0., 
         0., 0., 0., 0., 0., 0., time_step;

    //set the state inequality constraints
    xMax << 270./180*M_PI,
            180./180*M_PI,
            155./180*M_PI,
            180./180*M_PI,
            180./180*M_PI,
            270./180*M_PI,
            10;

    xMin << (-270.)/180*M_PI,
            (-180.)/180*M_PI,
            (-155.)/180*M_PI,
            (-180.)/180*M_PI,
            (-180.)/180*M_PI,
            (-270.)/180*M_PI,
            (-10);

    //set the input inequality constraints
    uMax << 180./180*M_PI,
            180./180*M_PI,
            180./180*M_PI,
            225./180*M_PI,
            225./180*M_PI,
            225./180*M_PI,
            0.5;

    uMin << (-180.)/180*M_PI,
            (-180.)/180*M_PI,
            (-180.)/180*M_PI,
            (-225.)/180*M_PI,
            (-225.)/180*M_PI,
            (-225.)/180*M_PI,
            (-0.5);

    aMax << 80./180*M_PI,
            80./180*M_PI,
            80./180*M_PI,
            80./180*M_PI,
            80./180*M_PI,
            80./180*M_PI,
            2;

    aMin << (-80.)/180*M_PI,
            (-80.)/180*M_PI,
            (-80.)/180*M_PI,
            (-80.)/180*M_PI,
            (-80.)/180*M_PI,
            (-80.)/180*M_PI,
            -2;
    //120 90

    x0 << 0, 0, 0, 0, 0, 0, 0;

    while(1)
    {
        try
        {
            listener.lookupTransform("/tm_base_link", "/tm_tool0", ros::Time(0), pose0);
            break;
        }
        catch (tf::TransformException &ex)
        {
            ROS_ERROR("%s",ex.what());
            ros::Duration(0.005).sleep();
        }
    }

    posef = pose0;

    trajectory_total_t = 1;
    trajectory_start_t = ros::Time::now().toSec();
    trajectory_exec_t = 0;

    trajectory_total_t_mob_plat = 1;
    trajectory_start_t_mob_plat = ros::Time::now().toSec();
    trajectory_exec_t_mob_plat = 0;
    trajectory_ini_v_mob_plat = 0;

    on_spot = false;

    starting_mobile_platform_position = 0;
    desired_mobile_platform_position = 0;

    desired_pose_is_global = false;

    castMPCToQPHessian();
    castMPCToQPGradient();
    castMPCToQPConstraintMatrix();
    castMPCToQPConstraintVectors();

    // settings
    //solver.settings()->setVerbosity(false);
    solver.settings()->setWarmStart(true);
    solver.settings()->setRelativeTolerance(0.000001);
    //solver.settings()->setPrimalInfeasibilityTollerance(0.001);

    // set the initial data of the QP solver
    solver.data()->setNumberOfVariables(7*(mpcWindow+1)+7*mpcWindow);
    solver.data()->setNumberOfConstraints(2*7*(mpcWindow+1)+2*7*mpcWindow);
    solver.data()->setHessianMatrix(hessianMatrix);
    solver.data()->setGradient(gradient);
    solver.data()->setLinearConstraintsMatrix(constraintMatrix);
    solver.data()->setLowerBound(lowerBound);
    solver.data()->setUpperBound(upperBound);

    // instantiate the solver
    solver.initSolver();
}

ModelPredictiveControl::~ModelPredictiveControl()
{
    stop();

    std::cout << "destructor is called" << std::endl;
}

void ModelPredictiveControl::castMPCToQPHessian()
{
    //input:Q, R, mpcWindow
    hessianMatrix.resize(7*(mpcWindow+1)+7*mpcWindow, 7*(mpcWindow+1)+7*mpcWindow);

    //populate hessian matrix
    for(int i = 0; i < 7*(mpcWindow+1)+7*mpcWindow; i++)
    {
        if(i < 7*(mpcWindow+1))
	    {
            int posQ = i%7;
            float value = Q.diagonal()[posQ];
            if(value != 0)
                hessianMatrix.insert(i,i) = value;
        }
        else
	    {
            int posR = i%7;
            float value = R.diagonal()[posR];
            if(value != 0)
                hessianMatrix.insert(i,i) = value;
        }
    }
}

int ModelPredictiveControl::castMPCToQPGradient()
{
    // a trajectory is planned here to be the reference trajectory.
    //input:Q, pose0, posef, mpcWindow, trajectory_total_t, trajectory_start_t;
    Eigen::Matrix<double, 7, 1> xRef, Qx_ref, last_xRef;
    tf::StampedTransform poseRef;
    double ik_sols[48];

    trajectory_exec_t += (ros::Time::now().toSec()-trajectory_start_t)*dec_factor_previous/**shift_factor_previous*/;
    trajectory_start_t = ros::Time::now().toSec();

    trajectory_exec_t_mob_plat += (ros::Time::now().toSec()-trajectory_start_t_mob_plat)*dec_factor_previous;
    trajectory_start_t_mob_plat = ros::Time::now().toSec();

    //when desired_pose_is_global is true, these variables are used;
    tf::StampedTransform arm_base_pose, current_mobile_platform_pose, desired_mobile_platform_pose;
    tf::Vector3 x_axis_vec;
    tf::StampedTransform ref_mobile_platform_pose;
    std_msgs::Float64MultiArray des_ee_state;

    des_ee_state.data.resize(6);

    if(desired_pose_is_global && trajectory_exec_t > trajectory_total_t)
    {
	    on_spot = true;
    }

    if(desired_pose_is_global)
    {
        while(1)
        {
            try
            {
                listener.lookupTransform("/odom", "/base_footprint", ros::Time(0), current_mobile_platform_pose);
                listener.lookupTransform("/base_footprint", "/tm_base_link", ros::Time(0), arm_base_pose);
                break;
            }
            catch (tf::TransformException &ex)
            {
                ROS_ERROR("%s",ex.what());
                ros::Duration(0.05).sleep();
            }
        }

	    x_axis_vec = current_mobile_platform_pose.getBasis().getColumn(0);

        desired_mobile_platform_pose.setBasis(current_mobile_platform_pose.getBasis());
        desired_mobile_platform_pose.setOrigin(current_mobile_platform_pose.getOrigin()+(desired_mobile_platform_position-x0(6, 0))*x_axis_vec);

        if(inverseKinematic(arm_base_pose.inverse()*desired_mobile_platform_pose.inverse()*posef, ik_sols) == (-1))
        {
            return -1;
        }
    }
    else
    {
	    if(inverseKinematic(posef, ik_sols) == (-1))
        {   
            return -1;
        }
    }

    xf << ik_sols[0], ik_sols[1], ik_sols[2], ik_sols[3], ik_sols[4], ik_sols[5], desired_mobile_platform_position;

    double ref_vel;
    tf::StampedTransform last_ref_mobile_platform_pose = current_mobile_platform_pose;
    double last_xRef_6 = x0[6];
    double stop_dec_ratio = 1./16;
    geometry_msgs::PoseStamped tra_pos;

    // populate the gradient vector
    gradient = Eigen::VectorXd::Zero(7*(mpcWindow+1)+7*mpcWindow, 1);
    for(int i = 0; i < 7*(mpcWindow+1); i++)
    {
        int posQ = i%7;

	    if(posQ == 0)
	    {
            if(trajectory_exec_t+(i/7)*time_step*dec_factor < trajectory_total_t)
            {
		        poseRef.setData(tf::Transform(pose0.getRotation().slerp(posef.getRotation(), trajectoryPlanning(0, 0, (trajectory_exec_t+(i/7)*time_step*dec_factor)/trajectory_total_t)), pose0.getOrigin().lerp(posef.getOrigin(), trajectoryPlanning(0, 0, (trajectory_exec_t+(i/7)*time_step*dec_factor)/trajectory_total_t))));
                
                if(desired_pose_is_global)
		        {
                    if(trajectory_exec_t_mob_plat+(i/7)*time_step*dec_factor < trajectory_total_t_mob_plat || on_spot)
                    {
			            ref_mobile_platform_pose.setBasis(current_mobile_platform_pose.getBasis());

                        if(on_spot)
			            {
                            if(current_mobile_platform_velocity.linear.x >= 0)
				                ref_vel = (current_mobile_platform_velocity.linear.x+(i/7)*time_step*aMin[6]*stop_dec_ratio > 0) ? (current_mobile_platform_velocity.linear.x+(i/7)*time_step*aMin[6]*stop_dec_ratio) : 0;
                            else
				                ref_vel = (current_mobile_platform_velocity.linear.x+(i/7)*time_step*aMax[6]*stop_dec_ratio < 0) ? (current_mobile_platform_velocity.linear.x+(i/7)*time_step*aMax[6]*stop_dec_ratio) : 0;

                            if(i/7 == 0)
			                    ref_mobile_platform_pose = last_ref_mobile_platform_pose;
			                else
                                ref_mobile_platform_pose.setOrigin(last_ref_mobile_platform_pose.getOrigin()+ref_vel*time_step*x_axis_vec);

                            last_ref_mobile_platform_pose = ref_mobile_platform_pose;
			            }
                        else
                            ref_mobile_platform_pose.setOrigin(current_mobile_platform_pose.getOrigin()+(starting_mobile_platform_position+(desired_mobile_platform_position-starting_mobile_platform_position)*trajectoryPlanning(trajectory_ini_v_mob_plat/(0.5/7.5)*copysign(1.0, (desired_mobile_platform_position-starting_mobile_platform_position)), (trajectory_exec_t_mob_plat+(i/7)*time_step*dec_factor)/trajectory_total_t_mob_plat)-x0(6, 0))*x_axis_vec);

			            if(inverseKinematic(arm_base_pose.inverse()*ref_mobile_platform_pose.inverse()*poseRef, ik_sols) == (-1))
		                {
	   	                    return -1;
		                }

                        if(on_spot)
			            {
                            if(i/7 == 0)
                                xRef << ik_sols[0], ik_sols[1], ik_sols[2], ik_sols[3], ik_sols[4], ik_sols[5], last_xRef_6;
                            else
			                    xRef << ik_sols[0], ik_sols[1], ik_sols[2], ik_sols[3], ik_sols[4], ik_sols[5], last_xRef_6+ref_vel*time_step;

                            last_xRef_6 = xRef[6];
			            }
                        else
                            xRef << ik_sols[0], ik_sols[1], ik_sols[2], ik_sols[3], ik_sols[4], ik_sols[5], starting_mobile_platform_position+(desired_mobile_platform_position-starting_mobile_platform_position)*trajectoryPlanning(trajectory_ini_v_mob_plat/(0.5/7.5)*copysign(1.0, (desired_mobile_platform_position-starting_mobile_platform_position)), (trajectory_exec_t_mob_plat+(i/7)*time_step*dec_factor)/trajectory_total_t_mob_plat);
                    }
                    else
                    {
			            if(inverseKinematic(arm_base_pose.inverse()*desired_mobile_platform_pose.inverse()*poseRef, ik_sols) == (-1))
                        {
                             return -1;
                        }

                        xRef << ik_sols[0], ik_sols[1], ik_sols[2], ik_sols[3], ik_sols[4], ik_sols[5], desired_mobile_platform_position;
                    }
		        }
                else
                {
                    if(inverseKinematic(poseRef, ik_sols) == (-1))
                    {   
                        return -1;
                    }

                    if(trajectory_exec_t_mob_plat+(i/7)*time_step*dec_factor < trajectory_total_t_mob_plat || on_spot)
                    {
                        if(current_mobile_platform_velocity.linear.x >= 0)
                            ref_vel = (current_mobile_platform_velocity.linear.x+(i/7)*time_step*aMin[6]*stop_dec_ratio > 0) ? (current_mobile_platform_velocity.linear.x+(i/7)*time_step*aMin[6]*stop_dec_ratio) : 0;
                        else
                            ref_vel = (current_mobile_platform_velocity.linear.x+(i/7)*time_step*aMax[6]*stop_dec_ratio < 0) ? (current_mobile_platform_velocity.linear.x+(i/7)*time_step*aMax[6]*stop_dec_ratio) : 0;

                        if(on_spot)
                        {
                            if(i/7 == 0)
                                xRef << ik_sols[0], ik_sols[1], ik_sols[2], ik_sols[3], ik_sols[4], ik_sols[5], last_xRef_6;
                            else
                                xRef << ik_sols[0], ik_sols[1], ik_sols[2], ik_sols[3], ik_sols[4], ik_sols[5], last_xRef_6+ref_vel*time_step;

                            last_xRef_6 = xRef[6];
                        }
                        else
                            xRef << ik_sols[0], ik_sols[1], ik_sols[2], ik_sols[3], ik_sols[4], ik_sols[5], starting_mobile_platform_position+(desired_mobile_platform_position-starting_mobile_platform_position)*trajectoryPlanning(trajectory_ini_v_mob_plat/(0.5/7.5)*copysign(1.0, (desired_mobile_platform_position-starting_mobile_platform_position)), (trajectory_exec_t_mob_plat+(i/7)*time_step*dec_factor)/trajectory_total_t_mob_plat);
                    }
                    else
                    {
                        xRef << ik_sols[0], ik_sols[1], ik_sols[2], ik_sols[3], ik_sols[4], ik_sols[5], desired_mobile_platform_position;
                    }
                }

                if(i == 0)
                {
                    tra_pos.pose.position.x = poseRef.getOrigin().getX();
                    tra_pos.pose.position.y = poseRef.getOrigin().getY();
                    tra_pos.pose.position.z = poseRef.getOrigin().getZ();
                    tra_pos.pose.orientation.x = poseRef.getRotation().getX();
                    tra_pos.pose.orientation.y = poseRef.getRotation().getY();
                    tra_pos.pose.orientation.z = poseRef.getRotation().getZ();
                    tra_pos.pose.orientation.w = poseRef.getRotation().getW();

                    des_ee_tra.poses.push_back(tra_pos);
                    des_ee_tra_pub.publish(des_ee_tra);
            
                    des_ee_state.data[0] = poseRef.getOrigin().getX();
                    des_ee_state.data[1] = poseRef.getOrigin().getY();
                    des_ee_state.data[2] = poseRef.getOrigin().getZ();
                    poseRef.getBasis().getRPY(des_ee_state.data[3], des_ee_state.data[4], des_ee_state.data[5]);
        
                    des_ee_state_pub.publish(des_ee_state);
                }
	        }
            else
            {
                if(desired_pose_is_global)
                {
                    if(trajectory_exec_t_mob_plat+(i/7)*time_step*dec_factor < trajectory_total_t_mob_plat || on_spot)
                    {
                        ref_mobile_platform_pose.setBasis(current_mobile_platform_pose.getBasis());
                            
                        if(on_spot)
                        {
                            if(current_mobile_platform_velocity.linear.x >= 0)
                                ref_vel = (current_mobile_platform_velocity.linear.x+(i/7)*time_step*aMin[6]*stop_dec_ratio > 0) ? (current_mobile_platform_velocity.linear.x+(i/7)*time_step*aMin[6]*stop_dec_ratio) : 0;
                            else
                                ref_vel = (current_mobile_platform_velocity.linear.x+(i/7)*time_step*aMax[6]*stop_dec_ratio < 0) ? (current_mobile_platform_velocity.linear.x+(i/7)*time_step*aMax[6]*stop_dec_ratio) : 0;

                            if(i/7 == 0)
                                ref_mobile_platform_pose = last_ref_mobile_platform_pose;
                            else
                                ref_mobile_platform_pose.setOrigin(last_ref_mobile_platform_pose.getOrigin()+ref_vel*time_step*x_axis_vec);

                            last_ref_mobile_platform_pose = ref_mobile_platform_pose;
                        }
                        else
                            ref_mobile_platform_pose.setOrigin(current_mobile_platform_pose.getOrigin()+(starting_mobile_platform_position+(desired_mobile_platform_position-starting_mobile_platform_position)*trajectoryPlanning(trajectory_ini_v_mob_plat/(0.5/7.5)*copysign(1.0, (desired_mobile_platform_position-starting_mobile_platform_position)), (trajectory_exec_t_mob_plat+(i/7)*time_step*dec_factor)/trajectory_total_t_mob_plat)-x0(6, 0))*x_axis_vec);

                        if(inverseKinematic(arm_base_pose.inverse()*ref_mobile_platform_pose.inverse()*posef, ik_sols) == (-1))
                        {
                                return -1;
                        }

                        if(on_spot)
                        {
                            if(i/7 == 0)
                                xRef << ik_sols[0], ik_sols[1], ik_sols[2], ik_sols[3], ik_sols[4], ik_sols[5], last_xRef_6;
                            else
                                xRef << ik_sols[0], ik_sols[1], ik_sols[2], ik_sols[3], ik_sols[4], ik_sols[5], last_xRef_6+ref_vel*time_step;

                            last_xRef_6 = xRef[6];
                        }
                            else
                                xRef << ik_sols[0], ik_sols[1], ik_sols[2], ik_sols[3], ik_sols[4], ik_sols[5], starting_mobile_platform_position+(desired_mobile_platform_position-starting_mobile_platform_position)*trajectoryPlanning(trajectory_ini_v_mob_plat/(0.5/7.5)*copysign(1.0, (desired_mobile_platform_position-starting_mobile_platform_position)), (trajectory_exec_t_mob_plat+(i/7)*time_step*dec_factor)/trajectory_total_t_mob_plat);
                    }
                    else
                    {
                        xRef << xf;
                    }
                }
                else
                {
                    xRef = xf;

                    if(current_mobile_platform_velocity.linear.x >= 0)
                    ref_vel = (current_mobile_platform_velocity.linear.x+(i/7)*time_step*aMin[6]*stop_dec_ratio > 0) ? (current_mobile_platform_velocity.linear.x+(i/7)*time_step*aMin[6]*stop_dec_ratio) : 0;
                    else
                        ref_vel = (current_mobile_platform_velocity.linear.x+(i/7)*time_step*aMax[6]*stop_dec_ratio < 0) ? (current_mobile_platform_velocity.linear.x+(i/7)*time_step*aMax[6]*stop_dec_ratio) : 0;

                    if(trajectory_exec_t_mob_plat+(i/7)*time_step*dec_factor < trajectory_total_t_mob_plat || on_spot)
                    {
                        if(on_spot)
                        {
                            if(i/7 == 0)
                                xRef << ik_sols[0], ik_sols[1], ik_sols[2], ik_sols[3], ik_sols[4], ik_sols[5], last_xRef_6;
                            else
                                xRef << ik_sols[0], ik_sols[1], ik_sols[2], ik_sols[3], ik_sols[4], ik_sols[5], last_xRef_6+ref_vel*time_step;

                            last_xRef_6 = xRef[6];
                        }
                        else
                            xRef(6, 0) = starting_mobile_platform_position+(desired_mobile_platform_position-starting_mobile_platform_position)*trajectoryPlanning(trajectory_ini_v_mob_plat/(0.5/7.5)*copysign(1.0, (desired_mobile_platform_position-starting_mobile_platform_position)), (trajectory_exec_t_mob_plat+(i/7)*time_step*dec_factor)/trajectory_total_t_mob_plat);
                    }
                }
            
                if(i == 0)
                {
                    des_ee_state.data[0] = posef.getOrigin().getX();
                    des_ee_state.data[1] = posef.getOrigin().getY();
                    des_ee_state.data[2] = posef.getOrigin().getZ();
                    posef.getBasis().getRPY(des_ee_state.data[3], des_ee_state.data[4], des_ee_state.data[5]);
        
                    des_ee_state_pub.publish(des_ee_state);

                    if(trajectory_exec_t < (trajectory_total_t+time_step))
                    {		
                        tra_pos.pose.position.x = posef.getOrigin().getX();
                        tra_pos.pose.position.y = posef.getOrigin().getY();
                        tra_pos.pose.position.z = posef.getOrigin().getZ();
                        tra_pos.pose.orientation.x = posef.getRotation().getX();
                        tra_pos.pose.orientation.y = posef.getRotation().getY();
                        tra_pos.pose.orientation.z = posef.getRotation().getZ();
                        tra_pos.pose.orientation.w = posef.getRotation().getW();

                        des_ee_tra.poses.push_back(tra_pos);
                        des_ee_tra_pub.publish(des_ee_tra);
                    }
                }
            }

            Qx_ref = Q*(-1)*xRef;

            if(i/7 != 0)
            {
                if(std::abs(last_xRef(0)-xRef(0)) > 1 || std::abs(last_xRef(1)-xRef(1)) > 1 || std::abs(last_xRef(2)-xRef(2)) > 1 || std::abs(last_xRef(3)-xRef(3)) > 1 || std::abs(last_xRef(4)-xRef(4)) > 1 || std::abs(last_xRef(5)-xRef(5)) > 1)
                    return -2;
            }

            last_xRef = xRef;
	    }

        float value = Qx_ref(posQ,0);
        gradient(i,0) = value;
    }

    dec_factor_previous = dec_factor;

    return 0;
}

void ModelPredictiveControl::castMPCToQPConstraintMatrix()
{
    //input:A, B, mpcWindow
    constraintMatrix.resize(7*(mpcWindow+1)+7*(mpcWindow+1)+7*mpcWindow+7*mpcWindow, 7*(mpcWindow+1)+7*mpcWindow);

    // populate linear constraint matrix
    for(int i = 0; i < 7*(mpcWindow+1); i++)
    {
        constraintMatrix.insert(i,i) = -1;
    }

    for(int i = 0; i < mpcWindow; i++)
        for(int j = 0; j < 7; j++)
            for(int k = 0; k < 7; k++)
	        {
                float value = A(j,k);
                if(value != 0)
		        {
                    constraintMatrix.insert(7*(i+1)+j, 7*i+k) = value;
                }
            }

    for(int i = 0; i < mpcWindow; i++)
        for(int j = 0; j < 7; j++)
            for(int k = 0; k < 7; k++)
	        {
                float value = B(j,k);
                if(value != 0)
		        {
                    constraintMatrix.insert(7*(i+1)+j, 7*i+k+7*(mpcWindow+1)) = value;
                }
            }

    for(int i = 0; i < 7*(mpcWindow+1)+7*mpcWindow; i++)
    {
        constraintMatrix.insert(i+(mpcWindow+1)*7, i) = 1;
    }

    for(int i = 0; i < 7*mpcWindow; i++)
    {
        constraintMatrix.insert(i+7*(mpcWindow+1)*2+7*mpcWindow, i+7*(mpcWindow+1)) = 1./time_step;
    }

    for(int i = 0; i < 7*(mpcWindow-1); i++)
    {
        constraintMatrix.insert(i+7*(mpcWindow+1)*2+7*mpcWindow+7, i+7*(mpcWindow+1)) = (-1.)/time_step;
    }
}

void ModelPredictiveControl::castMPCToQPConstraintVectors()
{
    //input:xMax, xMin, uMax, uMin, x0, mpcWindow
    // evaluate the lower and the upper inequality vectors
    Eigen::VectorXd lowerInequality = Eigen::MatrixXd::Zero(7*(mpcWindow+1)+2*7*mpcWindow, 1);
    Eigen::VectorXd upperInequality = Eigen::MatrixXd::Zero(7*(mpcWindow+1)+2*7*mpcWindow, 1);
    for(int i = 0; i < mpcWindow+1; i++)
    {
        lowerInequality.block(7*i, 0, 7, 1) = xMin;
        upperInequality.block(7*i, 0, 7, 1) = xMax;
    }
    for(int i = 0; i < mpcWindow; i++)
    {
        lowerInequality.block(7*i+7*(mpcWindow+1), 0, 7, 1) = uMin*dec_factor;
        upperInequality.block(7*i+7*(mpcWindow+1), 0, 7, 1) = uMax*dec_factor;
    }
    for(int i = 0; i < mpcWindow; i++)
    {
        if(i==0)
        {
            Eigen::Matrix<double, 7, 1> u_now;
            u_now << current_joint_state.velocity[0], current_joint_state.velocity[1], current_joint_state.velocity[2], current_joint_state.velocity[3], current_joint_state.velocity[4], current_joint_state.velocity[5], current_mobile_platform_velocity.linear.x;

            lowerInequality.block(7*i+7*(mpcWindow+1)+7*mpcWindow, 0, 7, 1) = u_now/time_step+aMin;
            upperInequality.block(7*i+7*(mpcWindow+1)+7*mpcWindow, 0, 7, 1) = u_now/time_step+aMax;
        }
        else
        {
	        lowerInequality.block(7*i+7*(mpcWindow+1)+7*mpcWindow, 0, 7, 1) = aMin;
            upperInequality.block(7*i+7*(mpcWindow+1)+7*mpcWindow, 0, 7, 1) = aMax;
	    }
    }

    // evaluate the lower and the upper equality vectors
    Eigen::VectorXd lowerEquality = Eigen::MatrixXd::Zero(7*(mpcWindow+1), 1);
    Eigen::VectorXd upperEquality;
    
    lowerEquality.block(0,0,7,1) = -x0;
    upperEquality = lowerEquality;
    lowerEquality = lowerEquality;

    // merge inequality and equality vectors
    lowerBound = Eigen::MatrixXd::Zero(2*7*(mpcWindow+1)+2*7*mpcWindow, 1);
    lowerBound << lowerEquality, lowerInequality;

    upperBound = Eigen::MatrixXd::Zero(2*7*(mpcWindow+1)+2*7*mpcWindow, 1);
    upperBound << upperEquality, upperInequality;
}

void ModelPredictiveControl::joint_state_callback(const sensor_msgs::JointState& joint_state)
{
    for(int i = 0; i < 6; i++)
    {
        current_joint_state.position[i] = joint_state.position[i];
        current_joint_state.velocity[i] = joint_state.velocity[i];
    }
}

void ModelPredictiveControl::mobile_platform_velocity_callback(const geometry_msgs::Twist& vel)
{
    current_mobile_platform_velocity.linear.x = vel.linear.x;

    if(callback_order == 0)
    {
        callback_order = 1;
    }
    else if(callback_order == 2)
    {
        trajectory_ini_v_mob_plat = current_mobile_platform_velocity.linear.x;
    }

    std::cout << "vel:" << current_mobile_platform_velocity.linear.x << std::endl << std::endl;
}

void ModelPredictiveControl::apriltag_detection_callback(const apriltags_ros::AprilTagDetectionArray& detection)
{
    for(int i = 0; i < detection.detections.size(); i++)
    {
        if(detection.detections[i].id == 13)
        {
            apriltag_detected = true;
	    }
    }
}

void ModelPredictiveControl::obstacles_detection_callback(const std_msgs::Float64MultiArray& obs_det_output)
{
    tf::StampedTransform camera_tf, current_mobile_platform_pose;
    tf::Vector3 initial_x_axis_vec, current_x_axis_vec, displacement;
    double mob_plat_position;//, base_link_to_camera_length;

    if(!obstacles_detection_enable) return;

    while(1)
    {
        try
        {
            // listener.lookupTransform("/base_link", "/depth_camera_1_depth_optical_frame", ros::Time(0), camera_tf);
	        listener.lookupTransform("/odom", "/base_footprint", ros::Time(0), current_mobile_platform_pose);
            break;
        }
        catch (tf::TransformException &ex)
        {
            ROS_ERROR("%s",ex.what());
            ros::Duration(0.005).sleep();
        }
    }

    initial_x_axis_vec = initial_mobile_platform_pose.getBasis().getColumn(0);
    current_x_axis_vec = current_mobile_platform_pose.getBasis().getColumn(0);

    displacement = current_mobile_platform_pose.getOrigin();
    displacement-= initial_mobile_platform_pose.getOrigin();

    mob_plat_position = initial_x_axis_vec.dot(displacement)/current_x_axis_vec.dot(initial_x_axis_vec);
    
    on_spot = false;

    if(obs_det_output.data[1] < -0.00001)
    {   
        if(desired_mobile_platform_position > mob_plat_position+obs_det_output.data[1] && desired_mobile_platform_position > -0.3+0.00001)
        {
            desired_mobile_platform_position = mob_plat_position+obs_det_output.data[1];

            if(desired_mobile_platform_position <= -0.3)
                desired_mobile_platform_position = -0.3;

	        starting_mobile_platform_position = mob_plat_position;

	        trajectory_total_t_mob_plat = (starting_mobile_platform_position-desired_mobile_platform_position)/0.5*7.5;

            if(current_mobile_platform_velocity.linear.x > 0)
		        trajectory_total_t_mob_plat += std::abs(current_mobile_platform_velocity.linear.x)/0.05;

	        trajectory_start_t_mob_plat = ros::Time::now().toSec();
            trajectory_exec_t_mob_plat = 0;

            if(callback_order == 0)
            { 
                callback_order = 2;
            }
            else if(callback_order == 1)
            { 
                trajectory_ini_v_mob_plat = current_mobile_platform_velocity.linear.x;
            }
        }
    }
    else if(obs_det_output.data[1] > 0.00001)
    {
        if(desired_mobile_platform_position < mob_plat_position+obs_det_output.data[1] && desired_mobile_platform_position < 0.3-0.00001) // 0.0
        {
            desired_mobile_platform_position = mob_plat_position+obs_det_output.data[1];

            if(desired_mobile_platform_position >= 0.3)  // 0.0
                desired_mobile_platform_position = 0.3;  // 0.0

	        starting_mobile_platform_position = mob_plat_position;

	        trajectory_total_t_mob_plat = (desired_mobile_platform_position-starting_mobile_platform_position)/0.5*7.5;

            if(current_mobile_platform_velocity.linear.x < 0)
                trajectory_total_t_mob_plat += std::abs(current_mobile_platform_velocity.linear.x)/0.05;  

	        trajectory_start_t_mob_plat = ros::Time::now().toSec();
            trajectory_exec_t_mob_plat = 0;

            if(callback_order == 0)
            { 
                callback_order = 2;
            }
            else if(callback_order == 1)
            { 
                trajectory_ini_v_mob_plat = current_mobile_platform_velocity.linear.x;
            }
        }
    }
    else
    {
        on_spot = true;
    }

    if(obs_det_output.data[0] <= distance_protective_field)
    {
        stop();

        dec_factor = 0;
    }
    else if(obs_det_output.data[0] < distance_warning_field)
    {
        dec_factor = (obs_det_output.data[0]-distance_protective_field)/(distance_warning_field-distance_protective_field)*1+0.0;
    }
    else
    {
	    dec_factor = 1;
    }
}

void ModelPredictiveControl::updateMPCx0()
{
    tf::StampedTransform current_mobile_platform_pose;
    tf::Vector3 initial_x_axis_vec, current_x_axis_vec, displacement;

    while(1)
    {
    	try
    	{
            listener.lookupTransform("/odom", "/base_footprint", ros::Time(0), current_mobile_platform_pose);
            break;
    	}
        catch (tf::TransformException &ex)
    	{
            ROS_ERROR("%s",ex.what());
            ros::Duration(0.005).sleep();
    	}
    }

    initial_x_axis_vec = initial_mobile_platform_pose.getBasis().getColumn(0);
    current_x_axis_vec = current_mobile_platform_pose.getBasis().getColumn(0);

    displacement = current_mobile_platform_pose.getOrigin();
    displacement-= initial_mobile_platform_pose.getOrigin();

    x0 << current_joint_state.position[0], current_joint_state.position[1], current_joint_state.position[2], current_joint_state.position[3], current_joint_state.position[4], current_joint_state.position[5], initial_x_axis_vec.dot(displacement)/current_x_axis_vec.dot(initial_x_axis_vec);

    if(on_spot)
    {
	    desired_mobile_platform_position = x0(6, 0);
        starting_mobile_platform_position = x0(6, 0);
    }
}

int ModelPredictiveControl::inverseKinematic(const tf::StampedTransform& transformation, double ik_sols[])
{
    double T[16];

    T[0] = transformation.getBasis()[0][0];
    T[1] = transformation.getBasis()[0][1];
    T[2] = transformation.getBasis()[0][2];
    T[3] = transformation.getOrigin().x();
    T[4] = transformation.getBasis()[1][0];
    T[5] = transformation.getBasis()[1][1];
    T[6] = transformation.getBasis()[1][2];
    T[7] = transformation.getOrigin().y();
    T[8] = transformation.getBasis()[2][0];
    T[9] = transformation.getBasis()[2][1];
    T[10] = transformation.getBasis()[2][2];
    T[11] = transformation.getOrigin().z();
    T[12] = 0;
    T[13] = 0;
    T[14] = 0;
    T[15] = 1;

    //std::cout << tm_kinematics::inverse(T, ik_sols, 0.0) << std::endl;
    //for (const auto& e : ik_sols) {
    //    std::cout << e << std::endl;
    //}

    if(tm_kinematics::inverse(T, ik_sols, 0.0) > 0)
    {
        //for (const auto& e : ik_sols) {
        //    std::cout << e << std::endl;
        //}

        if(ik_sols[0] > M_PI*3/4)
	    ik_sols[0] -= 2*M_PI; 

        return 1;
    }
    else
    {
        ROS_INFO("Fail to solve the ik problem.");
        return -1;
    }
}

int ModelPredictiveControl::inverseKinematic(const tf::Transform& transformation, double ik_sols[])
{
    double T[16];

    T[0] = transformation.getBasis()[0][0];
    T[1] = transformation.getBasis()[0][1];
    T[2] = transformation.getBasis()[0][2];
    T[3] = transformation.getOrigin().x();
    T[4] = transformation.getBasis()[1][0];
    T[5] = transformation.getBasis()[1][1];
    T[6] = transformation.getBasis()[1][2];
    T[7] = transformation.getOrigin().y();
    T[8] = transformation.getBasis()[2][0];
    T[9] = transformation.getBasis()[2][1];
    T[10] = transformation.getBasis()[2][2];
    T[11] = transformation.getOrigin().z();
    T[12] = 0;
    T[13] = 0;
    T[14] = 0;
    T[15] = 1;

    //std::cout << tm_kinematics::inverse(T, ik_sols, 0.0) << std::endl;
    //for (const auto& e : ik_sols) {
    //    std::cout << e << std::endl;
    //} 

    if(tm_kinematics::inverse(T, ik_sols, 0.0) > 0)
    {   
        //for (const auto& e : ik_sols) {
        //    std::cout << e << std::endl;
        //}

	    if(ik_sols[0] > M_PI*3/4)
            ik_sols[0] -= 2*M_PI;

        return 1;
    }   
    else
    {   
        ROS_INFO("Fail to solve the ik problem.");
        return -1; 
    }   
}

double ModelPredictiveControl::trajectoryPlanning(double ini_v, double t)
{
    return (ini_v-2)*pow(t, 3)+((-2)*ini_v+3)*pow(t, 2)+ini_v*t;
}

double ModelPredictiveControl::trajectoryPlanning(double ini_v, double ini_a, double t)
{   
    return ((-1/2)*ini_a-3*ini_v+6)*pow(t, 5)+(3/2*ini_a+8*ini_v-15)*pow(t, 4)+((-3/2)*ini_a-6*ini_v+10)*pow(t, 3)+1/2*ini_a*pow(t, 2)+ini_v*t;
}

void ModelPredictiveControl::stop()
{
    geometry_msgs::Twist mobile_platform_velocity_cmd;
    std_msgs::Float64MultiArray joint_velocity_cmd;
    int stop_count;

    joint_velocity_cmd.data.resize(6);

    while(1)
    {
        for(int i = 0; i <= 5; i++)
        {
	        if(current_joint_state.velocity[i] >= 0)
    	        joint_velocity_cmd.data[i] = (current_joint_state.velocity[i]+aMin[i]*time_step > 0) ? (current_joint_state.velocity[i]+aMin[i]*time_step) : 0;
	        else
                joint_velocity_cmd.data[i] = (current_joint_state.velocity[i]+aMax[i]*time_step < 0) ? (current_joint_state.velocity[i]+aMax[i]*time_step) : 0;
        }

        if(current_mobile_platform_velocity.linear.x >= 0)
            mobile_platform_velocity_cmd.linear.x = (current_mobile_platform_velocity.linear.x+aMin[6]*time_step > 0) ? (current_mobile_platform_velocity.linear.x+aMin[6]*time_step) : 0;
        else
            mobile_platform_velocity_cmd.linear.x = (current_mobile_platform_velocity.linear.x+aMax[6]*time_step < 0) ? (current_mobile_platform_velocity.linear.x+aMax[6]*time_step) : 0;
    
        joint_velocity_pub.publish(joint_velocity_cmd);
        mobile_platform_velocity_pub.publish(mobile_platform_velocity_cmd);

        stop_count = 0;

        for(int i = 0; i <= 5; i++)
        {
	        if(current_joint_state.velocity[i] < 0.00001 && current_joint_state.velocity[i] > -0.00001)
    	        stop_count++;
        }

        if(current_mobile_platform_velocity.linear.x < 0.00001 && current_mobile_platform_velocity.linear.x > -0.00001)
    	    stop_count++;

        if(stop_count == 7)
            break;

        ros::spinOnce();
    }
}

bool ModelPredictiveControl::reachDesiredPose(const tf::StampedTransform& ee_desired_pose, const bool& pose_is_global)
{
    //ROS_INFO("sssss");

    int reach_goal_count;
    geometry_msgs::Twist mobile_platform_velocity_cmd;
    std_msgs::Float64MultiArray joint_velocity_cmd, robot_state, robot_vel, ee_state;
    Eigen::VectorXd ctr, QPSolution;
    nav_msgs::Path ee_tra;
    geometry_msgs::PoseStamped tra_pos;
    tf::StampedTransform ee_pose_tf;
    //geometry_msgs::PoseStamped ee_pose; 
   
    joint_velocity_cmd.data.resize(6);
    robot_state.data.resize(7);
    robot_vel.data.resize(7);
    ee_state.data.resize(6);

    des_ee_tra.poses.clear();

    while(1)
    {
    	try
   	    {
            if(pose_is_global)
            {
                des_ee_tra.header.frame_id = "odom";
                ee_tra.header.frame_id = "odom";
                listener.lookupTransform("/odom", "/tm_tool0", ros::Time(0), pose0);
                break;
            }
            else
            {
                des_ee_tra.header.frame_id = "tm_base_link";
                ee_tra.header.frame_id = "tm_base_link";
                listener.lookupTransform("/tm_base_link", "/tm_tool0", ros::Time(0), pose0);
                break;
            }
    	}
    	catch (tf::TransformException &ex)
    	{
            ROS_ERROR("%s",ex.what());
            ros::Duration(0.005).sleep();
        }
    }

    tra_pos.pose.position.x = pose0.getOrigin().getX();
    tra_pos.pose.position.y = pose0.getOrigin().getY();
    tra_pos.pose.position.z = pose0.getOrigin().getZ();
    tra_pos.pose.orientation.x = pose0.getRotation().getX();
    tra_pos.pose.orientation.y = pose0.getRotation().getY();
    tra_pos.pose.orientation.z = pose0.getRotation().getZ();
    tra_pos.pose.orientation.w = pose0.getRotation().getW();

    ee_tra.poses.push_back(tra_pos);
    ee_tra_pub.publish(ee_tra);
   
    ee_state.data[0] = pose0.getOrigin().getX();
    ee_state.data[1] = pose0.getOrigin().getY();
    ee_state.data[2] = pose0.getOrigin().getZ();
    pose0.getBasis().getRPY(ee_state.data[3], ee_state.data[4], ee_state.data[5]);
    
    ee_state_pub.publish(ee_state);

    desired_pose_is_global = pose_is_global;

    posef = ee_desired_pose;

    callback_order = 0;
    ros::spinOnce();

    //on_spot = true;

    updateMPCx0();

    for(int i = 0; i < 7; i++)
    	robot_state.data[i] = x0[i];
    
    robot_state_pub.publish(robot_state);

    for(int i = 0; i < 6; i++)
    	robot_vel.data[i] = current_joint_state.velocity[i];
    robot_vel.data[6] = current_mobile_platform_velocity.linear.x; 

    robot_vel_pub.publish(robot_vel);

    trajectory_total_t = (pose0.getOrigin().distance(posef.getOrigin())/0.15*3 > pose0.getRotation().angleShortestPath(posef.getRotation())/0.8*4) ? pose0.getOrigin().distance(posef.getOrigin())/0.15*3 : pose0.getRotation().angleShortestPath(posef.getRotation())/0.8*4;
    std::cout << pose0.getOrigin().distance(posef.getOrigin()) << ":::" << pose0.getRotation().angleShortestPath(posef.getRotation()) << std::endl;
    trajectory_exec_t = 0;

    castMPCToQPHessian();
    castMPCToQPConstraintMatrix();

    // set the initial data of the QP solver
    if(!solver.updateHessianMatrix(hessianMatrix)) return false;
    if(!solver.updateLinearConstraintsMatrix(constraintMatrix)) return false;

    trajectory_start_t = ros::Time::now().toSec();

    int state = castMPCToQPGradient();

    if(state == -2)
    {
	    std::cout << "Set of configuration changes." << std::endl;
        return false;
    }
    else if(state == -1)
    {
        stop();
        dec_factor = 0;
	    dec_factor_previous = dec_factor;
    }
    
    if(!solver.updateGradient(gradient)) return false;

    castMPCToQPConstraintVectors();

    if(!solver.updateBounds(lowerBound, upperBound)) return false;

    //ROS_INFO("eeeee");

    while(ros::ok())
    {
        //std::cout << lowerBound << std::endl << std::endl;
        //std::cout << upperBound << std::endl << std::endl;  

        //std::cout << constraintMatrix << std::endl;

        //std::cout << x0 << std::endl;

        // solve the QP problem
        //if(!solver.solve()) return false;
        if(solver.solve())
	    {
            // get the controller input
            QPSolution = solver.getSolution();
            ctr = QPSolution.block(7*(mpcWindow+1), 0, 7, 1);
	    }
        else
        {
            for(int i = 0; i <= 5; i++)
            {
            if(current_joint_state.velocity[i] >= 0)
	    	    ctr(i) = (current_joint_state.velocity[i]+aMin[i]*time_step > 0) ? (current_joint_state.velocity[i]+aMin[i]*time_step) : 0;
		    else
                ctr(i) = (current_joint_state.velocity[i]+aMax[i]*time_step < 0) ? (current_joint_state.velocity[i]+aMax[i]*time_step) : 0;
	        }

            if(current_mobile_platform_velocity.linear.x >= 0)
                ctr(6) = (current_mobile_platform_velocity.linear.x+aMin[6]*time_step > 0) ? (current_mobile_platform_velocity.linear.x+aMin[6]*time_step) : 0;
            else
                ctr(6) = (current_mobile_platform_velocity.linear.x+aMax[6]*time_step < 0) ? (current_mobile_platform_velocity.linear.x+aMax[6]*time_step) : 0;
	    }
        
        std::cout << "vel_cmd:" << std::endl << ctr << std::endl << std::endl;

        reach_goal_count = 0;

        if((x0[6] - xf[6]) < 0.003 && (x0[6] - xf[6]) > -0.003 && std::abs(current_mobile_platform_velocity.linear.x) < 0.023)
        {
            std::cout << "vel:" << std::endl << current_mobile_platform_velocity.linear.x << std::endl << std::endl;

            mobile_platform_velocity_cmd.linear.x = 0;
            reach_goal_count++;
        }
        else
        {
            if(std::abs(ctr(6)) < 0.005)
		        mobile_platform_velocity_cmd.linear.x = 0;
            else
                mobile_platform_velocity_cmd.linear.x = ctr(6);
	    }

        std::cout << "vel_cmd:" << std::endl << mobile_platform_velocity_cmd.linear.x << std::endl << std::endl;

	    if((x0 - xf).block<6, 1>(0, 0).norm() < 0.0005)
    	{ 
            if(desired_pose_is_global)
            {
                if(reach_goal_count == 1)
                {
                    for(int i = 0; i < 6; i++)
                    {   
                        joint_velocity_cmd.data[i] = 0;
                    }
	            
	            reach_goal_count++;
		        }
                else
                {
                    for(int i = 0; i < 6; i++)
                    {
                        joint_velocity_cmd.data[i] = ctr(i);
                    }
                }
	        }
            else
	        {
            	for(int i = 0; i < 6; i++)
            	{
            	    joint_velocity_cmd.data[i] = 0;
            	}
 
            	reach_goal_count++;
		        on_spot = true;
	        }
        }
        else
        {
            for(int i = 0; i < 6; i++)
            {
                joint_velocity_cmd.data[i] = ctr(i);         
            }
        }

    	joint_velocity_pub.publish(joint_velocity_cmd);
    	mobile_platform_velocity_pub.publish(mobile_platform_velocity_cmd);
	
    	if(reach_goal_count == 2)
    	{
            /*while(1)
    	    {
    	    	try
   	    	{
	    	    if(pose_is_global)
	    	    {
            	    	listener.lookupTransform("/odom", "/tm_tool0", ros::Time(0), ee_pose_tf);
            	    	break;
	            }
	            else
	            {
		    	listener.lookupTransform("/tm_base_link", "/tm_tool0", ros::Time(0), ee_pose_tf);
            	    	break;
	    	    }
    	    	}
    	    	catch (tf::TransformException &ex)
    	    	{
            	    ROS_ERROR("%s",ex.what());
            	    ros::Duration(0.005).sleep();
            	}
    	    }

    	    tra_pos.pose.position.x = ee_pose_tf.getOrigin().getX();
    	    tra_pos.pose.position.y = ee_pose_tf.getOrigin().getY();
    	    tra_pos.pose.position.z = ee_pose_tf.getOrigin().getZ();
    	    tra_pos.pose.orientation.x = ee_pose_tf.getRotation().getX();
    	    tra_pos.pose.orientation.y = ee_pose_tf.getRotation().getY();
    	    tra_pos.pose.orientation.z = ee_pose_tf.getRotation().getZ();
    	    tra_pos.pose.orientation.w = ee_pose_tf.getRotation().getW();

    	    ee_tra.poses.push_back(tra_pos);
    	    ee_tra_pub.publish(ee_tra);*/

            break;
    	}

	    loop_rate.sleep();

	    // save data into file
        //auto x0Data = x0.data();

        // propagate the model
        callback_order = 0;
	    ros::spinOnce();

        //on_spot = true;

	    updateMPCx0();

        for(int i = 0; i < 7; i++)
    	    robot_state.data[i] = x0[i];
    
        robot_state_pub.publish(robot_state);

    	for(int i = 0; i < 6; i++)
    	    robot_vel.data[i] = current_joint_state.velocity[i];
    	robot_vel.data[6] = current_mobile_platform_velocity.linear.x; 

    	robot_vel_pub.publish(robot_vel);

        //std::cout << std::endl << ctr <<std::endl;
        //std::cout << std::endl << x0 << std::endl;

        state = castMPCToQPGradient();

        if(state == -2)
	    {
	        std::cout << "Set of configuration changes." << std::endl;
            return false;
	    }
	    else if(state == -1)
    	{
            stop();

            dec_factor = 0;
	        dec_factor_previous = dec_factor;
    	}

        if(!solver.updateGradient(gradient)) return false;

        // update the constraint bound
        castMPCToQPConstraintVectors();
        if(!solver.updateBounds(lowerBound, upperBound)) return false;

        while(1)
    	{
    	    try
   	        {
                if(pose_is_global)
                {
                    listener.lookupTransform("/odom", "/tm_tool0", ros::Time(0), ee_pose_tf);
                    break;
                }
                else
                {
                    listener.lookupTransform("/tm_base_link", "/tm_tool0", ros::Time(0), ee_pose_tf);
                    break;
                }
    	    }
    	    catch (tf::TransformException &ex)
    	    {
            	ROS_ERROR("%s",ex.what());
            	ros::Duration(0.005).sleep();
            }
    	}

    	tra_pos.pose.position.x = ee_pose_tf.getOrigin().getX();
    	tra_pos.pose.position.y = ee_pose_tf.getOrigin().getY();
    	tra_pos.pose.position.z = ee_pose_tf.getOrigin().getZ();
    	tra_pos.pose.orientation.x = ee_pose_tf.getRotation().getX();
    	tra_pos.pose.orientation.y = ee_pose_tf.getRotation().getY();
    	tra_pos.pose.orientation.z = ee_pose_tf.getRotation().getZ();
    	tra_pos.pose.orientation.w = ee_pose_tf.getRotation().getW();

    	ee_tra.poses.push_back(tra_pos);
    	ee_tra_pub.publish(ee_tra);

    	ee_state.data[0] = ee_pose_tf.getOrigin().getX();
    	ee_state.data[1] = ee_pose_tf.getOrigin().getY();
    	ee_state.data[2] = ee_pose_tf.getOrigin().getZ();
    	ee_pose_tf.getBasis().getRPY(ee_state.data[3], ee_state.data[4], ee_state.data[5]);
    
    	ee_state_pub.publish(ee_state);

        std::cout << "x0:" << std::endl << x0 << std::endl << std::endl;
        std::cout << "xf:" << std::endl << xf << std::endl << std::endl;
        //std::cout << "lowerBound:" << std::endl << lowerBound << std::endl << std::endl;
        //std::cout << "uppedBound:" << std::endl << upperBound << std::endl << std::endl;
        std::cout << "dec_factor:" << std::endl << dec_factor << std::endl << std::endl;
        std::cout << "gradient:" << std::endl << gradient.block(0, 0, (mpcWindow+1)*7, 1) << std::endl << std::endl;
    }

    return true;
}

void ModelPredictiveControl::performReplenishment()
{ 
    double task_start_t = ros::Time::now().toSec();
    double fun_fin_t[3];

    std_msgs::Bool apriltag_detection_enable, gripper_cmd;
    tf::StampedTransform apriltag_pose, placing_pose, pre_placing_pose; 

    tf::StampedTransform detection_pose;
    tf::StampedTransform base_tf;

    while(1)
    {
    	try
    	{
            listener.lookupTransform("/odom", "/tm_base_link", ros::Time(0), base_tf);
	        break;
        }
    	catch (tf::TransformException &ex)
    	{
            ROS_ERROR("%s",ex.what());
            ros::Duration(0.005).sleep();
    	}
    }

    //obstacles_detection_enable = false;
    obstacles_detection_enable = true;
    dec_factor = 1;

    while(0)
    {
        // GO TO SCAN TAG POSITION
        detection_pose.setData(base_tf*tf::Transform(tf::Transform(tf::Quaternion(0.7, 0, 0, 0.7), tf::Vector3(-0.12, -0.25, 0.55)))); 
        
        if(!reachDesiredPose(detection_pose, true))
        {
            ROS_INFO("Fail to reach the desired pose");
            stop();
            return;
        }

        tf::TransformListener listener;
        tf::StampedTransform tf_l;
        std::string tf_l_name = "/tag_308";

        static tf::TransformBroadcaster br;
        tf::Transform tf_b;    
        std::string tf_b_name = "/target_lemonade";
        

        listener.waitForTransform("/base_link", tf_l_name, ros::Time(0), ros::Duration(3.0));
        listener.lookupTransform("/base_link", tf_l_name, ros::Time(0), tf_l);

        tf_b.setOrigin(tf::Vector3(0.05, -0.07, 0.3421));
        tf_b.setRotation(tf::Quaternion(-0.500, 0.500, 0.500, 0.500));
        br.sendTransform(tf::StampedTransform(tf_b, ros::Time::now(), "/tag_308", tf_b_name));

        listener.waitForTransform("/tm_base_link", tf_b_name, ros::Time(0), ros::Duration(3.0));
        listener.lookupTransform("/tm_base_link", tf_b_name, ros::Time(0), tf_l);

        // std::cout<<"Relative Pose"<<std::endl;
        // std::cout<<"X: "<<tf_l.getOrigin().getX()<<std::endl;
        // std::cout<<"Y: "<<tf_l.getOrigin().getY()<<std::endl;
        // std::cout<<"Z: "<<tf_l.getOrigin().getZ()<<std::endl;

        // GO TO MID POSITION
        detection_pose.setData(base_tf*tf::Transform(tf::Transform(tf::Quaternion(0.7, 0, 0, 0.7), tf::Vector3(-0.24, -0.15, 0.55)))); 
        
        if(!reachDesiredPose(detection_pose, true))
        {
            ROS_INFO("Fail to reach the desired pose");
            stop();
            return;
        }

        //GO TO PLACE 1 MID
        detection_pose.setData(base_tf*tf::Transform(tf::Transform(tf::Quaternion(0.425, -0.436, -0.563, 0.559), tf::Vector3(-0.279, 0.123, 0.471)))); 
        
        if(!reachDesiredPose(detection_pose, true))
        {
            ROS_INFO("Fail to reach the desired pose");
            stop();
            return;
        }

        //GO TO PLACE 1
        detection_pose.setData(base_tf*tf::Transform(tf::Transform(tf::Quaternion(-0.498, 0.506, 0.502, -0.493), tf::Vector3(-0.372, 0.122, 0.332)))); 
        
        if(!reachDesiredPose(detection_pose, true))
        {
            ROS_INFO("Fail to reach the desired pose");
            stop();
            return;
        }

        gripper_cmd.data = true;
        gripper_pub.publish(gripper_cmd);
        sleep(1);

        //GO TO PLACE 1 MID
        detection_pose.setData(base_tf*tf::Transform(tf::Transform(tf::Quaternion(0.425, -0.436, -0.563, 0.559), tf::Vector3(-0.279, 0.123, 0.471)))); 
        
        if(!reachDesiredPose(detection_pose, true))
        {
            ROS_INFO("Fail to reach the desired pose");
            stop();
            return;
        }

        // GO TO MID POSITION
        detection_pose.setData(base_tf*tf::Transform(tf::Transform(tf::Quaternion(0.7, 0, 0, 0.7), tf::Vector3(-0.24, -0.15, 0.55)))); 
        
        if(!reachDesiredPose(detection_pose, true))
        {
            ROS_INFO("Fail to reach the desired pose");
            stop();
            return;
        }

        // GO TO LEMONADE POSITION

        detection_pose.setData(base_tf*tf::Transform(tf::Transform(tf::Quaternion(0.7, 0, 0, 0.7), tf::Vector3(tf_l.getOrigin().getX(), tf_l.getOrigin().getY()+0.05, tf_l.getOrigin().getZ()+0.05)))); 
        
        if(!reachDesiredPose(detection_pose, true))
        {
            ROS_INFO("Fail to reach the desired pose");
            stop();
            return;
        }

        detection_pose.setData(base_tf*tf::Transform(tf::Transform(tf::Quaternion(0.7, 0, 0, 0.7), tf::Vector3(tf_l.getOrigin().getX(), tf_l.getOrigin().getY(), tf_l.getOrigin().getZ())))); 
        
        if(!reachDesiredPose(detection_pose, true))
        {
            ROS_INFO("Fail to reach the desired pose");
            stop();
            return;
        }

        gripper_cmd.data = false;
        gripper_pub.publish(gripper_cmd);
        sleep(1);

        // GO TO SCAN TAG POSITION

        detection_pose.setData(base_tf*tf::Transform(tf::Transform(tf::Quaternion(0.7, 0, 0, 0.7), tf::Vector3(tf_l.getOrigin().getX(), tf_l.getOrigin().getY()+0.05, tf_l.getOrigin().getZ()+0.05)))); 
        
        if(!reachDesiredPose(detection_pose, true))
        {
            ROS_INFO("Fail to reach the desired pose");
            stop();
            return;
        }

        detection_pose.setData(base_tf*tf::Transform(tf::Transform(tf::Quaternion(0.7, 0, 0, 0.7), tf::Vector3(-0.12, -0.25, 0.55)))); 
        
        if(!reachDesiredPose(detection_pose, true))
        {
            ROS_INFO("Fail to reach the desired pose");
            stop();
            return;
        }

        ROS_INFO("REPLENISHMENT FINISHED !!!");
    }

    while(1)
    {
        detection_pose.setData(base_tf*tf::Transform(tf::Transform(tf::Quaternion(0.7, 0, 0, 0.7), tf::Vector3(-0.12, -0.25, 0.55)))); 
    
        if(!reachDesiredPose(detection_pose, true))
        {
            ROS_INFO("Fail to reach the desired pose");
            stop();
            return;
        }

        detection_pose.setData(base_tf*tf::Transform(tf::Transform(tf::Quaternion(0.7, 0, 0, 0.7), tf::Vector3(-0.12, -0.25, 0.67)))); 
    
        if(!reachDesiredPose(detection_pose, true))
        {
            ROS_INFO("Fail to reach the desired pose");
            stop();
            return;
        }

        detection_pose.setData(base_tf*tf::Transform(tf::Transform(tf::Quaternion(0.7, 0, 0, 0.7), tf::Vector3(-0.12, -0.25, 0.41)))); 
    
        if(!reachDesiredPose(detection_pose, true))
        {
            ROS_INFO("Fail to reach the desired pose");
            stop();
            return;
        }

        detection_pose.setData(base_tf*tf::Transform(tf::Transform(tf::Quaternion(0.7, 0, 0, 0.7), tf::Vector3(-0.12, -0.25, 0.55)))); 
    
        if(!reachDesiredPose(detection_pose, true))
        {
            ROS_INFO("Fail to reach the desired pose");
            stop();
            return;
        }
    }

    return;


    // OLD CODE
    while(0)
    {
        apriltag_detection_enable.data = true;
        apriltag_detection_cmd_pub.publish(apriltag_detection_enable);

        //consume the apriltag detection that may be out of date.
        ros::spinOnce();
        apriltag_detected = false;

        while(!apriltag_detected)
        {
            ros::spinOnce();
        }

        while(1)
        {
            try
            {
                listener.lookupTransform("/odom", "/at13", ros::Time(0), apriltag_pose);
                break;
            }
            catch (tf::TransformException &ex)
            {
                ROS_ERROR("%s",ex.what());
                ros::Duration(0.005).sleep();
            }
        }

        apriltag_detection_enable.data = false;
        apriltag_detection_cmd_pub.publish(apriltag_detection_enable);

        placing_pose = apriltag_pose;

        tf::StampedTransform bias;
        tf::Quaternion qu(-0.005, 0.998, 0.068, -0.003);

        bias.setData(tf::Transform(qu, tf::Vector3(0.0, 0.0, 0.0)));
        placing_pose*=bias;

        bias.setData(tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0.02, -0.28, -0.08)));
        placing_pose*=bias;

        pre_placing_pose = placing_pose;

        bias.setData(tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0, 0, -0.19)));
        pre_placing_pose*=bias;

        detection_pose.setData(tf::Transform(tf::Quaternion(0.7, 0, 0, 0.7), tf::Vector3(-0.2, -0.25, 0.47))); 

        if(!reachDesiredPose(detection_pose, false))
        {
            ROS_INFO("Fail to reach the desired pose");
            stop();
            return;
        }

        fun_fin_t[0] = ros::Time::now().toSec()-task_start_t;

        obstacles_detection_enable = true;

        detection_pose.setData(tf::Transform(tf::Quaternion(0.5, -0.5, -0.5, 0.5), tf::Vector3(-0.35, -0.0, 0.40))); 
        
        if(!reachDesiredPose(detection_pose, false))
        {
            ROS_INFO("Fail to reach the desired pose");
            stop();
            return;
        }

        detection_pose.setData(tf::Transform(tf::Quaternion(0.5, -0.5, -0.5, 0.5), tf::Vector3(-0.35, -0.0, 0.30))); 
        
        if(!reachDesiredPose(detection_pose, false))
        {
            ROS_INFO("Fail to reach the desired pose");
            stop();
            return;
        }

        gripper_cmd.data = true;
        gripper_pub.publish(gripper_cmd);
        ros::Duration(1).sleep();

        detection_pose.setData(tf::Transform(tf::Quaternion(0.5, -0.5, -0.5, 0.5), tf::Vector3(-0.35, -0.0, 0.40))); 
    
        if(!reachDesiredPose(detection_pose, false))
        {
            ROS_INFO("Fail to reach the desired pose");
            stop();
            return;
        }

        detection_pose.setData(tf::Transform(tf::Quaternion(0.7, 0, 0, 0.7), tf::Vector3(-0.2, -0.25, 0.47))); 
        
        if(!reachDesiredPose(detection_pose, false))
        {
            ROS_INFO("Fail to reach the desired pose");
            stop();
            return;
        }

        fun_fin_t[1] = ros::Time::now().toSec()-task_start_t-fun_fin_t[0];

        //detection_pose.setData(base_tf*tf::Transform(tf::Quaternion(0.7, 0, 0, 0.7), tf::Vector3(0.1, -0.25, 0.47))); 
        
        if(!reachDesiredPose(pre_placing_pose, true))
        {
            ROS_INFO("Fail to reach the desired pose");
            stop();
            return;
        }

        //detection_pose.setData(base_tf*tf::Transform(tf::Quaternion(0.7, 0, 0, 0.7), tf::Vector3(0.1, -0.43, 0.47))); 
        
        if(!reachDesiredPose(placing_pose, true))
        {
            ROS_INFO("Fail to reach the desired pose");
            stop();
            return;
        }
        
        gripper_cmd.data = false;
        gripper_pub.publish(gripper_cmd);
        ros::Duration(1).sleep();

        //detection_pose.setData(base_tf*tf::Transform(tf::Quaternion(0.7, 0, 0, 0.7), tf::Vector3(0.1, -0.25, 0.47))); 
        
        if(!reachDesiredPose(pre_placing_pose, true))
        {
            ROS_INFO("Fail to reach the desired pose");
            stop();
            return;
        }

        detection_pose.setData(tf::Transform(tf::Quaternion(0.7, 0, 0, 0.7), tf::Vector3(-0.2, -0.25, 0.47))); 

        if(!reachDesiredPose(detection_pose, false))
        {
            ROS_INFO("Fail to reach the desired pose");
            stop();
            return;
        }

        fun_fin_t[2] = ros::Time::now().toSec()-task_start_t-fun_fin_t[0]-fun_fin_t[1];

        std::cout << "detection_time:" << fun_fin_t[0] << std::endl;
        std::cout << "picking_time:" << fun_fin_t[1] << std::endl;
        std::cout << "placing_time:" << fun_fin_t[2] << std::endl;
        std::cout << "task_time:" << ros::Time::now().toSec()-task_start_t << std::endl;
        
        return;
    }
}
