/**
 * @file MPCExample.cpp
 * @author Giulio Romualdi
 * @copyright Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 * @date 2018
 */

#include <mpc/mpc.h>
#include <tm_kinematics/tm_kin.h>
#include <librealsense2/rsutil.h> 

#include <std_msgs/Bool.h>
#include <geometry_msgs/Twist.h>

#include <iostream>
#include <cmath>

ModelPredictiveControl::ModelPredictiveControl(ros::NodeHandle& nh) : loop_rate(25)
{
    joint_velocity_pub = nh.advertise<std_msgs::Float64MultiArray>("/velocity_cmd", 1);
    mobile_platform_velocity_pub = nh.advertise<geometry_msgs::Twist>("/mob_plat/cmd_vel", 1);
    apriltag_detection_cmd_pub = nh.advertise<std_msgs::Bool>("/apriltag_detection_enable", 1);
    joint_state_sub = nh.subscribe("/tm_joint_states", 1, &ModelPredictiveControl::joint_state_callback, this);
    mobile_platform_velocity_sub = nh.subscribe("/mob_plat/curr_vel", 1, &ModelPredictiveControl::mobile_platform_velocity_callback, this);
    apriltag_detection_sub = nh.subscribe("/tag_detections", 1, &ModelPredictiveControl::apriltag_detection_callback, this);
    //yolo_detection_sub = nh.subscribe("/darknet_ros/bounding_boxes_with_depth_image", 1, &ModelPredictiveControl::yolo_detection_callback, this);   
    obstacles_detection_sub = nh.subscribe("/obs_det_output", 1, &ModelPredictiveControl::obstacles_detection_callback, this);
    //realsense_intrinsic_matrix_sub = nh.subscribe("/realsense_camera/aligned_depth_to_color/camera_info", 1, &ModelPredictiveControl::realsense_intrinsic_matrix_callback, this);

    current_joint_state.position.resize(6);
    current_joint_state.velocity.resize(6);

    apriltag_detected = false;

    ros::Duration(0.5).sleep();

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
            10.55;

    xMin << (-270.)/180*M_PI,
            (-180.)/180*M_PI,
            (-155.)/180*M_PI,
            (-180.)/180*M_PI,
            (-180.)/180*M_PI,
            (-270.)/180*M_PI,
            (-10.55);

    //set the input inequality constraints
    uMax << 180./180*M_PI,
            180./180*M_PI,
            180./180*M_PI,
            225./180*M_PI,
            225./180*M_PI,
            225./180*M_PI,
            0.3;

    uMin << (-180.)/180*M_PI,
            (-180.)/180*M_PI,
            (-180.)/180*M_PI,
            (-225.)/180*M_PI,
            (-225.)/180*M_PI,
            (-225.)/180*M_PI,
            (-0.3);

    aMax << 120./180*M_PI,
            120./180*M_PI,
            120./180*M_PI,
            120./180*M_PI,
            120./180*M_PI,
            120./180*M_PI,
            0.5;

    aMin << (-120.)/180*M_PI,
            (-120.)/180*M_PI,
            (-120.)/180*M_PI,
            (-120.)/180*M_PI,
            (-120.)/180*M_PI,
            (-120.)/180*M_PI,
            (-0.5);
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
            ros::Duration(0.05).sleep();
        }
    }

    posef = pose0;

    trajectory_total_t = 5;
    trajectory_start_t = ros::Time::now().toSec();
    trajectory_exec_t = 0;

    trajectory_total_t_mob_plat = 2;
    trajectory_start_t_mob_plat = ros::Time::now().toSec();
    trajectory_exec_t_mob_plat = 0;

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
    solver.settings()->setPrimalInfeasibilityTollerance(0.001);
    
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
    Eigen::Matrix<double, 7, 1> xRef, Qx_ref;
    tf::StampedTransform poseRef;
    double ik_sols[48];

    trajectory_exec_t += (ros::Time::now().toSec()-trajectory_start_t);
    trajectory_start_t = ros::Time::now().toSec();

    trajectory_exec_t_mob_plat += (ros::Time::now().toSec()-trajectory_start_t_mob_plat);
    trajectory_start_t_mob_plat = ros::Time::now().toSec();

    //when desired_pose_is_global is true, these variables are used;
    tf::StampedTransform arm_base_pose, current_mobile_platform_pose, desired_mobile_platform_pose;
    tf::Vector3 x_axis_vec;
    tf::StampedTransform ref_mobile_platform_pose;

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

    // populate the gradient vector
    gradient = Eigen::VectorXd::Zero(7*(mpcWindow+1)+7*mpcWindow, 1);
    for(int i = 0; i < 7*(mpcWindow+1); i++)
    {
        int posQ = i%7;

	if(posQ == 0)
	{
            if(trajectory_exec_t+(i/7)*time_step < trajectory_total_t)
            {
		poseRef.setData(tf::Transform(pose0.getRotation().slerp(posef.getRotation(), trajectoryPlanning(0, (trajectory_exec_t+(i/7)*time_step)/trajectory_total_t)), pose0.getOrigin().lerp(posef.getOrigin(), trajectoryPlanning(0, (trajectory_exec_t+(i/7)*time_step)/trajectory_total_t))));
                
                if(desired_pose_is_global)
		{
                    if(trajectory_exec_t_mob_plat+(i/7)*time_step < trajectory_total_t_mob_plat)
                    {
			ref_mobile_platform_pose.setBasis(current_mobile_platform_pose.getBasis());
                        ref_mobile_platform_pose.setOrigin(current_mobile_platform_pose.getOrigin()+(starting_mobile_platform_position+(desired_mobile_platform_position-starting_mobile_platform_position)*trajectoryPlanning(current_mobile_platform_velocity.linear.x/(0.5/4), (trajectory_exec_t_mob_plat+(i/7)*time_step)/trajectory_total_t_mob_plat)-x0(6, 0))*x_axis_vec);

			if(inverseKinematic(arm_base_pose.inverse()*ref_mobile_platform_pose.inverse()*poseRef, ik_sols) == (-1))
		        {
	   	             return -1;
		        }

                        xRef << ik_sols[0], ik_sols[1], ik_sols[2], ik_sols[3], ik_sols[4], ik_sols[5], starting_mobile_platform_position+(desired_mobile_platform_position-starting_mobile_platform_position)*trajectoryPlanning(current_mobile_platform_velocity.linear.x/(0.5/4), (trajectory_exec_t_mob_plat+(i/7)*time_step)/trajectory_total_t_mob_plat);
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

		    if(trajectory_exec_t_mob_plat+(i/7)*time_step < trajectory_total_t_mob_plat)
            	    {
		        xRef << ik_sols[0], ik_sols[1], ik_sols[2], ik_sols[3], ik_sols[4], ik_sols[5], starting_mobile_platform_position+(desired_mobile_platform_position-starting_mobile_platform_position)*trajectoryPlanning(current_mobile_platform_velocity.linear.x/(0.5/4), (trajectory_exec_t_mob_plat+(i/7)*time_step)/trajectory_total_t_mob_plat);
	    	    }
                    else
            	    {
		        xRef << ik_sols[0], ik_sols[1], ik_sols[2], ik_sols[3], ik_sols[4], ik_sols[5], desired_mobile_platform_position;
	    	    }
		}
	    }
            else
            {
                if(desired_pose_is_global)
		{
		    if(trajectory_exec_t_mob_plat+(i/7)*time_step < trajectory_total_t_mob_plat)
                    {
                        ref_mobile_platform_pose.setBasis(current_mobile_platform_pose.getBasis());
                        ref_mobile_platform_pose.setOrigin(current_mobile_platform_pose.getOrigin()+(starting_mobile_platform_position+(desired_mobile_platform_position-starting_mobile_platform_position)*trajectoryPlanning(current_mobile_platform_velocity.linear.x/(0.5/4), (trajectory_exec_t_mob_plat+(i/7)*time_step)/trajectory_total_t_mob_plat)-x0(6, 0))*x_axis_vec);

                        if(inverseKinematic(arm_base_pose.inverse()*ref_mobile_platform_pose.inverse()*posef, ik_sols) == (-1))
                        {
                             return -1;
                        }

                        xRef << ik_sols[0], ik_sols[1], ik_sols[2], ik_sols[3], ik_sols[4], ik_sols[5], starting_mobile_platform_position+(desired_mobile_platform_position-starting_mobile_platform_position)*trajectoryPlanning(current_mobile_platform_velocity.linear.x/(0.5/4), (trajectory_exec_t_mob_plat+(i/7)*time_step)/trajectory_total_t_mob_plat);
                    }
                    else
                    {
                        xRef << xf;
                    }
		}
		else
		{
                    xRef = xf;

		    if(trajectory_exec_t_mob_plat+(i/7)*time_step < trajectory_total_t_mob_plat)
            	    {
		    	xRef(6, 0) = starting_mobile_platform_position+(desired_mobile_platform_position-starting_mobile_platform_position)*trajectoryPlanning(current_mobile_platform_velocity.linear.x/(0.5/4), (trajectory_exec_t_mob_plat+(i/7)*time_step)/trajectory_total_t_mob_plat);
	    	    }
		}
            }

            Qx_ref = Q*(-1)*xRef;
	}

        float value = Qx_ref(posQ,0);
        gradient(i,0) = value;
    }

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
        //constraintMatrix.insert(i+7*(mpcWindow+1)*2+7*mpcWindow, i+7*(mpcWindow+1)) = 1.;
    }

    for(int i = 0; i < 7*(mpcWindow-1); i++)
    {
        constraintMatrix.insert(i+7*(mpcWindow+1)*2+7*mpcWindow+7, i+7*(mpcWindow+1)) = (-1.)/time_step;
        //constraintMatrix.insert(i+7*(mpcWindow+1)*2+7*mpcWindow+7, i+7*(mpcWindow+1)) = (-1.);
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
        lowerInequality.block(7*i+7*(mpcWindow+1), 0, 7, 1) = uMin;
        upperInequality.block(7*i+7*(mpcWindow+1), 0, 7, 1) = uMax;
    }
    for(int i = 0; i < mpcWindow; i++)
    {
        if(i==0)
        {
            Eigen::Matrix<double, 7, 1> u_now;
            u_now << current_joint_state.velocity[0], current_joint_state.velocity[1], current_joint_state.velocity[2], current_joint_state.velocity[3], current_joint_state.velocity[4], current_joint_state.velocity[5], current_mobile_platform_velocity.linear.x;

            lowerInequality.block(7*i+7*(mpcWindow+1)+7*mpcWindow, 0, 7, 1) = u_now/time_step+aMin;
            upperInequality.block(7*i+7*(mpcWindow+1)+7*mpcWindow, 0, 7, 1) = u_now/time_step+aMax;
            //lowerInequality.block(7*i+7*(mpcWindow+1)+7*mpcWindow, 0, 7, 1) = u_now+aMin*time_step;
            //upperInequality.block(7*i+7*(mpcWindow+1)+7*mpcWindow, 0, 7, 1) = u_now+aMax*time_step;
        }
        else
        {
	    lowerInequality.block(7*i+7*(mpcWindow+1)+7*mpcWindow, 0, 7, 1) = aMin;
            upperInequality.block(7*i+7*(mpcWindow+1)+7*mpcWindow, 0, 7, 1) = aMax;
            //lowerInequality.block(7*i+7*(mpcWindow+1)+7*mpcWindow, 0, 7, 1) = aMin*time_step;
            //upperInequality.block(7*i+7*(mpcWindow+1)+7*mpcWindow, 0, 7, 1) = aMax*time_step;
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
    lowerBound << lowerEquality,
        lowerInequality;

    upperBound = Eigen::MatrixXd::Zero(2*7*(mpcWindow+1)+2*7*mpcWindow, 1);
    upperBound << upperEquality,
        upperInequality;
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
}

void ModelPredictiveControl::apriltag_detection_callback(const apriltags_ros::AprilTagDetectionArray& detection)
{
    for(int i = 0; i < detection.detections.size(); i++)
    {
        if(detection.detections[i].id == 10)
        {
            apriltag_detected = true;
	}
    }
}

/*void ModelPredictiveControl::yolo_detection_callback(const darknet_ros_msgs::BoundingBoxesWithDepthImage& detection)
{
    tf::StampedTransform camera_tf;
    tf::Vector3 position, point_nearest;
    float pixel[2], point[3], distance_x, distance_y, distance, distance_min;

    auto depthPtr = cv_bridge::toCvCopy(detection.depth_image, sensor_msgs::image_encodings::TYPE_16UC1);
    
    cv::Mat depth_image = depthPtr->image;

    if(depth_image.empty())
    {
        return;
    }

    while(1)
    {
        try
        {
            listener.lookupTransform("/base_link", "/realsense_camera_aligned_depth_to_color_frame", ros::Time(0), camera_tf);
            break;
        }
        catch (tf::TransformException &ex)
        {
            ROS_ERROR("%s",ex.what());
            ros::Duration(0.05).sleep();
        }
    }
    
    for(int i = 0; i < detection.bounding_boxes.size(); i++)
    {
        if(detection.bounding_boxes[i].Class == "person")
	{
            distance_min = 100;
	    //std::cout << det.bounding_boxes[i].xmin << std::endl;

            for(int j = detection.bounding_boxes[i].xmin; j <= detection.bounding_boxes[i].xmax; j++)
            {
                for(int k = detection.bounding_boxes[i].ymin; k <= detection.bounding_boxes[i].ymax; k++)
                {
                    if(j==640 || k==480)
		    {
    		        break;
		    }

                    pixel[0] = j;
                    pixel[1] = k;

                    auto depth = depth_image.at<unsigned short>(static_cast<int>(k), static_cast<int>(j))*0.001f;

                    if(depth < 0.0005)
                    {
			continue;
                    }

                    //the positive x-axis points to the right, the positive y-axis points down, and the positive z-axis points forward.
                    rs2_deproject_pixel_to_point(point, &realsense_intrinsic_matrix, pixel, depth);

		    position = camera_tf*(tf::Vector3(point[2], -point[0], -point[1]));

                    distance_x = std::max(std::max(position.getX()-0.465, -0.315-position.getX()), 0.0);
                    distance_y = std::max(std::max(position.getY()-0.255, -0.255-position.getY()), 0.0);

		    distance = sqrt(distance_x*distance_x+distance_y*distance_y);

                    if(distance < distance_min && distance > 0.001)
                    {
                        distance_min = distance;
                        point_nearest.setX(position.getX());
                        point_nearest.setY(position.getY());
                        point_nearest.setZ(position.getZ());
                    }
                }
            }

	    std::cout << "distance_min:" << distance_min << std::endl;

            distance_y = std::max(std::max(point_nearest.getY()-0.255, -0.255-point_nearest.getY()), 0.0);

            if(distance_min < distance_maintained)
	    {
		if(point_nearest.getX() >= 0)
		{
		    if(desired_mobile_platform_position > x0(6, 0)+std::abs(point_nearest.getX())-sqrt(distance_maintained*distance_maintained-distance_y*distance_y)-0.465 && desired_mobile_platform_position > xMin(6, 0)+0.020001)
		    {
		    	desired_mobile_platform_position = x0(6, 0)+std::abs(point_nearest.getX())-sqrt(distance_maintained*distance_maintained-distance_y*distance_y)-0.465;

                        if(desired_mobile_platform_position <= xMin(6, 0))
                            desired_mobile_platform_position = xMin(6, 0)+0.02;

			starting_mobile_platform_position = x0(6, 0);

			trajectory_total_t_mob_plat = 2;
			trajectory_start_t_mob_plat = ros::Time::now().toSec();
		    }
		}   
		else
		{
		    if(desired_mobile_platform_position < x0(6, 0)+sqrt(distance_maintained*distance_maintained-distance_y*distance_y)+0.315-std::abs(point_nearest.getX()) && desired_mobile_platform_position < xMax(6, 0)-0.020001)
		    {
		    	desired_mobile_platform_position = x0(6, 0)+sqrt(distance_maintained*distance_maintained-distance_y*distance_y)+0.315-std::abs(point_nearest.getX());

			if(desired_mobile_platform_position >= xMax(6, 0))
	                    desired_mobile_platform_position = xMax(6, 0)-0.02;

			starting_mobile_platform_position = x0(6, 0);

			trajectory_total_t_mob_plat = 2;
			trajectory_start_t_mob_plat = ros::Time::now().toSec();
		    }
		}         	
	    }        
	}
    }
}*/

void ModelPredictiveControl::obstacles_detection_callback(const std_msgs::Float64MultiArray& obs_det_output)
{
    tf::StampedTransform camera_tf, current_mobile_platform_pose;
    tf::Vector3 x_axis_vec, displacement;
    double mob_plat_position;//, base_link_to_camera_length;

    while(1)
    {
        try
        {
            listener.lookupTransform("/base_link", "/realsense_camera_depth_optical_frame", ros::Time(0), camera_tf);
	    listener.lookupTransform("/odom", "/base_footprint", ros::Time(0), current_mobile_platform_pose);
            break;
        }
        catch (tf::TransformException &ex)
        {
            ROS_ERROR("%s",ex.what());
            ros::Duration(0.05).sleep();
        }
    }

    x_axis_vec = current_mobile_platform_pose.getBasis().getColumn(0);

    displacement = current_mobile_platform_pose.getOrigin();
    displacement-= initial_mobile_platform_pose.getOrigin();

    mob_plat_position = copysign(1.0, x_axis_vec.dot(displacement))*displacement.length();

    if(obs_det_output.data[1] < 0.00001)
    {   
        if(desired_mobile_platform_position > mob_plat_position+obs_det_output.data[1] && desired_mobile_platform_position > xMin(6, 0)+10.05001)
        {
            desired_mobile_platform_position = mob_plat_position+obs_det_output.data[1];

            if(desired_mobile_platform_position <= xMin(6, 0)+10.05)
                desired_mobile_platform_position = xMin(6, 0)+10.05;

	    starting_mobile_platform_position = mob_plat_position;

	    trajectory_total_t_mob_plat = (starting_mobile_platform_position-desired_mobile_platform_position)/0.5*4;
	    trajectory_start_t_mob_plat = ros::Time::now().toSec();
            trajectory_exec_t_mob_plat = 0;
        }
    }
    else if(obs_det_output.data[1] > 0.00001)
    {
        if(desired_mobile_platform_position < mob_plat_position+obs_det_output.data[1] && desired_mobile_platform_position < xMax(6, 0)-10.050001)
        {
            desired_mobile_platform_position = mob_plat_position+obs_det_output.data[1];

            if(desired_mobile_platform_position >= xMax(6, 0)-10.05)
                desired_mobile_platform_position = xMax(6, 0)-10.05;

	    starting_mobile_platform_position = mob_plat_position;

	    trajectory_total_t_mob_plat = (desired_mobile_platform_position-starting_mobile_platform_position)/0.5*4;
	    trajectory_start_t_mob_plat = ros::Time::now().toSec();
            trajectory_exec_t_mob_plat = 0;
        }
    }

    /*base_link_to_camera_length = camera_tf.getOrigin().getX();

    xMax(6, 0) = mob_plat_position+base_link_to_camera_length+obs_distance.data-0.465-0.1;

    if(xMax(6, 0) > 0.5)
    {
	xMax(6, 0) = 0.5;
    }

    std::cout << "xMax: " << xMax(6, 0) << std::endl;*/
}

/*void ModelPredictiveControl::realsense_intrinsic_matrix_callback(const sensor_msgs::CameraInfo& cam)
{
    realsense_intrinsic_matrix.width = cam.width;
    realsense_intrinsic_matrix.height = cam.height;
    realsense_intrinsic_matrix.ppx = cam.K[2];
    realsense_intrinsic_matrix.ppy = cam.K[5];
    realsense_intrinsic_matrix.fx = cam.K[0];
    realsense_intrinsic_matrix.fy = cam.K[4];
    realsense_intrinsic_matrix.model = RS2_DISTORTION_NONE;
    //ri.coeffs[0] = cam.D[0];
    //ri.coeffs[1] = cam.D[1];
    //ri.coeffs[2] = cam.D[2];
    //ri.coeffs[3] = cam.D[3];
    //ri.coeffs[4] = cam.D[4];
}*/

void ModelPredictiveControl::updateMPCx0()
{
    tf::StampedTransform current_mobile_platform_pose;
    tf::Vector3 x_axis_vec, displacement;

    try
    {
        listener.lookupTransform("/odom", "/base_footprint", ros::Time(0), current_mobile_platform_pose);
    }
    catch (tf::TransformException &ex)
    {
        ROS_ERROR("%s",ex.what());
        ros::Duration(0.05).sleep();
    }

    x_axis_vec = current_mobile_platform_pose.getBasis().getColumn(0);

    displacement = current_mobile_platform_pose.getOrigin();
    displacement-= initial_mobile_platform_pose.getOrigin();

    x0 << current_joint_state.position[0], current_joint_state.position[1], current_joint_state.position[2], current_joint_state.position[3], current_joint_state.position[4], current_joint_state.position[5], copysign(1.0, x_axis_vec.dot(displacement))*displacement.length();
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
    return ((-3)*ini_v+6)*pow(t, 5)+(8*ini_v-15)*pow(t, 4)+((-6)*ini_v+10)*pow(t, 3)+ini_v*t;
}

void ModelPredictiveControl::stop()
{
    geometry_msgs::Twist mobile_platform_velocity_cmd;
    std_msgs::Float64MultiArray joint_velocity_cmd;

    joint_velocity_cmd.data.resize(6);

    for(int i = 0; i < 6; i++)
    {    
        joint_velocity_cmd.data[i] = 0;
    }
    
    mobile_platform_velocity_cmd.linear.x = 0;
    
    joint_velocity_pub.publish(joint_velocity_cmd);
    mobile_platform_velocity_pub.publish(mobile_platform_velocity_cmd);
}

bool ModelPredictiveControl::reachDesiredPose(const tf::StampedTransform& ee_desired_pose, const bool& pose_is_global)
{
    int reach_goal_count;
    geometry_msgs::Twist mobile_platform_velocity_cmd;
    std_msgs::Float64MultiArray joint_velocity_cmd;
    Eigen::VectorXd ctr, QPSolution;
    
    joint_velocity_cmd.data.resize(6);

    while(1)
    {
    	try
   	{
	    if(pose_is_global)
	    {
            	listener.lookupTransform("/odom", "/tm_tool0", ros::Time(0), pose0);
            	break;
	    }
	    else
	    {
		listener.lookupTransform("/tm_base_link", "/tm_tool0", ros::Time(0), pose0);
            	break;
	    }
    	}
    	catch (tf::TransformException &ex)
    	{
            ROS_ERROR("%s",ex.what());
            ros::Duration(0.05).sleep();
        }
    }

    desired_pose_is_global = pose_is_global;

    posef = ee_desired_pose;

    ros::spinOnce();

    updateMPCx0();

    //if(inverseKinematic(ee_desired_pose, ik_sols) == (-1))
    //{
    //    return false;
    //}

    //xRef << ik_sols[0], ik_sols[1], ik_sols[2], ik_sols[3], ik_sols[4], ik_sols[5], 0;

    trajectory_total_t = 10;
    trajectory_exec_t = 0;

    castMPCToQPHessian();
    castMPCToQPConstraintMatrix();
    castMPCToQPConstraintVectors();

    // set the initial data of the QP solver
    if(!solver.updateHessianMatrix(hessianMatrix)) return false;
    if(!solver.updateLinearConstraintsMatrix(constraintMatrix)) return false;
    if(!solver.updateBounds(lowerBound, upperBound)) return false;

    trajectory_start_t = ros::Time::now().toSec();

    if(castMPCToQPGradient() == (-1))
    {
    	return false;
    }

    if(!solver.updateGradient(gradient)) return false;

    while(ros::ok())
    {
        //std::cout << lowerBound << std::endl << std::endl;
        //std::cout << upperBound << std::endl << std::endl;  

        //std::cout << constraintMatrix << std::endl;

        //std::cout << x0 << std::endl;

        // solve the QP problem
        if(!solver.solve()) return false;

        // get the controller input
        QPSolution = solver.getSolution();
        ctr = QPSolution.block(7*(mpcWindow+1), 0, 7, 1);

	reach_goal_count = 0;

        if((x0[6] - xf[6]) < 0.005 && (x0[6] - xf[6]) > -0.005)
        {
            mobile_platform_velocity_cmd.linear.x = 0;
            reach_goal_count++;
        }
        else
        {
            mobile_platform_velocity_cmd.linear.x = ctr(6);
        }

	if((x0 - xf).block<6, 1>(0, 0).norm() < 0.001)
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
	    }
    	}
    	else
    	{
            for(int i = 0; i < 6; i++)
            {
            	joint_velocity_cmd.data[i] = ctr(i);         
            }
   	}

        std::cout << ctr(6) << std::endl<<std::endl;
        std::cout << x0 <<std::endl<<std::endl;
        std::cout << xf <<std::endl;

    	joint_velocity_pub.publish(joint_velocity_cmd);
    	mobile_platform_velocity_pub.publish(mobile_platform_velocity_cmd);

    	if(reach_goal_count == 2)
    	{
            break;
    	}

	loop_rate.sleep();

	// save data into file
        //auto x0Data = x0.data();

        // propagate the model
	ros::spinOnce();

	updateMPCx0();

	//std::cout << std::endl << ctr <<std::endl;
	//std::cout << std::endl << x0 << std::endl;

	if(castMPCToQPGradient() == (-1))
        {
            return false;
        }
        if(!solver.updateGradient(gradient)) return false;

        // update the constraint bound
        castMPCToQPConstraintVectors();
        if(!solver.updateBounds(lowerBound, upperBound)) return false;
    }

    return true;
}

void ModelPredictiveControl::performReplenishment()
{
    std_msgs::Bool apriltag_detection_enable;
    tf::StampedTransform apriltag_pose, ee_desired_pose; 

    tf::StampedTransform detection_pose;
    detection_pose.setData(tf::Transform(tf::Quaternion(0.7, 0, 0, 0.7), tf::Vector3(-0.12, -0.36, 0.3)));

    /*while(1)
    {
	ros::spinOnce();
    }*/

    int ccc = 0;

    tf::StampedTransform test;

    while(1)
    {
    	try
    	{
            listener.lookupTransform("/odom", "/tm_base_link", ros::Time(0), test);
	    break;
        }
    	catch (tf::TransformException &ex)
    	{
            ROS_ERROR("%s",ex.what());
            ros::Duration(0.05).sleep();
    	}
    }

    while(ccc<3)
    {
	detection_pose.setData(test*tf::Transform(tf::Quaternion(0.7, 0, 0, 0.7), tf::Vector3(-0.12, -0.36, 0.3)));

	if(!reachDesiredPose(detection_pose, true))
        {
            ROS_INFO("Fail to reach the desired pose");
            stop();
            return;
        }

	detection_pose.setData(test*tf::Transform(tf::Quaternion(0.7, 0, 0, 0.7), tf::Vector3(-0.12, -0.36, 0.4)));

	if(!reachDesiredPose(detection_pose, true))
        {   
            ROS_INFO("Fail to reach the desired pose");
            stop();
            return;
        }

        ccc++;
    }

    return;

    if(!reachDesiredPose(detection_pose, false))
    {
        ROS_INFO("Fail to reach the desired pose");
        stop();
        return;
    }
    
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
            listener.lookupTransform("/tm_base_link", "/at10", ros::Time(0), apriltag_pose);
            break;
    	}
    	catch (tf::TransformException &ex)
    	{
            ROS_ERROR("%s",ex.what());
            ros::Duration(0.05).sleep();
        }
    }

    apriltag_detection_enable.data = false;
    apriltag_detection_cmd_pub.publish(apriltag_detection_enable);

    ee_desired_pose = apriltag_pose;

    tf::StampedTransform bias;
    bias.setData(tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0, 0, 0.25)));

    ee_desired_pose*=bias;

    tf::Quaternion qu;
    qu.setEuler(M_PI, 0, 0.0); //YXZ
    bias.setData(tf::Transform(qu, tf::Vector3(0.0, 0.0, 0.0)));

    ee_desired_pose*=bias;

    if(!reachDesiredPose(ee_desired_pose, false))
    {
        ROS_INFO("Fail to reach the desired pose");
        stop();
        return;
    } 

    bias.setData(tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0, 0, 0.1)));

    ee_desired_pose*=bias;

    if(!reachDesiredPose(ee_desired_pose, false)) 
    {
        ROS_INFO("Fail to reach the desired pose");
        stop();
        return;
    } 
}
