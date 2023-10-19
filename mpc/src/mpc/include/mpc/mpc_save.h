/**
 * @file MPCExample.cpp
 * @author Giulio Romualdi
 * @copyright Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 * @date 2018
 */

// osqp-eigen
#include "OsqpEigen/OsqpEigen.h"

// eigen
#include <Eigen/Dense>

#include <cv_bridge/cv_bridge.h>
#include <librealsense2/h/rs_types.h>

#include <ros/ros.h>
#include <tf/transform_listener.h>

// msg
#include <std_msgs/Float64MultiArray.h>
#include <sensor_msgs/JointState.h>
#include <apriltags_ros/AprilTagDetectionArray.h>
#include <darknet_ros_msgs/BoundingBoxesWithDepthImage.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <geometry_msgs/Twist.h>

#define M_PI 3.14159265358979323846

class ModelPredictiveControl
{
    public:
        ModelPredictiveControl(ros::NodeHandle& nh);
        ~ModelPredictiveControl();

        void performReplenishment();
    private:  
	void castMPCToQPHessian();
        int castMPCToQPGradient();
        void castMPCToQPConstraintMatrix();
        void castMPCToQPConstraintVectors();

        void joint_state_callback(const sensor_msgs::JointState& joint_state);
	void mobile_platform_velocity_callback(const geometry_msgs::Twist& vel);
        void apriltag_detection_callback(const apriltags_ros::AprilTagDetectionArray& detection);
        //void yolo_detection_callback(const darknet_ros_msgs::BoundingBoxesWithDepthImage& detection);
        void obstacles_detection_callback(const std_msgs::Float64MultiArray& obs_det_output);
        //void realsense_intrinsic_matrix_callback(const sensor_msgs::CameraInfo& cam);

        int inverseKinematic(const tf::StampedTransform& transformation, double ik_sols[]);
	int inverseKinematic(const tf::Transform& transformation, double ik_sols[]);

        double trajectoryPlanning(double ini_v, double t);

        void updateMPCx0();
        void stop();
        bool reachDesiredPose(const tf::StampedTransform& ee_desired_pose, const bool& pose_is_global);

        //MPC problem parameters
        Eigen::Matrix<double, 7, 7> A, B;
        Eigen::Matrix<double, 7, 1> xMax, xMin, uMax, uMin, aMax, aMin, x0, xf;
        tf::StampedTransform pose0, posef;
	Eigen::DiagonalMatrix<double, 7> Q, R;
        float time_step;
	int mpcWindow;
        ros::Rate loop_rate;

        //Eigen::Matrix<double, 7, 1> initial_x0;
        double trajectory_total_t;
        double trajectory_start_t;
	double trajectory_total_t_mob_plat;
	double trajectory_start_t_mob_plat; 

        //QP problem parameters
        Eigen::SparseMatrix<double> hessianMatrix, constraintMatrix;
        Eigen::VectorXd gradient, lowerBound, upperBound;

        OsqpEigen::Solver solver;

        ros::Publisher joint_velocity_pub;
        ros::Publisher mobile_platform_velocity_pub;
        ros::Publisher apriltag_detection_cmd_pub;
        ros::Subscriber joint_state_sub;
	ros::Subscriber mobile_platform_velocity_sub;
        ros::Subscriber apriltag_detection_sub;
        //ros::Subscriber yolo_detection_sub;
        ros::Subscriber obstacles_detection_sub;
        //ros::Subscriber realsense_intrinsic_matrix_sub;
        tf::TransformListener listener;

	bool desired_pose_is_global;
	double starting_mobile_platform_position;
	double desired_mobile_platform_position;

        sensor_msgs::JointState current_joint_state;
        geometry_msgs::Twist current_mobile_platform_velocity;
        tf::StampedTransform initial_mobile_platform_pose;       

	bool apriltag_detected;

        //rs2_intrinsics realsense_intrinsic_matrix;
};
