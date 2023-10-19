#ifndef _MPC_H_
#define _MPC_H_

// osqp-eigen
#include "OsqpEigen/OsqpEigen.h"

// eigen
#include <Eigen/Dense>

//#include <cv_bridge/cv_bridge.h>
//#include <librealsense2/h/rs_types.h>

#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf/LinearMath/Matrix3x3.h>
#include <tf/LinearMath/Vector3.h>
#include <tf/transform_broadcaster.h>

// msg
#include <std_msgs/Float64MultiArray.h>
#include <sensor_msgs/JointState.h>
#include <apriltags_ros/AprilTagDetectionArray.h>
//#include <darknet_ros_msgs/BoundingBoxesWithDepthImage.h>
//#include <sensor_msgs/CameraInfo.h>
//#include <sensor_msgs/Image.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <detection_msgs/Det3D.h>
#include <detection_msgs/Det3DArray.h>


namespace mpc {

#define M_PI 3.14159265358979323846

class ModelPredictiveControl {
public:
	explicit ModelPredictiveControl(const ros::NodeHandle& nh = ros::NodeHandle());
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
	double trajectoryPlanning(double ini_v, double ini_a, double t);

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

	double trajectory_total_t, trajectory_start_t, trajectory_exec_t;
	double trajectory_total_t_mob_plat, trajectory_start_t_mob_plat, trajectory_exec_t_mob_plat;
	double trajectory_ini_v_mob_plat;

	bool on_spot;

	//QP problem parameters
	Eigen::SparseMatrix<double> hessianMatrix, constraintMatrix;
	Eigen::VectorXd gradient, lowerBound, upperBound;

	double distance_warning_field, distance_protective_field;
	double dec_factor, dec_factor_previous;

	OsqpEigen::Solver solver;

	ros::NodeHandle nh_;
	ros::Publisher gripper_pub;
	ros::Publisher joint_velocity_pub;
	ros::Publisher mobile_platform_velocity_pub;
	ros::Publisher apriltag_detection_cmd_pub;
	ros::Publisher des_ee_tra_pub;
	ros::Publisher ee_tra_pub;
	ros::Publisher des_ee_state_pub;
	ros::Publisher ee_state_pub;
	ros::Publisher robot_state_pub;
	ros::Publisher robot_vel_pub;
	ros::Subscriber joint_state_sub;
	ros::Subscriber mobile_platform_velocity_sub;
	ros::Subscriber apriltag_detection_sub;
	//ros::Subscriber yolo_detection_sub;
	ros::Subscriber obstacles_detection_sub;
	//ros::Subscriber realsense_intrinsic_matrix_sub;
	tf::TransformListener listener;

	bool desired_pose_is_global;
	double starting_mobile_platform_position, desired_mobile_platform_position;

	sensor_msgs::JointState current_joint_state;
	geometry_msgs::Twist current_mobile_platform_velocity;
	tf::StampedTransform initial_mobile_platform_pose;

	bool apriltag_detected;
	bool obstacles_detection_enable;

	int callback_order;

	nav_msgs::Path des_ee_tra;
	//rs2_intrinsics realsense_intrinsic_matrix;
};

}	// namespace mpc

#endif	// #define _MPC_H_