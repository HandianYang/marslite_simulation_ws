#ifndef _MPC_H_
#define _MPC_H_

#include <cmath>
#include <algorithm>

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

// #define M_PI 3.14159265358979323846

class ModelPredictiveControl {
public:
	explicit ModelPredictiveControl(const ros::NodeHandle& nh = ros::NodeHandle());
	~ModelPredictiveControl();

	void performReplenishment();

private:
	bool inverseKinematic(const tf::StampedTransform& transformation, double ik_sols[]);
	bool inverseKinematic(const tf::Transform& transformation, double ik_sols[]);

	double trajectoryPlanning(double ini_v, double t);
	double trajectoryPlanning(double ini_v, double ini_a, double t);

	void updateMPCx0();
	void stop();
	bool reachDesiredPose(const tf::StampedTransform& ee_desired_pose, const bool& pose_is_global);

	/* ************************* *
	 *       MPC parameters      *
	 * ************************* */
	float mpcTimeStep_;			// sample time
	int mpcWindow_;				// preview window (prediction horizon)

	Eigen::Matrix<double, 7, 7> A_;				// dynamic matrix
	Eigen::Matrix<double, 7, 7> B_;				// control matrix
	Eigen::DiagonalMatrix<double, 7> Q_, R_;	// weight matrices
	Eigen::Matrix<double, 7, 1> xMax_, xMin_;	// state inequality constraints
	Eigen::Matrix<double, 7, 1> uMax_, uMin_;	// input inequality constraints
	Eigen::Matrix<double, 7, 1> aMax_, aMin_;	// input inequality constraints
	Eigen::Matrix<double, 7, 1> x0_;			// initial state space
	Eigen::Matrix<double, 7, 1> xf_;			// reference state space

	/// ?
	double trajectoryTotalTime_;
	double trajectoryStartTime_;
	double trajectoryExecTime_;

	/// ?
	double mobilePlatformTrajectoryTotalTime_;
	double mobilePlatformTrajectoryStartTime_;
	double mobilePlatformTrajectoryExecTime_;

	double mobilePlatformTrajectoryInitialVelcity_;
	geometry_msgs::Twist mobilePlatformCurrentTwist_;

	tf::StampedTransform mobilePlatformInitialPose_;	// "/odom" <- "/base_footprint"
	tf::StampedTransform pose0_;	// "/tm_base_link" <- "/tm_tool0"
	tf::StampedTransform posef_;	

	/// ?
	double mobilePlatformBeginPosition_;
	double mobilePlatformDesiredPosition_;

	/// flags
	bool apriltagDetected_;				// whether any apriltag is detected
	bool obstaclesDetectionEnabled_;	// whether the obstacle detection feature is enabled
	bool onSpot_;					// ???
	bool desiredPoseIsGlobal_;		// ???
	int callbackOrder_;				// ???

	/// QP problem parameters
	Eigen::SparseMatrix<double> hessianMatrix_;
	Eigen::SparseMatrix<double> constraintMatrix_;
	Eigen::VectorXd gradient_, lowerBound_, upperBound_;

	/// config parameters (constant)
	double distanceWarningField_, distanceProtectiveField_;
	/// config parameters 
	double decFactor_, decFactorPrevious_;

	OsqpEigen::Solver solver_;

	/// ROS related
	ros::NodeHandle nh_;
	ros::Publisher gripperPublisher_;
	ros::Publisher jointVelocityPublisher_;
	ros::Publisher mobilePlatformVelocityPublisher_;
	ros::Publisher apriltagDetectionCmdPublisher_;
	ros::Publisher des_ee_tra_pub;
	ros::Publisher ee_tra_pub;
	ros::Publisher des_ee_state_pub;
	ros::Publisher ee_state_pub;
	ros::Publisher robotStatePublisher_;
	ros::Publisher robotVelocityPublisher_;
	ros::Subscriber jointStateSubscriber_;
	ros::Subscriber mobilePlatformTwistSubscriber_;
	ros::Subscriber apriltagDetectionSubscriber_;
	ros::Subscriber obstaclesDetectionSubscriber_;
	ros::Rate loopRate_;
	
	tf::TransformListener tfListener_;

	/// 
	sensor_msgs::JointState currentJointState_;

	nav_msgs::Path des_ee_tra;
	//rs2_intrinsics realsense_intrinsic_matrix;

private:
	/* ************************* *
	 *       MPC functions       *
	 * ************************* */

	/**
	 * @brief Populate the hessian matrix of the QP problem.
	 * @note This function requires `Q_`, `R_`, and `mpcWindow_` to work properly. Make sure that
	 * 		these class members are well declared and assigned values before invoking this function.
	 * @note [About hessian matrix] The size of the hessian matrix is `7(w+1)+7w` by `7(w+1)+7w`, where
	 *  `w` is the size of the preview window. The hessian matrix is formed as `diag(Q,...,Q,R,...,R)`.
	*/
	void castMPCToQPHessian();

	/**
	 * @brief Populate the gradient vector of the QP problem.
	 * @note This function requires 
	*/
	int  castMPCToQPGradient();
	void castMPCToQPConstraintMatrix();
	void castMPCToQPConstraintVectors();

	/// callback functions
	void jointStateCB(const sensor_msgs::JointStateConstPtr& jointStatePtr);
	void mobilePlatformTwistCB(const geometry_msgs::TwistConstPtr& twistPtr);
	void apriltagDetectionCB(const apriltags_ros::AprilTagDetectionArrayConstPtr& detectionPtr);
	void obstaclesDetectionCB(const std_msgs::Float64MultiArrayConstPtr& detectionPtr);
};

}	// namespace mpc

#endif	// #define _MPC_H_