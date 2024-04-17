#include "mpc/mpc.h"
#include "mpc/tm_kin.h"

// #include <std_msgs/Bool.h>

#include <iostream>

namespace mpc {

ModelPredictiveControl::ModelPredictiveControl(const ros::NodeHandle& nh) 
    : nh_(nh), loopRate_(25)
{
    /// config parameters (constant)
    distanceWarningField_ = 2;
    distanceProtectiveField_ = 1.5;

    /// config parameters 
    decFactor_ = 1;
    decFactorPrevious_ = 1;

    /// publisher
    gripperPublisher_ = nh_.advertise<bool>("/gripper/cmd_gripper", 1);
    jointVelocityPublisher_ = nh_.advertise<std_msgs::Float64MultiArray>("/velocity_cmd", 1);
    mobilePlatformVelocityPublisher_ = nh_.advertise<geometry_msgs::Twist>("/mob_plat/cmd_vel", 1);
    apriltagDetectionCmdPublisher_ = nh_.advertise<bool>("/apriltag_detection_enable", 1);
    des_ee_tra_pub = nh_.advertise<nav_msgs::Path>("/des_ee_tra", 1);
    ee_tra_pub = nh_.advertise<nav_msgs::Path>("/ee_tra", 1);
    des_ee_state_pub = nh_.advertise<std_msgs::Float64MultiArray>("/des_ee_state", 1);
    ee_state_pub = nh_.advertise<std_msgs::Float64MultiArray>("/ee_state", 1);
    robotStatePublisher_ = nh_.advertise<std_msgs::Float64MultiArray>("/robot_state", 1);
    robotVelocityPublisher_ = nh_.advertise<std_msgs::Float64MultiArray>("/robot_vel", 1);

    /// subscriber
    jointStateSubscriber_ = nh_.subscribe("/tm_joint_states", 1, &ModelPredictiveControl::jointStateCB, this);
    mobilePlatformTwistSubscriber_ = nh_.subscribe("/mob_plat/curr_vel", 1, &ModelPredictiveControl::mobilePlatformTwistCB, this);
    apriltagDetectionSubscriber_ = nh_.subscribe("/tag_detections", 1, &ModelPredictiveControl::apriltagDetectionCB, this);   
    obstaclesDetectionSubscriber_ = nh_.subscribe("/obs_det_output", 1, &ModelPredictiveControl::obstaclesDetectionCB, this);

    /// joint_state
    currentJointState_.position.resize(6);
    currentJointState_.velocity.resize(6);

    /// flags
    apriltagDetected_ = false;
    obstaclesDetectionEnabled_ = true;

    /// ???
    callbackOrder_ = 0;

    do {
        try {
            tfListener_.lookupTransform("/odom", "/base_footprint", ros::Time(0), mobilePlatformInitialPose_);
            tfListener_.lookupTransform("/tm_base_link", "/tm_tool0", ros::Time(0), pose0_);
        } catch (tf::TransformException &ex) {
            ROS_ERROR("%s", ex.what());
            ros::Duration(0.5).sleep();
            continue;
        }
    } while (false);

    /// MPC parameters (constant)
    mpcTimeStep_ = 0.04;
    mpcWindow_ = 6;

    // Initialize dynamic matrix
    A_ << 1., 0., 0., 0., 0., 0., 0., 
          0., 1., 0., 0., 0., 0., 0., 
          0., 0., 1., 0., 0., 0., 0., 
          0., 0., 0., 1., 0., 0., 0., 
          0., 0., 0., 0., 1., 0., 0., 
          0., 0., 0., 0., 0., 1., 0., 
          0., 0., 0., 0., 0., 0., 1.; 
    
    // Initialize control matrix
    B_ << mpcTimeStep_, 0., 0., 0., 0., 0., 0., 
          0., mpcTimeStep_, 0., 0., 0., 0., 0., 
          0., 0., mpcTimeStep_, 0., 0., 0., 0., 
          0., 0., 0., mpcTimeStep_, 0., 0., 0., 
          0., 0., 0., 0., mpcTimeStep_, 0., 0., 
          0., 0., 0., 0., 0., mpcTimeStep_, 0., 
          0., 0., 0., 0., 0., 0., mpcTimeStep_;
    
    // Initialize weight matrices
    Q_.diagonal() << 1., 1., 1., 1., 1., 1., 1.; 
    R_.diagonal() << 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01;

    // Initialize state inequality constraints
    xMax_ << 270./180*M_PI,
             180./180*M_PI,
             155./180*M_PI,
             180./180*M_PI,
             180./180*M_PI,
             270./180*M_PI,
             10;
    xMin_ << (-270.)/180*M_PI,
             (-180.)/180*M_PI,
             (-155.)/180*M_PI,
             (-180.)/180*M_PI,
             (-180.)/180*M_PI,
             (-270.)/180*M_PI,
             (-10);

    // Initialize input inequality constraints
    uMax_ << 180./180*M_PI,
             180./180*M_PI,
             180./180*M_PI,
             225./180*M_PI,
             225./180*M_PI,
             225./180*M_PI,
             0.5;
    uMin_ << (-180.)/180*M_PI,
             (-180.)/180*M_PI,
             (-180.)/180*M_PI,
             (-225.)/180*M_PI,
             (-225.)/180*M_PI,
             (-225.)/180*M_PI,
             (-0.5);
    aMax_ << 80./180*M_PI,
             80./180*M_PI,
             80./180*M_PI,
             80./180*M_PI,
             80./180*M_PI,
             80./180*M_PI,
             2;
    aMin_ << (-80.)/180*M_PI,
             (-80.)/180*M_PI,
             (-80.)/180*M_PI,
             (-80.)/180*M_PI,
             (-80.)/180*M_PI,
             (-80.)/180*M_PI,
             -2;

    // Initialize initial state space
    x0_ << 0, 0, 0, 0, 0, 0, 0;
    posef_ = pose0_;

    trajectoryTotalTime_ = 1;
    trajectoryStartTime_ = ros::Time::now().toSec();
    trajectoryExecTime_ = 0;

    mobilePlatformTrajectoryTotalTime_ = 1;
    mobilePlatformTrajectoryStartTime_ = ros::Time::now().toSec();
    mobilePlatformTrajectoryExecTime_ = 0;
    mobilePlatformTrajectoryInitialVelcity_ = 0;

    mobilePlatformBeginPosition_ = 0;
    mobilePlatformDesiredPosition_ = 0;

    onSpot_ = false;
    desiredPoseIsGlobal_ = false;

    castMPCToQPHessian();
    castMPCToQPGradient();
    castMPCToQPConstraintMatrix();
    castMPCToQPConstraintVectors();

    // settings
    //solver_.settings()->setVerbosity(false);
    solver_.settings()->setWarmStart(true);
    solver_.settings()->setRelativeTolerance(0.000001);
    //solver_.settings()->setPrimalInfeasibilityTollerance(0.001);

    // set the initial data of the QP solver_
    solver_.data()->setNumberOfVariables(7*(mpcWindow_+1)+7*mpcWindow_);
    solver_.data()->setNumberOfConstraints(2*7*(mpcWindow_+1)+2*7*mpcWindow_);
    solver_.data()->setHessianMatrix(hessianMatrix_);
    solver_.data()->setGradient(gradient_);
    solver_.data()->setLinearConstraintsMatrix(constraintMatrix_);
    solver_.data()->setLowerBound(lowerBound_);
    solver_.data()->setUpperBound(upperBound_);

    // instantiate the solver
    solver_.initSolver();
}

ModelPredictiveControl::~ModelPredictiveControl()
{
    stop();

    std::cout << "destructor is called" << std::endl;
}

void ModelPredictiveControl::castMPCToQPHessian()
{
    hessianMatrix_.resize(7*(mpcWindow_+1)+7*mpcWindow_, 7*(mpcWindow_+1)+7*mpcWindow_);

    double value;
    for (int i = 0; i < 7*(mpcWindow_+1)+7*mpcWindow_; ++i) {
        if (i < 7*(mpcWindow_+1))
            value = Q_.diagonal()[i%7];
        else
            value = R_.diagonal()[i%7];
        
        if (value != 0)
            hessianMatrix_.insert(i,i) = value;
    }
}

int ModelPredictiveControl::castMPCToQPGradient()
{
    // A_ trajectory is planned here to be the reference trajectory.
    //input:Q_, pose0_, posef_, mpcWindow_, trajectoryTotalTime_, trajectoryStartTime_;
    Eigen::Matrix<double, 7, 1> xRef, Qx_ref, last_xRef;
    tf::StampedTransform poseRef;
    double ik_sols[48];     // 6 joints * 8 IK-solutions

    //when desiredPoseIsGlobal_ is true, these variables are used;
    tf::StampedTransform armBasePose;
    tf::StampedTransform mobilePlatformCurrentPose;
    tf::StampedTransform mobilePlatformDesiredPose;
    tf::StampedTransform mobilePlatformRefPose;
    tf::Vector3 x_axis_vec;     // x axis of rotational element of mobilePlatformCurrentPose
    std_msgs::Float64MultiArray des_ee_state;
    des_ee_state.data.resize(6);

    trajectoryExecTime_ += (ros::Time::now().toSec()-trajectoryStartTime_)*decFactorPrevious_/**shift_factor_previous*/;
    trajectoryStartTime_ = ros::Time::now().toSec();

    mobilePlatformTrajectoryExecTime_ += (ros::Time::now().toSec()-mobilePlatformTrajectoryStartTime_)*decFactorPrevious_;
    mobilePlatformTrajectoryStartTime_ = ros::Time::now().toSec();

    if (desiredPoseIsGlobal_) {
        if (trajectoryExecTime_ > trajectoryTotalTime_) onSpot_ = true;

        do {
            try {
                tfListener_.lookupTransform("/odom", "/base_footprint", ros::Time(0), mobilePlatformCurrentPose);
                tfListener_.lookupTransform("/base_footprint", "/tm_base_link", ros::Time(0), armBasePose);
            }
            catch (tf::TransformException &ex) {
                ROS_ERROR("%s",ex.what());
                ros::Duration(0.5).sleep();
                continue;
            }
        } while (false);

	    x_axis_vec = mobilePlatformCurrentPose.getBasis().getColumn(0);

        /// Set the desired pose of mobile platform
        ///     Rotation: same as that of current pose
        ///     Translation: displacement on x axis from the current pose
        mobilePlatformDesiredPose.setBasis(mobilePlatformCurrentPose.getBasis());
        mobilePlatformDesiredPose.setOrigin(mobilePlatformCurrentPose.getOrigin()
                + (mobilePlatformDesiredPosition_ - x0_(6, 0)) * x_axis_vec);

        if (inverseKinematic(armBasePose.inverse() * mobilePlatformDesiredPose.inverse() * posef_, ik_sols))
            return -1;

    } else {
	    if (inverseKinematic(posef_, ik_sols))
            return -1;
    }

    xf_ << ik_sols[0], ik_sols[1], ik_sols[2], ik_sols[3], ik_sols[4], ik_sols[5], mobilePlatformDesiredPosition_;

    double ref_vel;
    tf::StampedTransform mobilePlatformRefPosePrev = mobilePlatformCurrentPose;
    double last_xRef_6 = x0_[6];
    double stop_dec_ratio = 1./16;
    geometry_msgs::PoseStamped tra_pos;

    // populate the gradient_ vector
    gradient_ = Eigen::VectorXd::Zero(7*(mpcWindow_+1)+7*mpcWindow_, 1);
    for (int i = 0; i < mpcWindow_+1; ++i) {
        /// ?    
        if (trajectoryExecTime_ + i * mpcTimeStep_ * decFactor_ < trajectoryTotalTime_) {
            poseRef.setData(
                tf::Transform(
                    pose0_.getRotation().slerp(posef_.getRotation(), trajectoryPlanning(0, 0, (trajectoryExecTime_ + i * mpcTimeStep_ * decFactor_) / trajectoryTotalTime_)), 
                    pose0_.getOrigin().lerp(   posef_.getOrigin(),   trajectoryPlanning(0, 0, (trajectoryExecTime_ + i * mpcTimeStep_ * decFactor_) / trajectoryTotalTime_))
                )
            );
            
            if (desiredPoseIsGlobal_) {
                if (onSpot_ || mobilePlatformTrajectoryExecTime_ + i * mpcTimeStep_ * decFactor_ < mobilePlatformTrajectoryTotalTime_) {

                    mobilePlatformRefPose.setBasis(mobilePlatformCurrentPose.getBasis());

                    if (onSpot_) {
                        if (mobilePlatformCurrentTwist_.linear.x >= 0) {
                            ref_vel = std::max(0.0, mobilePlatformCurrentTwist_.linear.x + i * mpcTimeStep_ * aMin_[6] * stop_dec_ratio);
                        } else {
                            ref_vel = std::min(0.0, mobilePlatformCurrentTwist_.linear.x + i * mpcTimeStep_ * aMax_[6] * stop_dec_ratio);
                        }

                        if (i == 0) {
                            mobilePlatformRefPose = mobilePlatformRefPosePrev;
                        } else {
                            mobilePlatformRefPose.setOrigin(mobilePlatformRefPosePrev.getOrigin() + ref_vel * mpcTimeStep_ * x_axis_vec);
                        }

                        mobilePlatformRefPosePrev = mobilePlatformRefPose;
                    } else {
                        mobilePlatformRefPose.setOrigin( mobilePlatformCurrentPose.getOrigin() 
                            + (mobilePlatformBeginPosition_ + (mobilePlatformDesiredPosition_ - mobilePlatformBeginPosition_) * trajectoryPlanning(
                                mobilePlatformTrajectoryInitialVelcity_ / (0.5/7.5) * copysign(1.0, (mobilePlatformDesiredPosition_ - mobilePlatformBeginPosition_)),
                                (mobilePlatformTrajectoryExecTime_ + i * mpcTimeStep_ * decFactor_) / mobilePlatformTrajectoryTotalTime_
                            ) - x0_(6, 0)) * x_axis_vec
                        );
                    }

                    if (inverseKinematic(armBasePose.inverse() * mobilePlatformRefPose.inverse() * poseRef, ik_sols))
                        return -1;

                    if (onSpot_) {
                        if(i == 0) {
                            xRef << ik_sols[0], ik_sols[1], ik_sols[2], ik_sols[3], ik_sols[4], ik_sols[5], last_xRef_6;
                        } else {
                            xRef << ik_sols[0], ik_sols[1], ik_sols[2], ik_sols[3], ik_sols[4], ik_sols[5], last_xRef_6 + ref_vel * mpcTimeStep_;
                        }
                        
                        last_xRef_6 = xRef[6];
                    } else {
                        xRef << ik_sols[0], ik_sols[1], ik_sols[2], ik_sols[3], ik_sols[4], ik_sols[5], 
                                mobilePlatformBeginPosition_ + (mobilePlatformDesiredPosition_ - mobilePlatformBeginPosition_) * trajectoryPlanning(
                                    mobilePlatformTrajectoryInitialVelcity_ / (0.5/7.5) * copysign(1.0, (mobilePlatformDesiredPosition_ - mobilePlatformBeginPosition_)),
                                    (mobilePlatformTrajectoryExecTime_ + i * mpcTimeStep_ * decFactor_) / mobilePlatformTrajectoryTotalTime_
                                );
                    }
                } else {
                    if (inverseKinematic(armBasePose.inverse() * mobilePlatformDesiredPose.inverse() * poseRef, ik_sols))
                        return -1;

                    xRef << ik_sols[0], ik_sols[1], ik_sols[2], ik_sols[3], ik_sols[4], ik_sols[5], mobilePlatformDesiredPosition_;
                }
            } else {
                if (inverseKinematic(poseRef, ik_sols)) 
                    return -1;

                if (onSpot_ || mobilePlatformTrajectoryExecTime_ + i * mpcTimeStep_ * decFactor_ < mobilePlatformTrajectoryTotalTime_) {
                    if (mobilePlatformCurrentTwist_.linear.x >= 0) {
                        ref_vel = std::max(0.0, mobilePlatformCurrentTwist_.linear.x + i * mpcTimeStep_ * aMin_[6] * stop_dec_ratio);
                    } else {
                        ref_vel = std::min(0.0, mobilePlatformCurrentTwist_.linear.x + i * mpcTimeStep_ * aMax_[6] * stop_dec_ratio);
                    }

                    if(onSpot_) {
                        if(i == 0)
                            xRef << ik_sols[0], ik_sols[1], ik_sols[2], ik_sols[3], ik_sols[4], ik_sols[5], last_xRef_6;
                        else
                            xRef << ik_sols[0], ik_sols[1], ik_sols[2], ik_sols[3], ik_sols[4], ik_sols[5], last_xRef_6 + ref_vel * mpcTimeStep_;

                        last_xRef_6 = xRef[6];
                    }
                    else {
                        xRef << ik_sols[0], ik_sols[1], ik_sols[2], ik_sols[3], ik_sols[4], ik_sols[5],
                                mobilePlatformBeginPosition_ + (mobilePlatformDesiredPosition_ - mobilePlatformBeginPosition_) * trajectoryPlanning(
                                    mobilePlatformTrajectoryInitialVelcity_ / (0.5/7.5) * copysign(1.0, (mobilePlatformDesiredPosition_ - mobilePlatformBeginPosition_)),
                                    (mobilePlatformTrajectoryExecTime_ + i * mpcTimeStep_ * decFactor_) / mobilePlatformTrajectoryTotalTime_
                                );
                    }
                        
                } else {
                    xRef << ik_sols[0], ik_sols[1], ik_sols[2], ik_sols[3], ik_sols[4], ik_sols[5], mobilePlatformDesiredPosition_;
                }
            }

            if (i == 0) {
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
        } else {
            if (desiredPoseIsGlobal_) {
                if (onSpot_ || mobilePlatformTrajectoryExecTime_ + i * mpcTimeStep_ * decFactor_ < mobilePlatformTrajectoryTotalTime_) {

                    mobilePlatformRefPose.setBasis(mobilePlatformCurrentPose.getBasis());
                        
                    if (onSpot_) {
                        if (mobilePlatformCurrentTwist_.linear.x >= 0) {
                            ref_vel = std::max(0.0, mobilePlatformCurrentTwist_.linear.x + i * mpcTimeStep_ * aMin_[6] * stop_dec_ratio);
                        } else {
                            ref_vel = std::min(0.0, mobilePlatformCurrentTwist_.linear.x + i * mpcTimeStep_ * aMax_[6] * stop_dec_ratio);
                        }
                        
                        if (i == 0)
                            mobilePlatformRefPose = mobilePlatformRefPosePrev;
                        else
                            mobilePlatformRefPose.setOrigin(mobilePlatformRefPosePrev.getOrigin() + ref_vel * mpcTimeStep_ * x_axis_vec);

                        mobilePlatformRefPosePrev = mobilePlatformRefPose;
                    } else {
                        mobilePlatformRefPose.setOrigin( mobilePlatformCurrentPose.getOrigin()
                            + (mobilePlatformBeginPosition_ + (mobilePlatformDesiredPosition_ - mobilePlatformBeginPosition_) * trajectoryPlanning(
                                mobilePlatformTrajectoryInitialVelcity_ / (0.5/7.5) * copysign(1.0, (mobilePlatformDesiredPosition_ - mobilePlatformBeginPosition_)),
                                (mobilePlatformTrajectoryExecTime_ + i * mpcTimeStep_ * decFactor_) / mobilePlatformTrajectoryTotalTime_
                            ) - x0_(6, 0)) * x_axis_vec
                        );
                    }  

                    if (inverseKinematic(armBasePose.inverse() * mobilePlatformRefPose.inverse() * posef_, ik_sols))
                        return -1;

                    if (onSpot_) {
                        if (i == 0)
                            xRef << ik_sols[0], ik_sols[1], ik_sols[2], ik_sols[3], ik_sols[4], ik_sols[5], last_xRef_6;
                        else
                            xRef << ik_sols[0], ik_sols[1], ik_sols[2], ik_sols[3], ik_sols[4], ik_sols[5], last_xRef_6 + ref_vel * mpcTimeStep_;
                        last_xRef_6 = xRef[6];
                    } else {
                        xRef << ik_sols[0], ik_sols[1], ik_sols[2], ik_sols[3], ik_sols[4], ik_sols[5],
                                mobilePlatformBeginPosition_ + (mobilePlatformDesiredPosition_ - mobilePlatformBeginPosition_) * trajectoryPlanning(
                                    mobilePlatformTrajectoryInitialVelcity_ / (0.5/7.5) * copysign(1.0, (mobilePlatformDesiredPosition_ - mobilePlatformBeginPosition_)),
                                    (mobilePlatformTrajectoryExecTime_ + i * mpcTimeStep_ * decFactor_) / mobilePlatformTrajectoryTotalTime_
                                );
                    }     
                } else {
                    xRef << xf_;
                }
            } else {
                xRef = xf_;

                if (mobilePlatformCurrentTwist_.linear.x >= 0) {
                    ref_vel = std::max(0.0, mobilePlatformCurrentTwist_.linear.x + i * mpcTimeStep_ * aMin_[6] * stop_dec_ratio);
                } else {
                    ref_vel = std::min(0.0, mobilePlatformCurrentTwist_.linear.x + i * mpcTimeStep_ * aMax_[6] * stop_dec_ratio);
                }

                if (onSpot_ || mobilePlatformTrajectoryExecTime_ + i * mpcTimeStep_ * decFactor_ < mobilePlatformTrajectoryTotalTime_) {
                    if (onSpot_) {
                        if (i == 0)
                            xRef << ik_sols[0], ik_sols[1], ik_sols[2], ik_sols[3], ik_sols[4], ik_sols[5], last_xRef_6;
                        else
                            xRef << ik_sols[0], ik_sols[1], ik_sols[2], ik_sols[3], ik_sols[4], ik_sols[5], last_xRef_6 + ref_vel * mpcTimeStep_;
                        last_xRef_6 = xRef[6];
                    }
                    else {
                        xRef(6, 0) = mobilePlatformBeginPosition_ + (mobilePlatformDesiredPosition_ - mobilePlatformBeginPosition_) * trajectoryPlanning(
                            mobilePlatformTrajectoryInitialVelcity_ / (0.5/7.5) * copysign(1.0, (mobilePlatformDesiredPosition_ - mobilePlatformBeginPosition_)),
                            (mobilePlatformTrajectoryExecTime_ + i * mpcTimeStep_ * decFactor_) / mobilePlatformTrajectoryTotalTime_
                        );
                    }
                }
            }
        
            if (i == 0) {
                des_ee_state.data[0] = posef_.getOrigin().getX();
                des_ee_state.data[1] = posef_.getOrigin().getY();
                des_ee_state.data[2] = posef_.getOrigin().getZ();
                posef_.getBasis().getRPY(des_ee_state.data[3], des_ee_state.data[4], des_ee_state.data[5]);
    
                des_ee_state_pub.publish(des_ee_state);

                if (trajectoryExecTime_ < (trajectoryTotalTime_ + mpcTimeStep_)) {		
                    tra_pos.pose.position.x = posef_.getOrigin().getX();
                    tra_pos.pose.position.y = posef_.getOrigin().getY();
                    tra_pos.pose.position.z = posef_.getOrigin().getZ();
                    tra_pos.pose.orientation.x = posef_.getRotation().getX();
                    tra_pos.pose.orientation.y = posef_.getRotation().getY();
                    tra_pos.pose.orientation.z = posef_.getRotation().getZ();
                    tra_pos.pose.orientation.w = posef_.getRotation().getW();

                    des_ee_tra.poses.push_back(tra_pos);
                    des_ee_tra_pub.publish(des_ee_tra);
                }
            }
        }

        /// Calculate the gradient vector
        Qx_ref = Q_ * (-xRef);
        for (int j = 0; j < 7; ++j)
            gradient_(i*7+j,0) = Qx_ref(j, 0);

        /// ?
        if (i > 0 && (abs(last_xRef(0)-xRef(0)) > 1 || abs(last_xRef(1)-xRef(1)) > 1
                || abs(last_xRef(2)-xRef(2)) > 1 || abs(last_xRef(3)-xRef(3)) > 1
                || abs(last_xRef(4)-xRef(4)) > 1 || abs(last_xRef(5)-xRef(5)) > 1))  { return -2; }
        last_xRef = xRef;
    }

    // decFactorPrevious_ = decFactor_;
    return 0;
}

void ModelPredictiveControl::castMPCToQPConstraintMatrix()
{
    //input:A_, B_, mpcWindow_
    constraintMatrix_.resize(7*(mpcWindow_+1)+7*(mpcWindow_+1)+7*mpcWindow_+7*mpcWindow_, 7*(mpcWindow_+1)+7*mpcWindow_);

    // populate linear constraint matrix
    for (int i = 0; i < 7*(mpcWindow_+1); ++i)
        constraintMatrix_.insert(i,i) = -1;

    float value;
    for (int i = 0; i < mpcWindow_; ++i) {
        for (int j = 0; j < 7; ++j) {
            for (int k = 0; k < 7; ++k) {
                value = A_(j,k);
                if (value != 0)
                    constraintMatrix_.insert(7*(i+1)+j, 7*i+k) = value;
                
                value = B_(j,k);
                if (value != 0)
                    constraintMatrix_.insert(7*(i+1)+j, 7*i+k+7*(mpcWindow_+1)) = value;
            }
        }
    }
    
    for (int i = 0; i < 7*(mpcWindow_+1)+7*mpcWindow_; ++i) {
        constraintMatrix_.insert(i+7*(mpcWindow_+1), i) = 1;
    }

    /// NEW
    for (int i = 0; i < 7*mpcWindow_; ++i) {
        constraintMatrix_.insert(i+7*(mpcWindow_+1)*2+7*mpcWindow_, i+7*(mpcWindow_+1)) = 1./mpcTimeStep_;
    }

    /// NEW
    for (int i = 0; i < 7*(mpcWindow_-1); ++i) {
        constraintMatrix_.insert(i+7*(mpcWindow_+1)*2+7*mpcWindow_+7, i+7*(mpcWindow_+1)) = (-1.)/mpcTimeStep_;
    }
}

void ModelPredictiveControl::castMPCToQPConstraintVectors()
{
    //input:xMax_, xMin_, uMax_, uMin_, x0_, mpcWindow_
    // evaluate the lower and the upper inequality vectors
    Eigen::VectorXd lowerInequality = Eigen::MatrixXd::Zero(7*(mpcWindow_+1)+2*7*mpcWindow_, 1);
    Eigen::VectorXd upperInequality = Eigen::MatrixXd::Zero(7*(mpcWindow_+1)+2*7*mpcWindow_, 1);
    for (int i = 0; i < mpcWindow_+1; ++i) {
        lowerInequality.block(7*i, 0, 7, 1) = xMin_;
        upperInequality.block(7*i, 0, 7, 1) = xMax_;
    }
    for (int i = 0; i < mpcWindow_; ++i) {
        lowerInequality.block(7*i+7*(mpcWindow_+1), 0, 7, 1) = uMin_*decFactor_;
        upperInequality.block(7*i+7*(mpcWindow_+1), 0, 7, 1) = uMax_*decFactor_;
    }

    /// NEW
    for (int i = 0; i < mpcWindow_; ++i) {
        if (i == 0) {
            Eigen::Matrix<double, 7, 1> u_now;
            u_now << currentJointState_.velocity[0], currentJointState_.velocity[1], currentJointState_.velocity[2],
                     currentJointState_.velocity[3], currentJointState_.velocity[4], currentJointState_.velocity[5],
                     mobilePlatformCurrentTwist_.linear.x;

            lowerInequality.block(7*i+7*(mpcWindow_+1)+7*mpcWindow_, 0, 7, 1) = u_now/mpcTimeStep_+aMin_;
            upperInequality.block(7*i+7*(mpcWindow_+1)+7*mpcWindow_, 0, 7, 1) = u_now/mpcTimeStep_+aMax_;
        } else {
	        lowerInequality.block(7*i+7*(mpcWindow_+1)+7*mpcWindow_, 0, 7, 1) = aMin_;
            upperInequality.block(7*i+7*(mpcWindow_+1)+7*mpcWindow_, 0, 7, 1) = aMax_;
	    }
    }

    // evaluate the lower and the upper equality vectors
    Eigen::VectorXd lowerEquality = Eigen::MatrixXd::Zero(7*(mpcWindow_+1), 1);
    Eigen::VectorXd upperEquality;
    
    lowerEquality.block(0,0,7,1) = -x0_;
    upperEquality = lowerEquality;
    lowerEquality = lowerEquality;

    // merge inequality and equality vectors
    lowerBound_ = Eigen::MatrixXd::Zero(2*7*(mpcWindow_+1)+2*7*mpcWindow_, 1);
    lowerBound_ << lowerEquality, lowerInequality;

    upperBound_ = Eigen::MatrixXd::Zero(2*7*(mpcWindow_+1)+2*7*mpcWindow_, 1);
    upperBound_ << upperEquality, upperInequality;
}

void ModelPredictiveControl::jointStateCB(const sensor_msgs::JointStateConstPtr& jointStatePtr)
{
    for (int i = 0; i < 6; ++i) {
        currentJointState_.position[i] = jointStatePtr->position[i];
        currentJointState_.velocity[i] = jointStatePtr->velocity[i];
    }
}

void ModelPredictiveControl::mobilePlatformTwistCB(const geometry_msgs::TwistConstPtr& twistPtr)
{
    mobilePlatformCurrentTwist_.linear.x = twistPtr->linear.x;

    if (callbackOrder_ == 0) {
        callbackOrder_ = 1;
    } else if (callbackOrder_ == 2) {
        mobilePlatformTrajectoryInitialVelcity_ = mobilePlatformCurrentTwist_.linear.x;
    }

    // std::cout << "vel:" << mobilePlatformCurrentTwist_.linear.x << std::endl << std::endl;
}

void ModelPredictiveControl::apriltagDetectionCB(const apriltags_ros::AprilTagDetectionArrayConstPtr& detectionPtr)
{
    for (int i = 0; i < detectionPtr->detections.size(); ++i) {
        if (detectionPtr->detections[i].id == 13) {
            apriltagDetected_ = true;
            break;
	    }
    }
}

void ModelPredictiveControl::obstaclesDetectionCB(const std_msgs::Float64MultiArrayConstPtr& detectionPtr)
{
    if(!obstaclesDetectionEnabled_) return;

    tf::StampedTransform camera_tf, mobilePlatformCurrentPose;
    tf::Vector3 initial_x_axis_vec, current_x_axis_vec, displacement;
    double mob_plat_position;//, base_link_to_camera_length;

    do {
        try {
            // tfListener_.lookupTransform("/base_link", "/depth_camera_1_depth_optical_frame", ros::Time(0), camera_tf);
	        tfListener_.lookupTransform("/odom", "/base_footprint", ros::Time(0), mobilePlatformCurrentPose);
        } catch (tf::TransformException &ex) {
            ROS_ERROR("%s",ex.what());
            ros::Duration(0.5).sleep();
            continue;
        }
    } while (false);

    initial_x_axis_vec = mobilePlatformInitialPose_.getBasis().getColumn(0);
    current_x_axis_vec = mobilePlatformCurrentPose.getBasis().getColumn(0);

    displacement  = mobilePlatformCurrentPose.getOrigin();
    displacement -= mobilePlatformInitialPose_.getOrigin();

    mob_plat_position = initial_x_axis_vec.dot(displacement)/current_x_axis_vec.dot(initial_x_axis_vec);
    
    onSpot_ = false;

    if (detectionPtr->data[1] < -1e-05) {   
        if (mobilePlatformDesiredPosition_ > mob_plat_position + detectionPtr->data[1] && mobilePlatformDesiredPosition_ > -0.3+0.00001) {
            
            mobilePlatformDesiredPosition_ = std::max(-0.3, mob_plat_position + detectionPtr->data[1]);

	        mobilePlatformBeginPosition_ = mob_plat_position;

	        mobilePlatformTrajectoryTotalTime_ = (mobilePlatformBeginPosition_ - mobilePlatformDesiredPosition_)/0.5*7.5;

            if (mobilePlatformCurrentTwist_.linear.x > 0)
		        mobilePlatformTrajectoryTotalTime_ += abs(mobilePlatformCurrentTwist_.linear.x)/0.05;

	        mobilePlatformTrajectoryStartTime_ = ros::Time::now().toSec();
            mobilePlatformTrajectoryExecTime_ = 0;

            if(callbackOrder_ == 0) { 
                callbackOrder_ = 2;
            } else if(callbackOrder_ == 1) { 
                mobilePlatformTrajectoryInitialVelcity_ = mobilePlatformCurrentTwist_.linear.x;
            }
        }
    } else if (detectionPtr->data[1] > 1e-05) {
        if (mobilePlatformDesiredPosition_ < mob_plat_position + detectionPtr->data[1] && mobilePlatformDesiredPosition_ < 0.3-0.00001) {
            mobilePlatformDesiredPosition_ = std::min(0.3, mob_plat_position + detectionPtr->data[1]);

	        mobilePlatformBeginPosition_ = mob_plat_position;

	        mobilePlatformTrajectoryTotalTime_ = (mobilePlatformDesiredPosition_ - mobilePlatformBeginPosition_)/0.5*7.5;

            if (mobilePlatformCurrentTwist_.linear.x < 0)
                mobilePlatformTrajectoryTotalTime_ += abs(mobilePlatformCurrentTwist_.linear.x)/0.05;  

	        mobilePlatformTrajectoryStartTime_ = ros::Time::now().toSec();
            mobilePlatformTrajectoryExecTime_ = 0;

            if(callbackOrder_ == 0) { 
                callbackOrder_ = 2;
            } else if(callbackOrder_ == 1) { 
                mobilePlatformTrajectoryInitialVelcity_ = mobilePlatformCurrentTwist_.linear.x;
            }
        }
    } else {
        onSpot_ = true;
    }

    if (detectionPtr->data[0] <= distanceProtectiveField_) {
        stop();
        decFactor_ = 0;
    } else if (detectionPtr->data[0] < distanceWarningField_) {
        decFactor_ = (detectionPtr->data[0] - distanceProtectiveField_) / (distanceWarningField_ - distanceProtectiveField_)*1 + 0.0;
    } else {
	    decFactor_ = 1;
    }
}

void ModelPredictiveControl::updateMPCx0()
{
    tf::StampedTransform mobilePlatformCurrentPose;
    tf::Vector3 initial_x_axis_vec, current_x_axis_vec, displacement;

    while(1)
    {
    	try
    	{
            tfListener_.lookupTransform("/odom", "/base_footprint", ros::Time(0), mobilePlatformCurrentPose);
            break;
    	}
        catch (tf::TransformException &ex)
    	{
            ROS_ERROR("%s",ex.what());
            ros::Duration(0.005).sleep();
    	}
    }

    initial_x_axis_vec = mobilePlatformInitialPose_.getBasis().getColumn(0);
    current_x_axis_vec = mobilePlatformCurrentPose.getBasis().getColumn(0);

    displacement = mobilePlatformCurrentPose.getOrigin();
    displacement-= mobilePlatformInitialPose_.getOrigin();

    x0_ << currentJointState_.position[0], currentJointState_.position[1], currentJointState_.position[2], currentJointState_.position[3], currentJointState_.position[4], currentJointState_.position[5], initial_x_axis_vec.dot(displacement)/current_x_axis_vec.dot(initial_x_axis_vec);

    if(onSpot_)
    {
	    mobilePlatformDesiredPosition_ = x0_(6, 0);
        mobilePlatformBeginPosition_ = x0_(6, 0);
    }
}

bool ModelPredictiveControl::inverseKinematic(const tf::StampedTransform& transformation, double ik_sols[])
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

        return true;
    }
    else
    {
        ROS_INFO("Fail to solve the ik problem.");
        return false;
    }
}

bool ModelPredictiveControl::inverseKinematic(const tf::Transform& transformation, double ik_sols[])
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

        return true;
    }
    else
    {   
        ROS_INFO("Fail to solve the ik problem.");
        return false; 
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
	        if(currentJointState_.velocity[i] >= 0)
    	        joint_velocity_cmd.data[i] = (currentJointState_.velocity[i]+aMin_[i]*mpcTimeStep_ > 0) ? (currentJointState_.velocity[i]+aMin_[i]*mpcTimeStep_) : 0;
	        else
                joint_velocity_cmd.data[i] = (currentJointState_.velocity[i]+aMax_[i]*mpcTimeStep_ < 0) ? (currentJointState_.velocity[i]+aMax_[i]*mpcTimeStep_) : 0;
        }

        if(mobilePlatformCurrentTwist_.linear.x >= 0)
            mobile_platform_velocity_cmd.linear.x = (mobilePlatformCurrentTwist_.linear.x+aMin_[6]*mpcTimeStep_ > 0) ? (mobilePlatformCurrentTwist_.linear.x+aMin_[6]*mpcTimeStep_) : 0;
        else
            mobile_platform_velocity_cmd.linear.x = (mobilePlatformCurrentTwist_.linear.x+aMax_[6]*mpcTimeStep_ < 0) ? (mobilePlatformCurrentTwist_.linear.x+aMax_[6]*mpcTimeStep_) : 0;
    
        jointVelocityPublisher_.publish(joint_velocity_cmd);
        mobilePlatformVelocityPublisher_.publish(mobile_platform_velocity_cmd);

        stop_count = 0;

        for(int i = 0; i <= 5; i++)
        {
	        if(currentJointState_.velocity[i] < 0.00001 && currentJointState_.velocity[i] > -0.00001)
    	        stop_count++;
        }

        if(mobilePlatformCurrentTwist_.linear.x < 0.00001 && mobilePlatformCurrentTwist_.linear.x > -0.00001)
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
                tfListener_.lookupTransform("/odom", "/tm_tool0", ros::Time(0), pose0_);
                break;
            }
            else
            {
                des_ee_tra.header.frame_id = "tm_base_link";
                ee_tra.header.frame_id = "tm_base_link";
                tfListener_.lookupTransform("/tm_base_link", "/tm_tool0", ros::Time(0), pose0_);
                break;
            }
    	}
    	catch (tf::TransformException &ex)
    	{
            ROS_ERROR("%s",ex.what());
            ros::Duration(0.005).sleep();
        }
    }

    tra_pos.pose.position.x = pose0_.getOrigin().getX();
    tra_pos.pose.position.y = pose0_.getOrigin().getY();
    tra_pos.pose.position.z = pose0_.getOrigin().getZ();
    tra_pos.pose.orientation.x = pose0_.getRotation().getX();
    tra_pos.pose.orientation.y = pose0_.getRotation().getY();
    tra_pos.pose.orientation.z = pose0_.getRotation().getZ();
    tra_pos.pose.orientation.w = pose0_.getRotation().getW();

    ee_tra.poses.push_back(tra_pos);
    ee_tra_pub.publish(ee_tra);
   
    ee_state.data[0] = pose0_.getOrigin().getX();
    ee_state.data[1] = pose0_.getOrigin().getY();
    ee_state.data[2] = pose0_.getOrigin().getZ();
    pose0_.getBasis().getRPY(ee_state.data[3], ee_state.data[4], ee_state.data[5]);
    
    ee_state_pub.publish(ee_state);

    desiredPoseIsGlobal_ = pose_is_global;

    posef_ = ee_desired_pose;

    callbackOrder_ = 0;
    ros::spinOnce();

    //onSpot_ = true;

    updateMPCx0();

    for(int i = 0; i < 7; i++)
    	robot_state.data[i] = x0_[i];
    
    robotStatePublisher_.publish(robot_state);

    for(int i = 0; i < 6; i++)
    	robot_vel.data[i] = currentJointState_.velocity[i];
    robot_vel.data[6] = mobilePlatformCurrentTwist_.linear.x; 

    robotVelocityPublisher_.publish(robot_vel);

    trajectoryTotalTime_ = (pose0_.getOrigin().distance(posef_.getOrigin())/0.15*3 > pose0_.getRotation().angleShortestPath(posef_.getRotation())/0.8*4) ? pose0_.getOrigin().distance(posef_.getOrigin())/0.15*3 : pose0_.getRotation().angleShortestPath(posef_.getRotation())/0.8*4;
    std::cout << pose0_.getOrigin().distance(posef_.getOrigin()) << ":::" << pose0_.getRotation().angleShortestPath(posef_.getRotation()) << std::endl;
    trajectoryExecTime_ = 0;

    castMPCToQPHessian();
    castMPCToQPConstraintMatrix();

    // set the initial data of the QP solver_
    if(!solver_.updateHessianMatrix(hessianMatrix_)) return false;
    if(!solver_.updateLinearConstraintsMatrix(constraintMatrix_)) return false;

    trajectoryStartTime_ = ros::Time::now().toSec();

    int state = castMPCToQPGradient();

    if(state == -2)
    {
	    std::cout << "Set of configuration changes." << std::endl;
        return false;
    }
    else if(state == -1)
    {
        stop();
        decFactor_ = 0;
	    decFactorPrevious_ = decFactor_;
    }
    
    if(!solver_.updateGradient(gradient_)) return false;

    castMPCToQPConstraintVectors();

    if(!solver_.updateBounds(lowerBound_, upperBound_)) return false;

    //ROS_INFO("eeeee");

    while(ros::ok())
    {
        //std::cout << lowerBound_ << std::endl << std::endl;
        //std::cout << upperBound_ << std::endl << std::endl;  

        //std::cout << constraintMatrix_ << std::endl;

        //std::cout << x0_ << std::endl;

        // solve the QP problem
        //if(!solver_.solve()) return false;
        if(solver_.solve())
	    {
            // get the controller input
            QPSolution = solver_.getSolution();
            ctr = QPSolution.block(7*(mpcWindow_+1), 0, 7, 1);
	    }
        else
        {
            for(int i = 0; i <= 5; i++)
            {
            if(currentJointState_.velocity[i] >= 0)
	    	    ctr(i) = (currentJointState_.velocity[i]+aMin_[i]*mpcTimeStep_ > 0) ? (currentJointState_.velocity[i]+aMin_[i]*mpcTimeStep_) : 0;
		    else
                ctr(i) = (currentJointState_.velocity[i]+aMax_[i]*mpcTimeStep_ < 0) ? (currentJointState_.velocity[i]+aMax_[i]*mpcTimeStep_) : 0;
	        }

            if(mobilePlatformCurrentTwist_.linear.x >= 0)
                ctr(6) = (mobilePlatformCurrentTwist_.linear.x+aMin_[6]*mpcTimeStep_ > 0) ? (mobilePlatformCurrentTwist_.linear.x+aMin_[6]*mpcTimeStep_) : 0;
            else
                ctr(6) = (mobilePlatformCurrentTwist_.linear.x+aMax_[6]*mpcTimeStep_ < 0) ? (mobilePlatformCurrentTwist_.linear.x+aMax_[6]*mpcTimeStep_) : 0;
	    }
        
        std::cout << "vel_cmd:" << std::endl << ctr << std::endl << std::endl;

        reach_goal_count = 0;

        if((x0_[6] - xf_[6]) < 0.003 && (x0_[6] - xf_[6]) > -0.003 && std::abs(mobilePlatformCurrentTwist_.linear.x) < 0.023)
        {
            std::cout << "vel:" << std::endl << mobilePlatformCurrentTwist_.linear.x << std::endl << std::endl;

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

	    if((x0_ - xf_).block<6, 1>(0, 0).norm() < 0.0005)
    	{ 
            if(desiredPoseIsGlobal_)
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
		        onSpot_ = true;
	        }
        }
        else
        {
            for(int i = 0; i < 6; i++)
            {
                joint_velocity_cmd.data[i] = ctr(i);         
            }
        }

    	jointVelocityPublisher_.publish(joint_velocity_cmd);
    	mobilePlatformVelocityPublisher_.publish(mobile_platform_velocity_cmd);
	
    	if(reach_goal_count == 2)
    	{
            /*while(1)
    	    {
    	    	try
   	    	{
	    	    if(pose_is_global)
	    	    {
            	    	tfListener_.lookupTransform("/odom", "/tm_tool0", ros::Time(0), ee_pose_tf);
            	    	break;
	            }
	            else
	            {
		    	tfListener_.lookupTransform("/tm_base_link", "/tm_tool0", ros::Time(0), ee_pose_tf);
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

	    loopRate_.sleep();

	    // save data into file
        //auto x0Data = x0_.data();

        // propagate the model
        callbackOrder_ = 0;
	    ros::spinOnce();

        //onSpot_ = true;

	    updateMPCx0();

        for(int i = 0; i < 7; i++)
    	    robot_state.data[i] = x0_[i];
    
        robotStatePublisher_.publish(robot_state);

    	for(int i = 0; i < 6; i++)
    	    robot_vel.data[i] = currentJointState_.velocity[i];
    	robot_vel.data[6] = mobilePlatformCurrentTwist_.linear.x; 

    	robotVelocityPublisher_.publish(robot_vel);

        //std::cout << std::endl << ctr <<std::endl;
        //std::cout << std::endl << x0_ << std::endl;

        state = castMPCToQPGradient();

        if(state == -2)
	    {
	        std::cout << "Set of configuration changes." << std::endl;
            return false;
	    }
	    else if(state == -1)
    	{
            stop();

            decFactor_ = 0;
	        decFactorPrevious_ = decFactor_;
    	}

        if(!solver_.updateGradient(gradient_)) return false;

        // update the constraint bound
        castMPCToQPConstraintVectors();
        if(!solver_.updateBounds(lowerBound_, upperBound_)) return false;

        while(1)
    	{
    	    try
   	        {
                if(pose_is_global)
                {
                    tfListener_.lookupTransform("/odom", "/tm_tool0", ros::Time(0), ee_pose_tf);
                    break;
                }
                else
                {
                    tfListener_.lookupTransform("/tm_base_link", "/tm_tool0", ros::Time(0), ee_pose_tf);
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

        std::cout << "x0_:" << std::endl << x0_ << std::endl << std::endl;
        std::cout << "xf_:" << std::endl << xf_ << std::endl << std::endl;
        //std::cout << "lowerBound_:" << std::endl << lowerBound_ << std::endl << std::endl;
        //std::cout << "uppedBound:" << std::endl << upperBound_ << std::endl << std::endl;
        std::cout << "decFactor_:" << std::endl << decFactor_ << std::endl << std::endl;
        std::cout << "gradient_:" << std::endl << gradient_.block(0, 0, (mpcWindow_+1)*7, 1) << std::endl << std::endl;
    }

    return true;
}

void ModelPredictiveControl::performReplenishment()
{ 
    double task_start_t = ros::Time::now().toSec();
    double fun_fin_t[3];

    bool apriltag_detection_enable, gripper_cmd;
    tf::StampedTransform apriltag_pose, placing_pose, pre_placing_pose; 

    tf::StampedTransform detection_pose;
    tf::StampedTransform base_tf;

    while(1)
    {
    	try
    	{
            tfListener_.lookupTransform("/odom", "/tm_base_link", ros::Time(0), base_tf);
	        break;
        }
    	catch (tf::TransformException &ex)
    	{
            ROS_ERROR("%s",ex.what());
            ros::Duration(0.005).sleep();
    	}
    }

    //obstaclesDetectionEnabled_ = false;
    obstaclesDetectionEnabled_ = true;
    decFactor_ = 1;

    // while(0)
    // {
    //     // GO TO SCAN TAG POSITION
    //     detection_pose.setData(base_tf*tf::Transform(tf::Transform(tf::Quaternion(0.7, 0, 0, 0.7), tf::Vector3(-0.12, -0.25, 0.55)))); 
        
    //     if(!reachDesiredPose(detection_pose, true))
    //     {
    //         ROS_INFO("Fail to reach the desired pose");
    //         stop();
    //         return;
    //     }

    //     tf::TransformtfListener_ tfListener_;
    //     tf::StampedTransform tf_l;
    //     std::string tf_l_name = "/tag_308";

    //     static tf::TransformBroadcaster br;
    //     tf::Transform tf_b;    
    //     std::string tf_b_name = "/target_lemonade";
        

    //     tfListener_.waitForTransform("/base_link", tf_l_name, ros::Time(0), ros::Duration(3.0));
    //     tfListener_.lookupTransform("/base_link", tf_l_name, ros::Time(0), tf_l);

    //     tf_b.setOrigin(tf::Vector3(0.05, -0.07, 0.3421));
    //     tf_b.setRotation(tf::Quaternion(-0.500, 0.500, 0.500, 0.500));
    //     br.sendTransform(tf::StampedTransform(tf_b, ros::Time::now(), "/tag_308", tf_b_name));

    //     tfListener_.waitForTransform("/tm_base_link", tf_b_name, ros::Time(0), ros::Duration(3.0));
    //     tfListener_.lookupTransform("/tm_base_link", tf_b_name, ros::Time(0), tf_l);

    //     // std::cout<<"Relative Pose"<<std::endl;
    //     // std::cout<<"X: "<<tf_l.getOrigin().getX()<<std::endl;
    //     // std::cout<<"Y: "<<tf_l.getOrigin().getY()<<std::endl;
    //     // std::cout<<"Z: "<<tf_l.getOrigin().getZ()<<std::endl;

    //     // GO TO MID POSITION
    //     detection_pose.setData(base_tf*tf::Transform(tf::Transform(tf::Quaternion(0.7, 0, 0, 0.7), tf::Vector3(-0.24, -0.15, 0.55)))); 
        
    //     if(!reachDesiredPose(detection_pose, true))
    //     {
    //         ROS_INFO("Fail to reach the desired pose");
    //         stop();
    //         return;
    //     }

    //     //GO TO PLACE 1 MID
    //     detection_pose.setData(base_tf*tf::Transform(tf::Transform(tf::Quaternion(0.425, -0.436, -0.563, 0.559), tf::Vector3(-0.279, 0.123, 0.471)))); 
        
    //     if(!reachDesiredPose(detection_pose, true))
    //     {
    //         ROS_INFO("Fail to reach the desired pose");
    //         stop();
    //         return;
    //     }

    //     //GO TO PLACE 1
    //     detection_pose.setData(base_tf*tf::Transform(tf::Transform(tf::Quaternion(-0.498, 0.506, 0.502, -0.493), tf::Vector3(-0.372, 0.122, 0.332)))); 
        
    //     if(!reachDesiredPose(detection_pose, true))
    //     {
    //         ROS_INFO("Fail to reach the desired pose");
    //         stop();
    //         return;
    //     }

    //     gripper_cmd.data = true;
    //     gripper_pub.publish(gripper_cmd);
    //     sleep(1);

    //     //GO TO PLACE 1 MID
    //     detection_pose.setData(base_tf*tf::Transform(tf::Transform(tf::Quaternion(0.425, -0.436, -0.563, 0.559), tf::Vector3(-0.279, 0.123, 0.471)))); 
        
    //     if(!reachDesiredPose(detection_pose, true))
    //     {
    //         ROS_INFO("Fail to reach the desired pose");
    //         stop();
    //         return;
    //     }

    //     // GO TO MID POSITION
    //     detection_pose.setData(base_tf*tf::Transform(tf::Transform(tf::Quaternion(0.7, 0, 0, 0.7), tf::Vector3(-0.24, -0.15, 0.55)))); 
        
    //     if(!reachDesiredPose(detection_pose, true))
    //     {
    //         ROS_INFO("Fail to reach the desired pose");
    //         stop();
    //         return;
    //     }

    //     // GO TO LEMONADE POSITION

    //     detection_pose.setData(base_tf*tf::Transform(tf::Transform(tf::Quaternion(0.7, 0, 0, 0.7), tf::Vector3(tf_l.getOrigin().getX(), tf_l.getOrigin().getY()+0.05, tf_l.getOrigin().getZ()+0.05)))); 
        
    //     if(!reachDesiredPose(detection_pose, true))
    //     {
    //         ROS_INFO("Fail to reach the desired pose");
    //         stop();
    //         return;
    //     }

    //     detection_pose.setData(base_tf*tf::Transform(tf::Transform(tf::Quaternion(0.7, 0, 0, 0.7), tf::Vector3(tf_l.getOrigin().getX(), tf_l.getOrigin().getY(), tf_l.getOrigin().getZ())))); 
        
    //     if(!reachDesiredPose(detection_pose, true))
    //     {
    //         ROS_INFO("Fail to reach the desired pose");
    //         stop();
    //         return;
    //     }

    //     gripper_cmd.data = false;
    //     gripper_pub.publish(gripper_cmd);
    //     sleep(1);

    //     // GO TO SCAN TAG POSITION

    //     detection_pose.setData(base_tf*tf::Transform(tf::Transform(tf::Quaternion(0.7, 0, 0, 0.7), tf::Vector3(tf_l.getOrigin().getX(), tf_l.getOrigin().getY()+0.05, tf_l.getOrigin().getZ()+0.05)))); 
        
    //     if(!reachDesiredPose(detection_pose, true))
    //     {
    //         ROS_INFO("Fail to reach the desired pose");
    //         stop();
    //         return;
    //     }

    //     detection_pose.setData(base_tf*tf::Transform(tf::Transform(tf::Quaternion(0.7, 0, 0, 0.7), tf::Vector3(-0.12, -0.25, 0.55)))); 
        
    //     if(!reachDesiredPose(detection_pose, true))
    //     {
    //         ROS_INFO("Fail to reach the desired pose");
    //         stop();
    //         return;
    //     }

    //     ROS_INFO("REPLENISHMENT FINISHED !!!");
    // }

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


    // // OLD CODE
    // while(0)
    // {
    //     apriltag_detection_enable.data = true;
    //     apriltag_detection_cmd_pub.publish(apriltag_detection_enable);

    //     //consume the apriltag detection that may be out of date.
    //     ros::spinOnce();
    //     apriltagDetected_ = false;

    //     while(!apriltagDetected_)
    //     {
    //         ros::spinOnce();
    //     }

    //     while(1)
    //     {
    //         try
    //         {
    //             tfListener_.lookupTransform("/odom", "/at13", ros::Time(0), apriltag_pose);
    //             break;
    //         }
    //         catch (tf::TransformException &ex)
    //         {
    //             ROS_ERROR("%s",ex.what());
    //             ros::Duration(0.005).sleep();
    //         }
    //     }

    //     apriltag_detection_enable.data = false;
    //     apriltag_detection_cmd_pub.publish(apriltag_detection_enable);

    //     placing_pose = apriltag_pose;

    //     tf::StampedTransform bias;
    //     tf::Quaternion qu(-0.005, 0.998, 0.068, -0.003);

    //     bias.setData(tf::Transform(qu, tf::Vector3(0.0, 0.0, 0.0)));
    //     placing_pose*=bias;

    //     bias.setData(tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0.02, -0.28, -0.08)));
    //     placing_pose*=bias;

    //     pre_placing_pose = placing_pose;

    //     bias.setData(tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0, 0, -0.19)));
    //     pre_placing_pose*=bias;

    //     detection_pose.setData(tf::Transform(tf::Quaternion(0.7, 0, 0, 0.7), tf::Vector3(-0.2, -0.25, 0.47))); 

    //     if(!reachDesiredPose(detection_pose, false))
    //     {
    //         ROS_INFO("Fail to reach the desired pose");
    //         stop();
    //         return;
    //     }

    //     fun_fin_t[0] = ros::Time::now().toSec()-task_start_t;

    //     obstaclesDetectionEnabled_ = true;

    //     detection_pose.setData(tf::Transform(tf::Quaternion(0.5, -0.5, -0.5, 0.5), tf::Vector3(-0.35, -0.0, 0.40))); 
        
    //     if(!reachDesiredPose(detection_pose, false))
    //     {
    //         ROS_INFO("Fail to reach the desired pose");
    //         stop();
    //         return;
    //     }

    //     detection_pose.setData(tf::Transform(tf::Quaternion(0.5, -0.5, -0.5, 0.5), tf::Vector3(-0.35, -0.0, 0.30))); 
        
    //     if(!reachDesiredPose(detection_pose, false))
    //     {
    //         ROS_INFO("Fail to reach the desired pose");
    //         stop();
    //         return;
    //     }

    //     gripper_cmd.data = true;
    //     gripper_pub.publish(gripper_cmd);
    //     ros::Duration(1).sleep();

    //     detection_pose.setData(tf::Transform(tf::Quaternion(0.5, -0.5, -0.5, 0.5), tf::Vector3(-0.35, -0.0, 0.40))); 
    
    //     if(!reachDesiredPose(detection_pose, false))
    //     {
    //         ROS_INFO("Fail to reach the desired pose");
    //         stop();
    //         return;
    //     }

    //     detection_pose.setData(tf::Transform(tf::Quaternion(0.7, 0, 0, 0.7), tf::Vector3(-0.2, -0.25, 0.47))); 
        
    //     if(!reachDesiredPose(detection_pose, false))
    //     {
    //         ROS_INFO("Fail to reach the desired pose");
    //         stop();
    //         return;
    //     }

    //     fun_fin_t[1] = ros::Time::now().toSec()-task_start_t-fun_fin_t[0];

    //     //detection_pose.setData(base_tf*tf::Transform(tf::Quaternion(0.7, 0, 0, 0.7), tf::Vector3(0.1, -0.25, 0.47))); 
        
    //     if(!reachDesiredPose(pre_placing_pose, true))
    //     {
    //         ROS_INFO("Fail to reach the desired pose");
    //         stop();
    //         return;
    //     }

    //     //detection_pose.setData(base_tf*tf::Transform(tf::Quaternion(0.7, 0, 0, 0.7), tf::Vector3(0.1, -0.43, 0.47))); 
        
    //     if(!reachDesiredPose(placing_pose, true))
    //     {
    //         ROS_INFO("Fail to reach the desired pose");
    //         stop();
    //         return;
    //     }
        
    //     gripper_cmd.data = false;
    //     gripper_pub.publish(gripper_cmd);
    //     ros::Duration(1).sleep();

    //     //detection_pose.setData(base_tf*tf::Transform(tf::Quaternion(0.7, 0, 0, 0.7), tf::Vector3(0.1, -0.25, 0.47))); 
        
    //     if(!reachDesiredPose(pre_placing_pose, true))
    //     {
    //         ROS_INFO("Fail to reach the desired pose");
    //         stop();
    //         return;
    //     }

    //     detection_pose.setData(tf::Transform(tf::Quaternion(0.7, 0, 0, 0.7), tf::Vector3(-0.2, -0.25, 0.47))); 

    //     if(!reachDesiredPose(detection_pose, false))
    //     {
    //         ROS_INFO("Fail to reach the desired pose");
    //         stop();
    //         return;
    //     }

    //     fun_fin_t[2] = ros::Time::now().toSec()-task_start_t-fun_fin_t[0]-fun_fin_t[1];

    //     std::cout << "detection_time:" << fun_fin_t[0] << std::endl;
    //     std::cout << "picking_time:" << fun_fin_t[1] << std::endl;
    //     std::cout << "placing_time:" << fun_fin_t[2] << std::endl;
    //     std::cout << "task_time:" << ros::Time::now().toSec()-task_start_t << std::endl;
        
    //     return;
    // }
}

}   // namespace mpc