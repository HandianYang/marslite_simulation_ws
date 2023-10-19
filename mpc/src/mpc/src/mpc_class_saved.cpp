/**
 * @file MPCExample.cpp
 * @author Giulio Romualdi
 * @copyright Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 * @date 2018
 */

#include <mpc/mpc.h> 

ModelPredictiveControl::ModelPredictiveControl(ros::NodeHandle& nh)
{
    time_step = 1;
    mpcWindow = 6;

    //set the weight matrices
    Q.diagonal() << 1., 1., 1., 1., 1., 1., 1.;
    R.diagonal() << 0.005, 0.005, 0.005, 0.005, 0.005, 0.005, 0.005;

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
            1.5;

    xMin << (-270.)/180*M_PI,
            (-180.)/180*M_PI,
            (-155.)/180*M_PI,
            (-180.)/180*M_PI,
            (-180.)/180*M_PI,
            (-270.)/180*M_PI,
            -1.5;

    //set the input inequality constraints
    uMax << 10./180*M_PI,
            10./180*M_PI,
            10./180*M_PI,
            10./180*M_PI,
            10./180*M_PI,
            10./180*M_PI,
            0.5;

    uMin << (-10.)/180*M_PI,
            (-10.)/180*M_PI,
            (-10.)/180*M_PI,
            (-10.)/180*M_PI,
            (-10.)/180*M_PI,
            (-10.)/180*M_PI,
            (-0.5);
}

ModelPredictiveControl::~ModelPredictiveControl()
{

}

void ModelPredictiveControl::castMPCToQPHessian()
{
    //input:Q, R, mpcWindow
    hessianMatrix.resize(7*(mpcWindow+1)+7*mpcWindow, 7*(mpcWindow+1)+7*mpcWindow);

    //populate hessian matrix
    for(int i = 0; i < 7*(mpcWindow+1)+7*mpcWindow; i++){
        if(i < 7*(mpcWindow+1)){
            int posQ = i%7;
            float value = Q.diagonal()[posQ];
            if(value != 0)
                hessianMatrix.insert(i,i) = value;
        }
        else{
            int posR = i%7;
            float value = R.diagonal()[posR];
            if(value != 0)
                hessianMatrix.insert(i,i) = value;
        }
    }
}

void ModelPredictiveControl::castMPCToQPGradient()
{
    //input:Q, x0, xRef, mpcWindow
    Eigen::Matrix<double,7,1> Qx_ref;

    // populate the gradient vector
    gradient = Eigen::VectorXd::Zero(7*(mpcWindow+1)+7*mpcWindow, 1);
    for(int i = 0; i < 7*(mpcWindow+1); i++){
        int posQ = i%7;

	if(posQ == 0)
	{
	    Qx_ref = Q*(-1)*(x0+(xRef-x0)/mpcWindow*(i/7));
	}

        float value = Qx_ref(posQ,0);
        gradient(i,0) = value;
    }
}

void ModelPredictiveControl::castMPCToQPConstraintMatrix()
{
    //input:A, B, mpcWindow
    constraintMatrix.resize(7*(mpcWindow+1)  + 7*(mpcWindow+1) + 7 * mpcWindow, 7*(mpcWindow+1) + 7 * mpcWindow);

    // populate linear constraint matrix
    for(int i = 0; i<7*(mpcWindow+1); i++){
        constraintMatrix.insert(i,i) = -1;
    }

    for(int i = 0; i < mpcWindow; i++)
        for(int j = 0; j<7; j++)
            for(int k = 0; k<7; k++){
                float value = A(j,k);
                if(value != 0){
                    constraintMatrix.insert(7 * (i+1) + j, 7 * i + k) = value;
                }
            }

    for(int i = 0; i < mpcWindow; i++)
        for(int j = 0; j < 7; j++)
            for(int k = 0; k < 7; k++){
                float value = B(j,k);
                if(value != 0){
                    constraintMatrix.insert(7*(i+1)+j, 7*i+k+7*(mpcWindow + 1)) = value;
                }
            }

    for(int i = 0; i<7*(mpcWindow+1) + 7*mpcWindow; i++){
        constraintMatrix.insert(i+(mpcWindow+1)*7,i) = 1;
    }
}

void ModelPredictiveControl::castMPCToQPConstraintVectors()
{
    //input:xMax, xMin, uMax, uMin, x0, mpcWindow
    // evaluate the lower and the upper inequality vectors
    Eigen::VectorXd lowerInequality = Eigen::MatrixXd::Zero(7*(mpcWindow+1) +  7 * mpcWindow, 1);
    Eigen::VectorXd upperInequality = Eigen::MatrixXd::Zero(7*(mpcWindow+1) +  7 * mpcWindow, 1);
    for(int i=0; i<mpcWindow+1; i++){
        lowerInequality.block(7*i,0,7,1) = xMin;
        upperInequality.block(7*i,0,7,1) = xMax;
    }
    for(int i=0; i<mpcWindow; i++){
        lowerInequality.block(7 * i + 7 * (mpcWindow + 1), 0, 7, 1) = uMin;
        upperInequality.block(7 * i + 7 * (mpcWindow + 1), 0, 7, 1) = uMax;
    }

    // evaluate the lower and the upper equality vectors
    Eigen::VectorXd lowerEquality = Eigen::MatrixXd::Zero(7*(mpcWindow+1),1 );
    Eigen::VectorXd upperEquality;
    lowerEquality.block(0,0,7,1) = -x0;
    upperEquality = lowerEquality;
    lowerEquality = lowerEquality;

    // merge inequality and equality vectors
    lowerBound = Eigen::MatrixXd::Zero(2*7*(mpcWindow+1) +  7*mpcWindow,1 );
    lowerBound << lowerEquality,
        lowerInequality;

    upperBound = Eigen::MatrixXd::Zero(2*7*(mpcWindow+1) +  7*mpcWindow,1 );
    upperBound << upperEquality,
        upperInequality;
}

/*
void callback(const sensor_msgs::JointState& joint_states)
{
    for(int i=0; i<7; i++)
    {
        current_joint_state.position[i] = joint_states.position[i];
	//std::cout << current_joint_state.position[i] << std::endl;
    }
}

void apriltag_callback(const apriltags_ros::AprilTagDetectionArray& det)
{
    tf::TransformListener listener;

    for(int i=0; i<det.detections.size(); i++)
    {
        if(det.detections[i].id==10)
        {
            apriltag_detected = true;
	    //std::cout << det.detections[i].id << std::endl;
	}
    }
    //std::cout << std::endl;
}

void yolo_callback(const darknet_ros_msgs::BoundingBoxes& det)
{
    for(int i=0; i<det.bounding_boxes.size(); i++)
    {
        if(det.bounding_boxes[i].Class=="person")
	{
	    std::cout << det.bounding_boxes[i].xmin << std::endl;
	}
    }
    std::cout << std::endl;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "mpc");
    ros::NodeHandle nh;

    ros::Publisher pub = nh.advertise<std_msgs::Float64MultiArray>("/velocity_cmd", 1);
    ros::Publisher pub2 = nh.advertise<geometry_msgs::Twist>("/mob_plat/cmd_vel", 1);
    ros::Publisher pub3 = nh.advertise<std_msgs::Bool>("/apriltag_detection_enable", 1);
    ros::Subscriber sub = nh.subscribe("/tm_joint_states", 1, callback);
    ros::Subscriber sub2 = nh.subscribe("/tag_detections", 1, apriltag_callback);
    ros::Subscriber sub3 = nh.subscribe("/bounding_boxes", 1, yolo_callback);
    tf::TransformListener listener;
    ros::Rate loop_rate(25);

    tf::StampedTransform transform, initial_p, now_p, p_apriltag;
    geometry_msgs::Twist v_cmd_p;
    std_msgs::Float64MultiArray v_cmd;
    int done_count = 0;

    apriltag_detected = false;

    v_cmd.data.resize(6);
    current_joint_state.position.resize(6);

    std_msgs::Bool apriltag_detection_enable;
    apriltag_detection_enable.data=true;

    ros::Duration(0.5).sleep();

    pub3.publish(apriltag_detection_enable);

    while(!apriltag_detected)
    {
    	ros::spinOnce();
    }

    ros::Duration(0.05).sleep();

    try
    {
        listener.lookupTransform("/tm_base_link", "/at10", ros::Time(0), p_apriltag);
    }
    catch (tf::TransformException &ex)
    {
        ROS_ERROR("%s",ex.what());
        ros::Duration(1.0).sleep();

        return 0;
    }

    apriltag_detection_enable.data=false;
    pub3.publish(apriltag_detection_enable);

    ros::spinOnce();

    // set the initial states
    x0 << current_joint_state.position[0], current_joint_state.position[1], current_joint_state.position[2], current_joint_state.position[3], current_joint_state.position[4], current_joint_state.position[5], 0;

    // set the desired states
    try
    {
        listener.lookupTransform("/tm_base_link", "/tm_tip_link", ros::Time(0), transform);
	listener.lookupTransform("/odom", "/base_footprint", ros::Time(0), initial_p);
    }
    catch (tf::TransformException &ex) 
    {
        ROS_ERROR("%s",ex.what());
        ros::Duration(1.0).sleep();

        return 0;
    }

    double T[16], ik_sols[48];

    tf::StampedTransform bias;
    bias.setData(tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0, 0, 0.2)));//0.0375/2, -0.0375/2, 0.2)));

    p_apriltag*=bias;

    tf::Quaternion qu;
    qu.setEuler(3.1415926, 0, 0.0); //YXZ
    bias.setData(tf::Transform(qu, tf::Vector3(0.0, 0.0, 0.0)));

    p_apriltag*=bias;


    T[0]=p_apriltag.getBasis()[0][0];
    T[1]=p_apriltag.getBasis()[0][1];
    T[2]=p_apriltag.getBasis()[0][2];
    T[3]=p_apriltag.getOrigin().x();
    T[4]=p_apriltag.getBasis()[1][0];
    T[5]=p_apriltag.getBasis()[1][1];
    T[6]=p_apriltag.getBasis()[1][2];
    T[7]=p_apriltag.getOrigin().y();
    T[8]=p_apriltag.getBasis()[2][0];
    T[9]=p_apriltag.getBasis()[2][1];
    T[10]=p_apriltag.getBasis()[2][2];
    T[11]=p_apriltag.getOrigin().z();
    T[12]=0;
    T[13]=0;
    T[14]=0;
    T[15]=1;

    //std::cout << tm_kinematics::inverse(T, ik_sols, 0.0) << std::endl;
    //for (const auto& e : ik_sols) {
    //    std::cout << e << std::endl;
    //}

    if(tm_kinematics::inverse(T, ik_sols, 0.0)>0)
    {
        xRef <<  ik_sols[0], ik_sols[1], ik_sols[2], ik_sols[3], ik_sols[4], ik_sols[5], 0;
    }
    else
    {
        ROS_INFO("IK failed.");
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
    solver.data()->setNumberOfVariables(7 * (mpcWindow + 1) + 7 * mpcWindow);
    solver.data()->setNumberOfConstraints(2 * 7 * (mpcWindow + 1) +  7 * mpcWindow);
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
        ctr = QPSolution.block(7 * (mpcWindow + 1), 0, 7, 1);

        done_count = 0;

	if((x0 - xRef).block<6, 1>(0, 0).norm() < 0.001)
	{
	    for(int i=0; i<6; i++)
            {
                v_cmd.data[i] = 0;
            }

	    done_count++;
	}
	else
	{
	    for(int i=0; i<6; i++)
	    {
	        v_cmd.data[i] = ctr(i);
	    }
	}

        if((x0[6] - xRef[6]) < 0.001 && (x0[6] - xRef[6]) > -0.001) 
        {
            v_cmd_p.linear.x = 0;
	    done_count++; 
        }
        else
	{
	    v_cmd_p.linear.x = ctr(6);
	}

	pub.publish(v_cmd);
        pub2.publish(v_cmd_p);

	if(done_count == 2)
	{
	    return 0;
	}

        // save data into file
        auto x0Data = x0.data();

        // propagate the model
	ros::spinOnce();

	try
        {
            listener.lookupTransform("/odom", "/base_footprint", ros::Time(0), now_p);
        }
        catch (tf::TransformException &ex)
        {
            ROS_ERROR("%s",ex.what());
            ros::Duration(1.0).sleep();
        }

	x0 << current_joint_state.position[0], current_joint_state.position[1], current_joint_state.position[2], current_joint_state.position[3], current_joint_state.position[4], current_joint_state.position[5], now_p.getOrigin().distance(initial_p.getOrigin());

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
}*/
