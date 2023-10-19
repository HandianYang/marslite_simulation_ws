#include <ros/ros.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>
// #include <mpc/DisMin.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <tf/transform_listener.h>
#include <cmath>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

ros::Publisher pub1;
ros::Publisher pub2;
//ros::Publisher pub3;
tf::StampedTransform depth_camera_1_tf, depth_camera_2_tf;
double distance_maintained = 3;

void pointcloud_cb(const sensor_msgs::PointCloud2ConstPtr& pc1, const sensor_msgs::PointCloud2ConstPtr& pc2)
{
    ROS_INFO("11111");

    // Container for original & filtered data
    pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2; 
    pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);
    pcl::PCLPointCloud2* cloud_filtered = new pcl::PCLPointCloud2;
    pcl::PCLPointCloud2Ptr cloudFilteredPtr(cloud_filtered);
    pcl::PointCloud<pcl::PointXYZ> *xyz_cloud = new pcl::PointCloud<pcl::PointXYZ>;
    pcl::PointCloud<pcl::PointXYZ>::Ptr xyzCloudPtr(xyz_cloud);
    pcl::PassThrough<pcl::PCLPointCloud2> pass;
    pcl::VoxelGrid<pcl::PCLPointCloud2> vg;
    sensor_msgs::PointCloud2 output;
    tf::Vector3 position, point_nearest;
    double distance_x, distance_y, distance, distance_min, distance_min_front, distance_min_back;
    std_msgs::Float64MultiArray obs_det_output;
    //mpc::DisMin min_distance;

    obs_det_output.data.resize(2);

    // Convert to PCL data type
    pcl_conversions::toPCL(*pc1, *cloud);

    // Create the filtering object
    pass.setInputCloud(cloudPtr);
    pass.setFilterFieldName ("x");
    pass.setFilterLimitsNegative (false);
    pass.setFilterLimits (-5.3, 0.30);
    pass.filter (*cloudFilteredPtr);

    pass.setInputCloud(cloudFilteredPtr);
    pass.setFilterFieldName ("z");
    pass.setFilterLimitsNegative (false);
    pass.setFilterLimits (0.8, 5.15);
    pass.filter (*cloudFilteredPtr);

    pass.setInputCloud(cloudFilteredPtr);
    pass.setFilterFieldName ("y");
    pass.setFilterLimitsNegative (false);
    pass.setFilterLimits (-1.3, 0.19);
    pass.filter (*cloudFilteredPtr);

    if((*cloud_filtered).data.empty())
    {
        distance_min_front = 100;

        pcl_conversions::fromPCL(*cloud_filtered, output);

        //pub1.publish(output);
    }
    else
    {
        vg.setInputCloud(cloudFilteredPtr);
    	vg.setLeafSize (0.025, 0.025, 0.025);
    	vg.filter(*cloudFilteredPtr);

    	pcl_conversions::fromPCL(*cloud_filtered, output);

    	//pub1.publish(output);

    	//pcl::PointCloud<pcl::PointXYZ> *xyz_cloud_filtered = new pcl::PointCloud<pcl::PointXYZ>;
    	//pcl::PointCloud<pcl::PointXYZ>::Ptr xyzCloudPtrFiltered(xyz_cloud_filtered);
    	pcl::fromPCLPointCloud2(*cloudFilteredPtr, *xyzCloudPtr);

    	/*pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
   	sor.setInputCloud(xyzCloudPtr);
    	sor.setMeanK(20);
    	sor.setStddevMulThresh (1.0);
    	sor.filter(*xyzCloudPtrFiltered);*/

    	//for obstacle avoidance module
    	/*std_msgs::Float64 obs_distance_min;

    	obs_distance_min.data = 100;

    	for(int nIndex = 0; nIndex < xyzCloudPtrFiltered->points.size(); nIndex++)
    	{
            if(obs_distance_min.data > xyzCloudPtrFiltered->points[nIndex].z && xyzCloudPtrFiltered->points[nIndex].x <= 0.40)
       	    {
            	obs_distance_min.data = xyzCloudPtrFiltered->points[nIndex].z;
            }
    	}

    	std::cout << "obs_distance_min:" << obs_distance_min.data << std::endl;*/

    	//for safety module
    	distance_min_front = 100;

    	for(int nIndex = 0; nIndex < xyzCloudPtr->points.size(); nIndex++)
    	{
            position = depth_camera_1_tf*(tf::Vector3(xyzCloudPtr->points[nIndex].x, xyzCloudPtr->points[nIndex].y, xyzCloudPtr->points[nIndex].z));

            distance_x = std::max(std::max(position.getX()-0.465, -0.39-position.getX()), 0.0);
            distance_y = std::max(std::max(position.getY()-0.255, -0.255-position.getY()), 0.0);

            distance = sqrt(distance_x*distance_x+distance_y*distance_y);

            if(distance < distance_min_front && distance > 0.001)
            {
            	distance_min_front = distance;
            	point_nearest.setX(position.getX());
            	point_nearest.setY(position.getY());
            	point_nearest.setZ(position.getZ());
            }
    	}
    }

    pass.setInputCloud(cloudPtr);
    pass.setFilterFieldName ("x");
    pass.setFilterLimitsNegative (false);
    pass.setFilterLimits (-5.3, 0.30);
    pass.filter (*cloudFilteredPtr);

    pass.setInputCloud(cloudFilteredPtr);
    pass.setFilterFieldName ("z");
    pass.setFilterLimitsNegative (false);
    pass.setFilterLimits (0.0, 0.8);
    pass.filter (*cloudFilteredPtr);

    pass.setInputCloud(cloudFilteredPtr);
    pass.setFilterFieldName ("y");
    pass.setFilterLimitsNegative (false);
    pass.setFilterLimits (-0.275, 0.19);
    pass.filter (*cloudFilteredPtr);

    if((*cloud_filtered).data.empty())
    {
        pcl_conversions::fromPCL(*cloud_filtered, output);

        pub1.publish(output);
    }
    else
    {
        vg.setInputCloud(cloudFilteredPtr);
    	vg.setLeafSize (0.025, 0.025, 0.025);
    	vg.filter(*cloudFilteredPtr);

    	pcl_conversions::fromPCL(*cloud_filtered, output);

    	pub1.publish(output);

    	pcl::fromPCLPointCloud2(*cloudFilteredPtr, *xyzCloudPtr);

    	for(int nIndex = 0; nIndex < xyzCloudPtr->points.size(); nIndex++)
    	{
            position = depth_camera_1_tf*(tf::Vector3(xyzCloudPtr->points[nIndex].x, xyzCloudPtr->points[nIndex].y, xyzCloudPtr->points[nIndex].z));

            distance_x = std::max(std::max(position.getX()-0.465, -0.39-position.getX()), 0.0);
            distance_y = std::max(std::max(position.getY()-0.26, -0.255-position.getY()), 0.0);

            distance = sqrt(distance_x*distance_x+distance_y*distance_y);

            if(distance < distance_min_front && distance > 0.001)
            {
            	distance_min_front = distance;
            	point_nearest.setX(position.getX());
            	point_nearest.setY(position.getY());
            	point_nearest.setZ(position.getZ());
            }
    	}
    }

    distance_min = distance_min_front;

    pcl_conversions::toPCL(*pc2, *cloud);

    // Create the filtering object
    pass.setInputCloud(cloudPtr);
    pass.setFilterFieldName ("y");
    pass.setFilterLimitsNegative (false);
    pass.setFilterLimits (-1.05, 0.45);
    pass.filter (*cloudFilteredPtr);

    pass.setInputCloud(cloudFilteredPtr);
    pass.setFilterFieldName ("x");
    pass.setFilterLimitsNegative (false);
    pass.setFilterLimits (-0.30, 5.3);
    pass.filter (*cloudFilteredPtr);

    if((*cloud_filtered).data.empty())
    {
	distance_min_back = 100;

        pcl_conversions::fromPCL(*cloud_filtered, output);

        //pub1.publish(output);
    }
    else
    {
        vg.setInputCloud(cloudFilteredPtr);
        vg.setLeafSize (0.025, 0.025, 0.025);
        vg.filter(*cloudFilteredPtr);

        pcl_conversions::fromPCL(*cloud_filtered, output);

        //pub1.publish(output);

        //pcl::PointCloud<pcl::PointXYZ> *xyz_cloud_filtered = new pcl::PointCloud<pcl::PointXYZ>;
        //pcl::PointCloud<pcl::PointXYZ>::Ptr xyzCloudPtrFiltered(xyz_cloud_filtered);
        pcl::fromPCLPointCloud2(*cloudFilteredPtr, *xyzCloudPtr);

        /*pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
        sor.setInputCloud(xyzCloudPtr);
        sor.setMeanK(20);
        sor.setStddevMulThresh (1.0);
        sor.filter(*xyzCloudPtrFiltered);*/

        //for obstacle avoidance module
        /*std_msgs::Float64 obs_distance_min;

        obs_distance_min.data = 100;

        for(int nIndex = 0; nIndex < xyzCloudPtrFiltered->points.size(); nIndex++)
        {
            if(obs_distance_min.data > xyzCloudPtrFiltered->points[nIndex].z && xyzCloudPtrFiltered->points[nIndex].x <= 0.40)
            {
                obs_distance_min.data = xyzCloudPtrFiltered->points[nIndex].z;
            }
        }

        std::cout << "obs_distance_min:" << obs_distance_min.data << std::endl;*/

        //for safety module
	distance_min_back = 100;

        for(int nIndex = 0; nIndex < xyzCloudPtr->points.size(); nIndex++)
        {
            position = depth_camera_2_tf*(tf::Vector3(xyzCloudPtr->points[nIndex].x, xyzCloudPtr->points[nIndex].y, xyzCloudPtr->points[nIndex].z));

            distance_x = std::max(std::max(position.getX()-0.465, -0.39-position.getX()), 0.0);
            distance_y = std::max(std::max(position.getY()-0.255, -0.255-position.getY()), 0.0);

            distance = sqrt(distance_x*distance_x+distance_y*distance_y);

            if(distance < distance_min_back && distance > 0.001)
            {
                distance_min_back = distance;

                if(distance < distance_min)
		{
		    distance_min = distance;

                    point_nearest.setX(position.getX());
                    point_nearest.setY(position.getY());
                    point_nearest.setZ(position.getZ());
		}
            }
        }

    }

    obs_det_output.data[0] = distance_min;

    distance_y = std::max(std::max(point_nearest.getY()-0.255, -0.255-point_nearest.getY()), 0.0);

    if((distance_min < distance_maintained) && ((distance_min_front-distance_min_back > 0.1) || (distance_min_front-distance_min_back < -0.1)))
    {
        if(point_nearest.getX() >= 0)
        {   
            obs_det_output.data[1] = std::abs(point_nearest.getX())-sqrt(distance_maintained*distance_maintained-distance_y*distance_y)-0.465;
        }   
        else
        {   
            obs_det_output.data[1] = sqrt(distance_maintained*distance_maintained-distance_y*distance_y)+0.315-std::abs(point_nearest.getX());
        }
    }
    else
    {
        obs_det_output.data[1] = 0;  
    }

    pub2.publish(obs_det_output);

    /*min_distance.header = output.header;
    min_distance.data = obs_det_output.data[0];

    pub3.publish(min_distance);*/

    std::cout << "distance_min:" << distance_min << std::endl << "base_desired_displacement:" << obs_det_output.data[1] << std::endl;

    ROS_INFO("99999");
}

int main (int argc, char** argv)
{
    // Initialize ROS
    ros::init (argc, argv, "test");
    ros::NodeHandle nh;

    // Create a message_filters subscriber for the input point cloud
    message_filters::Subscriber<sensor_msgs::PointCloud2> sub1(nh, "/depth_camera_1/depth/color/points", 1);
    message_filters::Subscriber<sensor_msgs::PointCloud2> sub2(nh, "/depth_camera_2/depth/color/points", 1);

    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, sensor_msgs::PointCloud2> SyncPolicy;

    message_filters::Synchronizer<SyncPolicy> sync(SyncPolicy(10), sub1, sub2);
    sync.registerCallback(boost::bind(&pointcloud_cb, _1, _2));

    // Create a ROS publisher for the output point cloud
    pub1 = nh.advertise<sensor_msgs::PointCloud2> ("/pcl_output", 1);
    pub2 = nh.advertise<std_msgs::Float64MultiArray> ("/obs_det_output", 1);
    //pub3 = nh.advertise<mpc::DisMin> ("/min_distance", 1);

    tf::TransformListener listener;

    while(1)
    {
        try
        {
            listener.lookupTransform("/base_link", "/depth_camera_1_depth_optical_frame", ros::Time(0), depth_camera_1_tf);
            listener.lookupTransform("/base_link", "/depth_camera_2_depth_optical_frame", ros::Time(0), depth_camera_2_tf);
            break;
        }
        catch (tf::TransformException &ex)
        {
            ROS_ERROR("%s",ex.what());
            ros::Duration(0.05).sleep();
        }
     }

    // Spin
    ros::spin ();
}
