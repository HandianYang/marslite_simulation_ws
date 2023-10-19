#include <ros/ros.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>
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

ros::Publisher pub;
ros::Publisher pub2;
tf::StampedTransform camera_tf;
double distance_maintained = 3;

void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input)
{
    ROS_INFO("11111");

    // Container for original & filtered data
    pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2; 
    pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);
    pcl::PCLPointCloud2* cloud_filtered = new pcl::PCLPointCloud2;
    pcl::PCLPointCloud2Ptr cloudFilteredPtr(cloud_filtered);

    // Convert to PCL data type
    pcl_conversions::toPCL(*input, *cloud);

    // Create the filtering object
    pcl::PassThrough<pcl::PCLPointCloud2> pass;
    pass.setInputCloud(cloudPtr);
    pass.setFilterFieldName ("y");
    pass.setFilterLimitsNegative (false);
    pass.setFilterLimits (-1.3, 0.2);
    pass.filter (*cloudFilteredPtr);

    pass.setInputCloud(cloudFilteredPtr);
    pass.setFilterFieldName ("z");
    pass.setFilterLimitsNegative (false);
    pass.setFilterLimits (0, 5);
    pass.filter (*cloudFilteredPtr);

    pass.setInputCloud(cloudFilteredPtr);
    pass.setFilterFieldName ("x");
    pass.setFilterLimitsNegative (false);
    pass.setFilterLimits (-0.40, 100);
    pass.filter (*cloudFilteredPtr);

    if((*cloud_filtered).data.empty())
    {
        sensor_msgs::PointCloud2 output;
        std_msgs::Float64 distance_min;
        distance_min.data = 100;

        std::cout << distance_min.data << std::endl;

        pcl_conversions::fromPCL(*cloud_filtered, output);

        pub.publish(output);
        pub2.publish(distance_min);

        return;
    }
 
    pcl::VoxelGrid<pcl::PCLPointCloud2> vg;
    vg.setInputCloud(cloudFilteredPtr);
    vg.setLeafSize (0.025, 0.025, 0.025);
    vg.filter(*cloudFilteredPtr);

    pcl::PointCloud<pcl::PointXYZ> *xyz_cloud = new pcl::PointCloud<pcl::PointXYZ>;
    pcl::PointCloud<pcl::PointXYZ>::Ptr xyzCloudPtr(xyz_cloud);
    pcl::PointCloud<pcl::PointXYZ> *xyz_cloud_filtered = new pcl::PointCloud<pcl::PointXYZ>;
    pcl::PointCloud<pcl::PointXYZ>::Ptr xyzCloudPtrFiltered(xyz_cloud_filtered);

    pcl::fromPCLPointCloud2(*cloudFilteredPtr, *xyzCloudPtr);

    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    sor.setInputCloud(xyzCloudPtr);
    sor.setMeanK(20);
    sor.setStddevMulThresh (1.0);
    sor.filter(*xyzCloudPtrFiltered);

    sensor_msgs::PointCloud2 output;
    pcl::PCLPointCloud2 outputPCL;

    pcl::toPCLPointCloud2(*xyzCloudPtrFiltered ,outputPCL);
    pcl_conversions::fromPCL(outputPCL, output);

    pub.publish(output); 

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
    tf::Vector3 position, point_nearest;
    double distance_x, distance_y, distance, distance_min;
    std_msgs::Float64MultiArray obs_det_output;
        
    obs_det_output.data.resize(2);

    distance_min = 100;

    for(int nIndex = 0; nIndex < xyzCloudPtrFiltered->points.size(); nIndex++)
    {
        position = camera_tf*(tf::Vector3(xyzCloudPtrFiltered->points[nIndex].x, xyzCloudPtrFiltered->points[nIndex].y, xyzCloudPtrFiltered->points[nIndex].z));

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

    std::cout << "distance_min:" << distance_min << std::endl;
    obs_det_output.data[0] = distance_min;

    distance_y = std::max(std::max(point_nearest.getY()-0.255, -0.255-point_nearest.getY()), 0.0);

    if(distance_min < distance_maintained)
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

    ROS_INFO("99999");
}

int main (int argc, char** argv)
{
    // Initialize ROS
    ros::init (argc, argv, "test");
    ros::NodeHandle nh;

    // Create a ROS subscriber for the input point cloud
    ros::Subscriber sub = nh.subscribe("/realsense_camera/depth/color/points", 1, cloud_cb);

    // Create a ROS publisher for the output point cloud
    pub = nh.advertise<sensor_msgs::PointCloud2> ("/pcl_output", 1);
    pub2 = nh.advertise<std_msgs::Float64MultiArray> ("/obs_det_output", 1);

    tf::TransformListener listener;

    while(1)
    {
        try
        {
            listener.lookupTransform("/base_link", "/realsense_camera_depth_optical_frame", ros::Time(0), camera_tf);
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
