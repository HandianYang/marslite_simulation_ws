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
#include <vector>
#include <algorithm>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <detection_msgs/Det3D.h>
#include <detection_msgs/Det3DArray.h>

ros::Publisher pub_obs;
tf::StampedTransform depth_camera_front_tf, depth_camera_back_tf;
double distance_maintained = 2;
// int print_con = 0;
std::vector<double> fil_input_front(5, 0), fil_input_back(5, 0);
tf::Vector3 position, point_nearest;
double distance_x, distance_y, distance, distance_min, distance_min_front, distance_min_back;
double prev_distance_min_front = 5.0, prev_distance_min_back = 5.0;
std_msgs::Float64MultiArray obs_det_output;
std::vector<double> vector_sorted;

void person_distance_front_cb(detection_msgs::Det3DArray person_distance_front)
{
    if(person_distance_front.dets_list.size() == 0)
        distance_min_front = 5.0;
    else
    {
        distance_min_front = 5.0;

        position = depth_camera_front_tf*(tf::Vector3(-person_distance_front.dets_list[0].y, -person_distance_front.dets_list[0].z, person_distance_front.dets_list[0].x));

        // std::cout<<"front=== "<<std::endl;
        // std::cout<<"x: "<<position.getX()<<std::endl;
        // std::cout<<"y: "<<position.getY()<<std::endl;
        // std::cout<<"z: "<<position.getZ()<<std::endl<<std::endl;

        distance_x = std::max(std::max(position.getX() - 0.465, -0.39 - position.getX()), 0.0);
        distance_y = std::max(std::max(position.getY() - 0.26, -0.255 - position.getY()), 0.0);

        distance = sqrt(distance_x * distance_x + distance_y * distance_y);

        if(distance < distance_min_front && distance > 0.001)
        {
            distance_min_front = distance;
            point_nearest.setX(position.getX());
            point_nearest.setY(position.getY());
            point_nearest.setZ(position.getZ());
        }
    }

    fil_input_front[0] = fil_input_front[1];
    fil_input_front[1] = fil_input_front[2];
    fil_input_front[2] = fil_input_front[3];
    fil_input_front[3] = fil_input_front[4];
    if(prev_distance_min_front != 5.0 && distance_min_front == 5.0)
        fil_input_front[4] = prev_distance_min_front;
    else
    {
        fil_input_front[4] = distance_min_front;
        prev_distance_min_front = distance_min_front;
    }
    vector_sorted = fil_input_front;
    std::sort(vector_sorted.begin(), vector_sorted.end());
    distance_min_front = vector_sorted[2];
    obs_det_output.data[2] = distance_min_front;
    distance_min = distance_min_front;
}

void person_distance_back_cb(detection_msgs::Det3DArray person_distance_back)
{
    if(person_distance_back.dets_list.size() == 0)
        distance_min_back = 5.0;
    else
    {
        distance_min_back = 5.0;

        position = depth_camera_back_tf*(tf::Vector3(-person_distance_back.dets_list[0].y, -person_distance_back.dets_list[0].z, person_distance_back.dets_list[0].x));

        // std::cout<<"back=== "<<std::endl;
        // std::cout<<"x: "<<position.getX()<<std::endl;
        // std::cout<<"y: "<<position.getY()<<std::endl;
        // std::cout<<"z: "<<position.getZ()<<std::endl<<std::endl;

        distance_x = std::max(std::max(position.getX() - 0.465, -0.39 - position.getX()), 0.0);
        distance_y = std::max(std::max(position.getY() - 0.26, -0.255 - position.getY()), 0.0);

        distance = sqrt(distance_x * distance_x + distance_y * distance_y);

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

    fil_input_back[0] = fil_input_back[1];
    fil_input_back[1] = fil_input_back[2];
    fil_input_back[2] = fil_input_back[3];
    fil_input_back[3] = fil_input_back[4];
    if(prev_distance_min_back != 5.0 && distance_min_back == 5.0)
        fil_input_back[4] = prev_distance_min_back;
    else
    {
        fil_input_back[4] = distance_min_back;
        prev_distance_min_back = distance_min_back;
    }
    vector_sorted = fil_input_back;
    std::sort(vector_sorted.begin(), vector_sorted.end());
    distance_min_back = vector_sorted[2];
    obs_det_output.data[3] = distance_min_back;
    obs_det_output.data[0] = distance_min;

    distance_y = std::max(std::max(point_nearest.getY()-0.26, -0.255 - point_nearest.getY()), 0.0);

    if((distance_min < distance_maintained) && ((distance_min_front - distance_min_back > 0.04) || (distance_min_front - distance_min_back < -0.04)))
    {
        if(point_nearest.getX() >= 0)
        {   
            obs_det_output.data[1] = std::abs(point_nearest.getX()) - sqrt(distance_maintained*distance_maintained-distance_y*distance_y) - 0.465;
        }   
        else
        {   
            obs_det_output.data[1] = sqrt(distance_maintained * distance_maintained - distance_y*distance_y) + 0.39 - std::abs(point_nearest.getX());
        }
    }
    else
    {
        obs_det_output.data[1] = 0;  
    }

    pub_obs.publish(obs_det_output);

    std::cout<<"front min: "<<obs_det_output.data[2]<<std::endl;
    std::cout<<"back min: "<<obs_det_output.data[3]<<std::endl;
    std::cout<<"total min: "<<obs_det_output.data[0]<<std::endl;
    std::cout<<"mars move distance: "<<obs_det_output.data[1]<<std::endl<<std::endl;
}


// void pointcloud_cb(const sensor_msgs::PointCloud2ConstPtr& pc1, const sensor_msgs::PointCloud2ConstPtr& pc2/*, const sensor_msgs::PointCloud2ConstPtr& pc3*/)
// {
//     print_con++;
//     print_con = print_con%10;

//     if(print_con == 0)
//     {
//     	std::cout << "   " << std::endl;
//     	ROS_INFO("11111");
//     }

//     //std::cout << pc1->header.stamp << "   " << ros::Time::now() << std::endl;

//     //double t_now = ros::Time::now().toSec();

//     // Container for original & filtered data
//     pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2; 
//     pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);
//     pcl::PCLPointCloud2* cloud_filtered = new pcl::PCLPointCloud2;
//     pcl::PCLPointCloud2Ptr cloudFilteredPtr(cloud_filtered);
//     pcl::PointCloud<pcl::PointXYZ> *xyz_cloud = new pcl::PointCloud<pcl::PointXYZ>;
//     pcl::PointCloud<pcl::PointXYZ>::Ptr xyzCloudPtr(xyz_cloud);
//     pcl::PassThrough<pcl::PCLPointCloud2> pass;
//     pcl::VoxelGrid<pcl::PCLPointCloud2> vg;
//     sensor_msgs::PointCloud2 output;
//     tf::Vector3 position, point_nearest;
//     double distance_x, distance_y, distance, distance_min, distance_min_front, distance_min_back;
//     std_msgs::Float64MultiArray obs_det_output;
//     std::vector<double> vector_sorted;

//     obs_det_output.data.resize(4);

//     // Convert to PCL data type
//     pcl_conversions::toPCL(*pc1, *cloud);

//     // Create the filtering object
//     pass.setInputCloud(cloudPtr);
//     pass.setFilterFieldName ("x");
//     pass.setFilterLimitsNegative (false);
//     pass.setFilterLimits (-3.5, 0.30);
//     pass.filter (*cloudFilteredPtr);

//     pass.setInputCloud(cloudFilteredPtr);
//     pass.setFilterFieldName ("z");
//     pass.setFilterLimitsNegative (false);
//     pass.setFilterLimits (0.8, 3.5);
//     pass.filter (*cloudFilteredPtr);

//     pass.setInputCloud(cloudFilteredPtr);
//     pass.setFilterFieldName ("y");
//     pass.setFilterLimitsNegative (false);
//     pass.setFilterLimits (-1.3, 0.19);
//     pass.filter (*cloudFilteredPtr);

//     if((*cloud_filtered).data.empty())
//     {
//         distance_min_front = 100;

//         pcl_conversions::fromPCL(*cloud_filtered, output);

//         pub1.publish(output);
//     }
//     else
//     {
//         vg.setInputCloud(cloudFilteredPtr);
//     	vg.setLeafSize (0.025, 0.025, 0.025);
//     	vg.filter(*cloudFilteredPtr);

//     	pcl_conversions::fromPCL(*cloud_filtered, output);

//     	pub1.publish(output);

//     	//pcl::PointCloud<pcl::PointXYZ> *xyz_cloud_filtered = new pcl::PointCloud<pcl::PointXYZ>;
//     	//pcl::PointCloud<pcl::PointXYZ>::Ptr xyzCloudPtrFiltered(xyz_cloud_filtered);
//     	pcl::fromPCLPointCloud2(*cloudFilteredPtr, *xyzCloudPtr);

//     	/*pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
//    	sor.setInputCloud(xyzCloudPtr);
//     	sor.setMeanK(20);
//     	sor.setStddevMulThresh (1.0);
//     	sor.filter(*xyzCloudPtrFiltered);*/

//     	//for obstacle avoidance module
//     	/*std_msgs::Float64 obs_distance_min;

//     	obs_distance_min.data = 100;

//     	for(int nIndex = 0; nIndex < xyzCloudPtrFiltered->points.size(); nIndex++)
//     	{
//             if(obs_distance_min.data > xyzCloudPtrFiltered->points[nIndex].z && xyzCloudPtrFiltered->points[nIndex].x <= 0.40)
//        	    {
//             	obs_distance_min.data = xyzCloudPtrFiltered->points[nIndex].z;
//             }
//     	}

//     	std::cout << "obs_distance_min:" << obs_distance_min.data << std::endl;*/

//     	//for safety module
//     	distance_min_front = 100;

//     	for(int nIndex = 0; nIndex < xyzCloudPtr->points.size(); nIndex++)
//     	{
//             position = depth_camera_front_tf*(tf::Vector3(xyzCloudPtr->points[nIndex].x, xyzCloudPtr->points[nIndex].y, xyzCloudPtr->points[nIndex].z));

//             distance_x = std::max(std::max(position.getX()-0.465, -0.39-position.getX()), 0.0);
//             distance_y = std::max(std::max(position.getY()-0.26, -0.255-position.getY()), 0.0);

//             distance = sqrt(distance_x*distance_x+distance_y*distance_y);

//             if(distance < distance_min_front && distance > 0.001)
//             {
//             	distance_min_front = distance;
//             	point_nearest.setX(position.getX());
//             	point_nearest.setY(position.getY());
//             	point_nearest.setZ(position.getZ());
//             }
//     	}
//     }

//     pass.setInputCloud(cloudPtr);
//     pass.setFilterFieldName ("x");
//     pass.setFilterLimitsNegative (false);
//     pass.setFilterLimits (-3.5, 0.30);
//     pass.filter (*cloudFilteredPtr);

//     pass.setInputCloud(cloudFilteredPtr);
//     pass.setFilterFieldName ("z");
//     pass.setFilterLimitsNegative (false);
//     pass.setFilterLimits (0.0, 0.8);
//     pass.filter (*cloudFilteredPtr);

//     pass.setInputCloud(cloudFilteredPtr);
//     pass.setFilterFieldName ("y");
//     pass.setFilterLimitsNegative (false);
//     pass.setFilterLimits (-0.275, 0.19);
//     pass.filter (*cloudFilteredPtr);

//     if((*cloud_filtered).data.empty())
//     {
//         pcl_conversions::fromPCL(*cloud_filtered, output);

//         //pub1.publish(output);
//     }
//     else
//     {
//         vg.setInputCloud(cloudFilteredPtr);
//     	vg.setLeafSize (0.025, 0.025, 0.025);
//     	vg.filter(*cloudFilteredPtr);

//     	pcl_conversions::fromPCL(*cloud_filtered, output);

//     	//pub1.publish(output);

//     	pcl::fromPCLPointCloud2(*cloudFilteredPtr, *xyzCloudPtr);

//     	for(int nIndex = 0; nIndex < xyzCloudPtr->points.size(); nIndex++)
//     	{
//             position = depth_camera_front_tf*(tf::Vector3(xyzCloudPtr->points[nIndex].x, xyzCloudPtr->points[nIndex].y, xyzCloudPtr->points[nIndex].z));

//             distance_x = std::max(std::max(position.getX()-0.465, -0.39-position.getX()), 0.0);
//             distance_y = std::max(std::max(position.getY()-0.26, -0.255-position.getY()), 0.0);

//             distance = sqrt(distance_x*distance_x+distance_y*distance_y);

//             if(distance < distance_min_front && distance > 0.001)
//             {
//             	distance_min_front = distance;
//             	point_nearest.setX(position.getX());
//             	point_nearest.setY(position.getY());
//             	point_nearest.setZ(position.getZ());
//             }
//     	}
//     } 

//     /*pcl_conversions::toPCL(*pc3, *cloud);

//     // Create the filtering object
//     pass.setInputCloud(cloudPtr);
//     pass.setFilterFieldName ("y");
//     pass.setFilterLimitsNegative (false);
//     pass.setFilterLimits (-0.945, 0.52);
//     pass.filter (*cloudFilteredPtr);

//     pass.setInputCloud(cloudFilteredPtr);
//     pass.setFilterFieldName ("x");
//     pass.setFilterLimitsNegative (false);
//     pass.setFilterLimits (0, 3.5);
//     pass.filter (*cloudFilteredPtr);

//     if((*cloud_filtered).data.empty())
//     {
//         pcl_conversions::fromPCL(*cloud_filtered, output);

//         pub1.publish(output);
//     }
//     else
//     {
//         vg.setInputCloud(cloudFilteredPtr);
//     	vg.setLeafSize (0.025, 0.025, 0.025);
//     	vg.filter(*cloudFilteredPtr);

//     	pcl_conversions::fromPCL(*cloud_filtered, output);

//     	pub1.publish(output);

//     	pcl::fromPCLPointCloud2(*cloudFilteredPtr, *xyzCloudPtr);

//     	for(int nIndex = 0; nIndex < xyzCloudPtr->points.size(); nIndex++)
//     	{
//             position = depth_camera_3_tf*(tf::Vector3(xyzCloudPtr->points[nIndex].x, xyzCloudPtr->points[nIndex].y, xyzCloudPtr->points[nIndex].z));

//             distance_x = std::max(std::max(position.getX()-0.465, -0.39-position.getX()), 0.0);
//             distance_y = std::max(std::max(position.getY()-0.26, -0.255-position.getY()), 0.0);

//             distance = sqrt(distance_x*distance_x+distance_y*distance_y);

//             if(distance < distance_min_front && distance > 0.001)
//             {
//             	distance_min_front = distance;
//             	point_nearest.setX(position.getX());
//             	point_nearest.setY(position.getY());
//             	point_nearest.setZ(position.getZ());
//             }
//     	}
//     }*/

//     fil_input_front[0] = fil_input_front[1];
//     fil_input_front[1] = fil_input_front[2];
//     fil_input_front[2] = fil_input_front[3];
//     fil_input_front[3] = fil_input_front[4];
//     fil_input_front[4] = distance_min_front;
//     vector_sorted = fil_input_front;
//     std::sort(vector_sorted.begin(), vector_sorted.end());
//     distance_min_front = vector_sorted[2];

//     obs_det_output.data[2] = distance_min_front;

//     if(print_con == 0)
//     	std::cout << "distance_min_front:" << distance_min_front << std::endl;

//     distance_min = distance_min_front;

//     /*pass.setInputCloud(cloudPtr);
//     pass.setFilterFieldName ("y");
//     pass.setFilterLimitsNegative (false);
//     pass.setFilterLimits (-0.945, 0.52);
//     pass.filter (*cloudFilteredPtr);

//     pass.setInputCloud(cloudFilteredPtr);
//     pass.setFilterFieldName ("x");
//     pass.setFilterLimitsNegative (false);
//     pass.setFilterLimits (-3.5, 0);
//     pass.filter (*cloudFilteredPtr);

//     if((*cloud_filtered).data.empty())
//     {
// 	distance_min_back = 100;

//         pcl_conversions::fromPCL(*cloud_filtered, output);

//         pub1.publish(output);
//     }
//     else
//     {
//         vg.setInputCloud(cloudFilteredPtr);
//         vg.setLeafSize (0.025, 0.025, 0.025);
//         vg.filter(*cloudFilteredPtr);

//         pcl_conversions::fromPCL(*cloud_filtered, output);

//         pub1.publish(output);

//         pcl::fromPCLPointCloud2(*cloudFilteredPtr, *xyzCloudPtr);

// 	distance_min_back = 100;

//         for(int nIndex = 0; nIndex < xyzCloudPtr->points.size(); nIndex++)
//         {
//             position = depth_camera_3_tf*(tf::Vector3(xyzCloudPtr->points[nIndex].x, xyzCloudPtr->points[nIndex].y, xyzCloudPtr->points[nIndex].z));

//             distance_x = std::max(std::max(position.getX()-0.465, -0.39-position.getX()), 0.0);
//             distance_y = std::max(std::max(position.getY()-0.26, -0.255-position.getY()), 0.0);

//             distance = sqrt(distance_x*distance_x+distance_y*distance_y);

//             if(distance < distance_min_back && distance > 0.001)
//             {
//                 distance_min_back = distance;

//                 if(distance < distance_min)
// 		{
// 		    distance_min = distance;

//                     point_nearest.setX(position.getX());
//                     point_nearest.setY(position.getY());
//                     point_nearest.setZ(position.getZ());
// 		}
//             }
//         }
//     }*/

//     pcl_conversions::toPCL(*pc2, *cloud);

//     // Create the filtering object
//     pass.setInputCloud(cloudPtr);
//     pass.setFilterFieldName ("y");
//     pass.setFilterLimitsNegative (false);
//     pass.setFilterLimits (-0.945, 0.52);
//     pass.filter (*cloudFilteredPtr);

//     pass.setInputCloud(cloudFilteredPtr);
//     pass.setFilterFieldName ("x");
//     pass.setFilterLimitsNegative (false);
//     pass.setFilterLimits (-0.30, 3.5);
//     pass.filter (*cloudFilteredPtr);

//     if((*cloud_filtered).data.empty())
//     {
// 	distance_min_back = 100; //if there are 3 cameras, delete it; 

//         pcl_conversions::fromPCL(*cloud_filtered, output);

//         //pub1.publish(output);
//     }
//     else
//     {
//         vg.setInputCloud(cloudFilteredPtr);
//         vg.setLeafSize (0.025, 0.025, 0.025);
//         vg.filter(*cloudFilteredPtr);

//         pcl_conversions::fromPCL(*cloud_filtered, output);

//         //pub1.publish(output);

//         //pcl::PointCloud<pcl::PointXYZ> *xyz_cloud_filtered = new pcl::PointCloud<pcl::PointXYZ>;
//         //pcl::PointCloud<pcl::PointXYZ>::Ptr xyzCloudPtrFiltered(xyz_cloud_filtered);
//         pcl::fromPCLPointCloud2(*cloudFilteredPtr, *xyzCloudPtr);

//         /*pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
//         sor.setInputCloud(xyzCloudPtr);
//         sor.setMeanK(20);
//         sor.setStddevMulThresh (1.0);
//         sor.filter(*xyzCloudPtrFiltered);*/

//         //for obstacle avoidance module
//         /*std_msgs::Float64 obs_distance_min;

//         obs_distance_min.data = 100;

//         for(int nIndex = 0; nIndex < xyzCloudPtrFiltered->points.size(); nIndex++)
//         {
//             if(obs_distance_min.data > xyzCloudPtrFiltered->points[nIndex].z && xyzCloudPtrFiltered->points[nIndex].x <= 0.40)
//             {
//                 obs_distance_min.data = xyzCloudPtrFiltered->points[nIndex].z;
//             }
//         }

//         std::cout << "obs_distance_min:" << obs_distance_min.data << std::endl;*/

// 	    distance_min_back = 100; //if there are 3 cameras, delete it;

//         //for safety module

//         for(int nIndex = 0; nIndex < xyzCloudPtr->points.size(); nIndex++)
//         {
//             position = depth_camera_back_tf*(tf::Vector3(xyzCloudPtr->points[nIndex].x, xyzCloudPtr->points[nIndex].y, xyzCloudPtr->points[nIndex].z));

//             distance_x = std::max(std::max(position.getX()-0.465, -0.39-position.getX()), 0.0);
//             distance_y = std::max(std::max(position.getY()-0.26, -0.255-position.getY()), 0.0);

//             distance = sqrt(distance_x*distance_x+distance_y*distance_y);

//             if(distance < distance_min_back && distance > 0.001)
//             {
//                 distance_min_back = distance;

//                 if(distance < distance_min)
//                 {
//                     distance_min = distance;

//                     point_nearest.setX(position.getX());
//                     point_nearest.setY(position.getY());
//                     point_nearest.setZ(position.getZ());
// 		        }
//             }
//         }
//     }

//     fil_input_back[0] = fil_input_back[1];
//     fil_input_back[1] = fil_input_back[2];
//     fil_input_back[2] = fil_input_back[3];
//     fil_input_back[3] = fil_input_back[4];
//     fil_input_back[4] = distance_min_back; 
//     vector_sorted = fil_input_back;
//     std::sort(vector_sorted.begin(), vector_sorted.end());
//     distance_min_back = vector_sorted[2];

//     obs_det_output.data[3] = distance_min_back;

//     if(print_con == 0)
//     	std::cout << "distance_min_back:" << distance_min_back << std::endl;

//     obs_det_output.data[0] = distance_min;

//     distance_y = std::max(std::max(point_nearest.getY()-0.26, -0.255-point_nearest.getY()), 0.0);

//     if((distance_min < distance_maintained) && ((distance_min_front-distance_min_back > 0.04) || (distance_min_front-distance_min_back < -0.04)))
//     {
//         if(point_nearest.getX() >= 0)
//         {   
//             obs_det_output.data[1] = std::abs(point_nearest.getX())-sqrt(distance_maintained*distance_maintained-distance_y*distance_y)-0.465;
//         }   
//         else
//         {   
//             obs_det_output.data[1] = sqrt(distance_maintained*distance_maintained-distance_y*distance_y)+0.39-std::abs(point_nearest.getX());
//         }
//     }
//     else
//     {
//         obs_det_output.data[1] = 0;  
//     }

//     if(print_con == 0)
//     	std::cout << "nearest point:" << point_nearest.getX() << ", " << point_nearest.getY() << ", " << point_nearest.getZ() << std::endl;

//     pub2.publish(obs_det_output);

//     if(print_con == 0)
//     	std::cout << "distance_min:" << distance_min << std::endl << "base_desired_displacement:" << obs_det_output.data[1] << std::endl;

//     //if(ros::Time::now().toSec()-t_now>max_t)
// 	//max_t = ros::Time::now().toSec()-t_now;

//     //std::cout << "max_t" << max_t << std::endl;

//     if(print_con == 0)
//     {
//     	ROS_INFO("99999");
//     	std::cout << "   " << std::endl;
//     }
// }


int main (int argc, char** argv)
{
    // Initialize ROS
    ros::init (argc, argv, "test");
    ros::NodeHandle nh;

    obs_det_output.data.resize(4);

    // Create a message_filters subscriber for the input point cloud
    // message_filters::Subscriber<sensor_msgs::PointCloud2> sub1(nh, "/depth_camera_1/depth/color/points", 1);
    // message_filters::Subscriber<sensor_msgs::PointCloud2> sub2(nh, "/depth_camera_2/depth/color/points", 1);
    //message_filters::Subscriber<sensor_msgs::PointCloud2> sub3(nh, "/depth_camera_3/depth/color/points", 1);

    // typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, sensor_msgs::PointCloud2/*, sensor_msgs::PointCloud2*/> SyncPolicy;

    // message_filters::Synchronizer<SyncPolicy> sync(SyncPolicy(10), sub1, sub2/*, sub3*/);
    // sync.registerCallback(boost::bind(&pointcloud_cb, _1, _2/*, _3*/));

    // message_filters::Subscriber<detection_msgs::Det3DArray> sub1(nh, "/scan_person_clustering_front_node/det3d_result", 1);
    // message_filters::Subscriber<detection_msgs::Det3DArray> sub2(nh, "/scan_person_clustering_back_node/det3d_result", 1);
    // typedef message_filters::sync_policies::ApproximateTime<detection_msgs::Det3DArray, detection_msgs::Det3DArray> SyncPolicy;
    // message_filters::Synchronizer<SyncPolicy> sync(SyncPolicy(10), sub1, sub2);
    // sync.registerCallback(boost::bind(&person_distance_cb, _1, _2));

    ros::Subscriber sub_front = nh.subscribe("/scan_person_clustering_front_node/det3d_result", 1, person_distance_front_cb);
    ros::Subscriber sub_back = nh.subscribe("/scan_person_clustering_back_node/det3d_result", 1, person_distance_back_cb);

    // Create a ROS publisher for the output point cloud
    // ros::Publisher pub1 = nh.advertise<sensor_msgs::PointCloud2> ("/pcl_output", 1);
    
    pub_obs = nh.advertise<std_msgs::Float64MultiArray> ("/obs_det_output", 1);

    tf::TransformListener listener;

    while(1)
    {
        try
        {
            listener.waitForTransform("/base_link", "/camera3_depth_optical_frame", ros::Time(0), ros::Duration(3.0));
            listener.lookupTransform("/base_link", "/camera3_depth_optical_frame", ros::Time(0), depth_camera_front_tf);
            listener.waitForTransform("/base_link", "/camera2_depth_optical_frame", ros::Time(0), ros::Duration(3.0));
            listener.lookupTransform("/base_link", "/camera2_depth_optical_frame", ros::Time(0), depth_camera_back_tf);
            break;
        }
        catch (tf::TransformException &ex)
        {
            ROS_ERROR("%s",ex.what());
            ros::Duration(0.05).sleep();
        }
    }

    // Spin
    ros::spin();
}
