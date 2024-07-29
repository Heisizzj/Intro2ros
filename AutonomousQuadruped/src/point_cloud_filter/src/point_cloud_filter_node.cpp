#include <ros/ros.h>
#include <ros/console.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/features/normal_3d.h>
#include <pcl/common/common.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/common/transforms.h>
#include <Eigen/Dense>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/segmentation/sac_segmentation.h>

ros::Publisher pub_filtered_cloud;

void pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg)
{
    // transform ROS message to PCL
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*cloud_msg, *cloud);

    // Remove the ground plane
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);

    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(0.01);

    seg.setInputCloud(cloud);
    seg.segment(*inliers, *coefficients);

    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud(cloud);
    extract.setIndices(inliers);
    extract.setNegative(true); // Extract everything except the ground plane
    extract.filter(*filtered_cloud);


    // transform PCL to ROS message
    sensor_msgs::PointCloud2 filtered_cloud_msg;
    pcl::toROSMsg(*filtered_cloud, filtered_cloud_msg);
    filtered_cloud_msg.header = cloud_msg->header;

    // publish enlarged point cloud
    pub_filtered_cloud.publish(filtered_cloud_msg);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "point_cloud_filter_node");
    ros::NodeHandle nh;
   
    pub_filtered_cloud = nh.advertise<sensor_msgs::PointCloud2>("filtered_cloud", 1);
    ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2>("point_cloud", 1, pointCloudCallback);
    

    ROS_INFO("Runing point_cloud_enlarge_node");

    ros::spin();

    return 0;
}