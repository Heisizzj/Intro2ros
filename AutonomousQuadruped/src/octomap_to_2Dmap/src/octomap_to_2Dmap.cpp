#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <octomap_msgs/Octomap.h>
#include <octomap/OcTree.h>
#include <octomap/AbstractOcTree.h>
#include <octomap_msgs/conversions.h>

ros::Publisher occupancy_grid_pub;

void octomapCallback(const octomap_msgs::Octomap::ConstPtr& msg)
{
    octomap::AbstractOcTree* tree = octomap_msgs::fullMsgToMap(*msg);
    octomap::OcTree* octree = dynamic_cast<octomap::OcTree*>(tree);

    if (octree)
    {
        ROS_INFO("Successfully converted Octomap message to OcTree.");

        // 创建OccupancyGrid消息
        nav_msgs::OccupancyGrid occ_grid;
        occ_grid.header.stamp = ros::Time::now();
        occ_grid.header.frame_id = "map";
        occ_grid.info.resolution = octree->getResolution();

        // 设置地图尺寸（这里假设地图的尺寸和八叉树一样大）
        double minX, minY, minZ;
        double maxX, maxY, maxZ;
        octree->getMetricMin(minX, minY, minZ);
        octree->getMetricMax(maxX, maxY, maxZ);
        occ_grid.info.width = (maxX - minX) / occ_grid.info.resolution;
        occ_grid.info.height = (maxY - minY) / occ_grid.info.resolution;
        occ_grid.info.origin.position.x = minX;
        occ_grid.info.origin.position.y = minY;
        occ_grid.info.origin.position.z = 0.0;
        occ_grid.info.origin.orientation.w = 1.0;

        occ_grid.data.resize(occ_grid.info.width * occ_grid.info.height, -1);

        for (octomap::OcTree::iterator it = octree->begin(), end = octree->end(); it != end; ++it)
        {
            if (octree->isNodeOccupied(*it))
            {
                double x = it.getX();
                double y = it.getY();
                double z = it.getZ();

                if (z < minZ + occ_grid.info.resolution)
                {
                    unsigned int idx_x = (x - minX) / occ_grid.info.resolution;
                    unsigned int idx_y = (y - minY) / occ_grid.info.resolution;
                    unsigned int index = idx_y * occ_grid.info.width + idx_x;

                    occ_grid.data[index] = 100;
                }
            }
        }

        // 发布OccupancyGrid消息
        occupancy_grid_pub.publish(occ_grid);
    }
    else
    {
        ROS_ERROR("Failed to convert Octomap message to OcTree.");
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "octomap_to_2Dmap");
    ros::NodeHandle nh;

    // 初始化发布器
    occupancy_grid_pub = nh.advertise<nav_msgs::OccupancyGrid>("occupancy_grid_2Dmap", 1);

    // 订阅Octomap消息
    ros::Subscriber sub = nh.subscribe("octomap_full", 1, octomapCallback);

    ros::spin();

    return 0;
}
