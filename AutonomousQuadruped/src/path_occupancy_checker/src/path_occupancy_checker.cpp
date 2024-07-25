#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <octomap_msgs/Octomap.h>
#include <octomap/OcTree.h>
#include <octomap/AbstractOcTree.h>
#include <octomap_msgs/conversions.h>

#include <nav_msgs/Path.h>
#include <std_msgs/Bool.h>
#include <octomap/OccupancyOcTreeBase.h>

/*
class PathOccupancyChecker
{
public:
    PathOccupancyChecker()
    {
        // 订阅路径点话题
        path_sub_ = nh_.subscribe("move_base_Quadruped/NavfnROS/plan", 1, &PathOccupancyChecker::pathCallback, this);

        // 订阅OctoMap话题
        octomap_sub_ = nh_.subscribe("octomap_full", 1, &PathOccupancyChecker::octomapCallback, this);

        // 发布占用状态
        occupancy_pub_ = nh_.advertise<std_msgs::Bool>("path_occupancy_status", 1);

        // 初始化标志
        octree_ = nullptr;
    }

private:
    void octomapCallback(const octomap_msgs::Octomap::ConstPtr& msg)
    {
        // 将OctoMap消息转换为Octree对象
        octomap::AbstractOcTree* abstract_tree = octomap_msgs::msgToMap(*msg);
        octree_ = dynamic_cast<octomap::OcTree*>(abstract_tree);
    }

    void pathCallback(const nav_msgs::Path::ConstPtr& path)
    {
        if (!octree_)
        {
            ROS_WARN("OctoMap has not been received yet.");
            return;
        }

        bool is_occupied = false;

        for (const auto& pose : path->poses)
        {
            octomap::point3d coord(pose.pose.position.x, pose.pose.position.y, 0.0);

            // 查找对应的栅格
            octomap::OcTreeNode* node = octree_->search(coord);

            if (node)
            {
                double occupancy_prob = node->getOccupancy();

                // 判断是否被占用
                if (occupancy_prob > 0.5) // 占用概率大于0.5表示被占用
                {
                    is_occupied = true;
                    break;
                }
            }
        }

        // 发布占用状态
        std_msgs::Bool occupancy_msg;
        occupancy_msg.data = is_occupied;
        occupancy_pub_.publish(occupancy_msg);
    }

    ros::NodeHandle nh_;
    ros::Subscriber path_sub_;
    ros::Subscriber octomap_sub_;
    ros::Publisher occupancy_pub_;
    octomap::OcTree* octree_;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "path_occupancy_checker");

    PathOccupancyChecker checker;

    ros::spin();

    return 0;
}
*/

class PathOccupancyChecker
{
public:
    PathOccupancyChecker()
    {
        // 订阅路径点话题
        path_sub_ = nh_.subscribe("move_base_Quadruped/NavfnROS/plan", 1, &PathOccupancyChecker::pathCallback, this);

        // 订阅OctoMap话题
        octomap_sub_ = nh_.subscribe("octomap_full", 1, &PathOccupancyChecker::octomapCallback, this);

        // 发布占用状态
        occupancy_pub_ = nh_.advertise<std_msgs::Bool>("path_occupancy_status", 1);

        // 初始化标志
        octree_ = nullptr;
    }

private:
    void octomapCallback(const octomap_msgs::Octomap::ConstPtr& msg)
    {
        // 将OctoMap消息转换为Octree对象
        octomap::AbstractOcTree* abstract_tree = octomap_msgs::msgToMap(*msg);
        octree_ = dynamic_cast<octomap::OcTree*>(abstract_tree);
    }

    void pathCallback(const nav_msgs::Path::ConstPtr& path)
    {
        if (!octree_)
        {
            ROS_WARN("OctoMap has not been received yet.");
            return;
        }

        bool is_occupied = false;

        // 使用ROS_INFO打印路径点的占用状态
        for (const auto& pose : path->poses)
        {
            octomap::point3d coord(pose.pose.position.x, pose.pose.position.y, 0.0);

            // 查找对应的栅格
            octomap::OcTreeNode* node = octree_->search(coord);

            if (node)
            {
                double occupancy_prob = node->getOccupancy();

                // 判断是否被占用
                if (occupancy_prob > 0.5) // 占用概率大于0.5表示被占用
                {
                    ROS_INFO("Point (%.2f, %.2f) is occupied.", pose.pose.position.x, pose.pose.position.y);
                    is_occupied = true;
                }
                else
                {
                    ROS_INFO("Point (%.2f, %.2f) is not occupied.", pose.pose.position.x, pose.pose.position.y);
                }
            }
            else
            {
                ROS_INFO("Point (%.2f, %.2f) is not in the map.", pose.pose.position.x, pose.pose.position.y);
            }
        }

        // 发布占用状态
        std_msgs::Bool occupancy_msg;
        occupancy_msg.data = is_occupied;
        occupancy_pub_.publish(occupancy_msg);
    }

    ros::NodeHandle nh_;
    ros::Subscriber path_sub_;
    ros::Subscriber octomap_sub_;
    ros::Publisher occupancy_pub_;
    octomap::OcTree* octree_;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "path_occupancy_checker");

    PathOccupancyChecker checker;

    ros::spin();

    return 0;
}