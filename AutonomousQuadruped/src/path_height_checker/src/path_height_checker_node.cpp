#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <octomap_msgs/Octomap.h>
#include <octomap/OcTree.h>
#include <octomap/AbstractOcTree.h>
#include <octomap_msgs/conversions.h>

#include <nav_msgs/Path.h>
#include <std_msgs/Bool.h>
#include <octomap/OccupancyOcTreeBase.h>

#include <std_msgs/Float32.h>
#include <path_height_checker/PathHeightStatus.h>
#include <path_height_checker/PathHeightStatusArray.h>

class PathOccupancyChecker
{
public:
    PathOccupancyChecker()
    {
        // 订阅路径点话题
        path_sub_ = nh_.subscribe("move_base_Quadruped/NavfnROS/plan", 1, &PathOccupancyChecker::pathCallback, this);

        // 订阅OctoMap话题
        octomap_sub_ = nh_.subscribe("octomap_full", 1, &PathOccupancyChecker::octomapCallback, this);

        // 发布高度状态路径
        path_height_pub_ = nh_.advertise<path_height_checker::PathHeightStatusArray>("path_height_status", 1);

        // 从参数服务器读取参数
        nh_.param("height_threshold", height_threshold_, 0.05);
        nh_.param("zmin", zmin_, 0.01);
        nh_.param("zmax", zmax_, 0.15);

        // 初始化标志
        octree_ = nullptr;
    }

    ~PathOccupancyChecker()
    {
        if (octree_) {
            delete octree_;
        }
    }

private:
    void octomapCallback(const octomap_msgs::Octomap::ConstPtr& msg)
    {
        // 将OctoMap消息转换为Octree对象
        octomap::AbstractOcTree* abstract_tree = octomap_msgs::msgToMap(*msg);
        if (octree_) {
            delete octree_;
        }
        octree_ = dynamic_cast<octomap::OcTree*>(abstract_tree);
    }

    void pathCallback(const nav_msgs::Path::ConstPtr& path)
    {
        if (!octree_)
        {
            ROS_WARN("OctoMap has not been received yet.");
            return;
        }

        path_height_checker::PathHeightStatusArray path_height_status_array;

        for (const auto& pose : path->poses)
        {
            path_height_checker::PathHeightStatus path_height_status;
            path_height_status.pose = pose;

            bool height_exceeds_threshold = false;
            for (double z = zmin_; z <= zmax_; z += octree_->getResolution())
            {
                octomap::point3d coord(pose.pose.position.x, pose.pose.position.y, z);

                // 查找对应的栅格
                octomap::OcTreeNode* node = octree_->search(coord);

                if (node)
                {
                    double occupancy_prob = node->getOccupancy();

                    // 如果占用概率大于0.8，并且高度超过阈值，则记录高度
                    if (occupancy_prob > 0.8 && z > height_threshold_)
                    {
                        height_exceeds_threshold = true;
                        path_height_status.height_status.data = z;
                        ROS_INFO("Point (%.2f, %.2f, %.2f) exceeds height threshold. Height: %.2f", pose.pose.position.x, pose.pose.position.y, z, z);
                        break; // 跳出当前路径点的高度检查
                    }
                }
            }
            if (!height_exceeds_threshold)
            {
                // 如果没有超过高度阈值的情况，则设置高度为0.0
                path_height_status.height_status.data = 0.0;
                ROS_INFO("Point (%.2f, %.2f) does not exceed height threshold. Published height: 0.0", pose.pose.position.x, pose.pose.position.y);
            }

            path_height_status_array.path_statuses.push_back(path_height_status);
        }

        // 发布路径高度状态
        path_height_pub_.publish(path_height_status_array);
    }

    ros::NodeHandle nh_;
    ros::Subscriber path_sub_;
    ros::Subscriber octomap_sub_;
    ros::Publisher path_height_pub_;
    octomap::OcTree* octree_;
    double height_threshold_;
    double zmin_;
    double zmax_;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "path_height_checker");

    PathOccupancyChecker checker;

    ros::spin();

    return 0;
}
