#include <ros/ros.h>
#include <ros/console.h>
#include <tf/transform_datatypes.h>
#include <tf_conversions/tf_eigen.h>
#include <eigen_conversions/eigen_msg.h>
#include <mav_msgs/Actuators.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <math.h>
#include <std_msgs/Float64.h>
#include <eigen3/Eigen/Dense>
#include <deque>

#define PI M_PI

class controllerNode {
  ros::NodeHandle nh;

  ros::Publisher commands;
  ros::Subscriber plan_sub;
  ros::Subscriber state_est_sub;
  ros::Timer control_timer;
  ros::Timer plan_update_timer;

  // Controller internals
  Eigen::Vector3d x;     // current position of the robot
  Eigen::Quaterniond q;  // current orientation of the robot
  Eigen::Vector3d xd;    // desired position of the robot

  double hz;             // frequency of the main control loop
  double distance_threshold; // distance threshold for path points
  bool first_plan_received; // flag to indicate if the first plan message has been received

public:
  controllerNode(): hz(100.0), distance_threshold(1.0), first_plan_received(false) {
    commands = nh.advertise<mav_msgs::Actuators>("commands", 1);
    plan_sub = nh.subscribe("/move_base_Quadruped/NavfnROS/plan", 1, &controllerNode::planCallback, this);
    state_est_sub = nh.subscribe("/current_state_est", 1, &controllerNode::stateEstCallback, this);
    control_timer = nh.createTimer(ros::Rate(hz), &controllerNode::controlLoop, this);
    plan_update_timer = nh.createTimer(ros::Duration(2.0), &controllerNode::planUpdateCallback, this);
  }

  void planCallback(const nav_msgs::Path::ConstPtr& msg) {
    if (!msg->poses.empty()) {
      for (const auto& pose : msg->poses) {
        Eigen::Vector3d path_point(pose.pose.position.x, pose.pose.position.y, pose.pose.position.z);
        double distance = (path_point - x).norm();
        if (distance > distance_threshold) {
          xd = path_point;
          first_plan_received = true; // Mark that the first plan message has been received
          ROS_INFO("Received new target pose: x=%f, y=%f, z=%f", xd.x(), xd.y(), xd.z());
          break;
        }
      }
    }
  }

  void stateEstCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    // Update current position and orientation
    x = Eigen::Vector3d(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);
    tf::quaternionMsgToEigen(msg->pose.pose.orientation, q);

    // ROS_INFO("Current Pose: x=%f, y=%f, z=%f", x.x(), x.y(), x.z());
  }

  void planUpdateCallback(const ros::TimerEvent& event) {
    // Trigger the plan callback manually
    ros::topic::waitForMessage<nav_msgs::Path>("/move_base_Quadruped/NavfnROS/plan", nh, ros::Duration(1.0));
  }

  void checkRotation(const Eigen::Vector3d& position, const Eigen::Vector3d& goal, mav_msgs::Actuators& msg) {
    Eigen::Vector2d pos_2d(position.x(), position.y());
    Eigen::Vector2d goal_2d(goal.x(), goal.y());

    // Current robot orientation as a 2D vector
    double yaw = tf::getYaw(tf::Quaternion(q.x(), q.y(), q.z(), q.w()));
    Eigen::Vector2d orientation_2d(cos(yaw), sin(yaw));

    // Vector from robot to goal
    Eigen::Vector2d dist = goal_2d - pos_2d;

    // Normalize vectors
    Eigen::Vector2d distance_normalized = dist / dist.norm();
    Eigen::Vector2d orientation_normalized = orientation_2d / orientation_2d.norm();

    // Cosine of the angle between the two vectors
    double cos_angle = distance_normalized.dot(orientation_normalized);

    // Use cross product to determine the rotation direction
    if (orientation_normalized.x() * distance_normalized.y() - orientation_normalized.y() * distance_normalized.x() <= 0) {
      msg.angular_velocities = {0, 45, 0, 0, 7}; // Rotate clockwise
    } else {
      msg.angular_velocities = {0, -45, 0, 0, 7}; // Rotate counterclockwise
    }

    // If the goal is directly in front, move forward
    if (cos_angle > 0.87) { // Adjust the threshold as needed
      msg.angular_velocities = {0, 90, 0, 0, 8}; // Move forward
    }
  }

  void controlLoop(const ros::TimerEvent& t) {
    mav_msgs::Actuators msg;

    if (!first_plan_received) {
      // If no plan received, keep the robot stationary
      msg.angular_velocities = {0, 0, 0, 0, 0};
      commands.publish(msg);
      return;
    }

    // Check rotation and set the control command
    checkRotation(x, xd, msg);

    commands.publish(msg);
  }
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "controller_node");
  ROS_INFO_NAMED("controller", "Controller started!");
  controllerNode n;
  ros::spin();
}