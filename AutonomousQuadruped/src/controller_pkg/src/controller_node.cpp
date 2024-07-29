#include <ros/ros.h>
#include <ros/console.h>
#include <tf/transform_datatypes.h>
#include <tf_conversions/tf_eigen.h>
#include <eigen_conversions/eigen_msg.h>
#include <mav_msgs/Actuators.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <math.h>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>
#include <eigen3/Eigen/Dense>
#include <deque>

#define PI M_PI

enum RobotState {
  ROTATING,
  MOVING_FORWARD,
  IDLE
};

class controllerNode {
  ros::NodeHandle nh;

  ros::Publisher commands;
  ros::Subscriber state_est_sub;
  ros::Subscriber path_sub;
  ros::Subscriber keyoverride_sub;
  ros::Timer control_timer;
  ros::Timer log_timer;
  ros::Timer path_update_timer;

  // Controller internals
  Eigen::Vector3d x;         // current position of the robot
  Eigen::Quaterniond q;      // current orientation of the robot
  std::deque<Eigen::Vector3d> path_points; // filtered path points
  Eigen::Vector3d xd;        // desired position of the robot
  Eigen::Vector3d initial_x; // initial position of the robot
  Eigen::Quaterniond initial_q; // initial orientation of the robot

  double hz;                // frequency of the main control loop
  double angle_tolerance;   // tolerance for angle alignment
  double position_tolerance; // tolerance for position alignment
  RobotState state;         // current state of the robot
  bool initial_state_set;   // flag to indicate if the initial state has been set
  std::string override_from_key;

public:
  controllerNode(): hz(40.0), angle_tolerance(0.1), position_tolerance(0.15), state(IDLE), initial_state_set(false) {
    commands = nh.advertise<mav_msgs::Actuators>("commands", 1);
    state_est_sub = nh.subscribe("/current_state_est", 1, &controllerNode::stateEstCallback, this);
    path_sub = nh.subscribe("/move_base_Quadruped/NavfnROS/plan", 1, &controllerNode::pathCallback, this);
    //keyoverride_sub = nh.subscribe("keyoverride", 1, &controllerNode::keyoverrideCallback, this);
    control_timer = nh.createTimer(ros::Rate(hz), &controllerNode::controlLoop, this);
    log_timer = nh.createTimer(ros::Duration(1.0), &controllerNode::logInfo, this); // Log info every second
    path_update_timer = nh.createTimer(ros::Duration(20.0), &controllerNode::updatePath, this); // Update path every second
  }

  void stateEstCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    // Update current position and orientation
    x = Eigen::Vector3d(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);
    tf::quaternionMsgToEigen(msg->pose.pose.orientation, q);

    // Set initial state if not already set
    if (!initial_state_set) {
      initial_x = x;
      initial_q = q;
      initial_state_set = true;
    }
  }

  void pathCallback(const nav_msgs::Path::ConstPtr& msg) {
    std::deque<Eigen::Vector3d> new_path_points;

    for (size_t i = 10; i < msg->poses.size(); i += 30) {
      const auto& pose = msg->poses[i];
      Eigen::Vector3d point(pose.pose.position.x, pose.pose.position.y, pose.pose.position.z + 0.26);
      new_path_points.push_back(point);
      if (new_path_points.size() >= 10) {
        break;
      }
    }

    path_points = new_path_points;
  }
  
  //void keyoverrideCallback(const std_msgs::String::ConstPtr& msg) {
  //  override_from_key = msg->data;
  //}
  
  void updatePath(const ros::TimerEvent&) {
    if (!path_points.empty() && initial_state_set) {
      xd = path_points.front();
      path_points.pop_front();
      state = ROTATING;
    }
  }

  void logInfo(const ros::TimerEvent& event) {
    if (initial_state_set) {
      ROS_INFO("Initial Pose: x=%f, y=%f, z=%f", initial_x.x(), initial_x.y(), initial_x.z());
      ROS_INFO("Current Pose: x=%f, y=%f, z=%f", x.x(), x.y(), x.z());
      ROS_INFO("Target Pose: x=%f, y=%f, z=%f", xd.x(), xd.y(), xd.z());
      if (state == ROTATING) {
        ROS_INFO("State: ROTATING");
      } else if (state == MOVING_FORWARD) {
        ROS_INFO("State: MOVING_FORWARD");
      } else if (state == IDLE) {
        ROS_INFO("State: IDLE");
      }
    }
  }

  void checkRotation(const Eigen::Vector3d& initial_position, const Eigen::Quaterniond& initial_orientation, const Eigen::Vector3d& current_position, const Eigen::Quaterniond& current_orientation, const Eigen::Vector3d& goal, mav_msgs::Actuators& msg) {
    // Initial robot orientation as a 2D vector
    double initial_yaw = tf::getYaw(tf::Quaternion(initial_orientation.x(), initial_orientation.y(), initial_orientation.z(), initial_orientation.w()));
    double desired_yaw = atan2(goal.y() - initial_position.y(), goal.x() - initial_position.x());

    double initial_yaw_error = desired_yaw - initial_yaw;

    // Normalize initial_yaw_error to the range [-pi, pi]
    while (initial_yaw_error > PI) initial_yaw_error -= 2 * PI;
    while (initial_yaw_error < -PI) initial_yaw_error += 2 * PI;

    // Determine rotation direction
    if (initial_yaw_error > 0) {
        msg.angular_velocities = {0, -45, 0, 0, 7}; // Rotate counterclockwise
    } else {
        msg.angular_velocities = {0, 45, 0, 0, 7}; // Rotate clockwise
    }

    // Current robot orientation as a 2D vector
    double current_yaw = tf::getYaw(tf::Quaternion(current_orientation.x(), current_orientation.y(), current_orientation.z(), current_orientation.w()));

    double current_yaw_error = desired_yaw - current_yaw;

    // Normalize current_yaw_error to the range [-pi, pi]
    while (current_yaw_error > PI) current_yaw_error -= 2 * PI;
    while (current_yaw_error < -PI) current_yaw_error += 2 * PI;

    // Determine rotation direction
    if (fabs(current_yaw_error) > angle_tolerance) {
      state = ROTATING;
    } else {
      state = MOVING_FORWARD;
    }
  }

  void controlLoop(const ros::TimerEvent& t) {
    mav_msgs::Actuators msg;
    msg.angular_velocities = {0, 0, 0, 0, 0};

    if (!initial_state_set) {
      // If no initial state set, keep the robot stationary
      msg.angular_velocities = {0, 0, 0, 0, 0};
      // 输出msg的值
      if (!msg.angular_velocities.empty()) {
        ROS_INFO("Publishing Actuator Commands: angular_velocities[0]: %f", msg.angular_velocities[0]);
      }
      commands.publish(msg);
      return;
    }

    if (state == ROTATING) {
      // Check rotation and set the control command
      checkRotation(initial_x, initial_q, x, q, xd, msg);
    } else if (state == MOVING_FORWARD) {
      // Calculate the difference between current and desired positions
      Eigen::Vector3d error = xd - x;
      double distance = error.norm();

      if (distance > position_tolerance) {
        msg.angular_velocities = {0, 90, 0, 0, 8}; // Move forward
      } else {
        // Stop moving when within position tolerance and update initial state
        msg.angular_velocities = {0, 0, 0, 0, 0};
        if (!path_points.empty()) {
          initial_x = x;
          initial_q = q;
          xd = path_points.front();
          path_points.pop_front();
          state = ROTATING; // Reset state to ROTATING
        } else {
          state = IDLE;
        }
      }
    }

    //if (override_from_key != "1") {
    //  commands.publish(msg);
    //}
    commands.publish(msg);
    
  }
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "controller_node");
  ROS_INFO_NAMED("controller", "Controller started!");
  controllerNode n;
  ros::spin();
}
