
#include <ros/ros.h>
#include <ros/console.h>
#include <tf/transform_datatypes.h>
#include <tf_conversions/tf_eigen.h>
#include <eigen_conversions/eigen_msg.h>
#include <mav_msgs/Actuators.h>
#include <nav_msgs/Odometry.h>
#include <trajectory_msgs/MultiDOFJointTrajectoryPoint.h>
#include <math.h>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>

#define PI M_PI

#include <eigen3/Eigen/Dense>

// If you choose to use Eigen, tf provides useful functions to convert tf 
// messages to eigen types and vice versa, have a look to the documentation:
// http://docs.ros.org/melodic/api/eigen_conversions/html/namespacetf.html
#include <eigen_conversions/eigen_msg.h>

class controllerNode{
  ros::NodeHandle nh;

  ros::Publisher commands;
  ros::Subscriber string_sub;
  ros::Timer timer;

  // Controller internals (you will have to set them below)
  // Current state
  Eigen::Vector3d x;     // current position of the UAV's c.o.m. in the world frame
  Eigen::Vector3d v;     // current velocity of the UAV's c.o.m. in the world frame
  Eigen::Matrix3d R;     // current orientation of the UAV
  Eigen::Vector3d omega; // current angular velocity of the UAV's c.o.m. in the *body* frame

  // Desired state
  Eigen::Vector3d xd;    // desired position of the UAV's c.o.m. in the world frame
  Eigen::Vector3d vd;    // desired velocity of the UAV's c.o.m. in the world frame
  Eigen::Vector3d ad;    // desired acceleration of the UAV's c.o.m. in the world frame
  double yawd;           // desired yaw angle

  double hz;             // frequency of the main control loop
  std::string received_string;  // store the received string

public:
  controllerNode():hz(1000.0), received_string(""){

      commands = nh.advertise<mav_msgs::Actuators>("commands", 1);
      timer = nh.createTimer(ros::Rate(hz), &controllerNode::controlLoop, this);

      // Subscribe to string messages
      string_sub = nh.subscribe("key_input", 1, &controllerNode::stringCallback, this);
  }

  void controlLoop(const ros::TimerEvent& t){
    if (!received_string.empty()) {
      // Print received message
      ROS_INFO("Processing command after receiving string message: %s", received_string.c_str());

      // Prepare mav_msgs::Actuators message
      mav_msgs::Actuators msg;
      msg.angular_velocities.resize(5);

      // Adjust the message based on received string
      if (received_string == "w") {
        msg.angular_velocities[0] = 0; // Phase between front and back legs (in degree)
        msg.angular_velocities[1] = 90; // Phase between front left + back right legs and front right and left back legs
        msg.angular_velocities[2] = 0; // Amplitude change of all legs
        msg.angular_velocities[3] = 0; // Amplitude change of back legs (added to angular_velocities[2])
        msg.angular_velocities[4] = 7; // Frequency of legs
      } else if (received_string == "a") {
        msg.angular_velocities[0] = 0;
        msg.angular_velocities[1] = -45;
        msg.angular_velocities[2] = 0;
        msg.angular_velocities[3] = 0;
        msg.angular_velocities[4] = 7;
      } else if (received_string == "s") {
        msg.angular_velocities[0] = -10;
        msg.angular_velocities[1] = 100;
        msg.angular_velocities[2] = -1;
        msg.angular_velocities[3] = -1;
        msg.angular_velocities[4] = 8;
      } else if (received_string == "d") {
        msg.angular_velocities[0] = 0;
        msg.angular_velocities[1] = 45;
        msg.angular_velocities[2] = 0;
        msg.angular_velocities[3] = 0;
        msg.angular_velocities[4] = 7;
      } else if (received_string == "q") {
        msg.angular_velocities[0] = 0;
        msg.angular_velocities[1] = 0;
        msg.angular_velocities[2] = 0;
        msg.angular_velocities[3] = 0;
        msg.angular_velocities[4] = 0;
      }

      // Publish the message
      commands.publish(msg);

      // Clear the received string
      received_string.clear();
    }
  }

  void stringCallback(const std_msgs::String::ConstPtr& msg) {
      ROS_INFO("Received string message: %s", msg->data.c_str());
      received_string = msg->data;
  }
};

int main(int argc, char** argv){
  ros::init(argc, argv, "controller_node");
  ROS_INFO_NAMED("controller", "Controller started!");
  controllerNode n;
  ros::spin();
}

