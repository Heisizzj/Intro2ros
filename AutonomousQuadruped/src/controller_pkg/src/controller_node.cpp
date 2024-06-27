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

#define PI M_PI


#include <eigen3/Eigen/Dense>

// If you choose to use Eigen, tf provides useful functions to convert tf 
// messages to eigen types and vice versa, have a look to the documentation:
// http://docs.ros.org/melodic/api/eigen_conversions/html/namespacetf.html
#include <eigen_conversions/eigen_msg.h>

class controllerNode{
  ros::NodeHandle nh;

  ros::Publisher commands;
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

public:
  controllerNode():hz(1000.0){
      
      commands = nh.advertise<mav_msgs::Actuators>("commands", 1);
      timer = nh.createTimer(ros::Rate(hz), &controllerNode::controlLoop, this);
  }



  void controlLoop(const ros::TimerEvent& t){

    mav_msgs::Actuators msg;

    msg.angular_velocities.resize(5);
    msg.angular_velocities[0] = 0; // Phase between front and back legs (in degree)
    msg.angular_velocities[1] = 90; // Phase between front left + back right legs and front right and left back legs
    msg.angular_velocities[2] = 0; // Amplitude change of all legs
    msg.angular_velocities[3] = 0; // Amplitude change of back legs (added to angular_velocities[2])
    msg.angular_velocities[4] = 7; // Frequency of legs

    commands.publish(msg);

  }
};

int main(int argc, char** argv){
  ros::init(argc, argv, "controller_node");
  ROS_INFO_NAMED("controller", "Controller started!");
  controllerNode n;
  ros::spin();
}
