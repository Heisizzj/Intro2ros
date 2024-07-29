#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <nav_msgs/Odometry.h>
#include <eigen3/Eigen/Dense>
#include <vector>

class GoalPublisherNode {
public:
  GoalPublisherNode() : ac_("move_base", true), current_goal_index_(0), tolerance_(0.3) {
    // Wait for the action server to come up
    ROS_INFO("Waiting for the move_base action server...");
    ac_.waitForServer();

    // Initialize goals
    initializeGoals();

    // Subscriber for current state
    state_sub_ = nh_.subscribe("/current_state_est", 1, &GoalPublisherNode::stateCallback, this);

    // Publish the first goal
    sendGoal();
  }

private:
  void initializeGoals() {
    // Define your five goals here
    goals_.emplace_back(createGoal(1.2, 1.70, 0.0));
    goals_.emplace_back(createGoal(3.4, 1.75, 0.0));
    goals_.emplace_back(createGoal(4.30, 0.80, 0.0));
    goals_.emplace_back(createGoal(5.45, 0.58, 0.0));
    goals_.emplace_back(createGoal(5.50, 1.29, 0.0));
  }

  move_base_msgs::MoveBaseGoal createGoal(double x, double y, double z) {
    move_base_msgs::MoveBaseGoal goal;
    goal.target_pose.header.frame_id = "world";
    goal.target_pose.header.stamp = ros::Time::now();
    goal.target_pose.pose.position.x = x;
    goal.target_pose.pose.position.y = y;
    goal.target_pose.pose.position.z = z;
    goal.target_pose.pose.orientation.w = 1.0; // No rotation
    return goal;
  }

  void sendGoal() {
    if (current_goal_index_ < goals_.size()) {
      move_base_msgs::MoveBaseGoal goal = goals_[current_goal_index_];
      goal.target_pose.header.stamp = ros::Time::now();
      ac_.sendGoal(goal, boost::bind(&GoalPublisherNode::doneCb, this, _1, _2),
                   boost::bind(&GoalPublisherNode::activeCb, this),
                   boost::bind(&GoalPublisherNode::feedbackCb, this, _1));
      ROS_INFO("Sent goal %zu: [%f, %f, %f]",
               current_goal_index_,
               goals_[current_goal_index_].target_pose.pose.position.x,
               goals_[current_goal_index_].target_pose.pose.position.y,
               goals_[current_goal_index_].target_pose.pose.position.z);
    } else {
      ROS_INFO("All goals have been sent.");
    }
  }

  void stateCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    if (current_goal_index_ >= goals_.size()) {
      return;
    }

    Eigen::Vector3d current_pos(msg->pose.pose.position.x,
                                msg->pose.pose.position.y,
                                msg->pose.pose.position.z);
    Eigen::Vector3d goal_pos(goals_[current_goal_index_].target_pose.pose.position.x,
                             goals_[current_goal_index_].target_pose.pose.position.y,
                             goals_[current_goal_index_].target_pose.pose.position.z);

    double distance = (current_pos - goal_pos).norm() - 0.26;
    ROS_INFO("Distance to goal %zu: %f", current_goal_index_, distance);
    if (distance < tolerance_) {
      ROS_INFO("Goal %zu reached", current_goal_index_);
      current_goal_index_++;
      sendGoal();
    }
  }

  void doneCb(const actionlib::SimpleClientGoalState& state,
              const move_base_msgs::MoveBaseResultConstPtr& result) {
    ROS_INFO("Goal reached with state: %s", state.toString().c_str());
  }

  void activeCb() {
    ROS_INFO("Goal just went active");
  }

  void feedbackCb(const move_base_msgs::MoveBaseFeedbackConstPtr& feedback) {
    ROS_INFO("Got feedback");
  }

  ros::NodeHandle nh_;
  actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ac_;
  ros::Subscriber state_sub_;
  std::vector<move_base_msgs::MoveBaseGoal> goals_;
  size_t current_goal_index_;
  double tolerance_;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "goal_publisher_node");
  GoalPublisherNode goal_publisher_node;
  ros::spin();
  return 0;
}
