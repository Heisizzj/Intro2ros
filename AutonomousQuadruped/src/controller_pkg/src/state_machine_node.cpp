
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <std_msgs/Int32.h>
#include <vector>
#include <eigen3/Eigen/Dense>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Quaternion.h>
#include <tf/transform_datatypes.h>
#include <tf/LinearMath/Matrix3x3.h>
#include <state_indicator_msgs/state_indicator.h>

//state definition
#define STANDBY 0
#define FORWARD 1
#define TROT 2
#define CCW_ROTATE 3
#define CW_ROTATE 4
#define RAISE_FORELEGS 5
#define HIGHAMP_WALK 6
#define END 7

//define destination coordinates
// change by Hao Guo for testing 2.5 9.5
// 0.5 1.70
#define GOAL_X 2.5
#define GOAL_Y 9.5

#define PI M_PI

class state_machine_node{
    ros::NodeHandle nh;
    ros::Subscriber robot_pos;  //subscribe to current pose
    ros::Subscriber path_point_sub;  //subscribe to path point   
    ros::Publisher goal_pub, goal_vis_pub;  //publish goal to move base
    ros::Publisher state_pub;   //publish state to controller

    ros::Timer timer;
    ros::Time start_time_1;
    double hz;  //frequency of publishing
    state_indicator_msgs::state_indicator state;

    int start = 0;
    int highamp_count = 0;
    int climbing = 0;

    Eigen::Vector2d current_position;
    Eigen::Vector2d current_velocity;
    Eigen::Vector2d current_direction;

    Eigen::Vector2d destination;

    Eigen::Vector2d path_point;
    Eigen::Vector3d path_point_3d;

    geometry_msgs::PoseStamped goal;
    visualization_msgs::Marker goal_marker;
    int flag_forward = 1;
public:
    state_machine_node():hz(200.0){
        // path_point, 

        state.state_msg = 0;  // STANDBY == 0
        // create a subscriber which can be used to subscribe the current pose
        robot_pos = nh.subscribe("current_state_est", 1, &state_machine_node::GetPose, this);
        // create a subscriber which can be used to subscribe the current GetPathPoint
        path_point_sub = nh.subscribe("path_point",100,&state_machine_node::GetPathPoint,this);
        // the timer will be call 200 times per second
        timer = nh.createTimer(ros::Rate(hz), &state_machine_node::ControllLoop, this);
        // create a publisher which can be used to publish goal
        // /move_base_simple/goal
        goal_pub = nh.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal",1);

        // create publishers which can be used to publish goal_marker and state
        goal_vis_pub = nh.advertise<visualization_msgs::Marker>("goal_marker",1);
        state_pub = nh.advertise<state_indicator_msgs::state_indicator>("state",1);

        // define the goal pos
        goal.pose.position.x = GOAL_X;
        goal.pose.position.y = GOAL_Y;
        goal.pose.position.z = 0;
        goal.header.frame_id = "world";
        goal.pose.orientation.w = 1.0;
        destination << GOAL_X,GOAL_Y;

        //visualize goal point in rviz
        goal_marker.header.frame_id = "world";
        goal_marker.ns = "state_machine_node";
        goal_marker.id = 11;
        goal_marker.action = visualization_msgs::Marker::ADD;
        goal_marker.pose.orientation.w = 1.0;
        goal_marker.type = visualization_msgs::Marker::CYLINDER;
        goal_marker.scale.x = goal_marker.scale.y = goal_marker.scale.z = 0.2;
        goal_marker.color.r = 1;
        goal_marker.color.a = 1;
        goal_marker.lifetime = ros::Duration();
        geometry_msgs::Point p;
        goal_marker.pose = goal.pose;

        ROS_INFO("state_machine_node start");
    }
    
    //get the current position, velocity and orientation
    void GetPose(const nav_msgs::Odometry& cur_state) {
        current_position << cur_state.pose.pose.position.x, cur_state.pose.pose.position.y;
        current_velocity << cur_state.twist.twist.linear.x,cur_state.twist.twist.linear.y;

        // Extract the face direction (yaw angle) from the quaternion orientation
        geometry_msgs::Quaternion orientation = cur_state.pose.pose.orientation;
        double yaw = tf::getYaw(orientation);
        // Calculate the 2D vector based on the yaw angle
        current_direction(1) = cos(yaw);
        current_direction(0) = sin(yaw);
        
        //check if close to destination
        Eigen::Vector2d d;
        d = (destination - current_position);
        if(d.norm() < 0.1){
            state.state_msg = END;
        } 
    }
    // where define this, pay attention
    // get path point from planning node
    // path point is our goal? here i dont know it is global or local 
    void GetPathPoint(const geometry_msgs::Point& path){
        path_point(0) = path.x;
        path_point(1) = path.y;
        path_point_3d(0) = path.x;
        path_point_3d(1) = path.y;
        path_point_3d(2) = path.z;
        start = 1;
    }

    //check if the robot should rotate
    void check_rotation(Eigen::Vector2d &position, Eigen::Vector2d &orientation, Eigen::Vector2d &goal){
        Eigen::Vector2d dist;
        //
        dist = (goal - position);

        double cos;
        Eigen::Vector2d distance_normalized;
        Eigen::Vector2d orientation_normalized;
        // the vector indicates the unit direction from current position to the goal position
        distance_normalized = dist / dist.norm();
        // the vector indicates the unit direction of current robotdog
        orientation_normalized = orientation / orientation.norm();
        // the cos angle between the two vector
        cos = std::abs(distance_normalized.dot(orientation_normalized));

        //  ROS_INFO("cos: %f", cos);
        // std::string distStr = "[" + std::to_string(distance_normalized[0]) + ", " + std::to_string(distance_normalized[1]) + "]";
        // ROS_INFO("Dist: %s", distStr.c_str());

        // std::string oritStr = "[" + std::to_string(orientation_normalized[0]) + ", " + std::to_string(orientation_normalized[1]) + "]";
        // ROS_INFO("Dist: %s", oritStr.c_str());

        // use cross product to difine the rotation direction
        if(orientation_normalized({0})*distance_normalized({1}) - orientation_normalized({1})*distance_normalized({0})<=0){
            state.state_msg = CW_ROTATE;
        }
        else{
            state.state_msg = CCW_ROTATE;
        }
        // if current goal is directly in front, move forward
        if(cos > 0.87){
            state.state_msg = FORWARD;
        }
    }
    // only publish state
    // if dont consider the jump, the state is only rotation and going forward. 
    // no matter which state we currently in, we always check whether to change the state
    void ControllLoop(const ros::TimerEvent& t){
    /*
    state_msg -> standby (0: default)
        |     -> ccw rotate
        |     -> cw  rotate
        |     -> forward	->   forward
        |           |          ->   trot
        |           |          ->   raise forelegs
        |           |          ->   highamp walk
                                          
    */  
      switch (state.state_msg){      
      	case STANDBY: //initialize
            // publish the final goal position
            ROS_INFO("state: initial");
            /*goal.header.stamp = goal_marker.header.stamp = ros::Time::now();
            goal_pub.publish(goal);
            goal_vis_pub.publish(goal_marker);
            ROS_INFO_ONCE("goal published to move base"); */
            
            if(start == 1)
                check_rotation(current_position, current_direction, path_point);

            // publish state
            state_pub.publish(state);
            break;
            
        case CCW_ROTATE:
            ROS_INFO("state: rotate counterclockwise");
            check_rotation(current_position, current_direction, path_point);
            
            // publish state
            state_pub.publish(state);
            break;

        case CW_ROTATE:
            ROS_INFO("state: rotate clockwise");
            check_rotation(current_position, current_direction, path_point);

            // publish state
            state_pub.publish(state);
            break;
        
        case END: //stop
            ROS_INFO("state: end");

            // publish state
            state_pub.publish(state);
            break;
        
        case FORWARD: //move forward. If there's a slope, try highamp. If it doesn't works, raise forelegs and trot upwards.
            ROS_INFO("state: slowly forward");
            check_rotation(current_position, current_direction, path_point);
            // if steps ahead, jump
            if(path_point_3d(2) > 0)
            	if(highamp_count < 500) {  // TODO: set count here to raise/lower the limit that triggers raise forelegs and trot
                  state.state_msg = HIGHAMP_WALK;
                  highamp_count += + 1;
                  check_rotation(current_position, current_direction, path_point);
                }
                else if(climbing < 500){  // TODO: set count here to give a proper number of climbing commands
                     state.state_msg = RAISE_FORELEGS;
                     climbing += 1;
                }
                else {
                       state.state_msg = TROT;
                       check_rotation(current_position, current_direction, path_point);
                }
            else {
                  highamp_count = 0;
                  climbing = 0;
                  check_rotation(current_position, current_direction, path_point);
                  state_pub.publish(state);
            }

            break;
	
	case TROT: //raise legs and walk over
            ROS_INFO("state: trot");
            // if no steps ahead, return to forward or rotate
            if(path_point_3d(2) == 0)
                check_rotation(current_position, current_direction, path_point);

            // publish state
            state_pub.publish(state);
            break;
	
	case RAISE_FORELEGS: //raise forelegs, get on the slope
            ROS_INFO("state: raise_foreleg");
            // if no steps ahead, return to forward or rotate
            if(path_point_3d(2) == 0)
                check_rotation(current_position, current_direction, path_point);

            // publish state
            state_pub.publish(state);
            break;
        
        case HIGHAMP_WALK: //walk forward while legs been given high amplitude
            ROS_INFO("state: slowly forward");
            check_rotation(current_position, current_direction, path_point);
            // if steps ahead, jump
            if(path_point_3d(2) > 0)
                state.state_msg = JUMP;

            // publish state
            state_pub.publish(state);
            break;
      }
    }

};

int main(int argc, char** argv){
    ros::init(argc, argv, "state_machine_node");
    state_machine_node n;
    ros::spin();
}
