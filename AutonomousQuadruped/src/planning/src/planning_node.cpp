#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <vector>
#include <eigen3/Eigen/Dense>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/OccupancyGrid.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Quaternion.h>
#include <tf/transform_datatypes.h>
#include <tf/LinearMath/Matrix3x3.h>

//state definition
#define INITIAL 0
#define SET_YAW 1
#define FOLLOW_PATH 2
#define UPSTAIR 3
#define DOWNSTAIR 4
#define SLOPE 5
#define END 6

//define destination coordinates
#define GOAL_X 5.6
#define GOAL_Y 6.1

#define PI M_PI



class planning_node{
    ros::NodeHandle nh;
    ros::Subscriber robot_pos;  //current position subscriber
    ros::Subscriber path_sub;     
    ros::Publisher next_waypoint_pub;  //next waypoint publisher
    ros::Publisher goal_pub, goal_vis_pub;  //publish the destination

    ros::Time start_time, finish_time;  //Record the start and end time of the task
    ros::Timer timer;
  
    double hz;  //frequency of the waypoint publishing
  
    int state, next_state;
    Eigen::Vector3d cur_pos;
    Eigen::Vector2d cur_v; 
    double yaw, yaw_desired; //orientation
    Eigen::Vector2d start_point_y,desired_pos, destination;
    std::vector<Eigen::Vector2d> path_list;

    geometry_msgs::PoseStamped goal;
    visualization_msgs::Marker goal_marker;
    
    int clockwise, start_flag, new_point, upstair_flag, slope_flag;
    double slope_y;

    //parameters used to check if there is step or slope
    int idx;
    int map_width;
    int map_height;
    double resolution;
    double origin_x;
    double origin_y;
    int rows;
    int columns;
    int occupancy;
    int stepth;
    double up_start_x;
    double up_start_y;
    Eigen::Vector2d nxgoals;
    ros::Subscriber map_sub;

public:
    planning_node():hz(50.0),state(SET_YAW),next_state(INITIAL),yaw_desired(0),start_flag(0),new_point(0),upstair_flag(0), slope_flag(0){
        robot_pos = nh.subscribe("current_state_est", 100, &planning_node::onCurrentState, this);
        path_sub = nh.subscribe("/move_base_Quadruped/NavfnROS/plan",100,&planning_node::getPath,this);
        map_sub = nh.subscribe("projected_map_step", 1, &planning_node::CheckStep, this);
        timer = nh.createTimer(ros::Duration(1/hz), &planning_node::planningLoop, this);
      
        next_waypoint_pub = nh.advertise<geometry_msgs::Point>("next_waypoint",1);
        goal_pub = nh.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal",1);
        goal_vis_pub = nh.advertise<visualization_msgs::Marker>("goal_marker",1);

        start_point_y << -0.6, 0.8;
        cur_pos <<-1,-1,-1;

        goal.pose.position.x = GOAL_X;
        goal.pose.position.y = GOAL_Y;
        goal.pose.position.z = 0;
        goal.header.frame_id = "world";
        goal.pose.orientation.w = 1.0;
        desired_pos << GOAL_X,GOAL_Y;
        destination << GOAL_X,GOAL_Y;

        //visualize goal point in rviz
        goal_marker.header.frame_id = "world";
        goal_marker.ns = "planning_node";
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

        ROS_INFO("[planning_node] Initialised");
    }
    
    //get the current position and velocity
    void onCurrentState(const nav_msgs::Odometry& cur_state) {
        cur_pos << cur_state.pose.pose.position.x, cur_state.pose.pose.position.y, cur_state.pose.pose.position.z;
        cur_v << cur_state.twist.twist.linear.x,cur_state.twist.twist.linear.y;

        // the incoming geometry_msgs::Quaternion is transformed to a tf::Quaternion
        tf::Quaternion quat;
        tf::quaternionMsgToTF(cur_state.pose.pose.orientation, quat);

        // the tf::Quaternion has a method to access roll pitch and yaw
        double roll, pitch;
        tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);

        if(state == INITIAL && start_flag == 0){
            start_flag = 1;
            clockwise = 0;
            desired_pos << cur_pos(0), start_point_y(0);
            ROS_INFO_STREAM("Going to the start point.");
        }
        
        //check if close to destination
        if (check_pos(cur_pos,destination,1.5) && (GOAL_Y-cur_pos(1))<1.0 && start_flag == 1){
            new_point = 0;
            state = END;
            desired_pos << cur_pos(0), GOAL_Y;
            }
    }

    //get path from global planner
    void getPath(const nav_msgs::Path& path_info){
        Eigen::Vector2d point;
        path_list.clear();
        for(int i=0; i<path_info.poses.size();i++){
            point<<path_info.poses[i].pose.position.x, path_info.poses[i].pose.position.y;
            path_list.push_back(point);
        }
    }

    // function that check if there is step or slope in front
    void CheckStep(const nav_msgs::OccupancyGrid& map_info){  
 
        map_width = map_info.info.width;
        map_height = map_info.info.height;
        resolution = map_info.info.resolution;
        origin_x = map_info.info.origin.position.x;
        origin_y = map_info.info.origin.position.y;
        stepth = 0;
        
        if(!path_list.empty() && desired_pos(1) > 1.0 && desired_pos(1) < 5.0){
            for(int i=0;i<=10;i++){
                nxgoals = path_list[i];
                columns = (int) ((nxgoals(0) - origin_x) / resolution);
                rows = (int) ((nxgoals(1)- origin_y) / resolution);
                idx = rows * map_width + columns;
                occupancy = map_info.data[idx];
                if(occupancy == 100){
                    stepth = stepth + 1;
                }
            }
        }

        if(stepth > 5 && upstair_flag == 0){
            state = SET_YAW;
            next_state = UPSTAIR;
            if (yaw <= 0)
                yaw_desired = -PI;
            else
                yaw_desired = PI;
            upstair_flag = 1;
            up_start_x = cur_pos(0);
            up_start_y = cur_pos(1);
            ROS_WARN("There is a step");
        }

         if(stepth > 5 && upstair_flag == 2){
            state = SET_YAW;
            next_state = SLOPE;
            yaw_desired = 0;
            slope_y = cur_pos(1);
            upstair_flag = -1;
            ROS_WARN("There is a slope");
        }
        
    } 

    void planningLoop(const ros::TimerEvent& t){  
      geometry_msgs::Point waypoint;

      //state machine
      switch (state){      
        case INITIAL: //initialize and go to start point
            if(!check_pos(cur_pos,desired_pos)){
                waypoint.x = desired_pos(0);
                waypoint.y = desired_pos(1);
                waypoint.z = 0;
                next_waypoint_pub.publish(waypoint); 
            }
            else{
                goal.header.stamp = goal_marker.header.stamp = ros::Time::now();
                goal_pub.publish(goal);
                goal_vis_pub.publish(goal_marker);
                ROS_INFO_ONCE("Here we go! Timer starts!");
                start_time = ros::Time::now();
                desired_pos << cur_pos(0), start_point_y(1);
                waypoint.x = cur_pos(0);
                waypoint.y = start_point_y(1);
                waypoint.z = 0;
                next_waypoint_pub.publish(waypoint);
                state = FOLLOW_PATH;
            }
            break;

        case SET_YAW: //set robot to desired orientation
            if(std::abs(yaw - yaw_desired) > 0.05){
                if(yaw - yaw_desired > 0)
                    waypoint.z = 8;
                else{
                    waypoint.z = -8;
                }
                waypoint.x = desired_pos(0);
                waypoint.y = desired_pos(1);
                next_waypoint_pub.publish(waypoint);
            }else{
                state = next_state;
                if (next_state == FOLLOW_PATH)
                    new_point = 1; 
            }
            break;

        case FOLLOW_PATH: // follow to desired waypoint
            if (new_point){
                int i;
                for(i=1;i<path_list.size();i++){
                    if(!check_pos(cur_pos,path_list[i],0.4) && check_pos(cur_pos,path_list[i],1.0)) //next desired point should be at least 0.4m far away
                        if(check_ori(cur_v,path_list[i])){
                            new_point = 0;
                            break;
                        }
                }
                            
                if(i == path_list.size()-1){
                    desired_pos << cur_pos(0) + 0.01*cur_v(0), cur_pos(1)+ 0.01*cur_v(1);
                }else{
                    desired_pos = path_list[i];
                }
            }

            if(!check_pos(cur_pos,desired_pos) && new_point == 0){
                waypoint.x = desired_pos(0);
                waypoint.y = desired_pos(1);
                waypoint.z = 0;
                next_waypoint_pub.publish(waypoint); 
            }
            else{
                new_point = 1;
            }
            break;

        case UPSTAIR: //go upstairs
            ROS_INFO_ONCE("Ready to upstair. It may take some time.");
            if((up_start_x - cur_pos(0))*(up_start_x - cur_pos(0)) + (up_start_y - cur_pos(1))*(up_start_y - cur_pos(1)) > 1 && cur_pos(2)>0.3){
                state = DOWNSTAIR;
                upstair_flag = 2;
            }
            waypoint.x = desired_pos(0);
            waypoint.y = desired_pos(1);
            waypoint.z = 2;
            next_waypoint_pub.publish(waypoint); 
            break;
        
        case DOWNSTAIR: //go downstairs
            ROS_INFO_ONCE("Ready to downstair.");
            yaw_desired = 0;
            state = SET_YAW;
            next_state = FOLLOW_PATH;
            break;

        case SLOPE: 
            waypoint.x = GOAL_X;
            waypoint.y = GOAL_Y;
            waypoint.z = 3;
            next_waypoint_pub.publish(waypoint); 
            if(cur_pos(1)-slope_y > 1.2){ 
                state = FOLLOW_PATH;
                new_point =1;
            }
            break;
        
        case END: //stop and calculate the time
            if(cur_pos(1)<5.9 && start_flag ==1){
                waypoint.x = desired_pos(0);
                waypoint.y = desired_pos(1);
                waypoint.z = 0;
                next_waypoint_pub.publish(waypoint); 
            }
            else{
                if(start_flag == 1){
                    finish_time = ros::Time::now();
                    ROS_WARN_ONCE("Arrive at destination! It took %f s",(finish_time - start_time).toSec());
                     start_flag = 0;
                }
                waypoint.x = GOAL_X;
                waypoint.y = GOAL_Y;
                waypoint.z = -1;
                next_waypoint_pub.publish(waypoint);
            }
            break;
      }
    }

    //check if the robot has arrived our desired point.
    bool check_pos(Eigen::Vector3d &cur_pos, Eigen::Vector2d &desired_pos, double distance = 0.2){
        Eigen::Vector2d cur_2d;
        cur_2d << cur_pos(0),cur_pos(1);
        double d = (cur_2d-desired_pos).norm();
        if(d <= distance)
            return true;
        else
            return false;
    }

    //check not to publish waypoint which is at back of robot
    bool check_ori(Eigen::Vector2d &cur_v, Eigen::Vector2d &path_point){
        Eigen::Vector2d cur_2d, ori;
        cur_2d << cur_pos(0),cur_pos(1);
        ori = path_point - cur_2d;
        double cosVal;
        if(cur_v.norm() * ori.norm() != 0)
            cosVal = cur_v.dot(ori) /(cur_v.norm()*ori.norm());
        else
            cosVal = 0;
        if(cosVal > -std::sqrt(3)/2)
            return true;
        else
            return false;
    }
};

int main(int argc, char** argv){
    ros::init(argc, argv, "planning_node");
    planning_node n;
    ros::spin();
}