#include "turtlebot_control/ros_header.h"

bool flag;
//callback functions
void GetFlagCallback(const std_msgs::BoolConstPtr& got_flag)
{  
  ROS_INFO("Got Flag");
  if(!got_flag->data){
    ROS_INFO("Got stop Flag from master");
    flag=false;
  }
}

void GetGoalCallback(const geometry_msgs::PoseStampedConstPtr& goal)
{
  ROS_INFO("Got goal");
  if(flag){
    ros::NodeHandle n;
    ros::Publisher enemy_goal_pub = n.advertise<geometry_msgs::PoseStamped>("simple_goal",1);
    enemy_goal_pub.publish(goal);
  }else{
    ROS_INFO("Flag = false, not published");
  }

}


int main(int argc, char**argv){
  flag = true;
  ros::init(argc, argv,"npc_node_parser");
  ros::NodeHandle nh;
  ros::Subscriber origin_goal_sub;
  ros::Subscriber flag_sub;
  flag_sub = nh.subscribe("flag_from_master",1,GetFlagCallback);
  origin_goal_sub = nh.subscribe("random_goal",1, GetGoalCallback);
  ros::spin();
  return 0;
}






