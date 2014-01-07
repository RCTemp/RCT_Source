/*====================================================================
このパッケージはフラグの状況によりPathをパブリッシュするかどうかを判断するためのもの。
flag_from_masterというrostopicを受け取った後はPathをパブリッシュしない。
======================================================================*/

#include "turtlebot_control/ros_header.h"

//global variable
 bool flag;
//callback functions
void GetFlagCallback(const std_msgs::BoolConstPtr& got_flag)
{
  bool tmp_flag = got_flag->data;
  ROS_INFO("Got Flag");
  if(!tmp_flag){
    ROS_INFO("Got Flag false from master");
    flag=false;
  }
}

void GetPathCallback(const nav_msgs::PathConstPtr& path)
{
  ROS_INFO("Got Path");
  if(flag){
    ROS_INFO("Publish path");
    ros::NodeHandle n;
    ros::Publisher control_path_pub = n.advertise<nav_msgs::Path>("local_path",1);    
    control_path_pub.publish(path);
  }else{
    ROS_INFO("flag = false, not published");
  }
}

void GetPathCallback(const geometry_msgs::Twist& cmd_vel)
{
  ROS_INFO("Got Velocity Command");
  if(flag){
    ROS_INFO("Publish cmd_vel");
    ros::NodeHandle n;
    ros::Publisher cmd_vel_pub = n.advertise<geometry_msgs::Twist>("cmd_vel_mux/input/teleop",1);    
    cmd_vel_pub.publish(cmd_vel);
  }else{
    ROS_INFO("flag = false, not published");
  }
}


int main(int argc,char** argv){
  flag = true;
  ros::init(argc, argv, "teleop_node_parser");
  ros::NodeHandle nh;
  ros::Subscriber origin_path_sub;
  ros::Subscriber cmd_vel_sub;
  ros::Subscriber flag_sub;

  flag_sub = nh.subscribe("flag_from_master",1,GetFlagCallback); 
  cmd_vel_sub = nh.subscribe("teleop/cmd_vel_mux/input/teleop",1,GetFlagCallback); 
  origin_path_sub = nh.subscribe("path_from_UI",1,GetPathCallback);
  ros::spin();
  return 0;
}












