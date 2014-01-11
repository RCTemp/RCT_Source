#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <std_msgs/Empty.h>
#include <geometry_msgs/PoseStamped.h>
#include "tf/transform_broadcaster.h"


class random_goal{  
 public:
  random_goal(ros::NodeHandle node_handle, ros::NodeHandle node_handle_private)
    : nh(node_handle), nh_private(node_handle_private)
    {
      if (!nh.getParam ("randMin", randMin_))
        randMin_ = -1.0;
      printf(" randMin_ is %.3f\n", randMin_);

      if (!nh.getParam ("randMax", randMax_))
        randMax_ = 1.0;
      printf(" randMax_ is %.3f\n", randMax_);

      if (!nh.getParam ("loopRate", loopRate_))
        loopRate_ = 1.0;
      printf(" loopRate_ is %.3f\n", loopRate_);


      goal_pub = nh.advertise<geometry_msgs::PoseStamped>("random_goal",1);
      start_sub = nh.subscribe<std_msgs::Empty>("start_flag_from_master", 1, &random_goal::startFlagCallback, this, ros::TransportHints().tcpNoDelay());

      srand((unsigned)time(NULL));

      start_flag = false;
    }

  ~random_goal(){}

  void goalpublish(){
    if(start_flag)
      {
        double random;
        random = randMin_ + (rand() % 100) / 100.0 * (randMax_ - randMin_); 
        random_x = random;
        random = randMin_ + (rand() % 100) / 100.0 * (randMax_ - randMin_); 
        random_y = random;
        ROS_INFO("random_x is :%f, random_y is :%f", random_x, random_y);

        goal_msg.header.stamp = ros::Time::now();
        goal_msg.header.frame_id = "/map";
        goal_msg.pose.position.x = random_x;
        goal_msg.pose.position.y = random_y;
        goal_msg.pose.position.z = 0;
        double yaw = 1.57;

        tf::Quaternion tmp = tf::createQuaternionFromYaw(yaw);
        goal_msg.pose.orientation.x = tmp.getX();
        goal_msg.pose.orientation.y = tmp.getY();
        goal_msg.pose.orientation.z = tmp.getZ();
        goal_msg.pose.orientation.w = tmp.getW();
        goal_pub.publish(goal_msg);
      }
  }


  void startFlagCallback(const std_msgs::EmptyConstPtr &start_msg)
  {
    start_flag = true;
  }

  double getLoopRate()
  {
    return loopRate_;
  }
  
 private:
  ros::NodeHandle nh;
  ros::NodeHandle nh_private;
  ros::Publisher goal_pub; 
  ros::Subscriber start_sub; 

  bool start_flag;
  geometry_msgs::PoseStamped goal_msg;
  double random_x;
  double random_y;
  
  double randMax_;
  double randMin_;

  double loopRate_;
};
