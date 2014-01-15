#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <std_msgs/Empty.h>
#include <std_msgs/UInt8.h>
#include <geometry_msgs/PoseStamped.h>
#include "tf/transform_broadcaster.h"


class random_goal{  
 public:
  random_goal(ros::NodeHandle node_handle, ros::NodeHandle node_handle_private)
    : nh(node_handle), nh_private(node_handle_private)
  {
      if (!nh_private.getParam ("randMinX", randMinX_))
        randMinX_ = -1.0;
      printf(" randMinX_ is %.3f\n", randMinX_);

      if (!nh_private.getParam ("randMaxX", randMaxX_))
        randMaxX_ = 1.0;
      printf(" randMaxX_ is %.3f\n", randMaxX_);

      if (!nh_private.getParam ("randMinY", randMinY_))
        randMinY_ = -1.0;
      printf(" randMinY_ is %.3f\n", randMinY_);

      if (!nh_private.getParam ("randMaxY", randMaxY_))
        randMaxY_ = 1.0;
      printf(" randMaxY_ is %.3f\n", randMaxY_);

      if (!nh_private.getParam ("loopRate", loopRate_))
        loopRate_ = 0.2;
      printf(" loopRate_ is %.3f\n", loopRate_);

      if (!nh_private.getParam ("cmdFromMasterTopicName", cmdFromMasterTopicName_))
        cmdFromMasterTopicName_ = std::string("cmd_from_master");
      printf(" cmdFromMasterTopicName_ is %s\n", cmdFromMasterTopicName_.c_str());


      if (!nh_private.getParam ("currPoseTopicName", currPoseTopicName_))
        currPoseTopicName_ = std::string("pose_pub");
      printf(" currPoseTopicName_ is %s\n", currPoseTopicName_.c_str());

      goal_pub = nh.advertise<geometry_msgs::PoseStamped>("move_base_simple/goal",1);
      cmd_sub = nh.subscribe<std_msgs::UInt8>(cmdFromMasterTopicName_, 1, &random_goal::cmdCallback, this, ros::TransportHints().tcpNoDelay());
      stop_flag_sub = nh.subscribe<std_msgs::Empty>("stop_flag_from_master",1, &random_goal::stopFlagCallback, this, ros::TransportHints().tcpNoDelay());
      clear_flag_sub = nh.subscribe<std_msgs::Empty>("clear_flag_from_master",1, &random_goal::clearFlagCallback, this, ros::TransportHints().tcpNoDelay());

      current_pose_sub = nh.subscribe<geometry_msgs::PoseStamped>(currPoseTopicName_, 1, &random_goal::currPoseCallback, this, ros::TransportHints().tcpNoDelay());

      srand((unsigned)time(NULL));

      start_flag = false;

      timer_ = nh_private.createTimer(ros::Duration(1.0 / loopRate_), &random_goal::mainFunc, this);

    }

  ~random_goal(){}

  void mainFunc(const ros::TimerEvent & e)
  {
    goalpublish();
  }

  void goalpublish(){
    if(start_flag)
      {
        double random;
        random = randMinX_ + (rand() % 100) / 100.0 * (randMaxX_ - randMinX_); 
        random_x = random;
        random = randMinY_ + (rand() % 100) / 100.0 * (randMaxY_ - randMinY_); 
        random_y = random;
	
	
        geometry_msgs::PoseStamped goal_msg;
        goal_msg.header.stamp = ros::Time::now();
        goal_msg.header.frame_id = "/map";
        goal_msg.pose.position.x = random_x;
        goal_msg.pose.position.y = random_y;
        goal_msg.pose.position.z = 0;
	

        float random_yaw = - M_PI + (rand() % 100) / 100.0 * 2 * M_PI; 
    
	ROS_INFO("random_x is :%f, random_y is :%f, yaw is : %f", random_x, random_y, random_yaw);

        tf::Quaternion tmp = tf::createQuaternionFromYaw(random_yaw);
        goal_msg.pose.orientation.x = tmp.getX();
        goal_msg.pose.orientation.y = tmp.getY();
        goal_msg.pose.orientation.z = tmp.getZ();
        goal_msg.pose.orientation.w = tmp.getW();
        goal_pub.publish(goal_msg);
      }
  }
  
  void stopFlagCallback(const std_msgs::EmptyConstPtr& msg)
  {
    ROS_ERROR("Game Over!!");
    start_flag = false;

#if 1         //TODO: set the current pose as goal
        geometry_msgs::PoseStamped goal_msg;
        goal_msg = current_pose;
        goal_msg.header.stamp = ros::Time::now();
        goal_msg.header.frame_id = "/map";
	ROS_WARN("goal pose: x, %f, y, %f", goal_msg.pose.position.x, goal_msg.pose.position.y);
        goal_pub.publish(goal_msg);
	
#endif

  }

  void clearFlagCallback(const std_msgs::EmptyConstPtr& msg)
  {
    ROS_WARN("Game Clear!!");
    start_flag = false;

#if 1         //TODO: set the current pose as goal
        geometry_msgs::PoseStamped goal_msg;
        goal_msg = current_pose;
        goal_msg.header.stamp = ros::Time::now();
        goal_msg.header.frame_id = "/map";
	ROS_WARN("goal pose: x, %f, y, %f", goal_msg.pose.position.x, goal_msg.pose.position.y);
        goal_pub.publish(goal_msg);
	
#endif


  }


  void cmdCallback(const std_msgs::UInt8ConstPtr &cmd_msg)
  {
    if(cmd_msg->data == START_CMD)
      {
        ROS_INFO("start send random goal");
        start_flag = true;
      }
    else if(cmd_msg->data == STOP_CMD)
      {
        ROS_INFO("stop send random goal");
        start_flag = false;

#if 1         //TODO: set the current pose as goal
        geometry_msgs::PoseStamped goal_msg;
        goal_msg = current_pose;
        goal_msg.header.stamp = ros::Time::now();
        goal_msg.header.frame_id = "/map";
	ROS_WARN("goal pose: x, %f, y, %f", goal_msg.pose.position.x, goal_msg.pose.position.y);
        goal_pub.publish(goal_msg);
	
#endif
      }
  }

  void currPoseCallback(const geometry_msgs::PoseStamped pose_msg)
  {
    //ROS_INFO("curr pose: x, %f, y, %f", current_pose.pose.position.x, current_pose.pose.position.y);
    current_pose = pose_msg;
  }


  double getLoopRate()
  {
    return loopRate_;
  }

  const static uint8_t STOP_CMD = 0;
  const static uint8_t START_CMD = 1;
  
 private:
  ros::NodeHandle nh;
  ros::NodeHandle nh_private;
  ros::Publisher goal_pub; 
  ros::Subscriber cmd_sub; 
  ros::Subscriber current_pose_sub; 
  ros::Subscriber clear_flag_sub;
  ros::Subscriber stop_flag_sub;

  ros::Timer  timer_;

  bool start_flag;

  geometry_msgs::PoseStamped current_pose;
  std::string currPoseTopicName_;
  std::string cmdFromMasterTopicName_;
  double random_x;
  double random_y;
  
  double randMaxX_;
  double randMinX_;
  double randMaxY_;
  double randMinY_;


  double loopRate_;
};
