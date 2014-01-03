#ifndef TURTLEBOTSMASTER_H
#define TURTLEBOTSMASTER_H

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>

class TurtlebotsMaster{
 public : 
  TurtlebotsMaster(ros::NodeHandle nh, ros::NodeHandle nh_private);
  ~TurtlebotsMaster();

  void masterFunc(const ros::TimerEvent & e);
  void paramInit();
  void multiMasterMsgRegistration();
  void teleopNodePoseCallback(const geometry_msgs::PoseStampedConstPtr & pose_msg);
  void npc1NodePoseCallback(const geometry_msgs::PoseStampedConstPtr & pose_msg);
  void npc2NodePoseCallback(const geometry_msgs::PoseStampedConstPtr & pose_msg);


 private:
  ros::NodeHandle turtlebotMasterNodeHandle_;
  ros::NodeHandle turtlebotMasterNodeHandlePrivate_;
  ros::Subscriber teleopNodeSub_;
  ros::Subscriber npc1NodeSub_;
  ros::Subscriber npc2NodeSub_;
  ros::Publisher teleopNodePub_;
  ros::Publisher npc1NodePub_;
  ros::Publisher npc2NodePub_;
  ros::Timer  timer_;

  double loopRate_;
  double multiMasterMsgRate_;
  std::string teleopNodeUri_;
  std::string teleopNodePubName_;
  std::string teleopNodePubType_;
  std::string teleopNodeSubName_;
  std::string teleopNodeSubType_;

  std::string npc1NodeUri_;
  std::string npc1NodePubName_;
  std::string npc1NodePubType_;
  std::string npc1NodeSubName_;
  std::string npc1NodeSubType_;

  std::string npc2NodeUri_;
  std::string npc2NodePubName_;
  std::string npc2NodePubType_;
  std::string npc2NodeSubName_;
  std::string npc2NodeSubType_;


  geometry_msgs::PoseStamped teleopNodePose;
  geometry_msgs::PoseStamped npc1NodePose;
  geometry_msgs::PoseStamped npc2NodePose;

};
#endif
