#ifndef TURTLEBOTSMASTER_H
#define TURTLEBOTSMASTER_H

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Empty.h>
#include <kobuki_msgs/BumperEvent.h>

#include <visualization_msgs/Marker.h> //debug

class TurtlebotsMaster{
 public : 
  TurtlebotsMaster(ros::NodeHandle nh, ros::NodeHandle nh_private);
  ~TurtlebotsMaster();


  void masterFunc(const ros::TimerEvent & e);
  void paramInit();
  void multiMasterMsgRegistration();
  void teleopNodePoseCallback(const geometry_msgs::PoseStampedConstPtr & pose_msg);
  void teleopNodeBumperCallback(const kobuki_msgs::BumperEventConstPtr & bumper_msg);
  void npc1NodePoseCallback(const geometry_msgs::PoseStampedConstPtr & pose_msg);
  void npc2NodePoseCallback(const geometry_msgs::PoseStampedConstPtr & pose_msg);
  float distanceBetweenTwoNodes(float x1, float x2, float y1, float y2);
  float inclinationBetweenTwoNodes(float x1, float x2, float y1, float y2);

 private:
  ros::NodeHandle turtlebotMasterNodeHandle_;
  ros::NodeHandle turtlebotMasterNodeHandlePrivate_;
  ros::Subscriber teleopNodeSub1_; //pose
  ros::Subscriber teleopNodeSub2_; //bumper
  ros::Subscriber npc1NodeSub1_;    //pose
  ros::Subscriber npc2NodeSub1_;    //pose
  ros::Publisher  teleopNodePub1_; //stop_flag
  ros::Publisher  teleopNodePub2_; //clear_flag
  ros::Publisher  npc1NodePub1_;   //stop_flag
  ros::Publisher  npc1NodePub2_;   //clear_flag
  ros::Publisher  npc2NodePub1_;   //stop_flag
  ros::Publisher  npc2NodePub2_;   //clear_flag

  ros::Timer  timer_;

  double loopRate_;
  double multiMasterMsgRate_;
  std::string teleopNodeUri_;
  std::string teleopNodePub1Name_;
  std::string teleopNodePub1Type_;
  std::string teleopNodePub2Name_;
  std::string teleopNodePub2Type_;
  std::string teleopNodeSub1Name_;
  std::string teleopNodeSub1Type_;
  std::string teleopNodeSub2Name_;
  std::string teleopNodeSub2Type_;

  std::string npc1NodeUri_;
  std::string npc1NodePub1Name_;
  std::string npc1NodePub1Type_;
  std::string npc1NodePub2Name_;
  std::string npc1NodePub2Type_;
  std::string npc1NodeSub1Name_;
  std::string npc1NodeSub1Type_;

  std::string npc2NodeUri_;
  std::string npc2NodePub1Name_;
  std::string npc2NodePub1Type_;
  std::string npc2NodePub2Name_;
  std::string npc2NodePub2Type_;
  std::string npc2NodeSub1Name_;
  std::string npc2NodeSub1Type_;

  double searchLightRadius_;
  double searchLightRange_;
  double inscribedRadius_;
  double nodeIntervalThre_;
  float dDash;


  float teleopNodeX;
  float teleopNodeY;
  float teleopNodeTheta;
  float npc1NodeX;
  float npc1NodeY;
  float npc1NodeTheta;
  float npc2NodeX;
  float npc2NodeY;
  float npc2NodeTheta;


  //debug 
  ros::Publisher npc1NodePub;
  ros::Publisher teleopNodePub;
  visualization_msgs::Marker teleopNodePoints;

};
#endif
