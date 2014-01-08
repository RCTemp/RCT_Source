#ifndef MULTIMASTERMSGREGISTRATION_H
#define MULTIMASTERMSGREGISTRATION_H

#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/PoseStamped.h>
#include "multi_master_server/TopicDescription.h"
#include "multi_master_server/RegisterMaster.h"
#include "tf/transform_broadcaster.h"
#include <vector>

class MultiMasterMsgRegistrator{
 public :
  MultiMasterMsgRegistrator(ros::NodeHandle nh, ros::NodeHandle nh_private, float msg_rate);
  ~MultiMasterMsgRegistrator();

  bool msgRegistration(std::string ros_master_uri, std::vector<std::string> pub_name_list, std::vector<std::string> pub_type_list, std::vector<std::string> sub_name_list, std::vector<std::string> sub_type_list);

 private:
  ros::NodeHandle multiMasterMsgRegistratorNodeHandle_;
  ros::NodeHandle multiMasterMsgRegistratorNodeHandlePrivate_;

  ros::ServiceClient client;
  double msgRate;
};

#endif
