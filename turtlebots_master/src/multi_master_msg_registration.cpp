#include "turtlebots_master/multi_master_msg_registration.h"

MultiMasterMsgRegistrator::MultiMasterMsgRegistrator(ros::NodeHandle nh, ros::NodeHandle nh_private, float msg_rate) : multiMasterMsgRegistratorNodeHandle_(nh), multiMasterMsgRegistratorNodeHandlePrivate_(nh_private), msgRate(msg_rate)
{
  client = multiMasterMsgRegistratorNodeHandle_.serviceClient<multi_master_server::RegisterMaster>("register_master");
}

MultiMasterMsgRegistrator::~MultiMasterMsgRegistrator()
{
}


bool MultiMasterMsgRegistrator::msgRegistration(std::string ros_master_uri, std::string pub_name, std::string pub_type, std::string sub_name, std::string sub_type)
{
  multi_master_server::RegisterMaster srv;
  srv.request.master = ros_master_uri;
  srv.request.rate = msgRate;
  multi_master_server::TopicDescription pub_info;
  pub_info.topic_name = pub_name;
  pub_info.topic_type = pub_type;
  srv.request.publishers.push_back(pub_info);

  multi_master_server::TopicDescription sub_info;
  sub_info.topic_name = sub_name;
  sub_info.topic_type = sub_type;
  srv.request.subscribers.push_back(sub_info);


  if (client.call(srv))
    {
      ROS_INFO("Result: %d",  srv.response.result);
      return true;
    }
  else
    {
      ROS_ERROR("Failed to call service for node %s ", ros_master_uri.c_str());
      return false;
    }

}
