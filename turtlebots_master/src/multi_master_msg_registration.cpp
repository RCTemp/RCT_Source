#include "turtlebots_master/multi_master_msg_registration.h"

MultiMasterMsgRegistrator::MultiMasterMsgRegistrator(ros::NodeHandle nh, ros::NodeHandle nh_private, float msg_rate) : multiMasterMsgRegistratorNodeHandle_(nh), multiMasterMsgRegistratorNodeHandlePrivate_(nh_private), msgRate(msg_rate)
{
  client = multiMasterMsgRegistratorNodeHandle_.serviceClient<multi_master_server::RegisterMaster>("register_master");
}

MultiMasterMsgRegistrator::~MultiMasterMsgRegistrator()
{
}


int MultiMasterMsgRegistrator::msgRegistration(std::string ros_master_uri, std::vector<std::string> pub_name_list, std::vector<std::string> pub_type_list, std::vector<std::string> sub_name_list, std::vector<std::string> sub_type_list)
{
  multi_master_server::RegisterMaster srv;
  srv.request.master = ros_master_uri;
  srv.request.rate = msgRate;

  for(int i = 0; i < (int)pub_name_list.size(); i++)
    {
      multi_master_server::TopicDescription pub_info;
      pub_info.topic_name = pub_name_list[i];
      pub_info.topic_type = pub_type_list[i];
      srv.request.publishers.push_back(pub_info);
    }

  for(int i = 0; i < (int)sub_name_list.size(); i++)
    {
      multi_master_server::TopicDescription sub_info;
      sub_info.topic_name = sub_name_list[i];
      sub_info.topic_type = sub_type_list[i];
      srv.request.subscribers.push_back(sub_info);
    }

  if (client.call(srv))
    {
      ROS_INFO("Result: %d",  srv.response.result);
      return srv.response.result;
    }
  else
    {
      ROS_ERROR("Failed to call service for node %s ", ros_master_uri.c_str());
      return 0;
    }

}
