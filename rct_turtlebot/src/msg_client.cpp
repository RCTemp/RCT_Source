#include "ros/ros.h"
#include "multi_master_server/TopicDescription.h"
#include "multi_master_server/RegisterMaster.h"



int main(int argc, char **argv)
{
  ros::init(argc, argv, "msg_client");

  ros::NodeHandle n;
  ros::ServiceClient client = n.serviceClient<multi_master_server::RegisterMaster>("register_master");
  multi_master_server::RegisterMaster srv;
  srv.request.master = std::string("http://157.82.4.78:11311");
  srv.request.rate = 50;
  multi_master_server::TopicDescription pub_info;
  pub_info.topic_name = std::string("/robot/pose_pub");
  pub_info.topic_type = std::string("geometry_msgs/PoseStamped");
  srv.request.publishers.push_back(pub_info);
  if (client.call(srv))
    {
      ROS_INFO("Result: %d",  srv.response.result);
    }
  else
    {
      ROS_ERROR("Failed to call service ");
      return 1;
    }

  return 0;
}
