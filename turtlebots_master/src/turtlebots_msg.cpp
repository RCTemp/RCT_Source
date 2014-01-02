#include "ros/ros.h"
#include "multi_master_server/TopicDescription.h"
#include "multi_master_server/RegisterMaster.h"



int main(int argc, char **argv)
{
  std::string ros_master_uri;
  double rate; 
  std::string topic_name;
  std::string topic_type;

  ros::init(argc, argv, "msg_client");
  ros::NodeHandle n;
  ros::NodeHandle private_nh("~");

  private_nh.param("ros_master_uri", ros_master_uri, std::string("empty"));
  private_nh.param("rate", rate, 10.0);
  private_nh.param("topic_name", topic_name, std::string("empty"));
  private_nh.param("topic_type", topic_type, std::string("empty"));

  ROS_INFO("ros_master_uri is %s, rate is %f, topic_name is %s, topic_type is %s",
           ros_master_uri.c_str(), rate, topic_name.c_str(), topic_type.c_str());

  ros::ServiceClient client = n.serviceClient<multi_master_server::RegisterMaster>("register_master");
  multi_master_server::RegisterMaster srv;


#if 0 //  depreacted
  srv.request.master = std::string("http://157.82.4.78:11311");
  srv.request.rate = 50;
  multi_master_server::TopicDescription pub_info;
  pub_info.topic_name = std::string("/robot/pose_pub");
  pub_info.topic_type = std::string("geometry_msgs/PoseStamped");
  srv.request.publishers.push_back(pub_info);
#else
  srv.request.master = ros_master_uri;
  srv.request.rate = rate;
  multi_master_server::TopicDescription pub_info;
  pub_info.topic_name = topic_name;
  pub_info.topic_type = topic_type;
  srv.request.publishers.push_back(pub_info);
#endif 

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
