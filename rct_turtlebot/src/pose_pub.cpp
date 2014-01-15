#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/PoseStamped.h>

int main(int argc, char** argv){
  std::string topic_name;
  double loop_rate;

  ros::init(argc, argv, "pose_pub");
  ros::NodeHandle node;
  ros::NodeHandle private_nh("~");
  private_nh.param("topic_name", topic_name, std::string("pose_pub"));
  private_nh.param("loop_rate", loop_rate, 10.0);

  ros::Publisher pose_pub =
    node.advertise<geometry_msgs::PoseStamped>(topic_name, 10);

  tf::TransformListener listener;

  ros::Rate rate(10.0);
  while (node.ok()){
    tf::StampedTransform transform;
    try{
      listener.lookupTransform("/map", "/base_footprint",
                               ros::Time(0), transform);
    }
    catch (tf::TransformException ex){

      //ROS_ERROR("%s",ex.what());
    }

    geometry_msgs::PoseStamped pose_msg;
    pose_msg.header.stamp = transform.stamp_;
    pose_msg.header.frame_id = "/map";
    pointTFToMsg(transform.getOrigin(), pose_msg.pose.position);
    quaternionTFToMsg(transform.getRotation(), pose_msg.pose.orientation);
    pose_pub.publish(pose_msg);

    rate.sleep();
  }
  return 0;
};
