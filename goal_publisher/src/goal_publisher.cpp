#include "ros/ros.h"
#include "std_msgs/Empty.h"
#include "tf/transform_broadcaster.h"
#include "geometry_msgs/PoseStamped.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "goal_sender");
  ros::NodeHandle n;
  ros::Publisher goal_pub = n.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 2);
  
  geometry_msgs::PoseStamped goal_msg;
  goal_msg.header.stamp = ros::Time::now();
  goal_msg.pose.position.x = 0.1;
  goal_msg.pose.position.y = 0.1;
  goal_msg.pose.position.z = 0;
  double yaw = 1.57;
#if 1
  tf::Quaternion tmp = tf::createQuaternionFromYaw(yaw);
  //tf::quaternionTFToMsg(goal_msg.pose.orientation, tmp);
  goal_msg.pose.orientation.x = tmp.getX();
  goal_msg.pose.orientation.y = tmp.getY();
  goal_msg.pose.orientation.z = tmp.getZ();
  goal_msg.pose.orientation.w = tmp.getW();
#else
  goal_msg.pose.orientation = geometry_msgs::Quaternion::createQuaternionFromYaw(yaw);
#endif
  ros::Rate loop_rate(1);

  while (ros::ok())
    {
      goal_pub.publish(goal_msg);
      ros::spinOnce();
      ros::spinOnce();
    }

  return 0;
}
