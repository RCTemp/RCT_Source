#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>

geometry_msgs::PoseStamped getgoal(int l);

int main(int argc, char** argv){
  ros::init(argc, argv, "simple_navigation_goals");
  ros::NodeHandle nh;
  ros::Publisher goal_pub = nh.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal",1);
  geometry_msgs::PoseStamped goal;
 
 
   ros::Rate loop_rate(0.1);
 for(int i=0;;i++){
  if(ros::ok())
    {
      goal = getgoal(i);      
      goal_pub.publish(goal);
      ROS_INFO("Send goal #%d",i);
      ros::spinOnce();
      loop_rate.sleep();
    }else{
    break;
  }
  }
  return 0;
}

geometry_msgs::PoseStamped getgoal(int l)
{
#if 1
  double yaw =0.0;
#else
  double yaw =1.57;
#endif
  geometry_msgs::PoseStamped goal;
  goal.header.frame_id = "/map";
  goal.header.stamp = ros::Time::now();
  goal.pose.position.x = 0.5*l;
  goal.pose.position.y = 0*l;
  goal.pose.position.z = 0;
 tf::Quaternion tmp = tf::createQuaternionFromYaw(yaw);
  goal.pose.orientation.x = tmp.getX();
  goal.pose.orientation.y = tmp.getY();
  goal.pose.orientation.z = tmp.getZ();
  goal.pose.orientation.w = tmp.getW();
  return goal;
}




