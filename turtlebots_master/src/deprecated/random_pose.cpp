#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <time.h>



int main(int argc, char** argv){
  ros::init(argc, argv, "random_pose");
  ros::NodeHandle nh;
  ros::Publisher goal_pub = nh.advertise<geometry_msgs::PoseStamped>("/teleop_node/pose_pub",1);


  ros::Rate rate(40);

  srand((unsigned)time(NULL));

  while(ros::ok()){

    //int time = ros::Time::now().toSec() * 1000 ; //ms

    double random_r = ((double)(rand() % 100) / 100.0) * 0.8; //radius:0.8 
    double random_theta = - M_PI / 3  + ((double)(rand() % 200) / 200.0) *  2 * M_PI / 3; //radius:0.8 
    //double random_theta =  ((double)(rand() % 200) / 200.0) *  2 * M_PI; //radius:0.8 
    //ROS_INFO("random_r: %f, random_theta: %f",random_r, random_theta);
    double random_x = random_r * cos(random_theta);
    double random_y = random_r * sin(random_theta);

    geometry_msgs::PoseStamped random_pose;
    random_pose.header.stamp = ros::Time::now();
    random_pose.pose.position.x = random_x;
    random_pose.pose.position.y = random_y;
    random_pose.pose.position.z = 0;
    random_pose.pose.orientation.x = 0;
    random_pose.pose.orientation.y = 0;
    random_pose.pose.orientation.z = 1;
    random_pose.pose.orientation.w = 0;
    goal_pub.publish(random_pose);
    //ROS_INFO("random_x: %f, random_y: %f",random_x, random_y);
    rate.sleep();
  }
  return 0;
}
  
