#include "../include/ros_header.h"
#include "tf/transform_broadcaster.h"

#define RANDMAX 20
#define RANDMIN -20


class random_goal{  
 public:
  geometry_msgs::PoseStamped goal_msg;
  double random_x;
  double random_y;
  
  ros::NodeHandle nh;
  ros::Publisher goal_pub; 
  
  void random_make_x(){
    double random;
    srand((unsigned)time(NULL));
    random = RANDMIN + (double)rand()/(RANDMAX - RANDMIN); 
    std::cout << "Random number is " <<random <<std::endl;
    random_x  = random;
  }

    void random_make_y(){
      double random;    
      srand((double)time(NULL));
    random = RANDMIN + (double)rand()/(RANDMAX - RANDMIN); 
    std::cout << "Random number is " <<random <<std::endl;
    random_y = random;
  }

  void makegoal(){
    goal_msg.header.stamp = ros::Time::now();
    goal_msg.header.frame_id = "/map";
    goal_msg.pose.position.x = random_x;
    goal_msg.pose.position.y = random_y;
    goal_msg.pose.position.z = 0;
    double yaw = 1.57;
    
    tf::Quaternion tmp = tf::createQuaternionFromYaw(yaw);
    goal_msg.pose.orientation.x = tmp.getX();
    goal_msg.pose.orientation.y = tmp.getY();
    goal_msg.pose.orientation.z = tmp.getZ();
    goal_msg.pose.orientation.w = tmp.getW();
  }
  
  void goalpublish(){
    goal_pub = nh.advertise<geometry_msgs::PoseStamped>("random_goal",1);
    goal_pub.publish(goal_msg);
  }
  
};
