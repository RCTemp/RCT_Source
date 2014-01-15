#include "rct_turtlebot/random_goals.h"


int main(int argc, char** argv){
  ros::init(argc, argv, "random_goals_publisher");
  ros::NodeHandle n;
  ros::NodeHandle np("~");

  random_goal RandomGoal(n, np);

  
  ros::spin();

  return 0;
}
  
