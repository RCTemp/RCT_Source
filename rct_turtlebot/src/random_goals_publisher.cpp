#include "rct_turtlebot/random_goals.h"


int main(int argc, char** argv){
  ros::init(argc, argv, "random_goals_publisher");
  ros::NodeHandle n;
  ros::NodeHandle np("~");

  random_goal RandomGoal(n, np);
  ros::Rate rate(0.2);
  
  while(ros::ok()){
    RandomGoal.random_make_goal();
    RandomGoal.goalpublish();
    rate.sleep();
  }
  return 0;
}
  
