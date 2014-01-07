#include "turtlebot_control/random_goals.h"


int main(int argc, char** argv){
  ros::init(argc, argv, "random_goals_publisher");
  random_goal RandomGoal;
  ros::Rate rate(1);
  
  while(ros::ok()){
    RandomGoal.random_make_x();
    RandomGoal.random_make_y();
    RandomGoal.makegoal();
    RandomGoal.goalpublish();
    rate.sleep();
  }
  return 0;
}
  