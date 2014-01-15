#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <std_msgs/Empty.h>
#include <geometry_msgs/PoseStamped.h>

class npcNodeConveyor{
public:
  npcNodeConveyor(ros::NodeHandle node_handle, ros::NodeHandle node_handle_private):nh(node_handle), nh_private(node_handle_private)
  {

    if (!nh_private.getParam ("goalFromUI", goalFromUI_))
      goalFromUI_ = "random_goal";
    printf(" goalFromUI_ is %s\n", goalFromUI_.c_str());

    if (!nh_private.getParam ("goalForTurtlebot", goalForTurtlebot_))
      goalForTurtlebot_ = "simple_goal";
    printf(" goalForTurtlebot_ is %s\n", goalForTurtlebot_.c_str());

    stopFlagSub = nh.subscribe<std_msgs::Empty>("stop_flag_from_master",1, &npcNodeConveyor::stopFlagCallback, this, ros::TransportHints().tcpNoDelay());
    clearFlagSub = nh.subscribe<std_msgs::Empty>("clear_flag_from_master",1, &npcNodeConveyor::clearFlagCallback, this, ros::TransportHints().tcpNoDelay());
    goalSub = nh.subscribe<geometry_msgs::PoseStamped>(goalFromUI_, 1, &npcNodeConveyor::goalCallback, this, ros::TransportHints().tcpNoDelay());
    goalPub = nh.advertise<geometry_msgs::PoseStamped>(goalForTurtlebot_, 1);

    controlFlag = true;
  }
  ~npcNodeConveyor(){};

  void clearFlagCallback(const std_msgs::EmptyConstPtr& msg)
  {
    ROS_WARN("Game Clear!!");
    controlFlag=false;
  }

  void stopFlagCallback(const std_msgs::EmptyConstPtr& msg)
  {
    ROS_WARN("Game Over!!");
    controlFlag=false;
  }

  void goalCallback(const geometry_msgs::PoseStamped goal)
  {
    if(controlFlag){
      ROS_INFO("Got goal");
      goalPub.publish(goal);
    }
  }

private:
  bool controlFlag;
  ros::NodeHandle nh;
  ros::NodeHandle nh_private;
  ros::Publisher goalPub;
  ros::Subscriber goalSub;
  ros::Subscriber clearFlagSub;
  ros::Subscriber stopFlagSub;

  std::string goalFromUI_;
  std::string goalForTurtlebot_;

};




int main(int argc, char**argv){
  ros::init(argc, argv,"npc_node_conveyor");
  ros::NodeHandle nodeHandle;
  ros::NodeHandle nodeHandlePrivate("~");
  npcNodeConveyor* npcNodeConveyorNode = new npcNodeConveyor(nodeHandle, nodeHandlePrivate);

  ros::spin();
  delete npcNodeConveyorNode;
  return 0;
}






