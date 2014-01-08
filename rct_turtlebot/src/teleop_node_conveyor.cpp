#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <std_msgs/Empty.h>
#include <geometry_msgs/PoseStamped.h>


class teleopNodeConveyor{
public:
  teleopNodeConveyor(ros::NodeHandle node_handle, ros::NodeHandle node_handle_private):nh(node_handle), nh_private(node_handle_private)
  {
    if (!nh_private.getParam ("pathFromUI", pathFromUI_))
      pathFromUI_ = "unknown";
    printf(" pathFromUI_ is %s\n", pathFromUI_.c_str());

    if (!nh_private.getParam ("pathForTurtlebot", pathForTurtlebot_))
      pathForTurtlebot_ = "unknown";
    printf(" pathForTurtlebot_ is %s\n", pathForTurtlebot_.c_str());

    if (!nh_private.getParam ("cmdVelFromUI", cmdVelFromUI_))
      cmdVelFromUI_ = "teleop/cmd_vel";
    printf(" cmdVelFromUI_ is %s\n", cmdVelFromUI_.c_str());

    if (!nh_private.getParam ("cmdVelForTurtlebot", cmdVelForTurtlebot_))
      cmdVelForTurtlebot_ = "cmd_vel_mux/input/teleop";
    printf(" cmdVelForTurtlebot_ is %s\n", cmdVelForTurtlebot_.c_str());

    stopFlagSub = nh.subscribe<std_msgs::Empty>("stop_flag_from_master",1, &teleopNodeConveyor::stopFlagCallback, this, ros::TransportHints().tcpNoDelay());
    clearFlagSub = nh.subscribe<std_msgs::Empty>("clear_flag_from_master",1, &teleopNodeConveyor::clearFlagCallback, this, ros::TransportHints().tcpNoDelay());

    cmdVelSub = nh.subscribe<geometry_msgs::Twist>(cmdVelFromUI_, 1, &teleopNodeConveyor::cmdVelCallback, this, ros::TransportHints().tcpNoDelay());
    cmdVelPub = nh.advertise<geometry_msgs::Twist>(cmdVelForTurtlebot_,1); 

    pathSub = nh.subscribe<nav_msgs::Path>(pathFromUI_, 1, &teleopNodeConveyor::pathCallback, this, ros::TransportHints().tcpNoDelay());
    pathPub = nh.advertise<nav_msgs::Path>(pathForTurtlebot_, 1);

    controlFlag = true;
  }
  ~teleopNodeConveyor(){};

  void clearFlagCallback(const std_msgs::EmptyConstPtr& msg)
  {
    ROS_INFO("Game Clear!!");
    controlFlag=false;
  }

  void stopFlagCallback(const std_msgs::EmptyConstPtr& msg)
  {
    ROS_INFO("Game Over!!");
    controlFlag=false;
  }

  void pathCallback(const nav_msgs::Path path)
  {
  if(controlFlag){
    ROS_INFO("Got Path");
    pathPub.publish(path);
  }
  }
  
  void cmdVelCallback(const geometry_msgs::Twist cmd_vel)
  {
    if(controlFlag){
      ROS_INFO("Got Velocity Command");
      cmdVelPub.publish(cmd_vel);
    }
  }

private:
  bool controlFlag;
  ros::NodeHandle nh;
  ros::NodeHandle nh_private;
  ros::Publisher cmdVelPub;
  ros::Subscriber cmdVelSub;
  ros::Publisher pathPub;
  ros::Subscriber pathSub;
  ros::Subscriber clearFlagSub;
  ros::Subscriber stopFlagSub;

  std::string pathFromUI_;
  std::string pathForTurtlebot_;

  std::string cmdVelFromUI_;
  std::string cmdVelForTurtlebot_;
};



int main(int argc,char** argv){
  ros::init(argc, argv, "teleop_node_conveyor");
  ros::NodeHandle nodeHandle;
  ros::NodeHandle nodeHandlePrivate("~");
  teleopNodeConveyor* teleopNodeConveyorNode = new teleopNodeConveyor(nodeHandle, nodeHandlePrivate);

  ros::spin();
  delete teleopNodeConveyorNode;
  return 0;
}












