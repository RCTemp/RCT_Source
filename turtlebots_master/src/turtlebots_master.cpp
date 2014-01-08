#include "turtlebots_master/turtlebots_master.h"
#include "turtlebots_master/multi_master_msg_registration.h"


TurtlebotsMaster::TurtlebotsMaster(ros::NodeHandle nh, ros::NodeHandle nh_private)
  : turtlebotMasterNodeHandle_(nh), turtlebotMasterNodeHandlePrivate_(nh_private)
{
  paramInit();
  multiMasterMsgRegistration();

  teleopNodePub1_ = turtlebotMasterNodeHandle_.advertise<std_msgs::Empty>(teleopNodePub1Name_, 1);
  teleopNodePub2_ = turtlebotMasterNodeHandle_.advertise<std_msgs::Empty>(teleopNodePub2Name_, 1);
  npc1NodePub1_ = turtlebotMasterNodeHandle_.advertise<std_msgs::Empty>(npc1NodePub1Name_, 1);
  npc1NodePub2_ = turtlebotMasterNodeHandle_.advertise<std_msgs::Empty>(npc1NodePub2Name_, 1);
  npc2NodePub1_ = turtlebotMasterNodeHandle_.advertise<std_msgs::Empty>(npc2NodePub1Name_, 1);
  npc2NodePub2_ = turtlebotMasterNodeHandle_.advertise<std_msgs::Empty>(npc2NodePub2Name_, 1);


  teleopNodeSub1_ = turtlebotMasterNodeHandle_.subscribe<geometry_msgs::PoseStamped>(teleopNodeSub1Name_, 1, &TurtlebotsMaster::teleopNodePoseCallback, this, ros::TransportHints().tcpNoDelay());
  teleopNodeSub2_ = turtlebotMasterNodeHandle_.subscribe<kobuki_msgs::BumperEvent>(teleopNodeSub2Name_, 1, &TurtlebotsMaster::teleopNodeBumperCallback, this, ros::TransportHints().tcpNoDelay());

  npc1NodeSub1_ = turtlebotMasterNodeHandle_.subscribe<geometry_msgs::PoseStamped>(npc1NodeSub1Name_, 1, &TurtlebotsMaster::npc1NodePoseCallback, this, ros::TransportHints().tcpNoDelay());
  npc2NodeSub1_ = turtlebotMasterNodeHandle_.subscribe<geometry_msgs::PoseStamped>(npc2NodeSub1Name_, 1, &TurtlebotsMaster::npc2NodePoseCallback, this, ros::TransportHints().tcpNoDelay());


  //temporarily initialization for turtlebots pose
  teleopNodeX = 100;
  teleopNodeY = 100;
  teleopNodeTheta = 0;

  npc1NodeX = 100;
  npc1NodeY = 100;
  npc1NodeTheta = 0;

  npc2NodeX = -100;
  npc2NodeY = -100;
  npc2NodeTheta = 0;


  timer_ = turtlebotMasterNodeHandlePrivate_.createTimer(ros::Duration(1.0 / loopRate_), &TurtlebotsMaster::masterFunc, this);

}


TurtlebotsMaster::~TurtlebotsMaster()
{
}


void TurtlebotsMaster::paramInit()
{

  if (!turtlebotMasterNodeHandlePrivate_.getParam ("loopRate", loopRate_))
    loopRate_ = 10;
  printf(" loopRate_ is %.3f\n", loopRate_);

  if (!turtlebotMasterNodeHandlePrivate_.getParam ("searchLightRadius", searchLightRadius_))
    searchLightRadius_ = 0.5;
  printf(" searchLightRadius_ is %.3f\n", searchLightRadius_);

  if (!turtlebotMasterNodeHandlePrivate_.getParam ("searchLightRange", searchLightRange_))
    searchLightRange_ = 1.047; //60circle
  printf(" searchLightRange_ is %.3f\n", searchLightRange_);

  if (!turtlebotMasterNodeHandlePrivate_.getParam ("inscribedRadius", inscribedRadius_))
    inscribedRadius_ = 0.2;
  printf(" inscribedRadius_ is %.3f\n", inscribedRadius_);

  dDash = inscribedRadius_ / sin(searchLightRange_ / 2);

  if (!turtlebotMasterNodeHandlePrivate_.getParam ("nodeIntervalThre", nodeIntervalThre_))
    nodeIntervalThre_ = 0.4;
  printf(" nodeIntervalThre_ is %.3f\n", nodeIntervalThre_);


  if (!turtlebotMasterNodeHandlePrivate_.getParam ("multiMasterMsgRate", multiMasterMsgRate_))
    multiMasterMsgRate_ = 40.0;
  printf(" multiMasterMsgRate_ is %.3f\n", multiMasterMsgRate_);

  if (!turtlebotMasterNodeHandlePrivate_.getParam ("teleopNodeUri", teleopNodeUri_))
    teleopNodeUri_ = std::string("empty");
  printf(" teleopNodeUri_ is %s\n", teleopNodeUri_.c_str());
  if (!turtlebotMasterNodeHandlePrivate_.getParam ("teleopNodePub1Name", teleopNodePub1Name_))
    teleopNodePub1Name_ = std::string("empty");
  printf(" teleopNodePub1Name_ is %s\n", teleopNodePub1Name_.c_str());
  if (!turtlebotMasterNodeHandlePrivate_.getParam ("teleopNodePub1Type", teleopNodePub1Type_))
    teleopNodePub1Type_ = std::string("empty");
  printf(" teleopNodePub1Type_ is %s\n", teleopNodePub1Type_.c_str());
  if (!turtlebotMasterNodeHandlePrivate_.getParam ("teleopNodePub2Name", teleopNodePub2Name_))
    teleopNodePub2Name_ = std::string("empty");
  printf(" teleopNodePub2Name_ is %s\n", teleopNodePub2Name_.c_str());
  if (!turtlebotMasterNodeHandlePrivate_.getParam ("teleopNodePub2Type", teleopNodePub2Type_))
    teleopNodePub2Type_ = std::string("empty");
  printf(" teleopNodePub2Type_ is %s\n", teleopNodePub2Type_.c_str());
  if (!turtlebotMasterNodeHandlePrivate_.getParam ("teleopNodeSub1Name", teleopNodeSub1Name_))
    teleopNodeSub1Name_ = std::string("empty");
  printf(" teleopNodeSub1Name_ is %s\n", teleopNodeSub1Name_.c_str());
  if (!turtlebotMasterNodeHandlePrivate_.getParam ("teleopNodeSub1Type", teleopNodeSub1Type_))
    teleopNodeSub1Type_ = std::string("empty");
  printf(" teleopNodeSub1Type_ is %s\n", teleopNodeSub1Type_.c_str());
  if (!turtlebotMasterNodeHandlePrivate_.getParam ("teleopNodeSub2Name", teleopNodeSub2Name_))
    teleopNodeSub2Name_ = std::string("empty");
  printf(" teleopNodeSub2Name_ is %s\n", teleopNodeSub2Name_.c_str());
  if (!turtlebotMasterNodeHandlePrivate_.getParam ("teleopNodeSub2Type", teleopNodeSub2Type_))
    teleopNodeSub2Type_ = std::string("empty");
  printf(" teleopNodeSub2Type_ is %s\n", teleopNodeSub2Type_.c_str());

  
  if (!turtlebotMasterNodeHandlePrivate_.getParam ("npc1NodeUri", npc1NodeUri_))
    npc1NodeUri_ = std::string("empty");
  printf(" npc1NodeUri_ is %s\n", npc1NodeUri_.c_str());
  if (!turtlebotMasterNodeHandlePrivate_.getParam ("npc1NodePub1Name", npc1NodePub1Name_))
    npc1NodePub1Name_ = std::string("empty");
  printf(" npc1NodePub1Name_ is %s\n", npc1NodePub1Name_.c_str());
  if (!turtlebotMasterNodeHandlePrivate_.getParam ("npc1NodePub1Type", npc1NodePub1Type_))
    npc1NodePub1Type_ = std::string("empty");
  printf(" npc1NodePub1Type_ is %s\n", npc1NodePub1Type_.c_str());
  if (!turtlebotMasterNodeHandlePrivate_.getParam ("npc1NodePub2Name", npc1NodePub2Name_))
    npc1NodePub2Name_ = std::string("empty");
  printf(" npc1NodePub2Name_ is %s\n", npc1NodePub2Name_.c_str());
  if (!turtlebotMasterNodeHandlePrivate_.getParam ("npc1NodePub2Type", npc1NodePub2Type_))
    npc1NodePub2Type_ = std::string("empty");
  printf(" npc1NodePub2Type_ is %s\n", npc1NodePub2Type_.c_str());
  if (!turtlebotMasterNodeHandlePrivate_.getParam ("npc1NodeSub1Name", npc1NodeSub1Name_))
    npc1NodeSub1Name_ = std::string("empty");
  printf(" npc1NodeSub1Name_ is %s\n", npc1NodeSub1Name_.c_str());
  if (!turtlebotMasterNodeHandlePrivate_.getParam ("npc1NodeSub1Type", npc1NodeSub1Type_))
    npc1NodeSub1Type_ = std::string("empty");
  printf(" npc1NodeSub1Type_ is %s\n", npc1NodeSub1Type_.c_str());


  if (!turtlebotMasterNodeHandlePrivate_.getParam ("npc2NodeUri", npc2NodeUri_))
    npc2NodeUri_ = std::string("empty");
  printf(" npc2NodeUri_ is %s\n", npc2NodeUri_.c_str());
  if (!turtlebotMasterNodeHandlePrivate_.getParam ("npc2NodePub1Name", npc2NodePub1Name_))
    npc2NodePub1Name_ = std::string("empty");
  printf(" npc2NodePub1Name_ is %s\n", npc2NodePub1Name_.c_str());
  if (!turtlebotMasterNodeHandlePrivate_.getParam ("npc2NodePub1Type", npc2NodePub1Type_))
    npc2NodePub1Type_ = std::string("empty");
  printf(" npc2NodePub1Type_ is %s\n", npc2NodePub1Type_.c_str());
  if (!turtlebotMasterNodeHandlePrivate_.getParam ("npc2NodePub2Name", npc2NodePub2Name_))
    npc2NodePub2Name_ = std::string("empty");
  printf(" npc2NodePub2Name_ is %s\n", npc2NodePub2Name_.c_str());
  if (!turtlebotMasterNodeHandlePrivate_.getParam ("npc2NodePub2Type", npc2NodePub2Type_))
    npc2NodePub2Type_ = std::string("empty");
  printf(" npc2NodePub2Type_ is %s\n", npc2NodePub2Type_.c_str());
  if (!turtlebotMasterNodeHandlePrivate_.getParam ("npc2NodeSub1Name", npc2NodeSub1Name_))
    npc2NodeSub1Name_ = std::string("empty");
  printf(" npc2NodeSub1Name_ is %s\n", npc2NodeSub1Name_.c_str());
  if (!turtlebotMasterNodeHandlePrivate_.getParam ("npc2NodeSub1Type", npc2NodeSub1Type_))
    npc2NodeSub1Type_ = std::string("empty");
  printf(" npc2NodeSub1Type_ is %s\n", npc2NodeSub1Type_.c_str());

  ROS_INFO("teleopNodeUri is %s, teleopNodePub1Name is %s, teleopNodePub1Type is %s, teleopNodePub1Name is %s, teleopNodePub1Type is %s, teleopNodeSub1Name is %s, teleopNodeSub1Type is %s, teleopNodeSub2Name is %s, teleopNodeSub2Type is %s", teleopNodeUri_.c_str(), teleopNodePub1Name_.c_str(), teleopNodePub1Type_.c_str(), teleopNodePub2Name_.c_str(), teleopNodePub2Type_.c_str(), teleopNodeSub1Name_.c_str(), teleopNodeSub1Type_.c_str(), teleopNodeSub2Name_.c_str(), teleopNodeSub2Type_.c_str());

  ROS_INFO("npc1NodeUri is %s, npc1NodePub1Name is %s, npc1NodePub1Type is %s, npc1NodePub2Name is %s, npc1NodePub2Type is %s, npc1NodeSub1Name is %s, npc1NodeSub1Type is %s", npc1NodeUri_.c_str(), npc1NodePub1Name_.c_str(), npc1NodePub1Type_.c_str(), npc1NodePub2Name_.c_str(), npc1NodePub2Type_.c_str(), npc1NodeSub1Name_.c_str(), npc1NodeSub1Type_.c_str());

  ROS_INFO("npc2NodeUri is %s, npc2NodePub1Name is %s, npc2NodePub1Type is %s, npc2NodePub2Name is %s, npc2NodePub2Type is %s, npc2NodeSub1Name is %s, npc2NodeSub1Type is %s", npc2NodeUri_.c_str(), npc2NodePub1Name_.c_str(), npc2NodePub1Type_.c_str(), npc2NodePub2Name_.c_str(), npc2NodePub2Type_.c_str(), npc2NodeSub1Name_.c_str(), npc2NodeSub1Type_.c_str());

}

void TurtlebotsMaster::multiMasterMsgRegistration()
{
  MultiMasterMsgRegistrator turtlebotsMsgRegistrator(turtlebotMasterNodeHandle_,
                                                     turtlebotMasterNodeHandlePrivate_, multiMasterMsgRate_);

  while(1)
    {
      std::vector<std::string> pub_name_list(2);
      pub_name_list[0] = teleopNodePub1Name_;
      pub_name_list[1] = teleopNodePub2Name_;
      std::vector<std::string> pub_type_list(2);
      pub_type_list[0] = teleopNodePub1Type_;
      pub_type_list[1] = teleopNodePub2Type_;
      std::vector<std::string> sub_name_list(2);
      sub_name_list[0] = teleopNodeSub1Name_;
      sub_name_list[1] = teleopNodeSub2Name_;
      std::vector<std::string> sub_type_list(2);
      sub_type_list[0] = teleopNodeSub1Type_;
      sub_type_list[1] = teleopNodeSub2Type_;

      if(turtlebotsMsgRegistrator.msgRegistration(teleopNodeUri_, pub_name_list, pub_type_list, sub_name_list, sub_type_list) == 1)
        {
          ROS_WARN("registration succeed");
          break;
        }
      ros::Duration(1.0).sleep();
    }

  ros::Duration(0.1).sleep();
  std::vector<std::string> pub_name_list1(2);
  pub_name_list1[0] = npc1NodePub1Name_;
  pub_name_list1[1] = npc1NodePub2Name_;
  std::vector<std::string> pub_type_list1(2);
  pub_type_list1[0] = npc1NodePub1Type_;
  pub_type_list1[1] = npc1NodePub2Type_;
  std::vector<std::string> sub_name_list1(1);
  sub_name_list1[0] = npc1NodeSub1Name_;
  std::vector<std::string> sub_type_list1(1);
  sub_type_list1[0] = npc1NodeSub1Type_;
  turtlebotsMsgRegistrator.msgRegistration(npc1NodeUri_, pub_name_list1, pub_type_list1,sub_name_list1, sub_type_list1);
  
  ros::Duration(0.1).sleep();
  std::vector<std::string> pub_name_list2(2);
  pub_name_list2[0] = npc2NodePub1Name_;
  pub_name_list2[1] = npc2NodePub2Name_;
  std::vector<std::string> pub_type_list2(2);
  pub_type_list2[0] = npc2NodePub1Type_;
  pub_type_list2[1] = npc2NodePub2Type_;
  std::vector<std::string> sub_name_list2(1);
  sub_name_list2[0] = npc2NodeSub1Name_;
  std::vector<std::string> sub_type_list2(1);
  sub_type_list2[0] = npc2NodeSub1Type_;
  turtlebotsMsgRegistrator.msgRegistration(npc2NodeUri_, pub_name_list2, pub_type_list2,sub_name_list2, sub_type_list2);

}

void TurtlebotsMaster::masterFunc(const ros::TimerEvent & e)
{
  //compare with npc1
  float npc1_x_dash = npc1NodeX - dDash * cos(npc1NodeTheta);
  float npc1_y_dash = npc1NodeY - dDash * sin(npc1NodeTheta);

  float delta_d 
    = distanceBetweenTwoNodes(teleopNodeX, npc1NodeX, teleopNodeY, npc1NodeY);
  float delta_theta 
    = inclinationBetweenTwoNodes(teleopNodeX, npc1_x_dash, teleopNodeY, npc1_y_dash) - npc1NodeTheta;

  ROS_INFO("npc1: delta_d is %f, delta_theta is %f", delta_d , delta_theta);

  if(delta_d < (searchLightRadius_ + inscribedRadius_) &&
     fabs(delta_theta) < (searchLightRange_ / 2))
    {//distance and angle

      float delta_theta_dash
        = inclinationBetweenTwoNodes(teleopNodeX, npc1NodeX, teleopNodeY, npc1NodeY) - npc1NodeTheta;
      if(delta_d > searchLightRadius_ && fabs(delta_theta_dash) > (searchLightRange_ / 2))
        {
          float delta_r;
          if(delta_theta_dash > 0)
            {
              float x_corner = npc1NodeX + searchLightRadius_ * cos(npc1NodeTheta + searchLightRange_ / 2);
              float y_corner = npc1NodeY + searchLightRadius_ * sin(npc1NodeTheta + searchLightRange_ / 2);
              delta_r = distanceBetweenTwoNodes(teleopNodeX, x_corner, teleopNodeY, y_corner);

            }
          else
            {
              float x_corner = npc1NodeX + searchLightRadius_ * cos(npc1NodeTheta - searchLightRange_ / 2);
              float y_corner = npc1NodeY + searchLightRadius_ * sin(npc1NodeTheta - searchLightRange_ / 2);
              delta_r = distanceBetweenTwoNodes(teleopNodeX, x_corner, teleopNodeY, y_corner);
            }

          if(delta_r < inscribedRadius_)
            {
              ROS_WARN("turtlebot is found");
              teleopNodePub1_.publish(std_msgs::Empty());
              npc1NodePub1_.publish(std_msgs::Empty());
              npc2NodePub1_.publish(std_msgs::Empty());
              return;
            }
          else
            { 
              ROS_INFO("turtlebot is not found");
            }
        }
      else
        {
          ROS_WARN("turtlebot is found");
          teleopNodePub1_.publish(std_msgs::Empty());
          npc1NodePub1_.publish(std_msgs::Empty());
          npc2NodePub1_.publish(std_msgs::Empty());
          return;
        }
    }


  //compare with npc2
  float npc2_x_dash = npc2NodeX - dDash * cos(npc2NodeTheta);
  float npc2_y_dash = npc2NodeY - dDash * sin(npc2NodeTheta);

  delta_d = distanceBetweenTwoNodes(teleopNodeX, npc2NodeX, teleopNodeY, npc2NodeY);
  delta_theta = inclinationBetweenTwoNodes(teleopNodeX, npc2_x_dash, teleopNodeY, npc2_y_dash) - npc2NodeTheta;

  ROS_INFO("npc2: delta_d is %f, delta_theta is %f", delta_d , delta_theta);

  if(delta_d < (searchLightRadius_ + inscribedRadius_) &&
     fabs(delta_theta) < (searchLightRange_ / 2))
    {//distance and angle

      float delta_theta_dash
        = inclinationBetweenTwoNodes(teleopNodeX, npc2NodeX, teleopNodeY, npc2NodeY) - npc2NodeTheta;
      if(delta_d > searchLightRadius_ && fabs(delta_theta_dash) > (searchLightRange_ / 2))
        {
          float delta_r;
          if(delta_theta_dash > 0)
            {
              float x_corner = npc2NodeX + searchLightRadius_ * cos(npc2NodeTheta + searchLightRange_ / 2);
              float y_corner = npc2NodeY + searchLightRadius_ * sin(npc2NodeTheta + searchLightRange_ / 2);
              delta_r = distanceBetweenTwoNodes(teleopNodeX, x_corner, teleopNodeY, y_corner);

            }
          else
            {
              float x_corner = npc2NodeX + searchLightRadius_ * cos(npc2NodeTheta - searchLightRange_ / 2);
              float y_corner = npc2NodeY + searchLightRadius_ * sin(npc2NodeTheta - searchLightRange_ / 2);
              delta_r = distanceBetweenTwoNodes(teleopNodeX, x_corner, teleopNodeY, y_corner);
            }

          if(delta_r < inscribedRadius_)
            {
              ROS_WARN("turtlebot is found");
              teleopNodePub1_.publish(std_msgs::Empty());
              npc1NodePub1_.publish(std_msgs::Empty());
              npc2NodePub1_.publish(std_msgs::Empty());

              return;
            }
          else
            { 
              ROS_INFO("turtlebot is not found");
            }
        }
      else
        {
          ROS_WARN("turtlebot is found");
          teleopNodePub1_.publish(std_msgs::Empty());
          npc1NodePub1_.publish(std_msgs::Empty());
          npc2NodePub1_.publish(std_msgs::Empty());
          return;
        }
    }
}

void TurtlebotsMaster::teleopNodePoseCallback(const geometry_msgs::PoseStampedConstPtr & pose_msg)
{
  ROS_INFO("get teleop node pose");
  teleopNodeX = pose_msg->pose.position.x;
  teleopNodeY = pose_msg->pose.position.y;
  double roll, pitch, yaw;
  tf::Quaternion q(pose_msg->pose.orientation.x,
                   pose_msg->pose.orientation.y,
                   pose_msg->pose.orientation.z,
                   pose_msg->pose.orientation.w);
  tf::Matrix3x3(q).getRPY(roll, pitch, yaw);
  teleopNodeTheta = yaw;
}

void TurtlebotsMaster::teleopNodeBumperCallback(const kobuki_msgs::BumperEventConstPtr & bumper_msg)
{
  float delta_d1
    = distanceBetweenTwoNodes(teleopNodeX, npc1NodeX, teleopNodeY, npc1NodeY);
  float delta_d2 
    = distanceBetweenTwoNodes(teleopNodeX, npc2NodeX, teleopNodeY, npc2NodeY);

  if(delta_d1 < nodeIntervalThre_ || delta_d2 < nodeIntervalThre_)
    {
      ROS_WARN("touch the npc");
      teleopNodePub2_.publish(std_msgs::Empty());
      npc1NodePub2_.publish(std_msgs::Empty());
      npc2NodePub2_.publish(std_msgs::Empty());
    }
}

void TurtlebotsMaster::npc1NodePoseCallback(const geometry_msgs::PoseStampedConstPtr & pose_msg)
{
  ROS_INFO("get npc1 node pose");
  npc1NodeX = pose_msg->pose.position.x;
  npc1NodeY = pose_msg->pose.position.y;
  double roll, pitch, yaw;
  tf::Quaternion q(pose_msg->pose.orientation.x,
                   pose_msg->pose.orientation.y,
                   pose_msg->pose.orientation.z,
                   pose_msg->pose.orientation.w);
  tf::Matrix3x3(q).getRPY(roll, pitch, yaw);
  npc1NodeTheta = yaw;
}

void TurtlebotsMaster::npc2NodePoseCallback(const geometry_msgs::PoseStampedConstPtr & pose_msg)
{
  ROS_INFO("get npc2 node pose");
  npc2NodeX = pose_msg->pose.position.x;
  npc2NodeY = pose_msg->pose.position.y;
  double roll, pitch, yaw;
  tf::Quaternion q(pose_msg->pose.orientation.x,
                   pose_msg->pose.orientation.y,
                   pose_msg->pose.orientation.z,
                   pose_msg->pose.orientation.w);
  tf::Matrix3x3(q).getRPY(roll, pitch, yaw);
  npc2NodeTheta = yaw;
}


 float TurtlebotsMaster::distanceBetweenTwoNodes(float x1, float x2, float y1, float y2)
 {
   return sqrt((x1 - x2) * (x1 -x2) + (y1 - y2) * (y1 - y2));
 }

 float TurtlebotsMaster::inclinationBetweenTwoNodes(float x1, float x2, float y1, float y2)
 {
   return atan2((x1 - x2), (y1 - y2));
 }
