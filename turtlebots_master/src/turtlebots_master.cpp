#include "turtlebots_master/turtlebots_master.h"
#include "turtlebots_master/multi_master_msg_registration.h"


TurtlebotsMaster::TurtlebotsMaster(ros::NodeHandle nh, ros::NodeHandle nh_private)
  : turtlebotMasterNodeHandle_(nh), turtlebotMasterNodeHandlePrivate_(nh_private)
{
  paramInit();
  multiMasterMsgRegistration();

  teleopNodePub_ = turtlebotMasterNodeHandle_.advertise<std_msgs::Bool>(teleopNodePubName_,1);
  npc1NodePub_ = turtlebotMasterNodeHandle_.advertise<std_msgs::Bool>(npc1NodePubName_,1);
  npc2NodePub_ = turtlebotMasterNodeHandle_.advertise<std_msgs::Bool>(npc1NodePubName_,1);

  teleopNodeSub_ = turtlebotMasterNodeHandle_.subscribe<geometry_msgs::PoseStamped>(teleopNodeSubName_, 1, &TurtlebotsMaster::teleopNodePoseCallback, this, ros::TransportHints().tcpNoDelay());
  npc1NodeSub_ = turtlebotMasterNodeHandle_.subscribe<geometry_msgs::PoseStamped>(npc1NodeSubName_, 1, &TurtlebotsMaster::teleopNodePoseCallback, this, ros::TransportHints().tcpNoDelay());
  npc2NodeSub_ = turtlebotMasterNodeHandle_.subscribe<geometry_msgs::PoseStamped>(npc2NodeSubName_, 1, &TurtlebotsMaster::teleopNodePoseCallback, this, ros::TransportHints().tcpNoDelay());


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

  if (!turtlebotMasterNodeHandlePrivate_.getParam ("multiMasterMsgRate", multiMasterMsgRate_))
    multiMasterMsgRate_ = 40.0;
  printf(" multiMasterMsgRate_ is %.3f\n", multiMasterMsgRate_);

  if (!turtlebotMasterNodeHandlePrivate_.getParam ("teleopNodeUri", teleopNodeUri_))
    teleopNodeUri_ = std::string("empty");
  printf(" teleopNodeUri_ is %s\n", teleopNodeUri_.c_str());
  if (!turtlebotMasterNodeHandlePrivate_.getParam ("teleopNodePubName", teleopNodePubName_))
    teleopNodePubName_ = std::string("empty");
  printf(" teleopNodePubName_ is %s\n", teleopNodePubName_.c_str());
  if (!turtlebotMasterNodeHandlePrivate_.getParam ("teleopNodePubType", teleopNodePubType_))
    teleopNodePubType_ = std::string("empty");
  printf(" teleopNodePubType_ is %s\n", teleopNodePubType_.c_str());
  if (!turtlebotMasterNodeHandlePrivate_.getParam ("teleopNodeSubName", teleopNodeSubName_))
    teleopNodeSubName_ = std::string("empty");
  printf(" teleopNodeSubName_ is %s\n", teleopNodeSubName_.c_str());
  if (!turtlebotMasterNodeHandlePrivate_.getParam ("teleopNodeSubType", teleopNodeSubType_))
    teleopNodeSubType_ = std::string("empty");
  printf(" teleopNodeSubType_ is %s\n", teleopNodeSubType_.c_str());


  if (!turtlebotMasterNodeHandlePrivate_.getParam ("npc1NodeUri", npc1NodeUri_))
    npc1NodeUri_ = std::string("empty");
  printf(" npc1NodeUri_ is %s\n", npc1NodeUri_.c_str());
  if (!turtlebotMasterNodeHandlePrivate_.getParam ("npc1NodePubName", npc1NodePubName_))
    npc1NodePubName_ = std::string("empty");
  printf(" npc1NodePubName_ is %s\n", npc1NodePubName_.c_str());
  if (!turtlebotMasterNodeHandlePrivate_.getParam ("npc1NodePubType", npc1NodePubType_))
    npc1NodePubType_ = std::string("empty");
  printf(" npc1NodePubType_ is %s\n", npc1NodePubType_.c_str());
  if (!turtlebotMasterNodeHandlePrivate_.getParam ("npc1NodeSubName", npc1NodeSubName_))
    npc1NodeSubName_ = std::string("empty");
  printf(" npc1NodeSubName_ is %s\n", npc1NodeSubName_.c_str());
  if (!turtlebotMasterNodeHandlePrivate_.getParam ("npc1NodeSubType", npc1NodeSubType_))
    npc1NodeSubType_ = std::string("empty");
  printf(" npc1NodeSubType_ is %s\n", npc1NodeSubType_.c_str());


  if (!turtlebotMasterNodeHandlePrivate_.getParam ("npc2NodeUri", npc2NodeUri_))
    npc2NodeUri_ = std::string("empty");
  printf(" npc2NodeUri_ is %s\n", npc2NodeUri_.c_str());
  if (!turtlebotMasterNodeHandlePrivate_.getParam ("npc2NodePubName", npc2NodePubName_))
    npc2NodePubName_ = std::string("empty");
  printf(" npc2NodePubName_ is %s\n", npc2NodePubName_.c_str());
  if (!turtlebotMasterNodeHandlePrivate_.getParam ("npc2NodePubType", npc2NodePubType_))
    npc2NodePubType_ = std::string("empty");
  printf(" npc2NodePubType_ is %s\n", npc2NodePubType_.c_str());
  if (!turtlebotMasterNodeHandlePrivate_.getParam ("npc2NodeSubName", npc2NodeSubName_))
    npc2NodeSubName_ = std::string("empty");
  printf(" npc2NodeSubName_ is %s\n", npc2NodeSubName_.c_str());
  if (!turtlebotMasterNodeHandlePrivate_.getParam ("npc2NodeSubType", npc2NodeSubType_))
    npc2NodeSubType_ = std::string("empty");
  printf(" npc2NodeSubType_ is %s\n", npc2NodeSubType_.c_str());

  ROS_INFO("teleopNodeUri is %s, teleopNodePubName is %s, teleopNodePubType is %s, teleopNodeSubName is %s, teleopNodeSubType is %s",
           teleopNodeUri_.c_str(), teleopNodePubName_.c_str(), teleopNodePubType_.c_str(), teleopNodeSubName_.c_str(), teleopNodeSubType_.c_str());

  ROS_INFO("npc1NodeUri is %s, npc1NodePubName is %s, npc1NodePubType is %s, npc1NodeSubName is %s, npc1NodeSubType is %s",
           npc1NodeUri_.c_str(), npc1NodePubName_.c_str(), npc1NodePubType_.c_str(), npc1NodeSubName_.c_str(), npc1NodeSubType_.c_str());

  ROS_INFO("npc2NodeUri is %s, npc2NodePubName is %s, npc2NodePubType is %s, npc2NodeSubName is %s, npc2NodeSubType is %s",
           npc2NodeUri_.c_str(), npc2NodePubName_.c_str(), npc2NodePubType_.c_str(), npc2NodeSubName_.c_str(), npc2NodeSubType_.c_str());

}

void TurtlebotsMaster::multiMasterMsgRegistration()
{
  MultiMasterMsgRegistrator turtlebotsMsgRegistrator(turtlebotMasterNodeHandle_,
                                                     turtlebotMasterNodeHandlePrivate_,
                                                     multiMasterMsgRate_);
  while(1)
    {
      if(turtlebotsMsgRegistrator.msgRegistration(teleopNodeUri_,
                                                 teleopNodePubName_,
                                                 teleopNodePubType_,
                                                 teleopNodeSubName_,
                                                 teleopNodeSubType_))
        {
          break;
        }
    }
  turtlebotsMsgRegistrator.msgRegistration(npc1NodeUri_,
                                          npc1NodePubName_, npc1NodePubType_,
                                          npc1NodeSubName_, npc1NodeSubType_);
  turtlebotsMsgRegistrator.msgRegistration(npc2NodeUri_,
                                          npc2NodePubName_, npc2NodePubType_,
                                          npc2NodeSubName_, npc2NodeSubType_);
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
              std_msgs::Bool found_flag;
              found_flag.data = true;
              teleopNodePub_.publish(found_flag);
              npc1NodePub_.publish(found_flag);
              npc2NodePub_.publish(found_flag);
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
          std_msgs::Bool found_flag;
          found_flag.data = true;
          teleopNodePub_.publish(found_flag);
          npc1NodePub_.publish(found_flag);
          npc2NodePub_.publish(found_flag);
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
              std_msgs::Bool found_flag;
              found_flag.data = true;
              teleopNodePub_.publish(found_flag);
              npc1NodePub_.publish(found_flag);
              npc2NodePub_.publish(found_flag);
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
          std_msgs::Bool found_flag;
          found_flag.data = true;
          teleopNodePub_.publish(found_flag);
          npc1NodePub_.publish(found_flag);
          npc2NodePub_.publish(found_flag);
          return;
        }
    }
}

void TurtlebotsMaster::teleopNodePoseCallback(const geometry_msgs::PoseStampedConstPtr & pose_msg)
{
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

void TurtlebotsMaster::npc1NodePoseCallback(const geometry_msgs::PoseStampedConstPtr & pose_msg)
{
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
