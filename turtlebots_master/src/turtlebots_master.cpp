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
  //TODO

}

void TurtlebotsMaster::teleopNodePoseCallback(const geometry_msgs::PoseStampedConstPtr & pose_msg)
{
  teleopNodePose.header = pose_msg->header;
  teleopNodePose.pose = pose_msg->pose;
}

void TurtlebotsMaster::npc1NodePoseCallback(const geometry_msgs::PoseStampedConstPtr & pose_msg)
{
  npc1NodePose.header = pose_msg->header;
  npc1NodePose.pose = pose_msg->pose;
}

void TurtlebotsMaster::npc2NodePoseCallback(const geometry_msgs::PoseStampedConstPtr & pose_msg)
{
  npc2NodePose.header = pose_msg->header;
  npc2NodePose.pose = pose_msg->pose;
}


