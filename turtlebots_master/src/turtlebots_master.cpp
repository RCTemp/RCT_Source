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
  npc1NodePub3_ = turtlebotMasterNodeHandle_.advertise<std_msgs::UInt8>(npc1NodePub3Name_, 1);
  npc2NodePub1_ = turtlebotMasterNodeHandle_.advertise<std_msgs::Empty>(npc2NodePub1Name_, 1);
  npc2NodePub2_ = turtlebotMasterNodeHandle_.advertise<std_msgs::Empty>(npc2NodePub2Name_, 1);
  npc2NodePub3_ = turtlebotMasterNodeHandle_.advertise<std_msgs::UInt8>(npc2NodePub3Name_, 1);


  teleopNodeSub1_ = turtlebotMasterNodeHandle_.subscribe<geometry_msgs::PoseStamped>(teleopNodeSub1Name_, 1, &TurtlebotsMaster::teleopNodePoseCallback, this, ros::TransportHints().tcpNoDelay());
  teleopNodeSub2_ = turtlebotMasterNodeHandle_.subscribe<kobuki_msgs::BumperEvent>(teleopNodeSub2Name_, 1, &TurtlebotsMaster::teleopNodeBumperCallback, this, ros::TransportHints().tcpNoDelay());

  npc1NodeSub1_ = turtlebotMasterNodeHandle_.subscribe<geometry_msgs::PoseStamped>(npc1NodeSub1Name_, 1, &TurtlebotsMaster::npc1NodePoseCallback, this, ros::TransportHints().tcpNoDelay());
  npc2NodeSub1_ = turtlebotMasterNodeHandle_.subscribe<geometry_msgs::PoseStamped>(npc2NodeSub1Name_, 1, &TurtlebotsMaster::npc2NodePoseCallback, this, ros::TransportHints().tcpNoDelay());

  //callback
  npcNodesStartSub_ = turtlebotMasterNodeHandle_.subscribe<std_msgs::UInt8>(npcNodesStartSubName_, 1, &TurtlebotsMaster::npcNodesCmdCallback, this, ros::TransportHints().tcpNoDelay());

  //temporarily initialization for turtlebots pose
  teleopNodeX = 100;
  teleopNodeY = 100;
  teleopNodeTheta = 0;

  npc1NodeX = -100;
  npc1NodeY = -100;
  npc1NodeTheta = 0;

  npc2NodeX = -100;
  npc2NodeY = -100;
  npc2NodeTheta = 0;

  timer_ = turtlebotMasterNodeHandlePrivate_.createTimer(ros::Duration(1.0 / loopRate_), &TurtlebotsMaster::masterFunc, this);

  //debug
  npc1NodePub = turtlebotMasterNodeHandle_.advertise<visualization_msgs::Marker>("npc1_node_point", 1);
  teleopNodePub = turtlebotMasterNodeHandle_.advertise<visualization_msgs::Marker>("teleop_node_point", 1);

  teleopNodePoints.header.frame_id = "/map";
  teleopNodePoints.type = visualization_msgs::Marker::SPHERE_LIST;
  teleopNodePoints.action = visualization_msgs::Marker::ADD;
  teleopNodePoints.scale.x = 0.02;
  teleopNodePoints.scale.y = 0.02;
  teleopNodePoints.scale.z = 0.02;

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

  if (!turtlebotMasterNodeHandlePrivate_.getParam ("teleopNodeConnectFlag", teleopNodeConnectFlag_))
    teleopNodeConnectFlag_ = true;
  printf(" teleopNodeConnectFlag_ is %s\n", teleopNodeConnectFlag_?("true"):("false"));
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


  if (!turtlebotMasterNodeHandlePrivate_.getParam ("npc1NodeConnectFlag", npc1NodeConnectFlag_))
    npc1NodeConnectFlag_ = true;
  printf(" npc1NodeConnectFlag_ is %s\n", npc1NodeConnectFlag_?("true"):("false"));  
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
  if (!turtlebotMasterNodeHandlePrivate_.getParam ("npc1NodePub3Name", npc1NodePub3Name_))
    npc1NodePub3Name_ = std::string("empty");
  printf(" npc1NodePub3Name_ is %s\n", npc1NodePub3Name_.c_str());
  if (!turtlebotMasterNodeHandlePrivate_.getParam ("npc1NodePub3Type", npc1NodePub3Type_))
    npc1NodePub3Type_ = std::string("empty");
  printf(" npc1NodePub3Type_ is %s\n", npc1NodePub3Type_.c_str());
  if (!turtlebotMasterNodeHandlePrivate_.getParam ("npc1NodeSub1Name", npc1NodeSub1Name_))
    npc1NodeSub1Name_ = std::string("empty");
  printf(" npc1NodeSub1Name_ is %s\n", npc1NodeSub1Name_.c_str());
  if (!turtlebotMasterNodeHandlePrivate_.getParam ("npc1NodeSub1Type", npc1NodeSub1Type_))
    npc1NodeSub1Type_ = std::string("empty");
  printf(" npc1NodeSub1Type_ is %s\n", npc1NodeSub1Type_.c_str());

  if (!turtlebotMasterNodeHandlePrivate_.getParam ("npc2NodeConnectFlag", npc2NodeConnectFlag_))
    npc2NodeConnectFlag_ = true;
  printf(" npc2NodeConnectFlag_ is %s\n", npc2NodeConnectFlag_?("true"):("false"));  
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
  if (!turtlebotMasterNodeHandlePrivate_.getParam ("npc2NodePub3Name", npc2NodePub3Name_))
    npc2NodePub3Name_ = std::string("empty");
  printf(" npc2NodePub3Name_ is %s\n", npc2NodePub3Name_.c_str());
  if (!turtlebotMasterNodeHandlePrivate_.getParam ("npc2NodePub3Type", npc2NodePub3Type_))
    npc2NodePub3Type_ = std::string("empty");
  printf(" npc2NodePub3Type_ is %s\n", npc2NodePub3Type_.c_str());
  if (!turtlebotMasterNodeHandlePrivate_.getParam ("npc2NodeSub1Name", npc2NodeSub1Name_))
    npc2NodeSub1Name_ = std::string("empty");
  printf(" npc2NodeSub1Name_ is %s\n", npc2NodeSub1Name_.c_str());
  if (!turtlebotMasterNodeHandlePrivate_.getParam ("npc2NodeSub1Type", npc2NodeSub1Type_))
    npc2NodeSub1Type_ = std::string("empty");
  printf(" npc2NodeSub1Type_ is %s\n", npc2NodeSub1Type_.c_str());

  if(teleopNodeConnectFlag_)
    ROS_INFO("teleopNodeUri is %s, teleopNodePub1Name is %s, teleopNodePub1Type is %s, teleopNodePub2Name is %s, teleopNodePub2Type is %s, teleopNodeSub1Name is %s, teleopNodeSub1Type is %s, teleopNodeSub2Name is %s, teleopNodeSub2Type is %s", teleopNodeUri_.c_str(), teleopNodePub1Name_.c_str(), teleopNodePub1Type_.c_str(), teleopNodePub2Name_.c_str(), teleopNodePub2Type_.c_str(), teleopNodeSub1Name_.c_str(), teleopNodeSub1Type_.c_str(), teleopNodeSub2Name_.c_str(), teleopNodeSub2Type_.c_str());

  if(npc1NodeConnectFlag_)
    ROS_INFO("npc1NodeUri is %s, npc1NodePub1Name is %s, npc1NodePub1Type is %s, npc1NodePub2Name is %s, npc1NodePub2Type is %s, npc1NodePub3Name is %s, npc1NodePub3Type is %s, npc1NodeSub1Name is %s, npc1NodeSub1Type is %s", npc1NodeUri_.c_str(), npc1NodePub1Name_.c_str(), npc1NodePub1Type_.c_str(), npc1NodePub2Name_.c_str(), npc1NodePub2Type_.c_str(), npc1NodePub3Name_.c_str(), npc1NodePub3Type_.c_str(), npc1NodeSub1Name_.c_str(), npc1NodeSub1Type_.c_str());

  if(npc2NodeConnectFlag_)
    ROS_INFO("npc2NodeUri is %s, npc2NodePub1Name is %s, npc2NodePub1Type is %s, npc2NodePub2Name is %s, npc2NodePub2Type is %s, npc2NodePub3Name is %s, npc2NodePub3Type is %s, npc2NodeSub1Name is %s, npc2NodeSub1Type is %s", npc2NodeUri_.c_str(), npc2NodePub1Name_.c_str(), npc2NodePub1Type_.c_str(), npc2NodePub2Name_.c_str(), npc2NodePub2Type_.c_str(), npc2NodePub3Name_.c_str(), npc2NodePub3Type_.c_str(), npc2NodeSub1Name_.c_str(), npc2NodeSub1Type_.c_str());

  if (!turtlebotMasterNodeHandlePrivate_.getParam ("npcNodesStartSubName", npcNodesStartSubName_))
    npcNodesStartSubName_ = std::string("empty");
  printf(" npcNodesStartSubName_ is %s\n", npcNodesStartSubName_.c_str());

}

void TurtlebotsMaster::multiMasterMsgRegistration()
{
  if(!teleopNodeConnectFlag_ && !npc1NodeConnectFlag_ && !npc2NodeConnectFlag_)
    return;

  MultiMasterMsgRegistrator turtlebotsMsgRegistrator(turtlebotMasterNodeHandle_,
                                                     turtlebotMasterNodeHandlePrivate_, multiMasterMsgRate_);
  
  if(teleopNodeConnectFlag_)
    {
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
    }

  if(npc1NodeConnectFlag_)
    {
      ros::Duration(0.1).sleep();
      std::vector<std::string> pub_name_list1(3);
      pub_name_list1[0] = npc1NodePub1Name_;
      pub_name_list1[1] = npc1NodePub2Name_;
      pub_name_list1[2] = npc1NodePub3Name_;
      std::vector<std::string> pub_type_list1(3);
      pub_type_list1[0] = npc1NodePub1Type_;
      pub_type_list1[1] = npc1NodePub2Type_;
      pub_type_list1[2] = npc1NodePub3Type_;
      std::vector<std::string> sub_name_list1(1);
      sub_name_list1[0] = npc1NodeSub1Name_;
      std::vector<std::string> sub_type_list1(1);
      sub_type_list1[0] = npc1NodeSub1Type_;
      turtlebotsMsgRegistrator.msgRegistration(npc1NodeUri_, pub_name_list1, pub_type_list1,sub_name_list1, sub_type_list1);
  
    }

  if(npc2NodeConnectFlag_)
    {
      ros::Duration(0.1).sleep();
      std::vector<std::string> pub_name_list2(3);
      pub_name_list2[0] = npc2NodePub1Name_;
      pub_name_list2[1] = npc2NodePub2Name_;
      pub_name_list2[2] = npc2NodePub3Name_;
      std::vector<std::string> pub_type_list2(3);
      pub_type_list2[0] = npc2NodePub1Type_;
      pub_type_list2[1] = npc2NodePub2Type_;
      pub_type_list2[2] = npc2NodePub3Type_;
      std::vector<std::string> sub_name_list2(1);
      sub_name_list2[0] = npc2NodeSub1Name_;
      std::vector<std::string> sub_type_list2(1);
      sub_type_list2[0] = npc2NodeSub1Type_;
      turtlebotsMsgRegistrator.msgRegistration(npc2NodeUri_, pub_name_list2, pub_type_list2,sub_name_list2, sub_type_list2);
    }
}


void TurtlebotsMaster::masterFunc(const ros::TimerEvent & e)
{
  bool teleop_node_found_flag = false; 

  //compare with npc1
  float npc1_x_dash = npc1NodeX - dDash * cos(npc1NodeTheta);
  float npc1_y_dash = npc1NodeY - dDash * sin(npc1NodeTheta);

  float delta_d 
    = distanceBetweenTwoNodes(teleopNodeX, npc1NodeX, teleopNodeY, npc1NodeY);
  float delta_theta 
    = inclinationBetweenTwoNodes(teleopNodeX, npc1_x_dash, teleopNodeY, npc1_y_dash) - npc1NodeTheta;
  float delta_theta_dash
    = inclinationBetweenTwoNodes(teleopNodeX, npc1NodeX, teleopNodeY, npc1NodeY) - npc1NodeTheta;


  if(delta_d < (searchLightRadius_ + inscribedRadius_) &&
     fabs(delta_theta) < (searchLightRange_ / 2) && 
     fabs(delta_theta_dash) <  ((searchLightRange_ / 2) + M_PI / 2 ))
    {//distance and angle


      if(delta_d > searchLightRadius_ &&
         fabs(delta_theta_dash) > (searchLightRange_ / 2))
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
              teleop_node_found_flag = true;
            }
        }
      else
        {
          teleop_node_found_flag = true;
        }
    }


  //compare with npc2
  float npc2_x_dash = npc2NodeX - dDash * cos(npc2NodeTheta);
  float npc2_y_dash = npc2NodeY - dDash * sin(npc2NodeTheta);

  delta_d = distanceBetweenTwoNodes(teleopNodeX, npc2NodeX, teleopNodeY, npc2NodeY);
  delta_theta = inclinationBetweenTwoNodes(teleopNodeX, npc2_x_dash, teleopNodeY, npc2_y_dash) - npc2NodeTheta;
  delta_theta_dash = inclinationBetweenTwoNodes(teleopNodeX, npc2NodeX, teleopNodeY, npc2NodeY) - npc2NodeTheta;

  if(delta_d < (searchLightRadius_ + inscribedRadius_) &&
     fabs(delta_theta) < (searchLightRange_ / 2) &&
     fabs(delta_theta_dash) <  ((searchLightRange_ / 2) + M_PI / 2 ))
    {//distance and angle

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
              teleop_node_found_flag = true;

        }
      else
        {
          teleop_node_found_flag = true;
        }
    }


  if(teleop_node_found_flag)
    {
      ROS_WARN("turtlebot is found");
#if 0      //debug
      if(delta_d < (searchLightRadius_ ) &&
         fabs(delta_theta_dash) < ((searchLightRange_ / 2)))
        {
          teleopNodePoints.header.stamp = ros::Time::now();
          geometry_msgs::Point teleop_node_point;
          teleop_node_point.x = teleopNodeX;
          teleop_node_point.y = teleopNodeY;
          teleop_node_point.z = 0;
          teleopNodePoints.points.push_back(teleop_node_point);
          std_msgs::ColorRGBA teleop_node_point_color;
          teleop_node_point_color.r = 1;
          teleop_node_point_color.g = 0.55;
          teleop_node_point_color.b = 0;
          teleop_node_point_color.a = 1;
          teleopNodePoints.colors.push_back(teleop_node_point_color);
          teleopNodePub.publish(teleopNodePoints);
        }
      else
        {
          teleopNodePoints.header.stamp = ros::Time::now();
          geometry_msgs::Point teleop_node_point;
          teleop_node_point.x = teleopNodeX;
          teleop_node_point.y = teleopNodeY;
          teleop_node_point.z = 0;
          teleopNodePoints.points.push_back(teleop_node_point);
          std_msgs::ColorRGBA teleop_node_point_color;
          teleop_node_point_color.r = 1;
          teleop_node_point_color.g = 0;
          teleop_node_point_color.b = 0;
          teleop_node_point_color.a = 1;
          teleopNodePoints.colors.push_back(teleop_node_point_color);
          teleopNodePub.publish(teleopNodePoints);

        }

#endif
      teleopNodePub1_.publish(std_msgs::Empty());
      npc1NodePub1_.publish(std_msgs::Empty());
      npc2NodePub1_.publish(std_msgs::Empty());
    }
  else
    {
      //ROS_INFO("turtlebot is not found");
#if 0 //debug
      teleopNodePoints.header.stamp = ros::Time::now();
      geometry_msgs::Point teleop_node_point;
      teleop_node_point.x = teleopNodeX;
      teleop_node_point.y = teleopNodeY;
      teleop_node_point.z = 0;
      teleopNodePoints.points.push_back(teleop_node_point);
      std_msgs::ColorRGBA teleop_node_point_color;
      teleop_node_point_color.r = 0;
      teleop_node_point_color.g = 0;
      teleop_node_point_color.b = 1;
      teleop_node_point_color.a = 1;
      teleopNodePoints.colors.push_back(teleop_node_point_color);
      teleopNodePub.publish(teleopNodePoints);
#endif
    }

#if 0  //deubg
  visualization_msgs::Marker npcNodePoint;
  npcNodePoint.header.stamp = ros::Time::now();
  npcNodePoint.header.frame_id = "/map";
  npcNodePoint.type = visualization_msgs::Marker::SPHERE;
  npcNodePoint.action = visualization_msgs::Marker::ADD;
  npcNodePoint.pose.position.x = 0;
  npcNodePoint.pose.position.y = 0;
  npcNodePoint.pose.position.z = 0;
  npcNodePoint.pose.orientation.x = 0.0;
  npcNodePoint.pose.orientation.y = 0.0;
  npcNodePoint.pose.orientation.z = 0.0;
  npcNodePoint.pose.orientation.w = 1.0;
  npcNodePoint.scale.x = 0.1;
  npcNodePoint.scale.y = 0.1;
  npcNodePoint.scale.z = 0.1;
  npcNodePoint.color.a = 1.0;
  npcNodePoint.color.r = 0.0;
  npcNodePoint.color.g = 1.0;
  npcNodePoint.color.b = 0.0;
  npc1NodePub.publish(npcNodePoint);
#endif
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


void TurtlebotsMaster::npcNodesCmdCallback(const std_msgs::UInt8ConstPtr & cmd_msg)
{
  if(cmd_msg->data == STOP_CMD)
    {
      ROS_INFO("get stop cmd for npc nodes");
      std_msgs::UInt8 cmd_msgs;
      cmd_msgs.data = STOP_CMD;
      npc1NodePub3_.publish(cmd_msgs);
      npc2NodePub3_.publish(cmd_msgs);
    }
  else if(cmd_msg->data == START_CMD)
    {
      ROS_INFO("get start cmd for npc nodes");
      std_msgs::UInt8 cmd_msgs;
      cmd_msgs.data = START_CMD;
      npc1NodePub3_.publish(cmd_msgs);
      npc2NodePub3_.publish(cmd_msgs);
    }
  else
    ROS_WARN("get unknown cmd for npc nodes");
}

 float TurtlebotsMaster::distanceBetweenTwoNodes(float x1, float x2, float y1, float y2)
 {
   return sqrt((x1 - x2) * (x1 -x2) + (y1 - y2) * (y1 - y2));
 }

 float TurtlebotsMaster::inclinationBetweenTwoNodes(float x1, float x2, float y1, float y2)
 {
   return atan2((y1 - y2), (x1 - x2));
 }
