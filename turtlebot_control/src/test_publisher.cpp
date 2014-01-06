#include "turtlebot_control/ros_header.h"


int main(int argc, char** argv){
  ros::init(argc, argv, "test_publisher");
  ros::NodeHandle nh;
  ros::Publisher flag_publisher  = nh.advertise<std_msgs::Bool>("flag_from_master",1);
  std_msgs::Bool flag;
  bool tmp_flag;
  int get_flag;
  tmp_flag =true;
  // while(tmp_flag){
  std::cout<<"Put flag 1 or 0"<<std::endl;
  std::cin>>get_flag;
  std::cout<<get_flag<<std::endl;
  if(get_flag == 1){
 flag.data = true;
 tmp_flag = true;
  }else if(get_flag == 0){
 flag.data =false;
 tmp_flag=false;
 if(!flag.data) std::cout<<"ffafafa"<<std::endl;
  }
  if(!tmp_flag) std::cout<<"jsjsjsjsj"<<std::endl;
  // else{ break;}
  //std::cout<<"Flag = "<<flag.data<<std::endl;
  flag_publisher.publish(flag);
  // }

  //  tmp_flag = flag.data;
  // std::cout<<"tmp_flag = "<<tmp_flag<<std::endl;

  /*  
if( !(tmp_flag==0 || tmp_flag==1)) {
    std::cout<<"Please put 0 or 1"<<std::endl;
  }
  */


    
  std::cout<<"deta"<<std::endl;

  nav_msgs::Path path;
  ros::Publisher path_pub_ = nh.advertise<nav_msgs::Path>("path_from_UI",1);
  while(ros::ok()){ 
 path_pub_.publish(path);
  }
    ros::spin();
 
    
return 0;

}















