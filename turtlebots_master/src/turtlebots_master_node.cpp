#include "turtlebots_master/turtlebots_master.h"

int main (int argc, char **argv)
{
  ros::init (argc, argv, "turtlebots_master");
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");
  TurtlebotsMaster*  turtlebotsMasterNode = new TurtlebotsMaster(nh, nh_private);
  ros::spin ();
  delete turtlebotsMasterNode;
  return 0;
}
