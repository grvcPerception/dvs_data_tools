#include "dvs_data_tools/events2frames.h"

void shutdown(int s);

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "eventFrame_node");
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");

  Events2Frames eventsRenderer(nh, nh_private);

  ros::spin();

  return 0;
}
