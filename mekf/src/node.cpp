#include <mekf/sensor_message_handler.h>

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "mekf");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  mekf::MessageHandler node(nh, pnh);
  ros::spin();
  return 0;
}
