#include "Fiesta.h"

int main(int argc, char **argv) {
  ros::init(argc, argv, "FIESTA");
  ros::NodeHandle node("~");
  fiesta::Fiesta<sensor_msgs::PointCloud2::ConstPtr, geometry_msgs::TransformStamped::ConstPtr> esdf_map(node);
  ros::spin();
  return 0;
}
