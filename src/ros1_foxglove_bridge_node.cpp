#include <foxglove_bridge/ros1_foxglove_bridge.hpp>

#include <ros/ros.h>

int main(int argc, char** argv) {
  ros::init(argc, argv, "foxglove_bridge");
  ros::NodeHandle nh;
  ros::NodeHandle privateNh("~");

  foxglove_bridge::Ros1FoxgloveBridge bridge(nh, privateNh);

  // Foxglove client requests (subscribe, publish, service calls, ...) arrive
  // on the SDK's own threads; the ROS spinner only services ROS message
  // callbacks and timers.
  ros::AsyncSpinner spinner(4);
  spinner.start();
  ros::waitForShutdown();

  return 0;
}
