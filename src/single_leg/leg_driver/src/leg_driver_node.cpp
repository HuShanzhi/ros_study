#include <ros/ros.h>
// #include "leg_driver/leg_driver.hpp"
#include "leg_driver.hpp"


int main(int argc, char** argv) {
  ros::init(argc, argv, "leg_driver_node");
  ros::NodeHandle nh;
  
  SingleLeg::LegDriver leg_driver(nh);
  leg_driver.spin();
  
  return 0;
}