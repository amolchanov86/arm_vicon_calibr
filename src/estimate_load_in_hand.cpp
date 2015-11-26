/*
 * estimate_load_in_hand.cpp
 *
 *  Created on: Jul 21, 2011
 *      Author: Righetti
 */


#include "ros/ros.h"
#include <arm_estimate_hand_load/load_estimator.h>


int main(int argc, char **argv)
{
  ros::init(argc, argv, "estimate_load_in_hand");
  ros::NodeHandle n("~");

  arm_estimate_hand_load::LoadEstimator estimator(n);

  ros::MultiThreadedSpinner spinner(3);
  spinner.spin();

  return 0;
}
