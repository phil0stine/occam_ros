/*
  Copyright 2011 - 2015 Occam Robotics Inc - All rights reserved.

  This software is licensed under GPLv2, as specified in the top-level
  LICENSE.txt file. Please contact us at info@occamvisiongroup.com for
  information regarding commercial licensing.

  __________________________________

  Adapted by JONAS EICHENBERGER: jo.eichenberger@gmail.com

  - classes to external file

*/


#include "../include/OccamClasses.h"

int main(int argc, char **argv) {
  OCCAM_CHECK(occamInitialize());
  ros::init(argc, argv, "occam");
  OccamNode a;
  a.spin();
  exit(0);
  return 0;
}
