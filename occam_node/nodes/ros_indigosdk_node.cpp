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
using namespace std;
int main(int argc, char **argv) {
  OCCAM_CHECK(occamInitialize());
  ros::init(argc, argv, "occam");

  // define data to advertise
  vector<_OccamDataName> vTopics = {OCCAM_RECTIFIED_IMAGE0,
                                    OCCAM_RECTIFIED_IMAGE1,
                                    OCCAM_RECTIFIED_IMAGE2,
                                    OCCAM_RECTIFIED_IMAGE3,
                                    OCCAM_RECTIFIED_IMAGE4,
                                    OCCAM_RECTIFIED_IMAGE5,
                                    OCCAM_RECTIFIED_IMAGE6,
                                    OCCAM_RECTIFIED_IMAGE7,
                                    OCCAM_RECTIFIED_IMAGE8,
                                    OCCAM_RECTIFIED_IMAGE9};
  //vector<_OccamDataName> vTopics = {OCCAM_STITCHED_POINT_CLOUD};

  // setup device and publishers
  OccamNode a(vTopics);

  a.spin();
  exit(0);
  return 0;
}
