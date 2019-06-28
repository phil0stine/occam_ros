#include <ros/ros.h>
#include <iostream>

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

    // set parameters TODO: use launch file, use OccamConfig
    //cout << "fps: " << a.getDeviceValue(OCCAM_TARGET_FPS) << endl;
    //a.setDeviceValue(OCCAM_TARGET_FPS, 15);
    //cout << "fps: " << a.getDeviceValue(OCCAM_TARGET_FPS) << endl;

    // publish data
    a.spin();

    exit(0);
    return 0;

}
