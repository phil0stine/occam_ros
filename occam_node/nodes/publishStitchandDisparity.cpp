#include <ros/ros.h>
#include <iostream>

#include "../include/OccamClasses.h"

using namespace std;


int main(int argc, char **argv) {


    OCCAM_CHECK(occamInitialize());

    ros::init(argc, argv, "occam");

    // define data to advertise
    vector<_OccamDataName> vTopics = {OCCAM_STITCHED_DISPARITY_IMAGE, OCCAM_STITCHED_IMAGE0};

    // setup device and publishers
    OccamNode a(vTopics);

    // set parameters TODO: use launch file, use OccamConfig
    cout << "fps: " << a.getDeviceValue(OCCAM_TARGET_FPS) << endl;
    cout << "Micro: " << a.getDeviceValue(OCCAM_EXPOSURE_MICROSECONDS) << endl;
    cout << "Exposure: " << a.getDeviceValue(OCCAM_EXPOSURE) << endl;
    a.setDeviceValue(OCCAM_TARGET_FPS, 15);
    a.setDeviceValue(OCCAM_EXPOSURE, 1);
    a.setDeviceValue(OCCAM_AUTO_EXPOSURE, 1);
    //a.setDeviceValue(OCCAM_EXPOSURE_MICROSECONDS, 1000);
    cout << "fps: " << a.getDeviceValue(OCCAM_TARGET_FPS) << endl;
    cout << "Micro: " << a.getDeviceValue(OCCAM_EXPOSURE_MICROSECONDS) << endl;
    cout << "Exposure: " << a.getDeviceValue(OCCAM_EXPOSURE) << endl;
    cout << "AutoExposure: " << a.getDeviceValue(OCCAM_AUTO_EXPOSURE) << endl;
    
    // publish data
    a.spin();

    exit(0);
    return 0;

}
