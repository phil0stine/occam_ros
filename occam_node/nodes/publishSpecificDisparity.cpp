#include <ros/ros.h>
#include <iostream>

#include "../include/OccamClasses.h"

using namespace std;


int main(int argc, char **argv) {


    OCCAM_CHECK(occamInitialize());

    ros::init(argc, argv, "occam");

    // define data to advertise
    vector<_OccamDataName> vTopics = {OCCAM_DISPARITY_IMAGE0};

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
