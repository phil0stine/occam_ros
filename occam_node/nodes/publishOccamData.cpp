#include <iostream>
#include <ros/ros.h>
#include <ros/package.h>

#include <opencv2/core/core.hpp>

#include "../include/OccamClasses.h"

using namespace std;


int main(int argc, char **argv) { // pass 2 arguments to set parameter: package_name, path to .yaml file


    OCCAM_CHECK(occamInitialize());

    ros::init(argc, argv, "occam");

    // define data to advertise
    vector<_OccamDataName> vTopics = {OCCAM_STITCHED_IMAGE0};

    // setup device and publishers
    OccamNode a(vTopics);

    // set parameters TODO: maybe use OccamConfig?
    if(argc == 3) {

        cout << "arg1: " << argv[1] << endl;
        cout << "arg2: " << argv[2] << endl;

        string strSettingsFile = ros::package::getPath(argv[1])+"/"+argv[2];
        cv::FileStorage fParams(strSettingsFile.c_str(), cv::FileStorage::READ); // TODO: maybe other method than opencv?
        if(!fParams.isOpened()) {
            ROS_ERROR("Wrong path to settings. No parameters will be changed.");
        }
        else {

            // load parameters
            int fps = fParams["Camera.fps"];
            int theta = fParams["Camera.theta"];
            int crop = fParams["Camera.crop"];
            float radius = fParams["Camera.radius"];
            int radiusInt = radius*1000000;
            int exposure = fParams["Camera.exposure"];

            // set parameters
            a.setDeviceValue(OCCAM_TARGET_FPS, fps);
            a.setDeviceValue(OCCAM_STITCHING_RADIUS, radiusInt);
            a.setDeviceValue(OCCAM_STITCHING_ROTATION, theta);
            a.setDeviceValue(OCCAM_STITCHING_CROP, crop);
            a.setDeviceValue(OCCAM_EXPOSURE, exposure);

            // display parameters
            cout << "--- OCCAM PARAMETERS ---" << endl;
            cout << "-- fps: " << a.getDeviceValue(OCCAM_TARGET_FPS) << endl;
            cout << "-- radius: " << a.getDeviceValue(OCCAM_STITCHING_RADIUS) << " [mm*1000]" << endl;
            cout << "-- theta: " << a.getDeviceValue(OCCAM_STITCHING_ROTATION) << endl;
            cout << "-- crop: " << a.getDeviceValue(OCCAM_STITCHING_CROP) << endl;

        }
    }

    // publish data
    a.spin();

    exit(0);
    return 0;

}
