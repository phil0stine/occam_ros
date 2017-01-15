This is the **ROS** node for occam omnicam.

Please firstly finish the [installzation](https://github.com/CityU-MBE/occam) for occam sdk.

------

- **nodes/publishOccamData**: See how to pass the cameria intrisic information to occam sdk through configure yaml file.

- **nodes/publishStitchandDisparity.cpp**:  See how to set autoexposure mode.

- **nodes/publishSpecificDisparity.cpp**: See how to publish specific disparity image for the specific camera pairs.

- **ros_indigosdk_node** automatically publishes all of the RGB (separate and stitch), disparity image (pair and stitch) and pointclouds (pair). But there is something wrong with the official sdk to publish the disparity for camera pairs (stitch disparity image is fine). **rviz** cannot show the pair disparity image. However, you can still subscribe the pair disparity image data (through subscriber or command line **echo**). 

- There is no stitch-pointcloud topic for all of the cameras. 
