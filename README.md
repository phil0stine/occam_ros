# OCCAM SETUP INSTRUCTION

# Installation

### 1. Clone this repository

```sh
git clone https://github.com/CityU-MBE/occam_ros.git
```
or
```sh
git clone git@github.com:CityU-MBE/occam.git
```

### 2. Setup udev rule for the occam

Execute the following command

```
echo 'SUBSYSTEM=="usb", ATTRS{idVendor}=="285e", ATTRS{idProduct}=="3efd", MODE="0666", GROUP="plugdev", SYMLINK+="occam"' | sudo tee /etc/udev/rules.d/occam.rules
```

### 3. Build the occam SDK

Go into 'occam_sdk'

```
mkdir release
cd release
cmake -DUSE_OPENCV=1 -DUSE_OPENGL=1 ..
make
```

**Test**

Execute the following program: `bin/read_images_opencv`
You should see the output from both camera sets
Make sure both USB cables are plugged in

### 3. Build the ROS node
Adapt the absolute path to the sdk in `occam_node/CMakeLists.txt` on line 20:

```
set(INDIGOSDK_PATH "/<absolute/path/to/the/repo>/occam/occam_sdk")
```

Link the package to your src folder in the catkin workspace:

```
ln -sf <path/to/occam_node> <path/to/catkin_ws/src>
```

Run `catkin_make` or `catkin build` in your catkin_ws

**Test**

There is an example roslaunch file in the occam_node:

```
roslaunch viewStichedImage.launch
```

You should see the stiched output stream from one camera rig
