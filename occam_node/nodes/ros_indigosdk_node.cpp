/*
  Copyright 2011 - 2018 Occam Robotics Inc - All rights reserved.

  This software is licensed under GPLv2, as specified in the top-level
  LICENSE.txt file. Please contact us at info@occamvisiongroup.com for
  information regarding commercial licensing.
*/

#include <stdio.h>
#include <iostream>
#include <atomic>
#include <ros/ros.h>
#include <sensor_msgs/fill_image.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/ChannelFloat32.h>
#include <sensor_msgs/CameraInfo.h>
#include <geometry_msgs/Point32.h>
#include <image_transport/image_transport.h>
#include <camera_info_manager/camera_info_manager.h>
#include <dynamic_reconfigure/Config.h>
#include <dynamic_reconfigure/ConfigDescription.h>
#include <dynamic_reconfigure/Reconfigure.h>
#include <opencv2/imgproc/imgproc.hpp>
#include "indigo.h"

static void reportError(int error_code) {
  ROS_INFO("Occam API Error: %i", error_code);
  abort();
}

#define OCCAM_CHECK(call) { int r = call; if (r != OCCAM_API_SUCCESS) reportError(r); }

static std::string dataNameString(OccamDataName data_name) {
  switch (data_name) {
  case OCCAM_IMAGE0: return "camera0/image";
  case OCCAM_IMAGE1: return "camera1/image";
  case OCCAM_IMAGE2: return "camera2/image";
  case OCCAM_IMAGE3: return "camera3/image";
  case OCCAM_IMAGE4: return "camera4/image";
  case OCCAM_IMAGE5: return "camera5/image";
  case OCCAM_IMAGE6: return "camera6/image";
  case OCCAM_IMAGE7: return "camera7/image";
  case OCCAM_IMAGE8: return "camera8/image";
  case OCCAM_IMAGE9: return "camera9/image";
  case OCCAM_RAW_IMAGE0: return "camera0/image_raw";
  case OCCAM_RAW_IMAGE1: return "camera1/image_raw";
  case OCCAM_RAW_IMAGE2: return "camera2/image_raw";
  case OCCAM_RAW_IMAGE3: return "camera3/image_raw";
  case OCCAM_RAW_IMAGE4: return "camera4/image_raw";
  case OCCAM_RAW_IMAGE5: return "camera5/image_raw";
  case OCCAM_RAW_IMAGE6: return "camera6/image_raw";
  case OCCAM_RAW_IMAGE7: return "camera7/image_raw";
  case OCCAM_RAW_IMAGE8: return "camera8/image_raw";
  case OCCAM_RAW_IMAGE9: return "camera9/image_raw";
  case OCCAM_IMAGE_TILES0: return "image_tiles";
  case OCCAM_IMAGE_TILES1: return "image_tiles_with_disparity";
  case OCCAM_RAW_IMAGE_TILES0: return "raw_image_tiles";
  case OCCAM_UNDISTORTED_IMAGE_TILES0: return "undistorted_image_tiles";
  case OCCAM_UNDISTORTED_IMAGE0: return "camera0/undistorted_image";
  case OCCAM_UNDISTORTED_IMAGE1: return "camera1/undistorted_image";
  case OCCAM_UNDISTORTED_IMAGE2: return "camera2/undistorted_image";
  case OCCAM_UNDISTORTED_IMAGE3: return "camera3/undistorted_image";
  case OCCAM_UNDISTORTED_IMAGE4: return "camera4/undistorted_image";
  case OCCAM_UNDISTORTED_IMAGE5: return "camera5/undistorted_image";
  case OCCAM_UNDISTORTED_IMAGE6: return "camera6/undistorted_image";
  case OCCAM_UNDISTORTED_IMAGE7: return "camera7/undistorted_image";
  case OCCAM_UNDISTORTED_IMAGE8: return "camera8/undistorted_image";
  case OCCAM_UNDISTORTED_IMAGE9: return "camera9/undistorted_image";
  case OCCAM_STITCHED_IMAGE0: return "stitched_image";
  case OCCAM_STITCHED_IMAGE1: return "stitched_image_with_disparity";
  case OCCAM_RECTIFIED_IMAGE0: return "camera0/image_rect";
  case OCCAM_RECTIFIED_IMAGE1: return "camera1/image_rect";
  case OCCAM_RECTIFIED_IMAGE2: return "camera2/image_rect";
  case OCCAM_RECTIFIED_IMAGE3: return "camera3/image_rect";
  case OCCAM_RECTIFIED_IMAGE4: return "camera4/image_rect";
  case OCCAM_RECTIFIED_IMAGE5: return "camera5/image_rect";
  case OCCAM_RECTIFIED_IMAGE6: return "camera6/image_rect";
  case OCCAM_RECTIFIED_IMAGE7: return "camera7/image_rect";
  case OCCAM_RECTIFIED_IMAGE8: return "camera8/image_rect";
  case OCCAM_RECTIFIED_IMAGE9: return "camera9/image_rect";
  case OCCAM_DISPARITY_IMAGE0: return "camera0/disparity_image";
  case OCCAM_DISPARITY_IMAGE1: return "camera2/disparity_image";
  case OCCAM_DISPARITY_IMAGE2: return "camera4/disparity_image";
  case OCCAM_DISPARITY_IMAGE3: return "camera6/disparity_image";
  case OCCAM_DISPARITY_IMAGE4: return "camera8/disparity_image";
  case OCCAM_TILED_DISPARITY_IMAGE: return "tiled_disparity_image";
  case OCCAM_STITCHED_DISPARITY_IMAGE: return "stitched_disparity_image";
  case OCCAM_POINT_CLOUD0: return "camera0/point_cloud";
  case OCCAM_POINT_CLOUD1: return "camera2/point_cloud";
  case OCCAM_POINT_CLOUD2: return "camera4/point_cloud";
  case OCCAM_POINT_CLOUD3: return "camera6/point_cloud";
  case OCCAM_POINT_CLOUD4: return "camera8/point_cloud";
  case OCCAM_STITCHED_POINT_CLOUD: return "stitched_point_cloud";
  }
  return std::string();
}

class Publisher {
  OccamDataName req;
public:
  Publisher(OccamDataName _req)
    : req(_req) {
  }
  Publisher(const Publisher& x) = delete;
  Publisher& operator= (const Publisher& rhs) = delete;
  OccamDataName dataName() const {
    return req;
  }
  virtual bool isRequested() = 0;
  virtual void publish(void* data) = 0;
};

class ImagePublisher : public Publisher {
  image_transport::Publisher pub;
  ros::Publisher info_pub;
  ros::NodeHandle nh;
  OccamDataName req;
  OccamDevice* device;
  bool is_color;
  bool relative_stamp;
  bool is_rect;
  std::atomic<int> subscribers;
  unsigned seq;
  int sid;
  std::map<int, int> camera_info_sid_map;
public:
  std::array<double, 5> D;
  std::array<double, 9> K;
  std::array<double, 9> R;
  std::array<double, 3> T;

  ImagePublisher(OccamDataName _req, image_transport::ImageTransport it, bool _is_color,
                 OccamDevice* dev, ros::NodeHandle nh)
    : Publisher(_req),
      req(_req),
      is_color(_is_color),
      seq(0),
      sid(-1),
      is_rect(false),
      nh("~") {

    std::string req_name = dataNameString(req);
    std::string rect_name = "/image_rect";
    std::string camera_name = "camera";
    std::size_t rect_in_pub = req_name.find(rect_name);
    std::size_t camera_in_pub = req_name.find(camera_name);
    relative_stamp = nh.param("relative_stamp", relative_stamp, false);
    ROS_INFO("advertising %s",req_name.c_str());
    pub = it.advertise(req_name, 1);
    if (camera_in_pub != std::string::npos) {
      sid = std::stoi(req_name.substr(camera_in_pub+camera_name.size(), 1));
    }

    if (rect_in_pub != std::string::npos) {
      is_rect = true;
      std::stringstream sout;
      sout<<req_name.substr(0, rect_in_pub)<<"/camera_info";
      info_pub = nh.advertise<sensor_msgs::CameraInfo>(sout.str(), 1, true);
      camera_info_sid_map[0] = 0;
      camera_info_sid_map[1] = 5;
      camera_info_sid_map[2] = 1;
      camera_info_sid_map[3] = 6;
      camera_info_sid_map[4] = 2;
      camera_info_sid_map[5] = 7;
      camera_info_sid_map[6] = 3;
      camera_info_sid_map[7] = 8;
      camera_info_sid_map[8] = 4;
      camera_info_sid_map[9] = 9;
    }
    K[0] = 0.0;
    device = dev;
  }
  sensor_msgs::CameraInfo getInfoMsg(const sensor_msgs::Image& img) {
    sensor_msgs::CameraInfo ci;

    double D[5], K[9], R[9], T[3];

    int binning_mode = OCCAM_BINNING_DISABLED;
    occamGetDeviceValuei(device, OCCAM_BINNING_MODE, &binning_mode);

    OCCAM_CHECK(occamGetDeviceValuerv
                (device, OccamParam(OCCAM_SENSOR_DISTORTION_COEFS0+sid), D, 5));
    OCCAM_CHECK(occamGetDeviceValuerv
                (device, OccamParam(OCCAM_SENSOR_INTRINSICS0+sid), K, 9));
    OCCAM_CHECK(occamGetDeviceValuerv
                (device, OccamParam(OCCAM_SENSOR_ROTATION0+sid), R, 9));
    OCCAM_CHECK(occamGetDeviceValuerv
                (device, OccamParam(OCCAM_SENSOR_TRANSLATION0+sid), T, 3));

    ci.header = img.header;
    ci.height = img.height;
    ci.width = img.width;


    ci.distortion_model = "plumb_bob";
    ci.D.assign(D,D+5);

    ci.K[0] = K[0];
    ci.K[1] = K[1];
    ci.K[2] = K[2];
    ci.K[3] = K[3];
    ci.K[4] = K[4];
    ci.K[5] = K[5];
    ci.K[6] = K[6];
    ci.K[7] = K[7];
    ci.K[8] = K[8];

    ci.R[0] = 1;
    ci.R[1] = 0;
    ci.R[2] = 0;
    ci.R[3] = 0;
    ci.R[4] = 1;
    ci.R[5] = 0;
    ci.R[6] = 0;
    ci.R[7] = 0;
    ci.R[8] = 1;

    ci.P[0] = K[0];
    ci.P[1] = K[1];
    ci.P[2] = K[2];
    ci.P[3] = 0;
    ci.P[4] = K[3];
    ci.P[5] = K[4];
    ci.P[6] = K[5];
    ci.P[7] = 0;
    ci.P[8] = K[6];
    ci.P[9] = K[7];
    ci.P[10] = K[8];
    ci.P[11] = 0;

    if (binning_mode == OCCAM_BINNING_2x2) {
      ci.binning_x = 2;
      ci.binning_y = 2;
    } else if (binning_mode == OCCAM_BINNING_4x4) {
      ci.binning_x = 4;
      ci.binning_y = 4;
    } else {
      ci.binning_x = 1;
      ci.binning_y = 1;
    }

    ci.roi.x_offset = 0;
    ci.roi.y_offset = 0;
    ci.roi.height = 0;
    ci.roi.width = 0;
    ci.roi.do_rectify = false;

    return ci;
  }
  virtual bool isRequested() {
    if (pub.getNumSubscribers()>0)
      ROS_INFO_THROTTLE(5,"subscribers of data %s = %i",dataNameString(req).c_str(),pub.getNumSubscribers());
    return pub.getNumSubscribers()>0;
  }
  virtual void publish(void* data) {
    OccamImage* img0 = (OccamImage*)data;
    if (!img0)
      return;

    const char* image_encoding = 0;
    int bpp = 1;
    switch (img0->format) {
    case OCCAM_GRAY8:
      bpp = 1;
      image_encoding = is_color ? "bayer_bggr8" : "mono8";
      break;
    case OCCAM_RGB24:
      bpp = 3;
      image_encoding = "rgb8";
      break;
    case OCCAM_SHORT1:
      bpp = 2;
      image_encoding = "16SC1";
      break;
    }

    ros::Time stamp;
    if (relative_stamp) stamp.fromNSec(img0->time_ns);
    else stamp = ros::Time::now();

    // Rect images are transposed for stereo proc. Undo transpose for msg
    int msg_width = is_rect ? img0->height : img0->width;
    int msg_height = is_rect ? img0->width : img0->height;

    sensor_msgs::Image img1;
    img1.header.seq = seq++;
    img1.header.frame_id = sid >= 0 ? "occam"+std::to_string(sid) : "occam";
    img1.header.stamp = stamp;
    img1.encoding = image_encoding;
    img1.height = msg_height;
    img1.width = msg_width;
    img1.step = msg_width*bpp;
    img1.data.resize(img1.height*img1.step);
    img1.is_bigendian = 0;
    const uint8_t* srcp = img0->data[0];
    int src_step = img0->step[0];
    uint8_t* dstp = &img1.data[0];
    int dst_step = img1.step;
    if (is_rect) {
      for (int i=0; i<img0->width; ++i) {
        srcp=img0->data[0]+i*bpp;
        for (int j=0; j<img0->height; ++j) {
          memcpy(dstp,srcp,bpp);
          dstp+=bpp;
          srcp+=src_step;
        }
      }
    }
    else {
      for (int j=0;j<img0->height;++j,dstp+=dst_step,srcp+=src_step) {
        memcpy(dstp,srcp,img0->width*bpp);
      }
    }

    pub.publish(img1);

    if (is_rect) {
      sensor_msgs::CameraInfo info1 = getInfoMsg(img1);
      info_pub.publish(info1);
    }
    occamFreeImage(img0);
  }
};

class PointCloudPublisher : public Publisher {
  OccamDataName req;
  ros::Publisher pub;
  bool relative_stamp;
  unsigned seq;
  int sid;
public:
  PointCloudPublisher(OccamDataName _req, ros::NodeHandle nh)
    : Publisher(_req),
      req(_req),
      sid(-1),
      seq(0) {

    std::string req_name = dataNameString(req);
    std::string camera_name = "camera";
    std::size_t camera_in_pub = req_name.find(camera_name);
    if (camera_in_pub != std::string::npos) {
      sid = std::stoi(req_name.substr(camera_in_pub+camera_name.size(), 1));
    }
    ROS_INFO("advertising %s",req_name.c_str());
    pub = nh.advertise<sensor_msgs::PointCloud2>(nh.resolveName(req_name), 1);
    relative_stamp = nh.param("relative_stamp", relative_stamp, false);
  }
  virtual bool isRequested() {
    if (pub.getNumSubscribers()>0)
      ROS_INFO_THROTTLE(5,"subscribers of data %s = %i",dataNameString(req).c_str(),pub.getNumSubscribers());
    return pub.getNumSubscribers()>0;
  }
  virtual void publish(void* data) {
    OccamPointCloud* pc0 = (OccamPointCloud*)data;
    if (!pc0)
      return;

    ROS_INFO_THROTTLE(1,"sending data %s...",dataNameString(req).c_str());

    ros::Time stamp;
    if (relative_stamp) stamp.fromNSec(pc0->time_ns);
    else stamp = ros::Time::now();

    sensor_msgs::PointCloud2 pc2;
    pc2.header.seq = seq++;
    pc2.header.frame_id = sid >= 0 ? "occam"+std::to_string(sid) : "occam";
    pc2.header.stamp = stamp;

    pc2.height = 1;
    pc2.width = pc0->point_count;

    unsigned point_step = 0;

    sensor_msgs::PointField& fx = *pc2.fields.insert(pc2.fields.end(), sensor_msgs::PointField());
    fx.name = "x";
    fx.offset = point_step;
    point_step += sizeof(float);
    fx.datatype = sensor_msgs::PointField::FLOAT32;
    fx.count = 1;

    sensor_msgs::PointField& fy = *pc2.fields.insert(pc2.fields.end(), sensor_msgs::PointField());
    fy.name = "y";
    fy.offset = point_step;
    point_step += sizeof(float);
    fy.datatype = sensor_msgs::PointField::FLOAT32;
    fy.count = 1;

    sensor_msgs::PointField& fz = *pc2.fields.insert(pc2.fields.end(), sensor_msgs::PointField());
    fz.name = "z";
    fz.offset = point_step;
    point_step += sizeof(float);
    fz.datatype = sensor_msgs::PointField::FLOAT32;
    fz.count = 1;

    if (pc0->rgb) {
      sensor_msgs::PointField& frgb = *pc2.fields.insert(pc2.fields.end(), sensor_msgs::PointField());
      frgb.name = "rgb";
      frgb.offset = point_step;
      point_step += sizeof(float);
      frgb.datatype = sensor_msgs::PointField::FLOAT32;
      frgb.count = 1;
    }

    pc2.is_bigendian = false;
    pc2.point_step = point_step;
    pc2.row_step = point_step * pc0->point_count;
    pc2.is_dense = true;

    for (int j=0,k=0;j<pc0->point_count;++j,k+=3) {
      float x = pc0->xyz[k+0] / 1000.f;
      float y = pc0->xyz[k+1] / 1000.f;
      float z = pc0->xyz[k+2] / 1000.f;

      pc2.data.insert(pc2.data.end(), (uint8_t*)&x,(uint8_t*)&x + sizeof(x));
      pc2.data.insert(pc2.data.end(), (uint8_t*)&y,(uint8_t*)&y + sizeof(y));
      pc2.data.insert(pc2.data.end(), (uint8_t*)&z,(uint8_t*)&z + sizeof(z));

      if (pc0->rgb) {
     	uint8_t r = pc0->rgb[k+0];
     	uint8_t g = pc0->rgb[k+1];
     	uint8_t b = pc0->rgb[k+2];
	uint32_t rgb = (uint32_t(r)<<16) | (uint32_t(g)<<8) | uint32_t(b);
	float rgbf = *(float*)&rgb;
	pc2.data.insert(pc2.data.end(), (uint8_t*)&rgbf,(uint8_t*)&rgbf + sizeof(rgbf));
      }
    }

    pub.publish(pc2);

    occamFreePointCloud(pc0);
  }
};

class OccamConfig {
  ros::NodeHandle nh;
  ros::ServiceServer set_service;
  ros::Publisher update_pub;
  ros::Publisher descr_pub;

  std::string cid;
  OccamDevice* device;
  OccamParamList* param_list;

  void initSensorConfig();
  void initImageProcessingConfig();
  void initBMConfig(int select_index);
  void initStereoConfig();
  void initCylindricalBlenderConfig(int select_index);
  void initBlendingConfig();
  void initAdvancedConfig();
  void initConfig();

  OccamParam findParamId(const std::string& name) {
    std::string::size_type p0 = name.find_first_of('-');
    std::string base_name;
    int module_index;
    if (p0 != std::string::npos) {
      base_name = std::string(name.begin(),name.begin()+p0);
      module_index = atoi(std::string(name.begin()+p0+1,name.end()).c_str());
    } else {
      base_name = name;
      module_index = 0;
    }

    for (int j=0;j<param_list->param_count;++j) {
      if (param_list->params[j].read_only)
	continue;
      if (param_list->params[j].name != base_name)
	continue;
      if (module_index && param_list->params[j].module_index != module_index)
	continue;
      return param_list->params[j].id;
    }

    return OccamParam(-1);
  }

  bool setConfigCallback(dynamic_reconfigure::Reconfigure::Request& req,
			 dynamic_reconfigure::Reconfigure::Response& rsp) {

    for (dynamic_reconfigure::IntParameter& param : req.config.ints) {
      OccamParam id = findParamId(param.name);
      OCCAM_CHECK(occamSetDeviceValuei(device, id, param.value));
    }
    for (dynamic_reconfigure::BoolParameter& param : req.config.bools) {
      OccamParam id = findParamId(param.name);
      OCCAM_CHECK(occamSetDeviceValuei(device, id, param.value?1:0));
    }

    generateConfig(rsp.config);
    publishConfigDescription();
    publishConfig();
    return true;
  }

  void publishConfigDescription() {
    dynamic_reconfigure::ConfigDescription msg;

    dynamic_reconfigure::Group& group = *msg.groups.emplace(msg.groups.end());
    group.name = "Default";
    group.type = "";
    group.parent = 0;
    group.id = 0;

    for (int j=0;j<param_list->param_count;++j) {
      OccamParamEntry& p0 = param_list->params[j];
      if (p0.read_only)
	continue;
      OccamParam id = OCCAM_MAKE_PARAM(p0.id,p0.module_index);

      if (p0.type == OCCAM_PARAM_INT) {
	{
	  dynamic_reconfigure::ParamDescription& desc = *group.parameters.emplace(group.parameters.end());
	  desc.name = p0.name;
	  desc.type = "int";
	  desc.level = 0;
	  desc.description = p0.name;
	  desc.edit_method = "";

	  int value_count = 0;
	  if (OCCAM_API_SUCCESS == occamGetDeviceValueCount(device, id, &value_count)) {
	    char** valuesv = (char**)alloca(sizeof(char*)*value_count);
	    memset(valuesv,0,sizeof(char*)*value_count);
	    int* valueiv = (int*)alloca(sizeof(int)*value_count);
	    if (OCCAM_API_SUCCESS == occamGetDeviceValuesv(device, id, valuesv, value_count) &&
		OCCAM_API_SUCCESS == occamGetDeviceValueiv(device, id, valueiv, value_count)) {
	      std::stringstream sout;
	      sout<<"{'enum_description': '"<<p0.name<<"', 'enum': [";
	      for (int j=0;j<value_count;++j) {
		if (j)
		  sout<<", ";
		sout<<"{'description': '"<<p0.name<<"', 'cconsttype': 'int', 'value': "<<valueiv[j]<<", 'ctype': 'int', 'type': 'int', 'name': '"<<valuesv[j]<<"'}";
	      }
	      sout<<"]}";
	      desc.edit_method = sout.str();
	    }
	    for (int j=0;j<value_count;++j)
	      occamFree(valuesv[j]);
	  }
	}

	{
	  dynamic_reconfigure::IntParameter& param = *msg.min.ints.emplace(msg.min.ints.end());
	  param.name = p0.name;
	  param.value = int(p0.min_value);
	}
	{
	  dynamic_reconfigure::IntParameter& param = *msg.max.ints.emplace(msg.max.ints.end());
	  param.name = p0.name;
	  param.value = int(p0.max_value);
	}
	{
	  dynamic_reconfigure::IntParameter& param = *msg.dflt.ints.emplace(msg.dflt.ints.end());
	  param.name = p0.name;
	  int value0 = 0;
	  OCCAM_CHECK(occamGetDeviceValuei(device, id, &value0));
	  param.value = value0;
	}
      }

      else if (p0.type == OCCAM_PARAM_BOOL) {
	{
	  dynamic_reconfigure::ParamDescription& desc = *group.parameters.emplace(group.parameters.end());
	  desc.name = p0.name;
	  desc.type = "bool";
	  desc.level = 0;
	  desc.description = p0.name;
	  desc.edit_method = "";
	}

	{
	  dynamic_reconfigure::BoolParameter& param = *msg.min.bools.emplace(msg.min.bools.end());
	  param.name = p0.name;
	  param.value = false;
	}
	{
	  dynamic_reconfigure::BoolParameter& param = *msg.max.bools.emplace(msg.max.bools.end());
	  param.name = p0.name;
	  param.value = true;
	}
	{
	  dynamic_reconfigure::BoolParameter& param = *msg.dflt.bools.emplace(msg.dflt.bools.end());
	  param.name = p0.name;
	  int value0 = 0;
	  OCCAM_CHECK(occamGetDeviceValuei(device, id, &value0));
	  param.value = bool(value0);
	}
      }

    }

    descr_pub.publish(msg);
  }
  void generateConfig(dynamic_reconfigure::Config& msg) {

    for (int j=0;j<param_list->param_count;++j) {
      OccamParamEntry& p0 = param_list->params[j];
      if (p0.read_only)
	continue;
      OccamParam id = OCCAM_MAKE_PARAM(p0.id,p0.module_index);

      if (p0.type == OCCAM_PARAM_INT) {
	dynamic_reconfigure::IntParameter& param = *msg.ints.emplace(msg.ints.end());
	param.name = p0.name;
	int value0 = 0;
	OCCAM_CHECK(occamGetDeviceValuei(device, id, &value0));
	param.value = value0;
      }

      else if (p0.type == OCCAM_PARAM_BOOL) {
	dynamic_reconfigure::BoolParameter& param = *msg.bools.emplace(msg.bools.end());
	param.name = p0.name;
	int value0 = 0;
	OCCAM_CHECK(occamGetDeviceValuei(device, id, &value0));
	param.value = bool(value0);
      }
    }
  }

  void publishConfig() {
    dynamic_reconfigure::Config msg;
    generateConfig(msg);

    update_pub.publish(msg);
  }

public:
  OccamConfig(ros::NodeHandle _nh,const std::string& cid, OccamDevice* _device)
    : nh(_nh),
      device(_device),
      param_list(0) {
    OCCAM_CHECK(occamEnumerateParamList(device, &param_list));

    set_service = nh.advertiseService("set_parameters",
				      &OccamConfig::setConfigCallback, this);
    descr_pub = nh.advertise<dynamic_reconfigure::ConfigDescription>("parameter_descriptions", 1, true);
    update_pub = nh.advertise<dynamic_reconfigure::Config>("parameter_updates", 1, true);

    publishConfigDescription();
    publishConfig();
  }
  virtual ~OccamConfig() {
    if (param_list)
      occamFreeParamList(param_list);
  }
};

class OccamNode {
public:
  ros::NodeHandle nh;

  std::string cid;
  OccamDevice* device;
  std::vector<std::shared_ptr<Publisher> > data_pubs;
  std::shared_ptr<OccamConfig> config;
  std::vector<ros::Publisher> camera_info_pubs;

  OccamNode() :
    nh("~"),
    device(0) {

    int r;

    OccamDeviceList* device_list;
    OCCAM_CHECK(occamEnumerateDeviceList(2000, &device_list));
    ROS_INFO("%i devices found", device_list->entry_count);
    int dev_index = 0;
    for (int i=0;i<device_list->entry_count;++i) {
      if (!cid.empty() && device_list->entries[i].cid == cid) {
	dev_index = i;
      }
      ROS_INFO("device[%i]: cid = %s",i,device_list->entries[i].cid);
    }
    if (dev_index<0 || dev_index >= device_list->entry_count)
      return;
    if (!cid.empty() && device_list->entries[dev_index].cid != cid) {
      ROS_INFO("Requested cid %s not found",cid.c_str());
      return;
    }

    OCCAM_CHECK(occamOpenDevice(device_list->entries[dev_index].cid, &device));
    OCCAM_CHECK(occamFreeDeviceList(device_list));

    int is_color = 0;
    OCCAM_CHECK(occamGetDeviceValuei(device, OCCAM_COLOR, &is_color));

    image_transport::ImageTransport it(nh);

    int req_count;
    OccamDataName* req;
    OccamDataType* types;
    OCCAM_CHECK(occamDeviceAvailableData(device, &req_count, &req, &types));
    for (int j=0;j<req_count;++j) {
      if (types[j] == OCCAM_IMAGE)
	data_pubs.push_back(std::make_shared<ImagePublisher>(req[j],it,is_color,device,nh));
      else if (types[j] == OCCAM_POINT_CLOUD)
	data_pubs.push_back(std::make_shared<PointCloudPublisher>(req[j],nh));
    }
    occamFree(req);
    occamFree(types);

    config = std::make_shared<OccamConfig>(nh,cid,device);
  }

  virtual ~OccamNode() {
    if (device)
      occamCloseDevice(device);
  }

  OccamNode(const OccamNode& x) = delete;
  OccamNode& operator= (const OccamNode& rhs) = delete;

  bool take_and_send_data() {
    int r;

    std::vector<OccamDataName> reqs;
    std::vector<std::shared_ptr<Publisher> > reqs_pubs;
    reqs.reserve(data_pubs.size());
    reqs_pubs.reserve(data_pubs.size());
    for (std::shared_ptr<Publisher> pub : data_pubs)
      if (pub->isRequested()) {
	reqs.push_back(pub->dataName());
	reqs_pubs.push_back(pub);
      }

    std::vector<OccamDataType> types;
    std::vector<void*> data;
    types.resize(reqs.size());
    data.resize(reqs.size());
    r = occamDeviceReadData(device, reqs.size(), &reqs[0], &types[0], &data[0], 0);
    if (r != OCCAM_API_SUCCESS && r != OCCAM_API_DATA_NOT_AVAILABLE) {
      char error_str[256] = {0};
      occamGetErrorString((OccamError)r, error_str, sizeof(error_str));
      ROS_ERROR_THROTTLE(10,"Driver returned error %s (%i)",error_str,r);
      return false;
    }
    if (r != OCCAM_API_SUCCESS)
      return false;

    for (int j=0;j<reqs.size();++j)
      reqs_pubs[j]->publish(data[j]);

    return true;
  }

  bool spin() {
    if (!device)
      return false;

    while (nh.ok()) {
      if (!take_and_send_data())
        usleep(1000);
      ros::spinOnce();
    }
    return true;
  }

};

int main(int argc, char **argv) {
  OCCAM_CHECK(occamInitialize());
  ros::init(argc, argv, "occam");
  OccamNode a;
  a.spin();
  exit(0);
  return 0;
}
