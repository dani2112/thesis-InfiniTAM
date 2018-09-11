#include "RosEngine.h"
#include <string>

namespace InputSource {

  // Constructor and destructor
  RosEngine::RosEngine(ros::NodeHandle& nh, const char*& calibration_filename)
    : rgb_ready_(false),
    depth_ready_(false),
    rgb_info_ready_(false),
    depth_info_ready_(false),
    data_available_(true) {

    nh.param<std::string>("rgb_camera_info_topic", rgb_camera_info_topic_, "/camera/rgb/camera_info");
    nh.param<std::string>("depth_camera_info_topic", depth_camera_info_topic_, "/camera/depth/camera_info");

    ros::Subscriber rgb_info_sub;
    ros::Subscriber depth_info_sub;

    depth_info_sub = nh.subscribe(depth_camera_info_topic_, 1,
                              &RosEngine::depthCameraInfoCallback,
                              static_cast<RosEngine*>(this));
    rgb_info_sub = nh.subscribe(rgb_camera_info_topic_, 1,
                              &RosEngine::rgbCameraInfoCallback,
                              static_cast<RosEngine*>(this));



  }

  RosEngine::~RosEngine() {
  }

  // Callbacks for getting new data from ROS
  void RosEngine::rgbCallback(const sensor_msgs::Image::ConstPtr& msg) {

  }

  void RosEngine::depthCallback(const sensor_msgs::Image::ConstPtr& msg) {

  }

  void RosEngine::rgbCameraInfoCallback(const sensor_msgs::CameraInfo::ConstPtr& msg) {

  }

  void RosEngine::depthCameraInfoCallback(const sensor_msgs::CameraInfo::ConstPtr& msg) {

  }


  // InputSourceEngine related
  ITMLib::ITMRGBDCalib RosEngine::getCalib(void) const {
    return ITMLib::ITMRGBDCalib();
  }


  void RosEngine::getImages(ITMUChar4Image* rgb_image, ITMShortImage* raw_depth_image) {


  }

  bool RosEngine::hasMoreImages(void) const {
  return NULL;
  }

  Vector2i RosEngine::getDepthImageSize(void) const {
  return Vector2i(0,0);
  }

  Vector2i RosEngine::getRGBImageSize(void) const {
  return Vector2i(0,0);
  }


}
