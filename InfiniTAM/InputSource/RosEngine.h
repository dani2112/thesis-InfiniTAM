#pragma once

#include <mutex>
#include <string>

#include "ImageSourceEngine.h"
#include "ros/ros.h"
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/image_encodings.h>

namespace InputSource {
  class RosEngine : public InputSource::ImageSourceEngine {

  public:
    // Constructor and destructor
    RosEngine(ros::NodeHandle& nh, const char*& calibration_filename);
    ~RosEngine();

    // Callbacks for getting new data from ROS
    void rgbCallback(const sensor_msgs::Image::ConstPtr& msg);
    void rgbCameraInfoCallback(const sensor_msgs::CameraInfo::ConstPtr& msg);
    void depthCallback(const sensor_msgs::Image::ConstPtr& msg);
    void depthCameraInfoCallback(const sensor_msgs::CameraInfo::ConstPtr& msg);

    // InputSourceEngine related
    ITMLib::ITMRGBDCalib getCalib(void) const;
    bool hasMoreImages(void) const;
    void getImages(ITMUChar4Image* rgb, ITMShortImage* raw_depth);
    Vector2i getDepthImageSize(void) const;
    Vector2i getRGBImageSize(void) const;


  private:
    //! True if the RGB data is available.
    bool rgb_ready_;
    //! True if the Depth data is available.
    bool depth_ready_;
    //! True if the Depth info is available.
    bool rgb_info_ready_;
    //! True if the Depth info is available.
    bool depth_info_ready_;

    bool data_available_;
    //! ROS topic name for the incoming rgb messages.
    std::string rgb_camera_info_topic_;
    //! ROS Topic name for the incoming depth messages.
    std::string depth_camera_info_topic_;
    std::mutex rgb_mutex_;
    std::mutex depth_mutex_;
    Vector2i image_size_rgb_, image_size_depth_;
    sensor_msgs::CameraInfo rgb_info_;
    sensor_msgs::CameraInfo depth_info_;
    ros::Time depth_msg_time_stamp_;
  };
}
