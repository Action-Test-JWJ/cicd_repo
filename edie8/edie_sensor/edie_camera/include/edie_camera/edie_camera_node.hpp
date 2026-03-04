#ifndef EDIE_CAMERA_NODE_HPP
#define EDIE_CAMERA_NODE_HPP

#include <chrono>
#include <functional>
#include <memory>
#include <sstream>
#include <vector>
#include <string>

// For V4L2 and system calls
#include <linux/videodev2.h>
#include <poll.h>

#include <yaml-cpp/yaml.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <image_transport/image_transport.hpp>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"


using namespace std::chrono_literals;

// Helper struct to manage V4L2 mmap buffers.
struct MMAPBuffer {
    void*  start{nullptr};
    size_t length{0};
};

class EdieCameraNode : public rclcpp::Node
{
public:
  EdieCameraNode();
  ~EdieCameraNode();
  void ProcessRun();

private:
  // --- V4L2 MMAP capture members ---
  int                      v4l2_fd_{-1};
  std::vector<MMAPBuffer>  v4l2_buffers_;
  bool                     streaming_{false};

  // V4L2 methods
  bool OpenV4L2(const std::string & dev);
  bool SetupFormatV4L2(int width, int height);
  bool SetExposureV4L2(int exposure);
  bool SetupBuffersV4L2(unsigned int count = 4);
  bool StartStreamV4L2();
  bool StopStreamV4L2();
  void CloseV4L2();
  bool DequeueFrameV4L2(struct v4l2_buffer & buf);
  bool QueueFrameV4L2(const struct v4l2_buffer & buf);

  // Other members
  bool initialized_{false};
  int width_;
  int height_;
  bool visualize_;
  bool use_undistort_;
  std::string camera_model_;
  std::string distortion_model_;
  std::string camera_name_;
  int exposure_;
  int frame_index_;

  std::string device_;
  std::vector<double> camera_matrix_;
  std::vector<double> distortion_coefficients_;
  std::vector<double> rectification_matrix_;
  std::vector<double> projection_matrix_;

  cv::Mat undist_map1_, undist_map2_;

  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr edie_camera_pub;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr edie_camera_mono_pub;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr edie_camera_undistorted_pub;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr edie_camera_info_pub;
  rclcpp::Time last_frame_time_;

  // Time synchronization members
  rclcpp::Clock sys_clock_{RCL_SYSTEM_TIME};
  std::chrono::nanoseconds steady_to_system_offset_{};

  // Time sync methods
  void CalibrateTimeOffset();
  rclcpp::Time ToSystemRosFromSteadyTP(std::chrono::steady_clock::time_point tp);

  // Config and undistort
  bool LoadConfigFromFile(const std::string & file_path);
  bool InitUndistortMap();
  std::vector<double> parseDoubleVector(const YAML::Node & node);
  std::string vectorToString(const std::vector<double> & vec);
};

#endif // EDIE_CAMERA_NODE_HPP
