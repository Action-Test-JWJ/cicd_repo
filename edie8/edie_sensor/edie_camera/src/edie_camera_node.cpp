#include "edie_camera/edie_camera_node.hpp"
#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <unistd.h>
#include <cerrno>

EdieCameraNode::EdieCameraNode()
  : Node("edie_camera_pub"),
    frame_index_(0)
{
    rclcpp::QoS qos(rclcpp::KeepLast(5)); //rclcpp::QoS qos(rclcpp::KeepLast(15));
    // qos.best_effort();
    qos.reliable();

    edie_camera_pub = this->create_publisher<sensor_msgs::msg::Image>("/edie8/vision/image_raw", qos);
    edie_camera_mono_pub = this->create_publisher<sensor_msgs::msg::Image>("/edie8/vision/image_raw_mono", qos);
    edie_camera_undistorted_pub = this->create_publisher<sensor_msgs::msg::Image>("/edie8/vision/image_undistorted", qos);
    edie_camera_info_pub = this->create_publisher<sensor_msgs::msg::CameraInfo>("/edie8/vision/camera_info", qos); //10);

    // Declare parameters
    this->declare_parameter<int>("width", 0);
    this->declare_parameter<int>("height", 0);
    this->declare_parameter<bool>("visualize", false);
    this->declare_parameter<std::string>("camera_name", "");
    this->declare_parameter<std::string>("device", "");
    this->declare_parameter<int>("exposure", 0);
    this->declare_parameter<bool>("use_undistort", false);
    this->declare_parameter<std::string>("calibration_file", "");

    this->declare_parameter<std::string>("camera_model", "");
    this->declare_parameter<std::string>("distortion_model", "");
    this->declare_parameter<std::vector<double>>("camera_matrix", std::vector<double>());
    this->declare_parameter<std::vector<double>>("distortion_coefficients", std::vector<double>());
    this->declare_parameter<std::vector<double>>("rectification_matrix", std::vector<double>());
    this->declare_parameter<std::vector<double>>("projection_matrix", std::vector<double>());

    // Load configuration
    std::string pkg_share_dir = ament_index_cpp::get_package_share_directory("edie_camera");
    std::string config_file = pkg_share_dir + "/config/config.yaml";
    if (!LoadConfigFromFile(config_file)) {
        RCLCPP_FATAL(this->get_logger(), "Failed to load configuration. Node will not start.");
        return;
    }

    InitUndistortMap();

    // === V4L2 Initialization ===
    if (!OpenV4L2(device_)) {
      RCLCPP_FATAL(this->get_logger(), "Failed to open V4L2 device %s", device_.c_str());
      return;
    }
    if (!SetupFormatV4L2(width_, height_)) {
      RCLCPP_FATAL(this->get_logger(), "Failed to set V4L2 format");
      CloseV4L2();
      return;
    }
    if (!SetExposureV4L2(exposure_)) 
    {
      RCLCPP_WARN(this->get_logger(), "Failed to set exposure.");
      // Not a fatal error, continue
    }
    if (!SetupBuffersV4L2(4)) {
      RCLCPP_FATAL(this->get_logger(), "Failed to setup V4L2 buffers");
      CloseV4L2();
      return;
    }
    if (!StartStreamV4L2()) {
      RCLCPP_FATAL(this->get_logger(), "Failed to start V4L2 stream");
      CloseV4L2();
      return;
    }

    CalibrateTimeOffset();

     RCLCPP_INFO_STREAM(this->get_logger(),
      "\n[DEBUG] Loaded Parameters:" << "\n"
      << " width: " << width_ << "\n"
      << " height: " << height_ << "\n"
      << " visualize: " << (visualize_ ? "true" : "false") << "\n"
      << " camera_name: " << camera_name_ << "\n"
      << " device: " << device_ << "\n"
      << " exposure: " << exposure_ << "\n"
      << " use_undistort: " << (use_undistort_ ? "true" : "false") << "\n"
      << " camera_model: " << camera_model_ << "\n"
      << " distortion_model: " << distortion_model_ << "\n"
      << " camera_matrix: " << vectorToString(camera_matrix_) << "\n"
      << " distortion_coefficients: " << vectorToString(distortion_coefficients_) << "\n"
      << " rectification_matrix: " << vectorToString(rectification_matrix_) << "\n"
      << " projection_matrix: " << vectorToString(projection_matrix_) << "\n"
    );
    RCLCPP_INFO(this->get_logger(), "Camera node initialized successfully.");
    initialized_ = true;
}

EdieCameraNode::~EdieCameraNode()
{
    CloseV4L2();
    cv::destroyAllWindows();
}

// 파라미터 값들을 파일에서 불러오기
bool EdieCameraNode::LoadConfigFromFile(const std::string &file_path)
{
  try {
    YAML::Node base = YAML::LoadFile(file_path);
    YAML::Node config = base["edie_camera_pub"]["ros__parameters"];
    if (!config) 
    {
        RCLCPP_ERROR(this->get_logger(), "Invalid structure in %s.", file_path.c_str());
        return false;
    }
    width_ = config["image_width"].as<int>();
    height_ = config["image_height"].as<int>();
    visualize_ = config["visualize"].as<bool>();
    device_ = config["device"].as<std::string>();
    exposure_ = config["exposure"].as<int>();
    use_undistort_ = config["use_undistort"].as<bool>();

    std::string calib_filename = config["calibration_file"].as<std::string>();
    if (calib_filename.empty()) 
    {
        use_undistort_ = false;
    } 
    else 
    {
      std::string pkg_share_dir = ament_index_cpp::get_package_share_directory("edie_camera");
      std::string calib_filepath = pkg_share_dir + "/config/" + calib_filename;
      YAML::Node calib_base = YAML::LoadFile(calib_filepath);
      YAML::Node calib_config = calib_base["edie_camera_pub"]["ros__parameters"];
      camera_model_ = calib_config["camera_model"].as<std::string>();
      distortion_model_ = calib_config["distortion_model"].as<std::string>();
      camera_matrix_ = parseDoubleVector(calib_config["camera_matrix"]["data"]);
      distortion_coefficients_ = parseDoubleVector(calib_config["distortion_coefficients"]["data"]);
      rectification_matrix_ = parseDoubleVector(calib_config["rectification_matrix"]["data"]);
      projection_matrix_ = parseDoubleVector(calib_config["projection_matrix"]["data"]);
    }
    return true;
  }
  catch (const std::exception &e) 
  {
    RCLCPP_ERROR(this->get_logger(), "Error loading config file: %s", e.what());
    return false;
  }
}

bool EdieCameraNode::InitUndistortMap()
{
  if (!use_undistort_ || camera_matrix_.size() != 9 || width_ == 0 || height_ == 0) {
    use_undistort_ = false;
    return false;
  }
  cv::Mat K(3, 3, CV_64F, camera_matrix_.data());
  cv::Mat D(distortion_coefficients_);
  cv::Size image_size(width_, height_);
  if (camera_model_ == "pinhole") {
    cv::Mat new_K = cv::getOptimalNewCameraMatrix(K, D, image_size, 0.0, image_size);
    cv::initUndistortRectifyMap(K, D, cv::Mat(), new_K, image_size, CV_16SC2, undist_map1_, undist_map2_);
  } else if (camera_model_ == "equidistant" || camera_model_ == "fisheye" || camera_model_ == "kannala_brandt") {
    cv::Mat new_K = K.clone();
    cv::fisheye::estimateNewCameraMatrixForUndistortRectify(K, D, image_size, cv::Mat::eye(3,3,CV_64F), new_K, 0.0, image_size);
    cv::fisheye::initUndistortRectifyMap(K, D, cv::Mat::eye(3,3,CV_64F), new_K, image_size, CV_16SC2, undist_map1_, undist_map2_);
  } else {
    use_undistort_ = false;
    return false;
  }
  return true;
}

std::vector<double> EdieCameraNode::parseDoubleVector(const YAML::Node &node) {
  std::vector<double> vec;
  if (node && node.IsSequence()) {
    for (const auto& item : node) {
      vec.push_back(item.as<double>());
    }
  }
  return vec;
}

std::string EdieCameraNode::vectorToString(const std::vector<double> &vec) {
  std::ostringstream oss;
  oss << "[";
  for (size_t i = 0; i < vec.size(); ++i) {
    oss << vec[i] << (i == vec.size() - 1 ? "" : ", ");
  }
  oss << "]";
  return oss.str();
}


void EdieCameraNode::CalibrateTimeOffset()
{
  using namespace std::chrono;
  struct timespec mono_time, real_time;
  clock_gettime(CLOCK_MONOTONIC, &mono_time);
  clock_gettime(CLOCK_REALTIME, &real_time);
  auto mono_ns = seconds(mono_time.tv_sec) + nanoseconds(mono_time.tv_nsec);
  auto real_ns = seconds(real_time.tv_sec) + nanoseconds(real_time.tv_nsec);
  steady_to_system_offset_ = real_ns - mono_ns;
}

rclcpp::Time EdieCameraNode::ToSystemRosFromSteadyTP(std::chrono::steady_clock::time_point st_tp)
{
    using namespace std::chrono;
    auto st_ns  = duration_cast<nanoseconds>(st_tp.time_since_epoch());
    auto sys_ns = st_ns + steady_to_system_offset_;
    return rclcpp::Time(sys_ns.count());
}


bool EdieCameraNode::OpenV4L2(const std::string & dev)
{
  v4l2_fd_ = ::open(dev.c_str(), O_RDWR | O_NONBLOCK, 0);
  if (v4l2_fd_ < 0) {
    RCLCPP_ERROR(this->get_logger(), "open(%s) failed: %s", dev.c_str(), strerror(errno));
    return false;
  }
  v4l2_capability cap{};
  if (ioctl(v4l2_fd_, VIDIOC_QUERYCAP, &cap) < 0) {
    RCLCPP_ERROR(this->get_logger(), "VIDIOC_QUERYCAP failed: %s", strerror(errno));
    return false;
  }
  return true;
}

bool EdieCameraNode::SetupFormatV4L2(int width, int height)
{
  v4l2_format fmt{};
  fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  fmt.fmt.pix.width = width;
  fmt.fmt.pix.height = height;
  fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_MJPEG;
  fmt.fmt.pix.field = V4L2_FIELD_NONE;
  if (ioctl(v4l2_fd_, VIDIOC_S_FMT, &fmt) < 0) {
    RCLCPP_ERROR(this->get_logger(), "VIDIOC_S_FMT MJPG failed: %s", strerror(errno));
    return false;
  }
  width_ = fmt.fmt.pix.width;
  height_ = fmt.fmt.pix.height;
  return true;
}

bool EdieCameraNode::SetExposureV4L2(int exposure)
{
  if (v4l2_fd_ < 0) {
    return false;
  }

  struct v4l2_control ctrl;

  if (exposure > 0)
  {
    // Manual exposure
    ctrl.id = V4L2_CID_EXPOSURE_AUTO;
    ctrl.value = V4L2_EXPOSURE_MANUAL;
    if (ioctl(v4l2_fd_, VIDIOC_S_CTRL, &ctrl) == 0) 
    {
      RCLCPP_INFO(this->get_logger(), "Set exposure mode to MANUAL.");
    } 
    else 
    {
      RCLCPP_WARN(this->get_logger(), "Failed to set exposure mode to MANUAL: %s", strerror(errno));
    }

    ctrl.id = V4L2_CID_EXPOSURE_ABSOLUTE;
    ctrl.value = exposure;
    if (ioctl(v4l2_fd_, VIDIOC_S_CTRL, &ctrl) == 0) 
    {
      RCLCPP_INFO(this->get_logger(), "Set exposure value to %d.", exposure);
    } 
    else 
    {
      RCLCPP_ERROR(this->get_logger(), "Failed to set exposure value: %s", strerror(errno));
      return false;
    }
  } 
  else 
  {
    // Auto exposure
    ctrl.id = V4L2_CID_EXPOSURE_AUTO;
    ctrl.value = V4L2_EXPOSURE_AUTO;
    if (ioctl(v4l2_fd_, VIDIOC_S_CTRL, &ctrl) == 0) 
    {
      RCLCPP_INFO(this->get_logger(), "Set exposure mode to AUTO.");
    } 
    else 
    {
      RCLCPP_WARN(this->get_logger(), "Failed to set exposure mode to AUTO: %s", strerror(errno));
    }
  }

  // Verify current settings
  ctrl.id = V4L2_CID_EXPOSURE_AUTO;
  if (ioctl(v4l2_fd_, VIDIOC_G_CTRL, &ctrl) == 0) 
  {
    RCLCPP_INFO(this->get_logger(), "Current exposure mode: %s", (ctrl.value == V4L2_EXPOSURE_MANUAL) ? "MANUAL" : "AUTO");
    if (ctrl.value == V4L2_EXPOSURE_MANUAL) 
    {
      ctrl.id = V4L2_CID_EXPOSURE_ABSOLUTE;
      if (ioctl(v4l2_fd_, VIDIOC_G_CTRL, &ctrl) == 0) 
      {
        RCLCPP_INFO(this->get_logger(), "Current exposure value: %d", ctrl.value);
      }
    }
  }

  return true;
}


bool EdieCameraNode::SetupBuffersV4L2(unsigned int count)
{
  v4l2_requestbuffers req{};
  req.count = count;
  req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  req.memory = V4L2_MEMORY_MMAP;
  if (ioctl(v4l2_fd_, VIDIOC_REQBUFS, &req) < 0) {
    RCLCPP_ERROR(this->get_logger(), "VIDIOC_REQBUFS failed: %s", strerror(errno));
    return false;
  }
  v4l2_buffers_.resize(req.count);
  for (unsigned int i = 0; i < req.count; ++i) {
    v4l2_buffer buf{};
    buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf.memory = V4L2_MEMORY_MMAP;
    buf.index = i;
    if (ioctl(v4l2_fd_, VIDIOC_QUERYBUF, &buf) < 0) return false;
    v4l2_buffers_[i].length = buf.length;
    v4l2_buffers_[i].start = mmap(NULL, buf.length, PROT_READ | PROT_WRITE, MAP_SHARED, v4l2_fd_, buf.m.offset);
    if (v4l2_buffers_[i].start == MAP_FAILED) return false;
    if (ioctl(v4l2_fd_, VIDIOC_QBUF, &buf) < 0) return false;
  }
  return true;
}

bool EdieCameraNode::StartStreamV4L2() 
{
  int type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  if (ioctl(v4l2_fd_, VIDIOC_STREAMON, &type) < 0) {
    RCLCPP_ERROR(this->get_logger(), "VIDIOC_STREAMON failed: %s", strerror(errno));
    return false;
  }
  streaming_ = true;
  return true;
}

bool EdieCameraNode::StopStreamV4L2() 
{
  if (!streaming_) return true;
  enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  if (ioctl(v4l2_fd_, VIDIOC_STREAMOFF, &type) < 0) {
    return false;
  }
  streaming_ = false;
  return true;
}

void EdieCameraNode::CloseV4L2() 
{
  if (streaming_) {
    StopStreamV4L2();
  }
  if (v4l2_fd_ >= 0) {
    for (auto & b : v4l2_buffers_) {
      if (b.start && b.start != MAP_FAILED) {
        munmap(b.start, b.length);
      }
    }
    v4l2_buffers_.clear();
    ::close(v4l2_fd_);
    v4l2_fd_ = -1;
  }
}

bool EdieCameraNode::DequeueFrameV4L2(struct v4l2_buffer & buf) 
{
  memset(&buf, 0, sizeof(buf));
  buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  buf.memory = V4L2_MEMORY_MMAP;

  struct pollfd pfd;
  pfd.fd = v4l2_fd_;
  pfd.events = POLLIN;
  pfd.revents = 0;
  int ret = poll(&pfd, 1, 2000);
  if (ret < 0) 
  {
    return false;
  }
  if (ret == 0) 
  {
    return false;
  }
  if (ioctl(v4l2_fd_, VIDIOC_DQBUF, &buf) < 0) 
  {
    return false;
  }

  return true;
}

bool EdieCameraNode::QueueFrameV4L2(const struct v4l2_buffer & buf) 
{
  if (ioctl(v4l2_fd_, VIDIOC_QBUF, &buf) < 0) {
    return false;
  }
  return true;
}

void EdieCameraNode::ProcessRun()
{
    if (!initialized_) {
        RCLCPP_ERROR_ONCE(this->get_logger(), "Node not initialized.");
        return;
    }
    RCLCPP_INFO(this->get_logger(), "Starting V4L2 capture loop");
    while (rclcpp::ok()) {
      struct v4l2_buffer buf{};
      if (!DequeueFrameV4L2(buf)) {
        rclcpp::spin_some(this->get_node_base_interface());
        continue;
      }

      auto tv = buf.timestamp;
      auto mono_ns = std::chrono::seconds(tv.tv_sec) + std::chrono::microseconds(tv.tv_usec);
      auto sys_ns = mono_ns + steady_to_system_offset_;
      rclcpp::Time stamp(sys_ns.count());

      uint8_t* data_start = static_cast<uint8_t*>(v4l2_buffers_[buf.index].start);
      std::vector<uint8_t> jpeg_data(data_start, data_start + buf.bytesused);
      cv::Mat raw_frame = cv::imdecode(jpeg_data, cv::IMREAD_COLOR);
      if (raw_frame.empty()) {
          QueueFrameV4L2(buf);
          continue;
      }
      cv::Mat gray_frame;
      cv::cvtColor(raw_frame, gray_frame, cv::COLOR_BGR2GRAY);

      cv::Mat undistored_frame;
      if (use_undistort_) 
      {
        cv::remap(gray_frame, undistored_frame, undist_map1_, undist_map2_, cv::INTER_LINEAR);
        auto undist_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "mono8", undistored_frame).toImageMsg();
        undist_msg->header.stamp    = stamp;
        undist_msg->header.frame_id = "edie_camera_optical_frame";
        edie_camera_undistorted_pub->publish(*undist_msg);
      }

      auto img_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", raw_frame).toImageMsg();
      img_msg->header.stamp = stamp;
      img_msg->header.frame_id = "edie_camera_optical_frame";
      edie_camera_pub->publish(*img_msg);
      
      auto mono_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "mono8", gray_frame).toImageMsg();
      mono_msg->header.stamp    = stamp;
      mono_msg->header.frame_id = "edie_camera_optical_frame";
      edie_camera_mono_pub->publish(*mono_msg);

      sensor_msgs::msg::CameraInfo cam_info;
      cam_info.header = img_msg->header;
      cam_info.width = width_;
      cam_info.height = height_;
      if (!camera_matrix_.empty()) std::copy_n(camera_matrix_.begin(), 9, cam_info.k.begin());
      if (!rectification_matrix_.empty()) std::copy_n(rectification_matrix_.begin(), 9, cam_info.r.begin());
      if (!projection_matrix_.empty()) std::copy_n(projection_matrix_.begin(), 12, cam_info.p.begin());
      if (!distortion_coefficients_.empty()) cam_info.d = distortion_coefficients_;
      edie_camera_info_pub->publish(cam_info);

      if (visualize_) {
          cv::imshow("raw Frame", raw_frame);
          cv::waitKey(1);
          if(use_undistort_)
          {
            cv::imshow("undistored Frame", undistored_frame);
            cv::waitKey(1);
          }
      }

      QueueFrameV4L2(buf);
    }
    CloseV4L2();
}

