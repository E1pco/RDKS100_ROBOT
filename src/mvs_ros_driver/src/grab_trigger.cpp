/**
 * MVS (Hikvision) camera driver for ROS2 Humble.
 *
 * Reads camera parameters from an OpenCV FileStorage YAML file,
 * grabs frames in a worker thread, and publishes via image_transport.
 * Supports hardware trigger mode with shared-memory timestamp injection.
 *
 * ROS1 → ROS2 port: catkin/rscpp → ament_cmake/rclcpp.
 */

#include "MvCameraControl.h"

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/header.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.hpp>

#include <opencv2/opencv.hpp>

#include <chrono>
#include <cstring>
#include <fstream>
#include <iostream>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <fcntl.h>
#include <signal.h>
#include <sys/ipc.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>

// =========================================================================
// Shared-memory timestamp (injected by external trigger hardware)
// =========================================================================
struct TimeStamp {
  int64_t high;
  int64_t low;
};

// =========================================================================
// Pixel format enum used in the YAML config
// =========================================================================
enum PixelFormat : unsigned int {
  RGB8            = 0x02180014,
  BayerRG8        = 0x01080009,
  BayerRG12Packed = 0x010C002B,
  BayerGB12Packed = 0x010C002C,
  BayerGB8        = 0x0108000A
};

static const std::vector<PixelFormat> PIXEL_FORMATS = {
  RGB8, BayerRG8, BayerRG12Packed, BayerGB12Packed, BayerGB8
};

static const char *EXPOSURE_AUTO_STR[3] = {"Off", "Once", "Continues"};
static const char *GAMMA_SELECTOR_STR[3] = {"User", "sRGB", "Off"};
static const char *GAIN_AUTO_STR[3] = {"Off", "Once", "Continues"};

// =========================================================================
// Helper: print device info
// =========================================================================
static bool PrintDeviceInfo(const MV_CC_DEVICE_INFO *info) {
  if (!info) {
    printf("Device info pointer is NULL!\n");
    return false;
  }
  if (info->nTLayerType == MV_GIGE_DEVICE) {
    const auto &g = info->SpecialInfo.stGigEInfo;
    int ip = g.nCurrentIp;
    printf("  Model : %s\n", g.chModelName);
    printf("  IP    : %d.%d.%d.%d\n",
           (ip >> 24) & 0xFF, (ip >> 16) & 0xFF,
           (ip >> 8) & 0xFF, ip & 0xFF);
    printf("  Serial: %s\n", g.chSerialNumber);
  } else if (info->nTLayerType == MV_USB_DEVICE) {
    const auto &u = info->SpecialInfo.stUsb3VInfo;
    printf("  Model : %s\n", u.chModelName);
    printf("  Serial: %s\n", u.chSerialNumber);
  } else {
    printf("  Unknown transport layer.\n");
  }
  return true;
}

// =========================================================================
// MvsCameraNode — wraps all ROS2 + camera logic in one class
// =========================================================================
class MvsCameraNode : public rclcpp::Node {
public:
  explicit MvsCameraNode(const rclcpp::NodeOptions &options)
      : Node("mvs_camera", options), handle_(nullptr), running_(true) {}

  ~MvsCameraNode() override { cleanup(); }

  /** Parse the YAML config file (OpenCV FileStorage format) and store values. */
  bool loadConfig(const std::string &path) {
    config_path_ = path;
    cv::FileStorage fs(path, cv::FileStorage::READ);
    if (!fs.isOpened()) {
      RCLCPP_ERROR(get_logger(), "Failed to open config file: %s", path.c_str());
      return false;
    }
    fs["TriggerEnable"]   >> trigger_enable_;
    fs["SerialNumber"]    >> serial_number_;
    fs["TopicName"]       >> topic_name_;
    fs["PixelFormat"]     >> pixel_format_idx_;
    fs["ExposureAutoMode"]>> exposure_auto_mode_;
    fs["ExposureTime"]    >> exposure_time_;
    fs["GainAuto"]        >> gain_auto_;
    fs["Gain"]            >> gain_;
    fs["Gamma"]           >> gamma_;
    fs["GammaSelector"]   >> gamma_selector_;
    fs["image_scale"]     >> image_scale_;
    fs.release();

    if (image_scale_ < 0.1f) image_scale_ = 1.0f;
    if (topic_name_.empty()) topic_name_ = "camera/image";

    RCLCPP_INFO(get_logger(), "Config loaded: serial=%s topic=%s trigger=%d scale=%.2f",
                serial_number_.c_str(), topic_name_.c_str(), trigger_enable_, image_scale_);
    return true;
  }

  /** Enumerate MVS devices, find the matching camera, open it, and start grabbing. */
  bool initCamera() {
    int nRet;

    // Enumerate
    MV_CC_DEVICE_INFO_LIST dev_list{};
    nRet = MV_CC_EnumDevices(MV_GIGE_DEVICE | MV_USB_DEVICE, &dev_list);
    if (nRet != MV_OK) {
      RCLCPP_ERROR(get_logger(), "MV_CC_EnumDevices failed [0x%x]", nRet);
      return false;
    }
    if (dev_list.nDeviceNum == 0) {
      RCLCPP_ERROR(get_logger(), "No MVS devices found!");
      return false;
    }

    for (unsigned i = 0; i < dev_list.nDeviceNum; ++i) {
      printf("[device %u]:\n", i);
      PrintDeviceInfo(dev_list.pDeviceInfo[i]);
    }

    // Select device
    unsigned sel = 0;
    if (dev_list.nDeviceNum > 1) {
      if (serial_number_.empty()) {
        RCLCPP_ERROR(get_logger(), "Multiple cameras but SerialNumber not set!");
        return false;
      }
      bool found = false;
      for (unsigned i = 0; i < dev_list.nDeviceNum; ++i) {
        std::string sn;
        auto *info = dev_list.pDeviceInfo[i];
        if (info->nTLayerType == MV_USB_DEVICE)
          sn = reinterpret_cast<const char *>(info->SpecialInfo.stUsb3VInfo.chSerialNumber);
        else if (info->nTLayerType == MV_GIGE_DEVICE)
          sn = reinterpret_cast<const char *>(info->SpecialInfo.stGigEInfo.chSerialNumber);
        if (sn == serial_number_) { sel = i; found = true; break; }
      }
      if (!found) {
        RCLCPP_ERROR(get_logger(), "Camera serial %s not found!", serial_number_.c_str());
        return false;
      }
    }

    // Create handle & open
    nRet = MV_CC_CreateHandle(&handle_, dev_list.pDeviceInfo[sel]);
    if (nRet != MV_OK) {
      RCLCPP_ERROR(get_logger(), "MV_CC_CreateHandle failed [0x%x]", nRet);
      return false;
    }
    nRet = MV_CC_OpenDevice(handle_);
    if (nRet != MV_OK) {
      RCLCPP_ERROR(get_logger(), "MV_CC_OpenDevice failed [0x%x]", nRet);
      return false;
    }

    // Disable auto frame rate
    MV_CC_SetBoolValue(handle_, "AcquisitionFrameRateEnable", false);

    // Pixel format
    if (pixel_format_idx_ < 0 || pixel_format_idx_ >= (int)PIXEL_FORMATS.size())
      pixel_format_idx_ = 0;
    nRet = MV_CC_SetEnumValue(handle_, "PixelFormat", PIXEL_FORMATS[pixel_format_idx_]);
    if (nRet != MV_OK) {
      RCLCPP_ERROR(get_logger(), "Set PixelFormat failed [0x%x]", nRet);
      return false;
    }

    // Exposure / gain / gamma
    applyParams();

    // Trigger mode
    MV_CC_SetEnumValue(handle_, "TriggerMode", trigger_enable_);
    if (trigger_enable_) {
      MV_CC_SetEnumValue(handle_, "TriggerSource", MV_TRIGGER_SOURCE_LINE0);
    }

    // Start grabbing
    nRet = MV_CC_StartGrabbing(handle_);
    if (nRet != MV_OK) {
      RCLCPP_ERROR(get_logger(), "MV_CC_StartGrabbing failed [0x%x]", nRet);
      return false;
    }

    RCLCPP_INFO(get_logger(), "Camera started grabbing.");
    return true;
  }

  /** Create the image_transport publisher and start the grab thread. */
  void startPublishing() {
    pub_ = image_transport::create_publisher(this, topic_name_);
    worker_ = std::thread(&MvsCameraNode::grabLoop, this);
  }

  /** Signal the grab thread to stop. */
  void requestStop() { running_ = false; }

  /** Block until the grab thread finishes. */
  void join() {
    if (worker_.joinable()) worker_.join();
  }

private:
  // ---- camera parameters (from YAML) ----
  std::string config_path_;
  std::string serial_number_;
  std::string topic_name_;
  int   trigger_enable_   = 0;
  int   pixel_format_idx_ = 0;
  int   exposure_auto_mode_ = 0;
  int   exposure_time_    = 5000;
  int   gain_auto_        = 0;
  float gain_             = 1.0f;
  float gamma_            = 1.0f;
  int   gamma_selector_   = 0;
  float image_scale_      = 1.0f;

  // ---- MVS handle ----
  void *handle_;

  // ---- ROS2 ----
  image_transport::Publisher pub_;
  std::thread worker_;
  std::atomic<bool> running_;

  // ---- shared-memory trigger timestamp ----
  TimeStamp *shm_ = nullptr;
  int shm_fd_ = -1;

  // ---- helpers ----

  void applyParams() {
    int nRet;

    nRet = MV_CC_SetExposureAutoMode(handle_, exposure_auto_mode_);
    if (nRet == MV_OK) {
      RCLCPP_INFO(get_logger(), "ExposureAutoMode = %s", EXPOSURE_AUTO_STR[exposure_auto_mode_]);
    } else {
      RCLCPP_WARN(get_logger(), "Failed to set ExposureAutoMode [0x%x]", nRet);
    }

    if (exposure_auto_mode_ == 2) {  // Continuous auto
      MV_CC_SetAutoExposureTimeLower(handle_, 100);
      MV_CC_SetAutoExposureTimeUpper(handle_, 20000);
    }
    if (exposure_auto_mode_ == 0) {  // Manual
      nRet = MV_CC_SetExposureTime(handle_, exposure_time_);
      if (nRet == MV_OK)
        RCLCPP_INFO(get_logger(), "ExposureTime = %d us", exposure_time_);
      else
        RCLCPP_WARN(get_logger(), "Failed to set ExposureTime [0x%x]", nRet);
    }

    nRet = MV_CC_SetEnumValue(handle_, "GainAuto", gain_auto_);
    RCLCPP_INFO(get_logger(), "GainAuto = %s", GAIN_AUTO_STR[gain_auto_]);

    if (gain_auto_ == 0) {
      MV_CC_SetGain(handle_, gain_);
      RCLCPP_INFO(get_logger(), "Gain = %.2f", gain_);
    }

    MV_CC_SetGammaSelector(handle_, gamma_selector_);
    RCLCPP_INFO(get_logger(), "GammaSelector = %s", GAMMA_SELECTOR_STR[gamma_selector_]);

    MV_CC_SetGamma(handle_, gamma_);
    RCLCPP_INFO(get_logger(), "Gamma = %.2f", gamma_);
  }

  /** Open shared memory for trigger timestamp injection. */
  void openShm() {
    const char *user = getlogin();
    if (!user) return;
    std::string path = "/home/" + std::string(user) + "/timeshare";
    shm_fd_ = open(path.c_str(), O_RDWR);
    if (shm_fd_ < 0) {
      RCLCPP_WARN(get_logger(), "Cannot open shared memory %s (trigger timestamps disabled)", path.c_str());
      return;
    }
    shm_ = static_cast<TimeStamp *>(mmap(nullptr, sizeof(TimeStamp), PROT_READ | PROT_WRITE, MAP_SHARED, shm_fd_, 0));
    if (shm_ == MAP_FAILED) {
      RCLCPP_WARN(get_logger(), "mmap failed (trigger timestamps disabled)");
      shm_ = nullptr;
    }
  }

  void closeShm() {
    if (shm_ && shm_ != MAP_FAILED) munmap(shm_, sizeof(TimeStamp));
    if (shm_fd_ >= 0) ::close(shm_fd_);
  }

  // ---- grab thread ----

  void grabLoop() {
    openShm();

    MVCC_INTVALUE param{};
    int nRet = MV_CC_GetIntValue(handle_, "PayloadSize", &param);
    if (nRet != MV_OK) {
      RCLCPP_ERROR(get_logger(), "Get PayloadSize failed [0x%x]", nRet);
      return;
    }

    const unsigned buf_size = param.nCurValue * 3;
    auto *raw_data   = static_cast<unsigned char *>(std::malloc(buf_size));
    auto *bgr_data   = static_cast<unsigned char *>(std::malloc(buf_size));
    if (!raw_data || !bgr_data) {
      RCLCPP_ERROR(get_logger(), "Memory allocation failed!");
      std::free(raw_data); std::free(bgr_data);
      return;
    }

    MV_FRAME_OUT_INFO_EX frame_info{};
    MV_CC_PIXEL_CONVERT_PARAM conv_param{};

    int fail_count = 0;
    while (running_ && rclcpp::ok()) {
      nRet = MV_CC_GetOneFrameTimeout(handle_, raw_data, buf_size, &frame_info, 1000);
      if (nRet != MV_OK) {
        if (fail_count < 5 || fail_count % 100 == 0)
          RCLCPP_WARN(get_logger(), "GetOneFrame failed [0x%x] (count=%d)", nRet, fail_count);
        ++fail_count;
        continue;
      }
      if (fail_count > 0) {
        RCLCPP_INFO(get_logger(), "Frame acquisition recovered after %d failures", fail_count);
        fail_count = 0;
      }

      // Timestamp
      rclcpp::Time stamp;
      if (trigger_enable_ && shm_ && shm_->low != 0) {
        double sec = shm_->low / 1e9;
        stamp = rclcpp::Time(static_cast<int64_t>(sec), static_cast<uint32_t>((sec - static_cast<int64_t>(sec)) * 1e9));
      } else {
        stamp = this->now();
      }

      // Convert pixel format to RGB8
      conv_param.nWidth        = frame_info.nWidth;
      conv_param.nHeight       = frame_info.nHeight;
      conv_param.pSrcData      = raw_data;
      conv_param.nSrcDataLen   = buf_size;
      conv_param.enSrcPixelType = frame_info.enPixelType;
      conv_param.enDstPixelType = PixelType_Gvsp_RGB8_Packed;
      conv_param.pDstBuffer    = bgr_data;
      conv_param.nDstBufferSize = buf_size;

      nRet = MV_CC_ConvertPixelType(handle_, &conv_param);
      if (nRet != MV_OK) {
        RCLCPP_WARN(get_logger(), "ConvertPixelType failed [0x%x], skipping frame", nRet);
        continue;
      }

      cv::Mat img(frame_info.nHeight, frame_info.nWidth, CV_8UC3, bgr_data);
      if (image_scale_ > 0.0f && image_scale_ != 1.0f) {
        cv::resize(img, img,
                   cv::Size(static_cast<int>(img.cols * image_scale_),
                            static_cast<int>(img.rows * image_scale_)),
                   cv::INTER_LINEAR);
      }

      auto msg = cv_bridge::CvImage(std_msgs::msg::Header(), "rgb8", img).toImageMsg();
      msg->header.stamp = stamp;
      msg->header.frame_id = "camera";
      pub_.publish(msg);
    }

    std::free(raw_data);
    std::free(bgr_data);
    closeShm();
  }

  void cleanup() {
    running_ = false;
    if (worker_.joinable()) worker_.join();
    if (handle_) {
      MV_CC_StopGrabbing(handle_);
      MV_CC_CloseDevice(handle_);
      MV_CC_DestroyHandle(handle_);
      handle_ = nullptr;
    }
  }
};

// =========================================================================
// main
// =========================================================================
int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  if (argc < 2) {
    RCLCPP_FATAL(rclcpp::get_logger("mvs_camera"), "Usage: grabImgWithTrigger <config.yaml>");
    return 1;
  }

  auto node = std::make_shared<MvsCameraNode>(rclcpp::NodeOptions());

  if (!node->loadConfig(argv[1])) return 1;
  if (!node->initCamera()) return 1;
  node->startPublishing();

  rclcpp::spin(node);

  node->requestStop();
  node->join();
  rclcpp::shutdown();
  return 0;
}
