#pragma once

// TODO
// - remove m_ before private members
// - add const to member functions
// fix includes in all files
// - should we rclcpp::shutdown in construction instead
//

// std
#include <atomic>      // min_publish_period_ns_
#include <chrono>      //chrono_literals
#include <functional>  // std::bind , std::placeholders
#include <mutex>       // device_mutex_
#include <thread>      // publish_thread_
#include <unordered_set>

// ros
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/timer.hpp>           // WallTimer
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <std_msgs/msg/header.hpp>
#include <sensor_msgs/msg/image.hpp>  //image msg published
#include <std_srvs/srv/trigger.hpp>   // Trigger
#include <camera_info_manager/camera_info_manager.hpp>

// arena sdk
#include "ArenaApi.h"

class ArenaCameraNode : public rclcpp::Node
{
 public:
  ArenaCameraNode() : Node("arena_camera_node")
  {
    // set stdout buffer size for ROS defined size BUFSIZE
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);

    log_info(std::string("Creating \"") + this->get_name() + "\" node");
    parse_parameters_();
    initialize_();
    log_info(std::string("Created \"") + this->get_name() + "\" node");
  }

  ~ArenaCameraNode()
  {
    log_info(std::string("Destroying \"") + this->get_name() + "\" node");
    if (publish_thread_.joinable()) {
      publish_thread_.join();
    }
  }

  void log_debug(std::string msg) { RCLCPP_DEBUG(this->get_logger(), msg.c_str()); };
  void log_info(std::string msg) { RCLCPP_INFO(this->get_logger(), msg.c_str()); };
  void log_warn(std::string msg) { RCLCPP_WARN(this->get_logger(), msg.c_str()); };
  void log_err(std::string msg) { RCLCPP_ERROR(this->get_logger(), msg.c_str()); };

 private:
  std::shared_ptr<Arena::ISystem> m_pSystem;
  std::shared_ptr<Arena::IDevice> m_pDevice;

  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr m_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr m_pub_camera_info_;
  std::shared_ptr<camera_info_manager::CameraInfoManager> m_camera_info_manager_;
  rclcpp::TimerBase::SharedPtr m_wait_for_device_timer_callback_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr m_trigger_an_image_srv_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr
      m_on_set_parameters_callback_handle_;

  // Image acquisition runs on its own thread so the executor thread stays
  // free to service ros2 param set/get (and the trigger service) while
  // streaming. Guards concurrent GenICam nodemap writes between the
  // executor thread (parameter callback, initial set_nodes_()) and any
  // future caller; the publish thread itself never touches the nodemap.
  std::thread publish_thread_;
  std::mutex device_mutex_;

  std::string serial_;
  bool is_passed_serial_;

  std::string ip_;
  bool is_passed_ip_;

  std::string host_iface_ip_;
  bool is_passed_host_iface_ip_;

  std::string topic_;

  size_t width_;
  bool is_passed_width;

  size_t height_;
  bool is_passed_height;

  double gain_;
  bool is_passed_gain_;

  bool auto_gain_;
  double auto_gain_min_;
  double auto_gain_max_;

  double exposure_time_;
  bool is_passed_exposure_time_;

  bool auto_exposure_;
  double auto_exposure_min_us_;
  double auto_exposure_max_us_;
  // Maps to the camera's TargetBrightness node, which is an *integer* and is
  // shared by the auto-exposure and auto-gain algorithms alike.
  double auto_exposure_target_;

  double frame_rate_hz_;
  bool is_passed_frame_rate_hz_;

  std::string pixelformat_pfnc_;
  std::string pixelformat_ros_;
  bool is_passed_pixelformat_ros_;

  bool trigger_mode_activated_;
  std::string trigger_source_;
  std::string trigger_selector_;
  std::string trigger_activation_;

  std::string pub_qos_history_;
  bool is_passed_pub_qos_history_;

  size_t pub_qos_history_depth_;
  bool is_passed_pub_qos_history_depth_;

  std::string pub_qos_reliability_;
  bool is_passed_pub_qos_reliability_;

  int discovery_timeout_ms_;

  // --- ROS integration -----------------------------------------------------
  // TF frame the images are stamped with. The driver used to put the image
  // sequence number here, which is not a frame at all.
  std::string frame_id_;
  std::string camera_info_url_;
  // The device clock free-runs from power-on and is unrelated to ROS time
  // unless PTP is locked, so default to stamping on arrival.
  bool use_ros_time_;
  bool ptp_enable_;

  // --- auto exposure / gain tuning -----------------------------------------
  double exposure_auto_damping_;
  std::string exposure_auto_algorithm_;
  bool ae_aoi_enable_;
  int64_t ae_aoi_width_;
  int64_t ae_aoi_height_;
  int64_t ae_aoi_offset_x_;
  int64_t ae_aoi_offset_y_;

  // --- image quality / sensor ----------------------------------------------
  std::string balance_white_auto_;
  double gamma_;
  double black_level_;
  std::string adc_bit_depth_;
  int64_t offset_x_;
  int64_t offset_y_;
  bool reverse_x_;
  bool reverse_y_;

  // --- transport / latency -------------------------------------------------
  std::string stream_buffer_handling_mode_;
  int64_t packet_size_;
  int64_t packet_delay_;
  int64_t throughput_limit_;

  // Written by the executor thread (parameter callback), read every frame by
  // the publish thread.
  std::atomic<uint64_t> min_publish_period_ns_{0};
  uint64_t last_publish_timestamp_ns_ = 0;

  void parse_parameters_();
  void initialize_();
  rcl_interfaces::msg::SetParametersResult on_set_parameters_(
      const std::vector<rclcpp::Parameter>& parameters);

  // Writes one GenICam feature, reporting failure instead of hiding it.
  // Many features are conditionally read-only (e.g. ExposureAutoLowerLimit is
  // RO while ExposureAutoLimitAuto=Continuous), and a silently dropped write
  // is indistinguishable from a setting that was applied -- always log.
  template <typename T, typename NodeMapT>
  bool try_set_node_(NodeMapT nodemap, const char* feature, T value)
  {
    try {
      Arena::SetNodeValue<T>(nodemap, feature, value);
      return true;
    } catch (const GenICam::GenericException& e) {
      log_warn(std::string("\t") + feature + " NOT applied: " + e.what());
      return false;
    } catch (const std::exception& e) {
      log_warn(std::string("\t") + feature + " NOT applied: " + e.what());
      return false;
    }
  }

  // Writes a float feature, clamping into the range the camera currently
  // allows. Several limits are bounded by other settings (the exposure upper
  // limit is capped by the frame rate, for instance), and a request just past
  // the edge would otherwise be dropped entirely rather than honoured as
  // closely as the hardware permits.
  template <typename NodeMapT>
  bool set_float_clamped_(NodeMapT nodemap, const char* feature, double value)
  {
    try {
      GenApi::CFloatPtr node = nodemap->GetNode(feature);
      if (!node.IsValid() || !GenApi::IsWritable(node)) {
        log_warn(std::string("\t") + feature + " NOT applied: not writable");
        return false;
      }

      const double lo = node->GetMin();
      const double hi = node->GetMax();
      const double applied = value < lo ? lo : (value > hi ? hi : value);

      node->SetValue(applied);

      if (applied != value) {
        log_warn(std::string("\t") + feature + " clamped: requested " +
                 std::to_string(value) + ", camera allows [" + std::to_string(lo) +
                 ", " + std::to_string(hi) + "], applied " + std::to_string(applied));
      }
      return true;
    } catch (const GenICam::GenericException& e) {
      log_warn(std::string("\t") + feature + " NOT applied: " + e.what());
      return false;
    } catch (const std::exception& e) {
      log_warn(std::string("\t") + feature + " NOT applied: " + e.what());
      return false;
    }
  }

  // Integer counterpart of set_float_clamped_ (ROI offsets, packet size, ...).
  template <typename NodeMapT>
  bool set_int_clamped_(NodeMapT nodemap, const char* feature, int64_t value)
  {
    try {
      GenApi::CIntegerPtr node = nodemap->GetNode(feature);
      if (!node.IsValid() || !GenApi::IsWritable(node)) {
        log_warn(std::string("\t") + feature + " NOT applied: not writable");
        return false;
      }

      const int64_t lo = node->GetMin();
      const int64_t hi = node->GetMax();
      const int64_t inc = node->GetInc() > 0 ? node->GetInc() : 1;
      int64_t applied = value < lo ? lo : (value > hi ? hi : value);
      // Many integer nodes only accept multiples of an increment.
      applied = lo + ((applied - lo) / inc) * inc;

      node->SetValue(applied);

      if (applied != value) {
        log_warn(std::string("\t") + feature + " adjusted: requested " +
                 std::to_string(value) + ", camera allows [" + std::to_string(lo) +
                 ", " + std::to_string(hi) + "] step " + std::to_string(inc) +
                 ", applied " + std::to_string(applied));
      }
      return true;
    } catch (const GenICam::GenericException& e) {
      log_warn(std::string("\t") + feature + " NOT applied: " + e.what());
      return false;
    } catch (const std::exception& e) {
      log_warn(std::string("\t") + feature + " NOT applied: " + e.what());
      return false;
    }
  }

  // Reads a feature back so logs report what the camera actually holds.
  template <typename T, typename NodeMapT>
  bool try_get_node_(NodeMapT nodemap, const char* feature, T& out)
  {
    try {
      out = Arena::GetNodeValue<T>(nodemap, feature);
      return true;
    } catch (const GenICam::GenericException&) {
      return false;
    } catch (const std::exception&) {
      return false;
    }
  }

  void wait_for_device_timer_callback_();
  uint32_t ipv4_to_u32_(const std::string& ip_str);
  void update_devices_for_selection_();

  void run_();
  // TODO :
  // - handle misconfigured device
  Arena::IDevice* create_device_ros_();
  void set_nodes_();
  void set_nodes_load_default_profile_();
  void set_nodes_roi_();
  void set_nodes_gain_();
  void set_nodes_pixelformat_();
  void set_nodes_exposure_();
  void set_nodes_frame_rate_();
  void set_nodes_trigger_mode_();
  void set_nodes_ae_tuning_();
  void set_nodes_image_quality_();
  void set_nodes_transport_();
  void set_nodes_ptp_();
  void publish_camera_info_(const std_msgs::msg::Header& header);
  static std::string image_transport_base_(const std::string& topic);
  void set_nodes_test_pattern_image_();
  void publish_images_();

  void publish_an_image_on_trigger_(
      std::shared_ptr<std_srvs::srv::Trigger::Request> request,
      std::shared_ptr<std_srvs::srv::Trigger::Response> response);
  void msg_form_image_(Arena::IImage* pImage,
                       sensor_msgs::msg::Image& image_msg);
};
