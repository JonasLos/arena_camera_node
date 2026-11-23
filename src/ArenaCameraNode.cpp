#include <cmath>      // std::lround
#include <cstring>    // memcopy
#include <stdexcept>  // std::runtime_err
#include <string>
#include <cstdio>

// ROS
#include "rmw/types.h"

// ArenaSDK
#include "ArenaCameraNode.h"
#include "light_arena/deviceinfo_helper.h"
#include "rclcpp_adapter/pixelformat_translation.h"
#include "rclcpp_adapter/quilty_of_service_translation.cpp"

void ArenaCameraNode::parse_parameters_()
{
  std::string nextParameterToDeclare = "";
  try {
    nextParameterToDeclare = "serial";
    // Must be declared: with default NodeOptions an undeclared parameter is
    // invisible to has_parameter(), so the previous has_parameter("serial")
    // check was always false and selecting a camera by serial never worked.
    // Declared as an integer because Lucid serials are numeric and that is what
    // "-p serial:=214000580" infers -- declaring it a string makes every such
    // launch abort on a parameter type mismatch.
    const int64_t serial_number = this->declare_parameter("serial", 0);
    is_passed_serial_ = serial_number > 0;
    serial_ = is_passed_serial_ ? std::to_string(serial_number) : std::string();

    nextParameterToDeclare = "ip";
    ip_ = this->declare_parameter("ip", "");
    is_passed_ip_ = ip_ != "";

    nextParameterToDeclare = "host_iface_ip";
    host_iface_ip_ = this->declare_parameter("host_iface_ip", "");
    is_passed_host_iface_ip_ = host_iface_ip_ != "";

    nextParameterToDeclare = "discovery_timeout_ms";
    discovery_timeout_ms_ = this->declare_parameter("discovery_timeout_ms", 1000);
    
    nextParameterToDeclare = "pixelformat";
    pixelformat_ros_ = this->declare_parameter("pixelformat", "");
    is_passed_pixelformat_ros_ = pixelformat_ros_ != "";

    nextParameterToDeclare = "width";
    width_ = this->declare_parameter("width", 0);
    is_passed_width = width_ > 0;

    nextParameterToDeclare = "height";
    height_ = this->declare_parameter("height", 0);
    is_passed_height = height_ > 0;

    nextParameterToDeclare = "gain";
    gain_ = this->declare_parameter("gain", -1.0);
    is_passed_gain_ = gain_ >= 0;

    // Defaults to false to preserve existing behaviour: the Default user set on
    // these cameras loads with GainAuto=Off, so brightness is driven by exposure
    // alone. Enabling auto gain adds sensor noise, so it must be opted into.
    nextParameterToDeclare = "auto_gain";
    auto_gain_ = this->declare_parameter("auto_gain", false);

    // -1 means "leave the camera's own limit alone"; usable gain ranges are
    // model specific (this sensor reports 0..48 dB).
    nextParameterToDeclare = "auto_gain_min";
    auto_gain_min_ = this->declare_parameter("auto_gain_min", -1.0);

    nextParameterToDeclare = "auto_gain_max";
    auto_gain_max_ = this->declare_parameter("auto_gain_max", -1.0);

    nextParameterToDeclare = "exposure_time";
    exposure_time_ = this->declare_parameter("exposure_time", -1.0);
    is_passed_exposure_time_ = exposure_time_ >= 0;

    nextParameterToDeclare = "auto_exposure";
    auto_exposure_ = this->declare_parameter("auto_exposure", true);

    nextParameterToDeclare = "auto_exposure_min_us";
    auto_exposure_min_us_ = this->declare_parameter("auto_exposure_min_us", 100.0);

    nextParameterToDeclare = "auto_exposure_max_us";
    auto_exposure_max_us_ = this->declare_parameter("auto_exposure_max_us", 60000.0);

    nextParameterToDeclare = "auto_exposure_target";
    auto_exposure_target_ = this->declare_parameter("auto_exposure_target", 50.0);

    nextParameterToDeclare = "frame_rate";
    frame_rate_hz_ = this->declare_parameter("frame_rate", -1.0);
    is_passed_frame_rate_hz_ = frame_rate_hz_ > 0.0;

    nextParameterToDeclare = "trigger_mode";
    trigger_mode_activated_ = this->declare_parameter("trigger_mode", false);
    // no need to is_passed_trigger_mode_ because it is already a boolean

    nextParameterToDeclare = "trigger_source";
    trigger_source_ = this->declare_parameter("trigger_source", "Software");

    nextParameterToDeclare = "trigger_selector";
    trigger_selector_ = this->declare_parameter("trigger_selector", "FrameStart");

    nextParameterToDeclare = "trigger_activation";
    trigger_activation_ = this->declare_parameter("trigger_activation", "");

    nextParameterToDeclare = "topic";
    topic_ = this->declare_parameter(
        "topic", std::string("/") + this->get_name() + "/images");
    // no need to is_passed_topic_

    nextParameterToDeclare = "qos_history";
    pub_qos_history_ = this->declare_parameter("qos_history", "");
    is_passed_pub_qos_history_ = pub_qos_history_ != "";

    nextParameterToDeclare = "qos_history_depth";
    pub_qos_history_depth_ = this->declare_parameter("qos_history_depth", 0);
    is_passed_pub_qos_history_depth_ = pub_qos_history_depth_ > 0;

    // --- ROS integration ---------------------------------------------------
    nextParameterToDeclare = "frame_id";
    frame_id_ = this->declare_parameter("frame_id", "");

    nextParameterToDeclare = "camera_info_url";
    camera_info_url_ = this->declare_parameter("camera_info_url", "");

    nextParameterToDeclare = "use_ros_time";
    use_ros_time_ = this->declare_parameter("use_ros_time", true);

    nextParameterToDeclare = "ptp_enable";
    ptp_enable_ = this->declare_parameter("ptp_enable", false);

    // --- auto exposure / gain tuning ---------------------------------------
    // -1 / "" consistently mean "leave the camera's own setting alone".
    nextParameterToDeclare = "exposure_auto_damping";
    exposure_auto_damping_ = this->declare_parameter("exposure_auto_damping", -1.0);

    nextParameterToDeclare = "exposure_auto_algorithm";
    exposure_auto_algorithm_ = this->declare_parameter("exposure_auto_algorithm", "");

    nextParameterToDeclare = "auto_exposure_aoi_enable";
    ae_aoi_enable_ = this->declare_parameter("auto_exposure_aoi_enable", false);

    nextParameterToDeclare = "auto_exposure_aoi_width";
    ae_aoi_width_ = this->declare_parameter("auto_exposure_aoi_width", -1);

    nextParameterToDeclare = "auto_exposure_aoi_height";
    ae_aoi_height_ = this->declare_parameter("auto_exposure_aoi_height", -1);

    nextParameterToDeclare = "auto_exposure_aoi_offset_x";
    ae_aoi_offset_x_ = this->declare_parameter("auto_exposure_aoi_offset_x", -1);

    nextParameterToDeclare = "auto_exposure_aoi_offset_y";
    ae_aoi_offset_y_ = this->declare_parameter("auto_exposure_aoi_offset_y", -1);

    // --- image quality / sensor --------------------------------------------
    nextParameterToDeclare = "balance_white_auto";
    balance_white_auto_ = this->declare_parameter("balance_white_auto", "");

    nextParameterToDeclare = "gamma";
    gamma_ = this->declare_parameter("gamma", -1.0);

    nextParameterToDeclare = "black_level";
    black_level_ = this->declare_parameter("black_level", -1.0);

    nextParameterToDeclare = "adc_bit_depth";
    adc_bit_depth_ = this->declare_parameter("adc_bit_depth", "");

    nextParameterToDeclare = "offset_x";
    offset_x_ = this->declare_parameter("offset_x", -1);

    nextParameterToDeclare = "offset_y";
    offset_y_ = this->declare_parameter("offset_y", -1);

    nextParameterToDeclare = "reverse_x";
    reverse_x_ = this->declare_parameter("reverse_x", false);

    nextParameterToDeclare = "reverse_y";
    reverse_y_ = this->declare_parameter("reverse_y", false);

    // --- transport / latency -----------------------------------------------
    nextParameterToDeclare = "stream_buffer_handling_mode";
    stream_buffer_handling_mode_ =
        this->declare_parameter("stream_buffer_handling_mode", "");

    nextParameterToDeclare = "packet_size";
    packet_size_ = this->declare_parameter("packet_size", -1);

    nextParameterToDeclare = "packet_delay";
    packet_delay_ = this->declare_parameter("packet_delay", -1);

    nextParameterToDeclare = "device_link_throughput_limit";
    throughput_limit_ = this->declare_parameter("device_link_throughput_limit", -1);

    nextParameterToDeclare = "qos_reliability";
    pub_qos_reliability_ = this->declare_parameter("qos_reliability", "");
    is_passed_pub_qos_reliability_ = pub_qos_reliability_ != "";

  } catch (const rclcpp::exceptions::InvalidParameterTypeException& e) {
    // rclcpp throws this (not ParameterTypeException) when a launch/CLI
    // override has a different type than the declaration, and an uncaught
    // throw here aborts the process before any log explains why.
    log_err("Parameter '" + nextParameterToDeclare + "' was given the wrong type: " +
            e.what());
    throw;
  } catch (rclcpp::ParameterTypeException& e) {
    log_err(nextParameterToDeclare + " argument");
    throw;
  }
}

// "/camera_rear_1/image" -> "/camera_rear_1" so camera_info sits beside it.
std::string ArenaCameraNode::image_transport_base_(const std::string& topic)
{
  const auto slash = topic.find_last_of('/');
  if (slash == std::string::npos || slash == 0) {
    return topic;
  }
  return topic.substr(0, slash);
}

void ArenaCameraNode::initialize_()
{
  using namespace std::chrono_literals;
  if (is_passed_frame_rate_hz_) {
    min_publish_period_ns_ = static_cast<uint64_t>(1e9 / frame_rate_hz_);
  }
  // ARENASDK ---------------------------------------------------------------
  // Custom deleter for system
  m_pSystem =
      std::shared_ptr<Arena::ISystem>(nullptr, [=](Arena::ISystem* pSystem) {
        if (pSystem) {  // this is an issue for multi devices
          Arena::CloseSystem(pSystem);
          log_info("System is destroyed");
        }
      });
  m_pSystem.reset(Arena::OpenSystem());

  // Custom deleter for device
  m_pDevice =
      std::shared_ptr<Arena::IDevice>(nullptr, [=](Arena::IDevice* pDevice) {
        if (m_pSystem && pDevice) {
          m_pSystem->DestroyDevice(pDevice);
          log_info("Device is destroyed");
        }
      });

  //
  // CHECK DEVICE CONNECTION ( timer ) --------------------------------------
  //
  // TODO
  // - Think of design that allow the node to start stream as soon as
  // it is initialized without waiting for spin to be called
  // - maybe change 1s to a smaller value
  m_wait_for_device_timer_callback_ = this->create_wall_timer(
      1s, std::bind(&ArenaCameraNode::wait_for_device_timer_callback_, this));

  //
  // TRIGGER (service) ------------------------------------------------------
  //
  using namespace std::placeholders;
  m_trigger_an_image_srv_ = this->create_service<std_srvs::srv::Trigger>(
      std::string(this->get_name()) + "/trigger_image",
      std::bind(&ArenaCameraNode::publish_an_image_on_trigger_, this, _1, _2));

  //
  // Publisher --------------------------------------------------------------
  //
  // m_pub_qos is rclcpp::SensorDataQoS has these defaults
  // https://github.com/ros2/rmw/blob/fb06b57975373b5a23691bb00eb39c07f1660ed7/rmw/include/rmw/qos_profiles.h#L25

  /*
  static const rmw_qos_profile_t rmw_qos_profile_sensor_data =
  {
    RMW_QOS_POLICY_HISTORY_KEEP_LAST,
    5, // history depth
    RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
    RMW_QOS_POLICY_DURABILITY_VOLATILE,
    RMW_QOS_DEADLINE_DEFAULT,
    RMW_QOS_LIFESPAN_DEFAULT,
    RMW_QOS_POLICY_LIVELINESS_SYSTEM_DEFAULT,
    RMW_QOS_LIVELINESS_LEASE_DURATION_DEFAULT,
    false // avoid ros namespace conventions
  };
  */
  rclcpp::SensorDataQoS pub_qos_;
  // QoS history
  if (is_passed_pub_qos_history_) {
    if (is_supported_qos_histroy_policy(pub_qos_history_)) {
      pub_qos_.history(
          K_CMDLN_PARAMETER_TO_QOS_HISTORY_POLICY[pub_qos_history_]);
    } else {
      log_err(pub_qos_history_ + " is not supported for this node");
      // TODO
      // should thorow instead??
      // should this keeps shutting down if for some reasons this node is kept
      // alive
      throw;
    }
  }
  // QoS depth. This used to test the *requested* history string against a map
  // that has no "" key, so an unset qos_history yielded SYSTEM_DEFAULT and the
  // depth was silently dropped -- qos_history_depth did nothing on its own.
  // Test the effective profile instead, since SensorDataQoS is already
  // keep_last.
  if (is_passed_pub_qos_history_depth_) {
    if (pub_qos_.get_rmw_qos_profile().history == RMW_QOS_POLICY_HISTORY_KEEP_LAST) {
      pub_qos_.keep_last(pub_qos_history_depth_);
    } else {
      log_warn("qos_history_depth is ignored because qos_history is not keep_last");
    }
  }

  // Qos reliability
  if (is_passed_pub_qos_reliability_) {
    if (is_supported_qos_reliability_policy(pub_qos_reliability_)) {
      pub_qos_.reliability(
          K_CMDLN_PARAMETER_TO_QOS_RELIABILITY_POLICY[pub_qos_reliability_]);
    } else {
      log_err(pub_qos_reliability_ + " is not supported for this node");
      throw;
    }
  }

  // rmw_qos_history_policy_t history_policy_ = RMW_QOS_
  // rmw_qos_history_policy_t;
  // auto pub_qos_init = rclcpp::QoSInitialization(history_policy_, );

  m_pub_ = this->create_publisher<sensor_msgs::msg::Image>(
      this->get_parameter("topic").as_string(), pub_qos_);

  // Fall back to the node's namespace so the frame is at least a stable,
  // meaningful name rather than an image counter.
  if (frame_id_.empty()) {
    std::string ns = this->get_namespace();
    while (!ns.empty() && ns.front() == '/') {
      ns.erase(ns.begin());
    }
    frame_id_ = ns.empty() ? std::string(this->get_name()) : ns;
    log_warn("frame_id was not set; defaulting to \"" + frame_id_ +
             "\". Set it to the camera's TF frame (e.g. camera_rc) for TF to work.");
  }

  // CameraInfo shares the image's namespace, which is what image_proc and the
  // rest of the pipeline expect.
  const std::string info_topic =
      image_transport_base_(this->get_parameter("topic").as_string()) + "/camera_info";
  m_camera_info_manager_ = std::make_shared<camera_info_manager::CameraInfoManager>(
      this, frame_id_, camera_info_url_);
  m_pub_camera_info_ =
      this->create_publisher<sensor_msgs::msg::CameraInfo>(info_topic, pub_qos_);

  if (camera_info_url_.empty()) {
    log_warn("camera_info_url is empty: CameraInfo will be published uncalibrated. "
             "Point it at this camera's intrinsics file to enable rectification.");
  } else if (!m_camera_info_manager_->isCalibrated()) {
    log_warn("camera_info_url=" + camera_info_url_ +
             " did not yield a valid calibration; CameraInfo will be uncalibrated.");
  } else {
    log_info("Loaded camera calibration from " + camera_info_url_);
  }
  log_info("Publishing CameraInfo on " + info_topic + " with frame_id=" + frame_id_);

  if (is_passed_ip_) {
    log_info("Camera selection mode: ip=" + ip_);
  } else if (is_passed_serial_) {
    log_info("Camera selection mode: serial=" + serial_);
  } else {
    log_info("Camera selection mode: first discovered device");
  }
  log_info("Arena discovery timeout (ms): " + std::to_string(discovery_timeout_ms_));
  if (is_passed_host_iface_ip_) {
    log_info("Forced discovery interface ip: " + host_iface_ip_);
  }

  std::stringstream pub_qos_info;
  auto pub_qos_profile = pub_qos_.get_rmw_qos_profile();
  pub_qos_info
      << '\t' << "QoS history     = "
      << K_QOS_HISTORY_POLICY_TO_CMDLN_PARAMETER[pub_qos_profile.history]
      << '\n';
  pub_qos_info << "\t\t\t\t"
               << "QoS depth       = " << pub_qos_profile.depth << '\n';
  pub_qos_info << "\t\t\t\t"
               << "QoS reliability = "
               << K_QOS_RELIABILITY_POLICY_TO_CMDLN_PARAMETER[pub_qos_profile
                                                                  .reliability]
               << '\n';

  log_info(pub_qos_info.str());

  m_on_set_parameters_callback_handle_ = this->add_on_set_parameters_callback(
      std::bind(&ArenaCameraNode::on_set_parameters_, this, std::placeholders::_1));
}

rcl_interfaces::msg::SetParametersResult ArenaCameraNode::on_set_parameters_(
    const std::vector<rclcpp::Parameter>& parameters)
{
  // Parameters that select/identify the device, size its buffers, or shape
  // the publisher can't be changed without tearing down and recreating the
  // device connection / stream / publisher. Reject them with a clear reason
  // instead of silently accepting a value that never takes effect.
  static const std::unordered_set<std::string> kStructuralParams = {
      "serial",       "ip",       "host_iface_ip", "discovery_timeout_ms",
      "topic",        "qos_history", "qos_history_depth", "qos_reliability",
      "width",        "height",   "pixelformat",
      "trigger_mode", "trigger_source", "trigger_selector", "trigger_activation",
      // Change buffer geometry, transport sizing or clock discipline, none of
      // which can be reworked underneath a running stream.
      "offset_x",     "offset_y", "reverse_x", "reverse_y", "adc_bit_depth",
      "packet_size",  "packet_delay", "device_link_throughput_limit",
      "stream_buffer_handling_mode", "ptp_enable",
  };

  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;

  for (const auto& param : parameters) {
    if (kStructuralParams.count(param.get_name())) {
      result.successful = false;
      result.reason = param.get_name() +
          " cannot be changed at runtime; restart arena_camera_node to apply it.";
      return result;
    }
  }

  if (!m_pDevice) {
    // Device hasn't connected yet; the new values are already staged by
    // rclcpp's parameter storage and will be applied by set_nodes_() once
    // the device is discovered and streaming starts.
    return result;
  }

  bool gain_dirty = false;
  bool exposure_dirty = false;
  bool frame_rate_dirty = false;
  bool ae_tuning_dirty = false;
  bool image_quality_dirty = false;
  bool camera_info_dirty = false;

  // Snapshot every member this callback can touch. If applying to the camera
  // throws, these are restored so the driver's state cannot drift out of sync
  // with rclcpp's stored parameter values (which are not committed on failure).
  const double prev_gain = gain_;
  const bool prev_is_passed_gain = is_passed_gain_;
  const bool prev_auto_gain = auto_gain_;
  const double prev_auto_gain_min = auto_gain_min_;
  const double prev_auto_gain_max = auto_gain_max_;
  const double prev_exposure_time = exposure_time_;
  const bool prev_is_passed_exposure_time = is_passed_exposure_time_;
  const bool prev_auto_exposure = auto_exposure_;
  const double prev_auto_exposure_min_us = auto_exposure_min_us_;
  const double prev_auto_exposure_max_us = auto_exposure_max_us_;
  const double prev_auto_exposure_target = auto_exposure_target_;
  const double prev_frame_rate_hz = frame_rate_hz_;
  const bool prev_is_passed_frame_rate = is_passed_frame_rate_hz_;
  const uint64_t prev_min_publish_period_ns = min_publish_period_ns_.load();
  const double prev_exposure_auto_damping = exposure_auto_damping_;
  const std::string prev_exposure_auto_algorithm = exposure_auto_algorithm_;
  const bool prev_ae_aoi_enable = ae_aoi_enable_;
  const int64_t prev_ae_aoi_width = ae_aoi_width_;
  const int64_t prev_ae_aoi_height = ae_aoi_height_;
  const int64_t prev_ae_aoi_offset_x = ae_aoi_offset_x_;
  const int64_t prev_ae_aoi_offset_y = ae_aoi_offset_y_;
  const std::string prev_balance_white_auto = balance_white_auto_;
  const double prev_gamma = gamma_;
  const double prev_black_level = black_level_;
  const std::string prev_frame_id = frame_id_;
  const bool prev_use_ros_time = use_ros_time_;
  const std::string prev_camera_info_url = camera_info_url_;

  for (const auto& param : parameters) {
    const auto& name = param.get_name();
    if (name == "gain") {
      gain_ = param.as_double();
      is_passed_gain_ = gain_ >= 0;
      gain_dirty = true;
    } else if (name == "auto_gain") {
      auto_gain_ = param.as_bool();
      gain_dirty = true;
    } else if (name == "auto_gain_min") {
      auto_gain_min_ = param.as_double();
      gain_dirty = true;
    } else if (name == "auto_gain_max") {
      auto_gain_max_ = param.as_double();
      gain_dirty = true;
    } else if (name == "exposure_time") {
      exposure_time_ = param.as_double();
      is_passed_exposure_time_ = exposure_time_ >= 0;
      exposure_dirty = true;
    } else if (name == "auto_exposure") {
      auto_exposure_ = param.as_bool();
      exposure_dirty = true;
    } else if (name == "auto_exposure_min_us") {
      auto_exposure_min_us_ = param.as_double();
      exposure_dirty = true;
    } else if (name == "auto_exposure_max_us") {
      auto_exposure_max_us_ = param.as_double();
      exposure_dirty = true;
    } else if (name == "auto_exposure_target") {
      auto_exposure_target_ = param.as_double();
      exposure_dirty = true;
    } else if (name == "frame_rate") {
      frame_rate_hz_ = param.as_double();
      is_passed_frame_rate_hz_ = frame_rate_hz_ > 0.0;
      frame_rate_dirty = true;
    } else if (name == "exposure_auto_damping") {
      exposure_auto_damping_ = param.as_double();
      ae_tuning_dirty = true;
    } else if (name == "exposure_auto_algorithm") {
      exposure_auto_algorithm_ = param.as_string();
      ae_tuning_dirty = true;
    } else if (name == "auto_exposure_aoi_enable") {
      ae_aoi_enable_ = param.as_bool();
      ae_tuning_dirty = true;
    } else if (name == "auto_exposure_aoi_width") {
      ae_aoi_width_ = param.as_int();
      ae_tuning_dirty = true;
    } else if (name == "auto_exposure_aoi_height") {
      ae_aoi_height_ = param.as_int();
      ae_tuning_dirty = true;
    } else if (name == "auto_exposure_aoi_offset_x") {
      ae_aoi_offset_x_ = param.as_int();
      ae_tuning_dirty = true;
    } else if (name == "auto_exposure_aoi_offset_y") {
      ae_aoi_offset_y_ = param.as_int();
      ae_tuning_dirty = true;
    } else if (name == "balance_white_auto") {
      balance_white_auto_ = param.as_string();
      image_quality_dirty = true;
    } else if (name == "gamma") {
      gamma_ = param.as_double();
      image_quality_dirty = true;
    } else if (name == "black_level") {
      black_level_ = param.as_double();
      image_quality_dirty = true;
    } else if (name == "frame_id") {
      // Header-only; nothing to push to the camera.
      frame_id_ = param.as_string();
    } else if (name == "use_ros_time") {
      use_ros_time_ = param.as_bool();
    } else if (name == "camera_info_url") {
      camera_info_url_ = param.as_string();
      camera_info_dirty = true;
    }
  }

  // Nothing may escape a parameter callback: an exception here propagates into
  // the executor and terminates the process.
  try {
    std::lock_guard<std::mutex> lock(device_mutex_);
    if (gain_dirty) {
      set_nodes_gain_();
    }
    if (exposure_dirty) {
      set_nodes_exposure_();
    }
    if (frame_rate_dirty) {
      min_publish_period_ns_.store(is_passed_frame_rate_hz_
          ? static_cast<uint64_t>(1e9 / frame_rate_hz_)
          : 0);
      set_nodes_frame_rate_();
    }
    if (image_quality_dirty) {
      set_nodes_image_quality_();
    }
    if (ae_tuning_dirty) {
      set_nodes_ae_tuning_();
    }
    if (camera_info_dirty && m_camera_info_manager_) {
      if (!m_camera_info_manager_->loadCameraInfo(camera_info_url_)) {
        log_warn("camera_info_url=" + camera_info_url_ + " could not be loaded");
      }
    }
  } catch (const std::exception& e) {
    result.successful = false;
    result.reason = std::string("Failed to apply parameter change: ") + e.what();
  } catch (...) {
    result.successful = false;
    result.reason = "Failed to apply parameter change: unknown exception";
  }

  if (!result.successful) {
    gain_ = prev_gain;
    is_passed_gain_ = prev_is_passed_gain;
    auto_gain_ = prev_auto_gain;
    auto_gain_min_ = prev_auto_gain_min;
    auto_gain_max_ = prev_auto_gain_max;
    exposure_time_ = prev_exposure_time;
    is_passed_exposure_time_ = prev_is_passed_exposure_time;
    auto_exposure_ = prev_auto_exposure;
    auto_exposure_min_us_ = prev_auto_exposure_min_us;
    auto_exposure_max_us_ = prev_auto_exposure_max_us;
    auto_exposure_target_ = prev_auto_exposure_target;
    frame_rate_hz_ = prev_frame_rate_hz;
    is_passed_frame_rate_hz_ = prev_is_passed_frame_rate;
    min_publish_period_ns_.store(prev_min_publish_period_ns);
    exposure_auto_damping_ = prev_exposure_auto_damping;
    exposure_auto_algorithm_ = prev_exposure_auto_algorithm;
    ae_aoi_enable_ = prev_ae_aoi_enable;
    ae_aoi_width_ = prev_ae_aoi_width;
    ae_aoi_height_ = prev_ae_aoi_height;
    ae_aoi_offset_x_ = prev_ae_aoi_offset_x;
    ae_aoi_offset_y_ = prev_ae_aoi_offset_y;
    balance_white_auto_ = prev_balance_white_auto;
    gamma_ = prev_gamma;
    black_level_ = prev_black_level;
    frame_id_ = prev_frame_id;
    use_ros_time_ = prev_use_ros_time;
    camera_info_url_ = prev_camera_info_url;
  }

  return result;
}

void ArenaCameraNode::wait_for_device_timer_callback_()
{
  // something happend while checking for cameras
  if (!rclcpp::ok()) {
    log_err("Interrupted while waiting for arena camera. Exiting.");
    rclcpp::shutdown();
  }

  // camera discovery
  update_devices_for_selection_();
  auto device_infos = m_pSystem->GetDevices();

  // no camera is connected
  if (!device_infos.size()) {
    log_info("No arena camera is connected. Waiting for device(s)...");
  }
  // at least on is found
  else {
    m_wait_for_device_timer_callback_->cancel();
    log_info(std::to_string(device_infos.size()) +
             " arena device(s) has been discoved.");
    run_();
  }
}

uint32_t ArenaCameraNode::ipv4_to_u32_(const std::string& ip_str)
{
  unsigned int o1 = 0, o2 = 0, o3 = 0, o4 = 0;
  if (std::sscanf(ip_str.c_str(), "%u.%u.%u.%u", &o1, &o2, &o3, &o4) != 4 ||
      o1 > 255 || o2 > 255 || o3 > 255 || o4 > 255) {
    throw std::invalid_argument("Invalid IPv4 string: " + ip_str);
  }

  return (o1 << 24) | (o2 << 16) | (o3 << 8) | o4;
}

void ArenaCameraNode::update_devices_for_selection_()
{
  if (!is_passed_ip_) {
    m_pSystem->UpdateDevices(discovery_timeout_ms_);
    return;
  }

  const auto target_ip = ipv4_to_u32_(ip_);
  auto interfaces = m_pSystem->GetInterfaces();
  bool updated_matching_iface = false;
  uint32_t forced_iface_ip = 0;
  if (is_passed_host_iface_ip_) {
    forced_iface_ip = ipv4_to_u32_(host_iface_ip_);
  }

  for (auto& iface : interfaces) {
    const auto iface_ip = iface.IpAddress();
    const auto mask = iface.SubnetMask();
    if (is_passed_host_iface_ip_ && iface_ip != forced_iface_ip) {
      continue;
    }
    const bool same_subnet = ((iface_ip & mask) == (target_ip & mask));

    if (!same_subnet) {
      continue;
    }

    updated_matching_iface = true;
    log_info("Targeted discovery on interface ip=" +
             std::string(iface.IpAddressStr().c_str()) +
             " subnet=" + std::string(iface.SubnetMaskStr().c_str()));
    m_pSystem->UpdateDevices(iface, discovery_timeout_ms_);
  }

  if (!updated_matching_iface) {
    log_warn("No matching host interface subnet for camera ip=" + ip_ +
             "; falling back to global discovery");
    m_pSystem->UpdateDevices(discovery_timeout_ms_);
  }
}

void ArenaCameraNode::run_()
{
  auto device = create_device_ros_();
  m_pDevice.reset(device);
  {
    std::lock_guard<std::mutex> lock(device_mutex_);
    set_nodes_();
  }
  m_pDevice->StartStream();

  if (!trigger_mode_activated_ || trigger_source_ != "Software") {
    // Run on a dedicated thread so the executor thread stays free to
    // service ros2 param set/get and the trigger service while streaming.
    publish_thread_ = std::thread(&ArenaCameraNode::publish_images_, this);
  } else {
    // else ros::spin will
  }
}

void ArenaCameraNode::publish_images_()
{
  Arena::IImage* pImage = nullptr;
  while (rclcpp::ok()) {
    try {
      pImage = m_pDevice->GetImage(1000);

      if (min_publish_period_ns_ > 0) {
        // Allow 10% slack: when the camera runs at the requested rate, its
        // frame period jitters around (and on some firmware sits just under)
        // the nominal period, and a strict compare drops every such frame,
        // halving the effective rate in bursts.
        const uint64_t min_period_ns = min_publish_period_ns_.load() * 9 / 10;
        const uint64_t current_timestamp_ns = pImage->GetTimestampNs();
        if (last_publish_timestamp_ns_ > 0 &&
            (current_timestamp_ns - last_publish_timestamp_ns_) < min_period_ns) {
          this->m_pDevice->RequeueBuffer(pImage);
          pImage = nullptr;
          continue;
        }
        last_publish_timestamp_ns_ = current_timestamp_ns;
      }

      auto p_image_msg = std::make_unique<sensor_msgs::msg::Image>();
      msg_form_image_(pImage, *p_image_msg);

      const auto header = p_image_msg->header;
      m_pub_->publish(std::move(p_image_msg));
      publish_camera_info_(header);

      log_info(std::string("image ") + std::to_string(pImage->GetFrameId()) +
               " published to " + topic_);
      this->m_pDevice->RequeueBuffer(pImage);

    } catch (std::exception& e) {
      if (pImage) {
        this->m_pDevice->RequeueBuffer(pImage);
        pImage = nullptr;
        log_warn(std::string("Exception occurred while publishing an image\n") +
                 e.what());
      }
    }
  };
}

void ArenaCameraNode::msg_form_image_(Arena::IImage* pImage,
                                      sensor_msgs::msg::Image& image_msg)
{
  try {
    // 1 ) Header
    //      - stamp.sec
    //      - stamp.nanosec
    //      - Frame ID
    // The device clock counts from power-on, so unless PTP has disciplined it
    // to the same epoch as the rest of the system it must not be used as a ROS
    // stamp -- it lands in 1970 and breaks every time-based lookup.
    if (use_ros_time_) {
      image_msg.header.stamp = this->now();
    } else {
      image_msg.header.stamp.sec =
          static_cast<uint32_t>(pImage->GetTimestampNs() / 1000000000);
      image_msg.header.stamp.nanosec =
          static_cast<uint32_t>(pImage->GetTimestampNs() % 1000000000);
    }
    // frame_id is a TF frame, not the image sequence number.
    image_msg.header.frame_id = frame_id_;

    //
    // 2 ) Height
    //
    image_msg.height = height_;

    //
    // 3 ) Width
    //
    image_msg.width = width_;

    //
    // 4 ) encoding
    //
    image_msg.encoding = pixelformat_ros_;

    //
    // 5 ) is_big_endian
    //
    // TODO what to do if unknown
    image_msg.is_bigendian = pImage->GetPixelEndianness() ==
                             Arena::EPixelEndianness::PixelEndiannessBig;
    //
    // 6 ) step
    //
    // TODO could be optimized by moving it out
    auto pixel_length_in_bytes = pImage->GetBitsPerPixel() / 8;
    auto width_length_in_bytes = pImage->GetWidth() * pixel_length_in_bytes;
    image_msg.step =
        static_cast<sensor_msgs::msg::Image::_step_type>(width_length_in_bytes);

    //
    // 7) data
    //
    auto image_data_length_in_bytes = width_length_in_bytes * height_;
    image_msg.data.resize(image_data_length_in_bytes);
    auto x = pImage->GetData();
    std::memcpy(&image_msg.data[0], pImage->GetData(),
                image_data_length_in_bytes);

  } catch (...) {
    log_warn(
        "Failed to create Image ROS MSG. Published Image Msg might be "
        "corrupted");
  }
}

void ArenaCameraNode::publish_camera_info_(const std_msgs::msg::Header& header)
{
  if (!m_pub_camera_info_ || !m_camera_info_manager_) {
    return;
  }

  auto info = m_camera_info_manager_->getCameraInfo();
  // Keep intrinsics consistent with the frame actually being published; a
  // calibration taken at a different resolution would otherwise go unnoticed.
  if (info.width == 0 || info.height == 0) {
    info.width = width_;
    info.height = height_;
  }
  info.header = header;
  m_pub_camera_info_->publish(info);
}

void ArenaCameraNode::publish_an_image_on_trigger_(
    std::shared_ptr<std_srvs::srv::Trigger::Request> request /*unused*/,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  if (!trigger_mode_activated_ || trigger_source_ != "Software") {
    std::string msg =
        "Failed to trigger image because Software trigger mode is not enabled."
        " Configure trigger_mode:=true and trigger_source:=Software to use this service.";
    log_warn(msg);
    response->message = msg;
    response->success = false;
    return;
  }

  log_info("A client triggered an image request");

  Arena::IImage* pImage = nullptr;
  try {
    // trigger
    bool triggerArmed = false;
    auto waitForTriggerCount = 10;
    do {
      // infinite loop when I step in (sometimes)
      triggerArmed =
          Arena::GetNodeValue<bool>(m_pDevice->GetNodeMap(), "TriggerArmed");

      if (triggerArmed == false && (waitForTriggerCount % 10) == 0) {
        log_info("waiting for trigger to be armed");
      }

    } while (triggerArmed == false);

    log_debug("trigger is armed; triggering an image");
    Arena::ExecuteNode(m_pDevice->GetNodeMap(), "TriggerSoftware");

    // get image
    auto p_image_msg = std::make_unique<sensor_msgs::msg::Image>();

    log_debug("getting an image");
    pImage = m_pDevice->GetImage(1000);
    auto msg = std::string("image ") + std::to_string(pImage->GetFrameId()) +
               " published to " + topic_;
    msg_form_image_(pImage, *p_image_msg);
    const auto header = p_image_msg->header;
    m_pub_->publish(std::move(p_image_msg));
    publish_camera_info_(header);
    response->message = msg;
    response->success = true;

    log_info(msg);
    this->m_pDevice->RequeueBuffer(pImage);

  }

  catch (std::exception& e) {
    if (pImage) {
      this->m_pDevice->RequeueBuffer(pImage);
      pImage = nullptr;
    }
    auto msg =
        std::string("Exception occurred while grabbing an image\n") + e.what();
    log_warn(msg);
    response->message = msg;
    response->success = false;

  }

  catch (GenICam::GenericException& e) {
    if (pImage) {
      this->m_pDevice->RequeueBuffer(pImage);
      pImage = nullptr;
    }
    auto msg =
        std::string("GenICam Exception occurred while grabbing an image\n") +
        e.what();
    log_warn(msg);
    response->message = msg;
    response->success = false;
  }
}

Arena::IDevice* ArenaCameraNode::create_device_ros_()
{
  update_devices_for_selection_();
  auto device_infos = m_pSystem->GetDevices();
  if (!device_infos.size()) {
    throw std::runtime_error("No Arena camera discovered");
  }

  log_info("Discovered " + std::to_string(device_infos.size()) + " camera(s)");

  auto index = 0;
  // Order must match initialize_()'s "Camera selection mode" log and
  // update_devices_for_selection_(), which only narrows discovery to a subnet
  // when an ip is given. Selecting by serial here while discovering by ip
  // could otherwise return a different camera than either setting names.
  if (is_passed_ip_) {
    if (is_passed_serial_) {
      log_warn("Both ip and serial were provided; selecting by ip=" + ip_ +
               " and ignoring serial=" + serial_);
    }
    log_info("Selecting camera by ip: " + ip_);
    index = DeviceInfoHelper::get_index_of_ip(device_infos, ip_);
  } else if (is_passed_serial_) {
    log_info("Selecting camera by serial: " + serial_);
    index = DeviceInfoHelper::get_index_of_serial(device_infos, serial_);
  } else {
    log_info("Selecting first discovered camera (no serial/ip provided)");
  }

  auto pDevice = m_pSystem->CreateDevice(device_infos.at(index));
  log_info(std::string("device created ") +
           DeviceInfoHelper::info(device_infos.at(index)));
  return pDevice;
}

void ArenaCameraNode::set_nodes_()
{
  set_nodes_load_default_profile_();
  // Transport and PTP first: packet sizing and clock discipline should be in
  // place before any buffers are sized or the stream starts.
  set_nodes_transport_();
  set_nodes_ptp_();
  // ADC bit depth and image flips change what the ROI and pixel format can be.
  set_nodes_image_quality_();
  set_nodes_roi_();
  set_nodes_pixelformat_();
  set_nodes_gain_();
  // Frame rate must be set before exposure: the camera derives the maximum
  // allowed ExposureAutoUpperLimit from the current frame rate, so applying
  // exposure first clamps it against the default rate (e.g. a 33.5 Hz default
  // caps the upper limit at ~29.7 ms, whereas 15 Hz allows ~66.6 ms).
  set_nodes_frame_rate_();
  set_nodes_exposure_();
  // Metering AOI is expressed in sensor coordinates, so it follows the ROI.
  set_nodes_ae_tuning_();
  set_nodes_trigger_mode_();
  // Auto-negotiate the packet size only when the user has not pinned one --
  // this runs after set_nodes_transport_(), so unconditionally enabling it
  // would let the SDK renegotiate over an explicit packet_size.
  if (packet_size_ <= 0) {
    Arena::SetNodeValue<bool>(m_pDevice->GetTLStreamNodeMap(),
                              "StreamAutoNegotiatePacketSize", true);
  }
  Arena::SetNodeValue<bool>(m_pDevice->GetTLStreamNodeMap(), "StreamPacketResendEnable", true);

  //set_nodes_test_pattern_image_();
}

void ArenaCameraNode::set_nodes_load_default_profile_()
{
  auto nodemap = m_pDevice->GetNodeMap();
  // device run on default profile all the time if no args are passed
  // otherwise, overwise only these params
  Arena::SetNodeValue<GenICam::gcstring>(nodemap, "UserSetSelector", "Default");
  // execute the profile
  Arena::ExecuteNode(nodemap, "UserSetLoad");
  log_info("\tdefault profile is loaded");
}

void ArenaCameraNode::set_nodes_roi_()
{
  auto nodemap = m_pDevice->GetNodeMap();

  // Width -------------------------------------------------
  if (is_passed_width) {
    Arena::SetNodeValue<int64_t>(nodemap, "Width", width_);
  } else {
    width_ = Arena::GetNodeValue<int64_t>(nodemap, "Width");
  }

  // Height ------------------------------------------------
  if (is_passed_height) {
    Arena::SetNodeValue<int64_t>(nodemap, "Height", height_);
  } else {
    height_ = Arena::GetNodeValue<int64_t>(nodemap, "Height");
  }

  // Offsets position a cropped ROI on the sensor. Without these, width/height
  // alone can only ever produce a top-left crop.
  if (offset_x_ >= 0) {
    set_int_clamped_(nodemap, "OffsetX", offset_x_);
  }
  if (offset_y_ >= 0) {
    set_int_clamped_(nodemap, "OffsetY", offset_y_);
  }

  // TODO only if it was passed by ros arg
  log_info(std::string("\tROI set to ") + std::to_string(width_) + "X" +
           std::to_string(height_));
}

void ArenaCameraNode::set_nodes_gain_()
{
  auto nodemap = m_pDevice->GetNodeMap();

  // A manual gain only sticks if the auto-gain algorithm is off first,
  // otherwise it is overwritten on the next frame.
  if (is_passed_gain_) {
    try_set_node_<GenICam::gcstring>(nodemap, "GainAuto", "Off");
    try_set_node_<double>(nodemap, "Gain", gain_);

    double applied = 0.0;
    const bool got = try_get_node_<double>(nodemap, "Gain", applied);
    log_info(std::string("\tManual gain enabled. Gain=") +
             (got ? std::to_string(applied) : std::string("<unreadable>")));
    return;
  }

  if (!auto_gain_) {
    try_set_node_<GenICam::gcstring>(nodemap, "GainAuto", "Off");
    log_info("\tAuto gain disabled");
    return;
  }

  if (!try_set_node_<GenICam::gcstring>(nodemap, "GainAuto", "Continuous")) {
    return;
  }

  // Only touch the limits the user actually asked for; the sensor's own
  // range is model specific.
  if (auto_gain_min_ >= 0.0) {
    set_float_clamped_(nodemap, "GainAutoLowerLimit", auto_gain_min_);
  }
  if (auto_gain_max_ >= 0.0) {
    set_float_clamped_(nodemap, "GainAutoUpperLimit", auto_gain_max_);
  }

  double applied_min = 0.0;
  double applied_max = 0.0;
  const bool got_min = try_get_node_<double>(nodemap, "GainAutoLowerLimit", applied_min);
  const bool got_max = try_get_node_<double>(nodemap, "GainAutoUpperLimit", applied_max);

  log_info(
      std::string("\tAuto gain enabled (read back from camera). min=") +
      (got_min ? std::to_string(applied_min) : std::string("<unreadable>")) +
      " max=" + (got_max ? std::to_string(applied_max) : std::string("<unreadable>")));
}

void ArenaCameraNode::set_nodes_pixelformat_()
{
  auto nodemap = m_pDevice->GetNodeMap();
  // TODO ---------------------------------------------------------------------
  // PIXEL FORMAT HANDLEING

  if (is_passed_pixelformat_ros_) {
    pixelformat_pfnc_ = K_ROS2_PIXELFORMAT_TO_PFNC[pixelformat_ros_];
    if (pixelformat_pfnc_.empty()) {
      throw std::invalid_argument("pixelformat is not supported!");
    }

    try {
      Arena::SetNodeValue<GenICam::gcstring>(nodemap, "PixelFormat",
                                             pixelformat_pfnc_.c_str());
      log_info(std::string("\tPixelFormat set to ") + pixelformat_pfnc_);

    } catch (GenICam::GenericException& e) {
      // TODO
      // an rcl expectation might be expected
      auto x = std::string("pixelformat is not supported by this camera");
      x.append(e.what());
      throw std::invalid_argument(x);
    }
  } else {
    pixelformat_pfnc_ =
        Arena::GetNodeValue<GenICam::gcstring>(nodemap, "PixelFormat");
    pixelformat_ros_ = K_PFNC_TO_ROS2_PIXELFORMAT[pixelformat_pfnc_];

    if (pixelformat_ros_.empty()) {
      log_warn(
          "the device current pixelfromat value is not supported by ROS2. "
          "please use --ros-args -p pixelformat:=\"<supported pixelformat>\".");
      // TODO
      // print list of supported pixelformats
    }
  }
}

void ArenaCameraNode::set_nodes_exposure_()
{
  auto nodemap = m_pDevice->GetNodeMap();

  if (is_passed_exposure_time_) {
    // ExposureTime is read-only while ExposureAuto is running, so disable it first.
    try_set_node_<GenICam::gcstring>(nodemap, "ExposureAuto", "Off");
    try_set_node_<double>(nodemap, "ExposureTime", exposure_time_);

    double applied = 0.0;
    const bool got = try_get_node_<double>(nodemap, "ExposureTime", applied);
    log_info(std::string("\tManual exposure enabled. ExposureTime=") +
             (got ? std::to_string(applied) : std::string("<unreadable>")));
    return;
  }

  if (!auto_exposure_) {
    try_set_node_<GenICam::gcstring>(nodemap, "ExposureAuto", "Off");
    log_info("\tAuto exposure disabled");
    return;
  }

  if (!try_set_node_<GenICam::gcstring>(nodemap, "ExposureAuto", "Continuous")) {
    return;
  }

  // ExposureAutoLowerLimit/UpperLimit are read-only while the camera is
  // managing the limits itself, so this must be turned off *first* or both
  // writes below are rejected.
  try_set_node_<GenICam::gcstring>(nodemap, "ExposureAutoLimitAuto", "Off");

  set_float_clamped_(nodemap, "ExposureAutoLowerLimit", auto_exposure_min_us_);
  set_float_clamped_(nodemap, "ExposureAutoUpperLimit", auto_exposure_max_us_);

  // TargetBrightness is an integer node; passing a double throws.
  try_set_node_<int64_t>(nodemap, "TargetBrightness",
                         static_cast<int64_t>(std::lround(auto_exposure_target_)));

  // Report what the camera actually holds, not what we asked for.
  double applied_min = 0.0;
  double applied_max = 0.0;
  int64_t applied_target = 0;
  const bool got_min = try_get_node_<double>(nodemap, "ExposureAutoLowerLimit", applied_min);
  const bool got_max = try_get_node_<double>(nodemap, "ExposureAutoUpperLimit", applied_max);
  const bool got_target = try_get_node_<int64_t>(nodemap, "TargetBrightness", applied_target);

  log_info(
      std::string("\tAuto exposure enabled (read back from camera). min_us=") +
      (got_min ? std::to_string(applied_min) : std::string("<unreadable>")) +
      " max_us=" + (got_max ? std::to_string(applied_max) : std::string("<unreadable>")) +
      " target_brightness=" +
      (got_target ? std::to_string(applied_target) : std::string("<unreadable>")));
}

void ArenaCameraNode::set_nodes_frame_rate_()
{
  if (!is_passed_frame_rate_hz_) {
    return;
  }

  auto nodemap = m_pDevice->GetNodeMap();

  // AcquisitionFrameRate is read-only until this toggle is enabled.
  try_set_node_<bool>(nodemap, "AcquisitionFrameRateEnable", true);

  if (!try_set_node_<double>(nodemap, "AcquisitionFrameRate", frame_rate_hz_)) {
    return;
  }

  double applied = 0.0;
  const bool got = try_get_node_<double>(nodemap, "AcquisitionFrameRate", applied);
  log_info(std::string("\tFrame rate capped to ") +
           (got ? std::to_string(applied) : std::to_string(frame_rate_hz_)) + " Hz");
}

void ArenaCameraNode::set_nodes_ae_tuning_()
{
  auto nodemap = m_pDevice->GetNodeMap();

  if (!exposure_auto_algorithm_.empty()) {
    try_set_node_<GenICam::gcstring>(nodemap, "ExposureAutoAlgorithm",
                                     exposure_auto_algorithm_.c_str());
  }

  // Damping controls how quickly auto exposure chases a change in scene
  // brightness -- lower reacts faster, higher is steadier.
  if (exposure_auto_damping_ >= 0.0) {
    set_float_clamped_(nodemap, "ExposureAutoDamping", exposure_auto_damping_);
  }

  if (!ae_aoi_enable_) {
    // Only write when disabling explicitly; the sub-nodes are unavailable
    // until the AOI is switched on.
    try_set_node_<bool>(nodemap, "AutoExposureAOIEnable", false);
    return;
  }

  if (!try_set_node_<bool>(nodemap, "AutoExposureAOIEnable", true)) {
    return;
  }

  // Offsets before sizes: a stale large size can block a new offset.
  if (ae_aoi_offset_x_ >= 0) {
    set_int_clamped_(nodemap, "AutoExposureAOIOffsetX", ae_aoi_offset_x_);
  }
  if (ae_aoi_offset_y_ >= 0) {
    set_int_clamped_(nodemap, "AutoExposureAOIOffsetY", ae_aoi_offset_y_);
  }
  if (ae_aoi_width_ > 0) {
    set_int_clamped_(nodemap, "AutoExposureAOIWidth", ae_aoi_width_);
  }
  if (ae_aoi_height_ > 0) {
    set_int_clamped_(nodemap, "AutoExposureAOIHeight", ae_aoi_height_);
  }

  log_info("\tAuto exposure metering AOI enabled");
}

void ArenaCameraNode::set_nodes_image_quality_()
{
  auto nodemap = m_pDevice->GetNodeMap();

  if (!adc_bit_depth_.empty()) {
    try_set_node_<GenICam::gcstring>(nodemap, "ADCBitDepth", adc_bit_depth_.c_str());
  }

  // Written unconditionally: writing only on true would make reverse_x:=false
  // a silent no-op that depends on the loaded user set already being unflipped.
  try_set_node_<bool>(nodemap, "ReverseX", reverse_x_);
  try_set_node_<bool>(nodemap, "ReverseY", reverse_y_);

  if (!balance_white_auto_.empty()) {
    try_set_node_<GenICam::gcstring>(nodemap, "BalanceWhiteAuto",
                                     balance_white_auto_.c_str());
  }

  if (gamma_ >= 0.0) {
    // Gamma is ignored unless the correction block is enabled.
    try_set_node_<bool>(nodemap, "GammaEnable", true);
    set_float_clamped_(nodemap, "Gamma", gamma_);
  }

  if (black_level_ >= 0.0) {
    set_float_clamped_(nodemap, "BlackLevel", black_level_);
  }
}

void ArenaCameraNode::set_nodes_transport_()
{
  auto nodemap = m_pDevice->GetNodeMap();

  // Larger packets cut per-packet overhead substantially, but the whole path
  // (NIC MTU and any switch) must carry them or every frame is lost.
  //
  // Note ICMP is not a valid probe here: these cameras cap their control stack
  // at a 1500 byte MTU (a 9000 byte ping fails) while the streaming path will
  // happily emit 9000 byte packets, so test with actual streaming.
  if (packet_size_ > 0) {
    // This toggle is on the stream nodemap, not the device nodemap -- setting
    // it on the device nodemap silently does nothing.
    try_set_node_<bool>(m_pDevice->GetTLStreamNodeMap(),
                        "StreamAutoNegotiatePacketSize", false);
    set_int_clamped_(nodemap, "GevSCPSPacketSize", packet_size_);
  }

  // Always report the size actually in force: with auto-negotiation enabled the
  // SDK picks this, so the only trustworthy value is the one read back.
  int64_t applied_packet_size = 0;
  if (try_get_node_<int64_t>(nodemap, "GevSCPSPacketSize", applied_packet_size)) {
    log_info(std::string("\tStream packet size in force: ") +
             std::to_string(applied_packet_size) + " bytes" +
             (packet_size_ > 0 ? " (explicit)" : " (auto-negotiated)"));
  }

  // Inter-packet delay paces the camera so a shared link or a slower host
  // does not drop bursts.
  if (packet_delay_ >= 0) {
    set_int_clamped_(nodemap, "GevSCPD", packet_delay_);
  }

  if (throughput_limit_ > 0) {
    if (try_set_node_<GenICam::gcstring>(nodemap, "DeviceLinkThroughputLimitMode", "On")) {
      set_int_clamped_(nodemap, "DeviceLinkThroughputLimit", throughput_limit_);
    }
  }

  // Buffer handling lives on the stream nodemap, not the device nodemap.
  if (!stream_buffer_handling_mode_.empty()) {
    try_set_node_<GenICam::gcstring>(m_pDevice->GetTLStreamNodeMap(),
                                     "StreamBufferHandlingMode",
                                     stream_buffer_handling_mode_.c_str());
  }
}

void ArenaCameraNode::set_nodes_ptp_()
{
  auto nodemap = m_pDevice->GetNodeMap();

  if (!ptp_enable_) {
    try_set_node_<bool>(nodemap, "PtpEnable", false);
    return;
  }

  if (!try_set_node_<bool>(nodemap, "PtpEnable", true)) {
    return;
  }

  // Status is only meaningful once the servo has had time to settle; report
  // whatever it says now so a misconfigured PTP network is visible early.
  GenICam::gcstring status;
  if (try_get_node_<GenICam::gcstring>(nodemap, "PtpStatus", status)) {
    log_info(std::string("\tPTP enabled. PtpStatus=") + status.c_str() +
             " (Master/Slave once locked; Listening or Disabled means no grandmaster yet)");
  } else {
    log_info("\tPTP enabled");
  }
}

void ArenaCameraNode::set_nodes_trigger_mode_()
{
  auto nodemap = m_pDevice->GetNodeMap();
  if (trigger_mode_activated_) {
    if (exposure_time_ < 0) {
      log_warn(
          "\tavoid long waits wating for triggered images by providing proper "
          "exposure_time.");
    }
    // Enable trigger mode before setting the source and selector
    // and before starting the stream. Trigger mode cannot be turned
    // on and off while the device is streaming.

    // Make sure Trigger Mode set to 'Off' after finishing this example
    Arena::SetNodeValue<GenICam::gcstring>(nodemap, "TriggerMode", "On");

    Arena::SetNodeValue<GenICam::gcstring>(nodemap, "TriggerSource",
                                           trigger_source_.c_str());
    Arena::SetNodeValue<GenICam::gcstring>(nodemap, "TriggerSelector",
                                           trigger_selector_.c_str());

    if (!trigger_activation_.empty()) {
      try {
        Arena::SetNodeValue<GenICam::gcstring>(nodemap, "TriggerActivation",
                                               trigger_activation_.c_str());
      } catch (GenICam::GenericException & e) {
        log_warn(std::string("\tUnable to set TriggerActivation to ") +
                 trigger_activation_ + ": " + e.what());
      }
    }

    if (trigger_source_ == "Software") {
      auto msg =
          std::string(
              "\ttrigger_mode is activated with TriggerSource=Software. To trigger an image run `ros2 service call /") +
          this->get_name() +
          "/trigger_image std_srvs/srv/Trigger`";
      log_warn(msg);
    } else {
      log_info(std::string("\ttrigger_mode is activated with TriggerSource=") +
               trigger_source_ + ", TriggerSelector=" + trigger_selector_ +
               (trigger_activation_.empty() ? "" : ", TriggerActivation=" + trigger_activation_));
    }
  }
  // unset device from being in trigger mode if user did not pass trigger
  // mode parameter because the trigger nodes are not rest when loading
  // the user default profile
  else {
    Arena::SetNodeValue<GenICam::gcstring>(nodemap, "TriggerMode", "Off");
  }
}

// just for debugging
void ArenaCameraNode::set_nodes_test_pattern_image_()
{
  auto nodemap = m_pDevice->GetNodeMap();
  Arena::SetNodeValue<GenICam::gcstring>(nodemap, "TestPattern", "Pattern3");
}
