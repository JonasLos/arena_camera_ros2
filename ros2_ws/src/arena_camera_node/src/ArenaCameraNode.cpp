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
    if (this->has_parameter("serial")) {
        int serial_integer;
        this->get_parameter<int>("serial", serial_integer);
        serial_ = std::to_string(serial_integer);
        is_passed_serial_ = true;
} else {
    serial_ = ""; // Set it to an empty string to indicate it's not passed.
    is_passed_serial_ = false;
}

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

    nextParameterToDeclare = "qos_reliability";
    pub_qos_reliability_ = this->declare_parameter("qos_reliability", "");
    is_passed_pub_qos_reliability_ = pub_qos_reliability_ != "";

  } catch (rclcpp::ParameterTypeException& e) {
    log_err(nextParameterToDeclare + " argument");
    throw;
  }
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
  // QoS depth
  if (is_passed_pub_qos_history_depth_ &&
      K_CMDLN_PARAMETER_TO_QOS_HISTORY_POLICY[pub_qos_history_] ==
          RMW_QOS_POLICY_HISTORY_KEEP_LAST) {
    // TODO
    // test err msg withwhen -1
    pub_qos_.keep_last(pub_qos_history_depth_);
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
  set_nodes_();
  m_pDevice->StartStream();

  if (!trigger_mode_activated_ || trigger_source_ != "Software") {
    publish_images_();
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
        const uint64_t current_timestamp_ns = pImage->GetTimestampNs();
        if (last_publish_timestamp_ns_ > 0 &&
            (current_timestamp_ns - last_publish_timestamp_ns_) < min_publish_period_ns_) {
          this->m_pDevice->RequeueBuffer(pImage);
          pImage = nullptr;
          continue;
        }
        last_publish_timestamp_ns_ = current_timestamp_ns;
      }

      auto p_image_msg = std::make_unique<sensor_msgs::msg::Image>();
      msg_form_image_(pImage, *p_image_msg);

      m_pub_->publish(std::move(p_image_msg));

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
    image_msg.header.stamp.sec =
        static_cast<uint32_t>(pImage->GetTimestampNs() / 1000000000);
    image_msg.header.stamp.nanosec =
        static_cast<uint32_t>(pImage->GetTimestampNs() % 1000000000);
    image_msg.header.frame_id = std::to_string(pImage->GetFrameId());

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
    m_pub_->publish(std::move(p_image_msg));
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
  if (is_passed_serial_) {
    log_info("Selecting camera by serial: " + serial_);
    index = DeviceInfoHelper::get_index_of_serial(device_infos, serial_);
  } else if (is_passed_ip_) {
    log_info("Selecting camera by ip: " + ip_);
    index = DeviceInfoHelper::get_index_of_ip(device_infos, ip_);
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
  set_nodes_roi_();
  set_nodes_gain_();
  set_nodes_pixelformat_();
  set_nodes_exposure_();
  set_nodes_frame_rate_();
  set_nodes_trigger_mode_();
  // configure Auto Negotiate Packet Size and Packet Resend
  Arena::SetNodeValue<bool>(m_pDevice->GetTLStreamNodeMap(), "StreamAutoNegotiatePacketSize", true);
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

  // TODO only if it was passed by ros arg
  log_info(std::string("\tROI set to ") + std::to_string(width_) + "X" +
           std::to_string(height_));
}

void ArenaCameraNode::set_nodes_gain_()
{
  if (is_passed_gain_) {  // not default
    auto nodemap = m_pDevice->GetNodeMap();
    Arena::SetNodeValue<double>(nodemap, "Gain", gain_);
    log_info(std::string("\tGain set to ") + std::to_string(gain_));
  }
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
    Arena::SetNodeValue<GenICam::gcstring>(nodemap, "ExposureAuto", "Off");
    Arena::SetNodeValue<double>(nodemap, "ExposureTime", exposure_time_);
    log_info(std::string("\tManual exposure enabled. ExposureTime=") + std::to_string(exposure_time_));
    return;
  }

  if (!auto_exposure_) {
    Arena::SetNodeValue<GenICam::gcstring>(nodemap, "ExposureAuto", "Off");
    log_info("\tAuto exposure disabled");
    return;
  }

  try {
    Arena::SetNodeValue<GenICam::gcstring>(nodemap, "ExposureAuto", "Continuous");
  } catch (GenICam::GenericException & e) {
    log_warn(std::string("\tUnable to set ExposureAuto=Continuous: ") + e.what());
    return;
  }

  try {
    Arena::SetNodeValue<double>(nodemap, "AutoExposureExposureTimeLowerLimit", auto_exposure_min_us_);
  } catch (GenICam::GenericException & e) {
    (void)e;
  }

  try {
    Arena::SetNodeValue<double>(nodemap, "AutoExposureExposureTimeUpperLimit", auto_exposure_max_us_);
  } catch (GenICam::GenericException & e) {
    (void)e;
  }

  try {
    Arena::SetNodeValue<double>(nodemap, "AutoExposureTargetGreyValue", auto_exposure_target_);
  } catch (GenICam::GenericException & e) {
    (void)e;
  }

  log_info(
      std::string("\tAuto exposure enabled. min_us=") + std::to_string(auto_exposure_min_us_) +
      " max_us=" + std::to_string(auto_exposure_max_us_) +
      " target=" + std::to_string(auto_exposure_target_));
}

void ArenaCameraNode::set_nodes_frame_rate_()
{
  if (!is_passed_frame_rate_hz_) {
    return;
  }

  auto nodemap = m_pDevice->GetNodeMap();
  try {
    Arena::SetNodeValue<bool>(nodemap, "AcquisitionFrameRateEnable", true);
  } catch (GenICam::GenericException & e) {
    (void)e;
    // Some cameras do not expose this toggle; continue and try setting rate directly.
  }

  try {
    Arena::SetNodeValue<double>(nodemap, "AcquisitionFrameRate", frame_rate_hz_);
    log_info(std::string("\tFrame rate capped to ") + std::to_string(frame_rate_hz_) + " Hz");
    return;
  } catch (GenICam::GenericException & e) {
    (void)e;
  }

  try {
    Arena::SetNodeValue<double>(nodemap, "AcquisitionFrameRateAbs", frame_rate_hz_);
    log_info(std::string("\tFrame rate capped to ") + std::to_string(frame_rate_hz_) + " Hz");
  } catch (GenICam::GenericException & e) {
    log_warn(
      std::string("\tUnable to set frame_rate to ") + std::to_string(frame_rate_hz_) +
      " Hz (camera nodemap does not expose AcquisitionFrameRate/AcquisitionFrameRateAbs): " +
      e.what());
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
