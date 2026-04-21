/*
 * flir_ptu_driver ROS package
 * Copyright (C) 2014 Mike Purvis (mpurvis@clearpathrobotics.com)
 *
 * PTU ROS Package
 * Copyright (C) 2009 Erik Karulf (erik@cse.wustl.edu)
 *
 * Author: Toby Collett (University of Auckland)
 * Date: 2003-02-10
 *
 * Player - One Hell of a Robot Server
 * Copyright (C) 2000  Brian Gerkey   &  Kasper Stoy
 *                     gerkey@usc.edu    kaspers@robotics.usc.edu
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */

#include <memory>
#include <string>

#include <flir_ptu_driver/driver.h>
#include <flir_ptu_driver/serial_transport.h>
#include <flir_ptu_driver/tcp_transport.h>
#include <diagnostic_updater/diagnostic_updater.hpp>
#include <diagnostic_updater/publisher.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/empty.hpp>

namespace flir_ptu_driver
{

class PtuNode : public rclcpp::Node
{
public:
  PtuNode() : rclcpp::Node("ptu_driver"), m_pantilt(nullptr)
  {
    // Declare parameters
    this->declare_parameter<std::string>("connection_type", "tcp");
    this->declare_parameter<std::string>("port", PTU_SERIAL_DEFAULT_PORT);
    this->declare_parameter<int>("baud", PTU_SERIAL_DEFAULT_BAUD);
    this->declare_parameter<std::string>("ip_addr", PTU_DEFAULT_TCP_IP);
    this->declare_parameter<int>("tcp_port", PTU_DEFAULT_TCP_PORT);
    this->declare_parameter<bool>("limits_enabled", true);
    this->declare_parameter<double>("default_velocity", PTU_DEFAULT_VEL);
    this->declare_parameter<int>("hz", PTU_DEFAULT_HZ);
    this->declare_parameter<std::string>("joint_name_prefix", "ptu_");
    this->declare_parameter<bool>("invert_pan", true);

    m_joint_name_prefix = this->get_parameter("joint_name_prefix").as_string();
    m_invert_pan = this->get_parameter("invert_pan").as_bool();
    default_velocity_ = this->get_parameter("default_velocity").as_double();

    // Diagnostic updater
    m_updater = std::make_unique<diagnostic_updater::Updater>(this);
    m_updater->setHardwareID("unknown");
    m_updater->add("PTU Status", this, &PtuNode::produce_diagnostics);
  }

  void connect()
  {
    // If reconnecting, disconnect first
    if (ok())
    {
      disconnect();
    }

    std::string connection_type = this->get_parameter("connection_type").as_string();

    std::unique_ptr<Transport> transport;

    if (connection_type == "tcp")
    {
      std::string ip_addr = this->get_parameter("ip_addr").as_string();
      int tcp_port = this->get_parameter("tcp_port").as_int();

      RCLCPP_INFO(
        this->get_logger(), "Attempting to connect to FLIR PTU via TCP on %s:%d", ip_addr.c_str(),
        tcp_port);

      auto tcp = std::make_unique<TcpTransport>(ip_addr, tcp_port);
      if (!tcp->open())
      {
        RCLCPP_ERROR(this->get_logger(), "Unable to connect to %s:%d", ip_addr.c_str(), tcp_port);
        return;
      }

      // Drain the banner that the PTU sends on TCP connect
      tcp->drainBanner();
      transport = std::move(tcp);

      m_connection_type = "tcp";
      m_connection_endpoint = ip_addr + ":" + std::to_string(tcp_port);
    }
    else if (connection_type == "tty")
    {
      std::string port = this->get_parameter("port").as_string();
      int baud = this->get_parameter("baud").as_int();

      RCLCPP_INFO(
        this->get_logger(), "Attempting to connect to FLIR PTU via serial on %s at %d baud",
        port.c_str(), baud);

      auto serial = std::make_unique<SerialTransport>(port, baud);
      if (!serial->open())
      {
        RCLCPP_ERROR(this->get_logger(), "Unable to open port %s", port.c_str());
        return;
      }
      transport = std::move(serial);

      m_connection_type = "tty";
      m_connection_endpoint = port + "@" + std::to_string(baud);
    }
    else
    {
      RCLCPP_ERROR(
        this->get_logger(), "Unknown connection_type: '%s' (use 'tty' or 'tcp')",
        connection_type.c_str());
      return;
    }

    RCLCPP_INFO(this->get_logger(), "FLIR PTU port opened, now initializing.");

    m_pantilt = std::make_unique<PTU>(std::move(transport));

    if (!m_pantilt->initialize())
    {
      RCLCPP_ERROR(this->get_logger(), "Could not initialize FLIR PTU.");
      disconnect();
      return;
    }

    bool limit = this->get_parameter("limits_enabled").as_bool();
    if (!limit)
    {
      m_pantilt->disableLimits();
      RCLCPP_INFO(this->get_logger(), "FLIR PTU limits disabled.");
    }
    m_limits_enabled = limit;

    // Update hardware ID now that we know what we're talking to.
    m_updater->setHardwareID("flir_ptu:" + m_connection_endpoint);

    // Cache the firmware/version banner so it can be reported via diagnostics
    // and logged on startup.
    m_firmware_version = m_pantilt->getVersion();
    if (!m_firmware_version.empty())
    {
      RCLCPP_INFO(this->get_logger(), "FLIR PTU firmware: %s", m_firmware_version.c_str());
    }

    RCLCPP_INFO(this->get_logger(), "FLIR PTU initialized.");

    // Log limits
    RCLCPP_INFO(
      this->get_logger(), "Pan range: [%.4f, %.4f] rad, speed: [%.4f, %.4f] rad/s",
      m_pantilt->getMin(PTU_PAN), m_pantilt->getMax(PTU_PAN), m_pantilt->getMinSpeed(PTU_PAN),
      m_pantilt->getMaxSpeed(PTU_PAN));
    RCLCPP_INFO(
      this->get_logger(), "Tilt range: [%.4f, %.4f] rad, speed: [%.4f, %.4f] rad/s",
      m_pantilt->getMin(PTU_TILT), m_pantilt->getMax(PTU_TILT), m_pantilt->getMinSpeed(PTU_TILT),
      m_pantilt->getMaxSpeed(PTU_TILT));

    // Publishers
    m_joint_pub = this->create_publisher<sensor_msgs::msg::JointState>("state", 1);

    // Frequency diagnostic on published joint_states (tolerate +/- 10%).
    int hz = this->get_parameter("hz").as_int();
    m_expected_hz_min = static_cast<double>(hz) * 0.9;
    m_expected_hz_max = static_cast<double>(hz) * 1.1;
    m_joint_freq = std::make_shared<diagnostic_updater::HeaderlessTopicDiagnostic>(
      "joint_states", *m_updater,
      diagnostic_updater::FrequencyStatusParam(&m_expected_hz_min, &m_expected_hz_max, 0.1, 5));

    // Subscribers
    m_joint_sub = this->create_subscription<sensor_msgs::msg::JointState>(
      "cmd", 1, std::bind(&PtuNode::cmdCallback, this, std::placeholders::_1));

    m_recalibrate_sub = this->create_subscription<std_msgs::msg::Empty>(
      "recalibrate", 1, std::bind(&PtuNode::recalibrateCallback, this, std::placeholders::_1));

    // Timer
    m_timer = this->create_wall_timer(
      std::chrono::milliseconds(1000 / hz), std::bind(&PtuNode::spinCallback, this));
  }

  bool ok() const { return m_pantilt != nullptr; }

  void disconnect()
  {
    m_timer.reset();
    m_joint_pub.reset();
    m_joint_sub.reset();
    m_recalibrate_sub.reset();
    m_joint_freq.reset();
    m_pantilt.reset();
  }

private:
  void cmdCallback(const sensor_msgs::msg::JointState::ConstSharedPtr msg)
  {
    RCLCPP_DEBUG(this->get_logger(), "PTU command callback.");
    if (!ok())
    {
      return;
    }

    if (msg->position.size() != 2)
    {
      RCLCPP_ERROR(
        this->get_logger(), "JointState command to PTU has wrong number of position elements.");
      return;
    }

    double pan = msg->position[0];
    double tilt = msg->position[1];
    double panspeed, tiltspeed;

    if (msg->velocity.size() == 2)
    {
      panspeed = msg->velocity[0];
      tiltspeed = msg->velocity[1];
    }
    else
    {
      RCLCPP_WARN_ONCE(
        this->get_logger(),
        "JointState command to PTU has wrong number of velocity elements; using default velocity.");
      panspeed = default_velocity_;
      tiltspeed = default_velocity_;
    }

    if (m_invert_pan)
    {
      pan = -pan;
    }

    m_pantilt->setPosition(PTU_PAN, pan);
    m_pantilt->setPosition(PTU_TILT, tilt);
    m_pantilt->setSpeed(PTU_PAN, panspeed);
    m_pantilt->setSpeed(PTU_TILT, tiltspeed);
  }

  void recalibrateCallback(const std_msgs::msg::Empty::ConstSharedPtr /*msg*/)
  {
    RCLCPP_INFO(this->get_logger(), "Recalibrating the PTU");
    m_pantilt->home();
  }

  void produce_diagnostics(diagnostic_updater::DiagnosticStatusWrapper & stat)
  {
    if (!ok())
    {
      stat.summary(diagnostic_updater::DiagnosticStatusWrapper::ERROR, "Not connected");
      stat.add("Connection type", m_connection_type);
      stat.add("Endpoint", m_connection_endpoint);
      stat.add("Firmware", m_firmware_version.empty() ? "unknown" : m_firmware_version);
      return;
    }

    if (m_comm_errors > 0)
    {
      stat.summary(
        diagnostic_updater::DiagnosticStatusWrapper::WARN,
        "Connected, recent communication errors");
    }
    else
    {
      stat.summary(diagnostic_updater::DiagnosticStatusWrapper::OK, "Connected");
    }

    stat.add("Connection type", m_connection_type);
    stat.add("Endpoint", m_connection_endpoint);
    stat.add("Firmware", m_firmware_version.empty() ? "unknown" : m_firmware_version);
    stat.add("Limits enabled", m_limits_enabled);
    stat.add(
      "PTU Mode", m_last_mode == PTU_POSITION
                    ? "Position"
                    : (m_last_mode == PTU_VELOCITY ? "Velocity" : "Unknown"));
    stat.add("Pan position (rad)", m_last_pan);
    stat.add("Tilt position (rad)", m_last_tilt);
    stat.add("Pan velocity (rad/s)", m_last_pan_speed);
    stat.add("Tilt velocity (rad/s)", m_last_tilt_speed);
    stat.add(
      "Pan range (rad)", std::to_string(m_pantilt->getMin(PTU_PAN)) + " to " +
                           std::to_string(m_pantilt->getMax(PTU_PAN)));
    stat.add(
      "Tilt range (rad)", std::to_string(m_pantilt->getMin(PTU_TILT)) + " to " +
                            std::to_string(m_pantilt->getMax(PTU_TILT)));
    stat.add("Communication errors", m_comm_errors);
  }

  void spinCallback()
  {
    if (!ok())
    {
      return;
    }

    // Read Position & Speed
    double pan = m_pantilt->getPosition(PTU_PAN);
    double tilt = m_pantilt->getPosition(PTU_TILT);

    double panspeed = m_pantilt->getSpeed(PTU_PAN);
    double tiltspeed = m_pantilt->getSpeed(PTU_TILT);

    // Track communication health. getPosition()/getSpeed() return -1 on error.
    const bool comm_ok = (pan != -1.0 && tilt != -1.0 && panspeed != -1.0 && tiltspeed != -1.0);
    if (!comm_ok)
    {
      m_comm_errors++;
    }
    else
    {
      if (m_invert_pan)
      {
        pan = -pan;
        panspeed = -panspeed;
      }
      // Cache latest values so diagnostics don't re-query the hardware.
      m_last_pan = pan;
      m_last_tilt = tilt;
      m_last_pan_speed = panspeed;
      m_last_tilt_speed = tiltspeed;
    }
    m_last_mode = m_pantilt->getMode();

    // Publish Position & Speed
    sensor_msgs::msg::JointState joint_state;
    joint_state.header.stamp = this->now();
    joint_state.name.resize(2);
    joint_state.position.resize(2);
    joint_state.velocity.resize(2);
    joint_state.name[0] = m_joint_name_prefix + "pan";
    joint_state.position[0] = pan;
    joint_state.velocity[0] = panspeed;
    joint_state.name[1] = m_joint_name_prefix + "tilt";
    joint_state.position[1] = tilt;
    joint_state.velocity[1] = tiltspeed;
    m_joint_pub->publish(joint_state);

    // Tick the joint_states frequency monitor so diagnostics can flag stalls.
    if (m_joint_freq)
    {
      m_joint_freq->tick();
    }

    m_updater->force_update();
  }

  std::unique_ptr<diagnostic_updater::Updater> m_updater;
  std::shared_ptr<diagnostic_updater::HeaderlessTopicDiagnostic> m_joint_freq;
  std::unique_ptr<PTU> m_pantilt;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr m_joint_pub;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr m_joint_sub;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr m_recalibrate_sub;
  rclcpp::TimerBase::SharedPtr m_timer;

  std::string m_joint_name_prefix;
  double default_velocity_;
  bool m_invert_pan{true};

  // Cached state for diagnostics.
  std::string m_connection_type{"unknown"};
  std::string m_connection_endpoint;
  std::string m_firmware_version;
  bool m_limits_enabled{true};
  double m_last_pan{0.0};
  double m_last_tilt{0.0};
  double m_last_pan_speed{0.0};
  double m_last_tilt_speed{0.0};
  signed char m_last_mode{-1};
  size_t m_comm_errors{0};
  double m_expected_hz_min{1.0};
  double m_expected_hz_max{1.0};
};

}  // namespace flir_ptu_driver

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<flir_ptu_driver::PtuNode>();
  node->connect();

  if (!node->ok())
  {
    RCLCPP_ERROR(node->get_logger(), "Failed to connect to FLIR PTU. Exiting.");
    return 1;
  }

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
