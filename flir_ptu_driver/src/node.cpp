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

#include <diagnostic_updater/diagnostic_updater.h>
#include <diagnostic_updater/publisher.h>
#include <flir_ptu_driver/driver.h>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <serial/serial.h>
#include <std_msgs/Bool.h>

#include <string>

namespace flir_ptu_driver
{

class Node
{
public:
  explicit Node(ros::NodeHandle& node_handle);
  ~Node();

  // Service Control
  void connect();
  bool ok()
  {
    return m_pantilt != NULL;
  }
  void disconnect();

  // Service Execution
  void spinCallback(const ros::TimerEvent&);

  // Callback Methods
  void cmdCallback(const sensor_msgs::JointState::ConstPtr& msg);
  void resetCallback(const std_msgs::Bool::ConstPtr& msg);

  void produce_diagnostics(diagnostic_updater::DiagnosticStatusWrapper &stat);

protected:
  diagnostic_updater::Updater* m_updater;
  PTU* m_pantilt;
  ros::NodeHandle m_node;
  ros::Publisher  m_joint_pub;
  ros::Subscriber m_joint_sub;
  ros::Subscriber m_reset_sub;

  serial::Serial m_ser;
  std::string m_joint_name_prefix;
  double default_velocity_;
};

Node::Node(ros::NodeHandle& node_handle)
  : m_pantilt(NULL), m_node(node_handle)
{
  m_updater = new diagnostic_updater::Updater();
  m_updater->setHardwareID("none");
  m_updater->add("PTU Status", this, &Node::produce_diagnostics);

  ros::param::param<std::string>("~joint_name_prefix", m_joint_name_prefix, "ptu_");
}

Node::~Node()
{
  disconnect();
  delete m_updater;
}

/** Opens the connection to the PTU and sets appropriate parameters.
    Also manages subscriptions/publishers */
void Node::connect()
{
  // If we are reconnecting, first make sure to disconnect
  if (ok())
  {
    disconnect();
  }

  // Query for serial configuration
  std::string port;
  int32_t baud;
  bool limit;
  ros::param::param<std::string>("~port", port, PTU_DEFAULT_PORT);
  ros::param::param<bool>("~limits_enabled", limit, true);
  ros::param::param<int32_t>("~baud", baud, PTU_DEFAULT_BAUD);
  ros::param::param<double>("~default_velocity", default_velocity_, PTU_DEFAULT_VEL);

  // Connect to the PTU
  ROS_INFO_STREAM("Attempting to connect to FLIR PTU on " << port);

  try
  {
    m_ser.setPort(port);
    m_ser.setBaudrate(baud);
    serial::Timeout to = serial::Timeout(200, 200, 0, 200, 0);
    m_ser.setTimeout(to);
    m_ser.open();
  }
  catch (serial::IOException& e)
  {
    ROS_ERROR_STREAM("Unable to open port " << port);
    return;
  }

  ROS_INFO_STREAM("FLIR PTU serial port opened, now initializing.");

  m_pantilt = new PTU(&m_ser);

  if (!m_pantilt->initialize())
  {
    ROS_ERROR_STREAM("Could not initialize FLIR PTU on " << port);
    disconnect();
    return;
  }

  if (!limit)
  {
    m_pantilt->disableLimits();
    ROS_INFO("FLIR PTU limits disabled.");
  }

  ROS_INFO("FLIR PTU initialized.");

  m_node.setParam("min_tilt", m_pantilt->getMin(PTU_TILT));
  m_node.setParam("max_tilt", m_pantilt->getMax(PTU_TILT));
  m_node.setParam("min_tilt_speed", m_pantilt->getMinSpeed(PTU_TILT));
  m_node.setParam("max_tilt_speed", m_pantilt->getMaxSpeed(PTU_TILT));
  m_node.setParam("tilt_step", m_pantilt->getResolution(PTU_TILT));

  m_node.setParam("min_pan", m_pantilt->getMin(PTU_PAN));
  m_node.setParam("max_pan", m_pantilt->getMax(PTU_PAN));
  m_node.setParam("min_pan_speed", m_pantilt->getMinSpeed(PTU_PAN));
  m_node.setParam("max_pan_speed", m_pantilt->getMaxSpeed(PTU_PAN));
  m_node.setParam("pan_step", m_pantilt->getResolution(PTU_PAN));

  // Publishers : Only publish the most recent reading
  m_joint_pub = m_node.advertise
                <sensor_msgs::JointState>("state", 1);

  // Subscribers : Only subscribe to the most recent instructions
  m_joint_sub = m_node.subscribe
                <sensor_msgs::JointState>("cmd", 1, &Node::cmdCallback, this);

  m_reset_sub = m_node.subscribe
                <std_msgs::Bool>("reset", 1, &Node::resetCallback, this);
}

/** Disconnect */
void Node::disconnect()
{
  if (m_pantilt != NULL)
  {
    delete m_pantilt;   // Closes the connection
    m_pantilt = NULL;   // Marks the service as disconnected
  }
}

/** Callback for resetting PTU */
void Node::resetCallback(const std_msgs::Bool::ConstPtr& msg)
{
  ROS_INFO("Resetting the PTU");
  m_pantilt->home();
}

/** Callback for getting new Goal JointState */
void Node::cmdCallback(const sensor_msgs::JointState::ConstPtr& msg)
{
  ROS_DEBUG("PTU command callback.");
  if (!ok()) return;

  if (msg->position.size() != 2)
  {
    ROS_ERROR("JointState command to PTU has wrong number of position elements.");
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
    ROS_WARN_ONCE("JointState command to PTU has wrong number of velocity elements; using default velocity.");
    panspeed = default_velocity_;
    tiltspeed = default_velocity_;
  }

  m_pantilt->setPosition(PTU_PAN, pan);
  m_pantilt->setPosition(PTU_TILT, tilt);
  m_pantilt->setSpeed(PTU_PAN, panspeed);
  m_pantilt->setSpeed(PTU_TILT, tiltspeed);
}

void Node::produce_diagnostics(diagnostic_updater::DiagnosticStatusWrapper &stat)
{
  stat.summary(diagnostic_msgs::DiagnosticStatus::OK, "All normal.");
  stat.add("PTU Mode", m_pantilt->getMode() == PTU_POSITION ? "Position" : "Velocity");
}


/**
 * Publishes a joint_state message with position and speed.
 * Also sends out updated TF info.
 */
void Node::spinCallback(const ros::TimerEvent&)
{
  if (!ok()) return;

  // Read Position & Speed
  double pan  = m_pantilt->getPosition(PTU_PAN);
  double tilt = m_pantilt->getPosition(PTU_TILT);

  double panspeed  = m_pantilt->getSpeed(PTU_PAN);
  double tiltspeed = m_pantilt->getSpeed(PTU_TILT);

  // Publish Position & Speed
  sensor_msgs::JointState joint_state;
  joint_state.header.stamp = ros::Time::now();
  joint_state.name.resize(2);
  joint_state.position.resize(2);
  joint_state.velocity.resize(2);
  joint_state.name[0] = m_joint_name_prefix + "pan";
  joint_state.position[0] = pan;
  joint_state.velocity[0] = panspeed;
  joint_state.name[1] = m_joint_name_prefix + "tilt";
  joint_state.position[1] = tilt;
  joint_state.velocity[1] = tiltspeed;
  m_joint_pub.publish(joint_state);

  m_updater->update();
}

}  // namespace flir_ptu_driver

int main(int argc, char** argv)
{
  ros::init(argc, argv, "ptu");
  ros::NodeHandle n;

  while (ros::ok())
  {
    // Connect to PTU
    flir_ptu_driver::Node ptu_node(n);
    ptu_node.connect();

    // Set up polling callback
    int hz;
    ros::param::param<int>("~hz", hz, PTU_DEFAULT_HZ);
    ros::Timer spin_timer = n.createTimer(ros::Duration(1 / hz),
        &flir_ptu_driver::Node::spinCallback, &ptu_node);

    // Spin until there's a problem or we're in shutdown
    ros::spin();

    if (!ptu_node.ok())
    {
      ROS_ERROR("FLIR PTU disconnected, attempting reconnection.");
      ros::Duration(1.0).sleep();
    }
  }

  return 0;
}
