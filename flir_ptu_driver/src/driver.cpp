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

#include <flir_ptu_driver/driver.h>

#include <unistd.h>
#include <algorithm>
#include <cmath>
#include <iostream>
#include <string>

namespace flir_ptu_driver
{

/** Templated wrapper function to assist with extracting
 * values from response strings.
 */
template <typename T>
T parseResponse(const std::string & responseBuffer)
{
  T parsed = T();

  try
  {
    std::string trimmed = responseBuffer.substr(1);  // skip the leading '*'
    // Trim whitespace
    auto start = trimmed.find_first_not_of(" \t\r\n");
    auto end = trimmed.find_last_not_of(" \t\r\n");
    if (start == std::string::npos)
    {
      trimmed = "0";
    }
    else
    {
      trimmed = trimmed.substr(start, end - start + 1);
    }

    if constexpr (std::is_same_v<T, double>)
    {
      parsed = std::stod(trimmed);
    }
    else if constexpr (std::is_same_v<T, int>)
    {
      parsed = std::stoi(trimmed);
    }

    // Debug: parsed response value
  }
  catch (const std::exception & e)
  {
    std::cerr << "[flir_ptu_driver] Unable to parse '" << responseBuffer << "': " << e.what()
              << std::endl;
  }

  return parsed;
}

bool PTU::initialized() { return transport_ && transport_->isOpen() && initialized_; }

bool PTU::disableLimits()
{
  transport_->write("ld ");  // Disable Limits
  transport_->read(20);
  Lim = false;
  return true;
}

bool PTU::initialize()
{
  transport_->write("ft ");  // terse feedback
  transport_->read(20);
  transport_->write("ed ");  // disable echo
  transport_->read(20);
  transport_->write("ci ");  // position mode
  transport_->read(20);

  // get pan tilt encoder res
  tr = getRes(PTU_TILT);
  pr = getRes(PTU_PAN);

  PMin = getLimit(PTU_PAN, PTU_MIN);
  PMax = getLimit(PTU_PAN, PTU_MAX);
  TMin = getLimit(PTU_TILT, PTU_MIN);
  TMax = getLimit(PTU_TILT, PTU_MAX);
  PSMin = getLimit(PTU_PAN, PTU_MIN_SPEED);
  PSMax = getLimit(PTU_PAN, PTU_MAX_SPEED);
  TSMin = getLimit(PTU_TILT, PTU_MIN_SPEED);
  TSMax = getLimit(PTU_TILT, PTU_MAX_SPEED);
  Lim = true;

  if (tr <= 0 || pr <= 0 || PMin == -1 || PMax == -1 || TMin == -1 || TMax == -1)
  {
    initialized_ = false;
  }
  else
  {
    initialized_ = true;
  }

  return initialized();
}

std::string PTU::sendCommand(std::string command)
{
  // Drain any stale bytes in the receive buffer to keep TX/RX synchronized.
  transport_->flush();
  transport_->write(command);
  // Debug: TX/RX
  std::string buffer = transport_->readline(PTU_BUFFER_LEN);
  return buffer;
}

bool PTU::home()
{
  std::cout << "[flir_ptu_driver] Sending command to reset PTU." << std::endl;

  // Issue reset command
  transport_->flush();
  transport_->write(" r ");

  std::string actual_response, expected_response("!T!T!P!P*");

  // 30 seconds to receive full confirmation of reset action completed.
  for (int i = 0; i < 300; i++)
  {
    usleep(100000);

    if (transport_->available() >= expected_response.length())
    {
      std::cout << "[flir_ptu_driver] PTU reset command response received." << std::endl;
      actual_response = transport_->read(expected_response.length());
      return actual_response == expected_response;
    }
  }

  std::cerr << "[flir_ptu_driver] PTU reset command response not received before timeout."
            << std::endl;
  return false;
}

// get radians/count resolution
float PTU::getRes(char type)
{
  if (!transport_ || !transport_->isOpen())
  {
    return -1;
  }

  std::string buffer = sendCommand(std::string() + type + "r ");

  if (buffer.length() < 3 || buffer[0] != '*')
  {
    std::cerr << "[flir_ptu_driver] Error getting pan-tilt res" << std::endl;
    return -1;
  }

  double z = parseResponse<double>(buffer);
  z = z / 3600;           // degrees/count
  return z * M_PI / 180;  // radians/count
}

// get position limit
int PTU::getLimit(char type, char limType)
{
  if (!transport_ || !transport_->isOpen())
  {
    return -1;
  }

  std::string buffer = sendCommand(std::string() + type + limType + " ");

  if (buffer.length() < 3 || buffer[0] != '*')
  {
    std::cerr << "[flir_ptu_driver] Error getting pan-tilt limit" << std::endl;
    return -1;
  }

  return parseResponse<int>(buffer);
}

// get firmware/version banner
std::string PTU::getVersion()
{
  if (!transport_ || !transport_->isOpen())
  {
    return std::string();
  }

  std::string buffer = sendCommand("v ");

  if (buffer.length() < 2 || buffer[0] != '*')
  {
    std::cerr << "[flir_ptu_driver] Error getting PTU version" << std::endl;
    return std::string();
  }

  std::string trimmed = buffer.substr(1);
  auto start = trimmed.find_first_not_of(" \t\r\n");
  auto end = trimmed.find_last_not_of(" \t\r\n");
  if (start == std::string::npos)
  {
    return std::string();
  }
  return trimmed.substr(start, end - start + 1);
}

// get position in radians
float PTU::getPosition(char type)
{
  if (!initialized())
  {
    return -1;
  }

  std::string buffer = sendCommand(std::string() + type + "p ");

  if (buffer.length() < 3 || buffer[0] != '*')
  {
    std::cerr << "[flir_ptu_driver] Error getting pan-tilt pos" << std::endl;
    return -1;
  }

  return parseResponse<double>(buffer) * getResolution(type);
}

// set position in radians
bool PTU::setPosition(char type, float pos, bool block)
{
  if (!initialized())
  {
    return false;
  }

  // get raw encoder count to move
  int count = static_cast<int>(pos / getResolution(type));

  // Check limits
  if (Lim)
  {
    if (count < (type == PTU_TILT ? TMin : PMin) || count > (type == PTU_TILT ? TMax : PMax))
    {
      std::cerr << "[flir_ptu_driver] Pan Tilt Value out of Range: " << type << " " << pos << "("
                << count << ") (" << (type == PTU_TILT ? TMin : PMin) << "-"
                << (type == PTU_TILT ? TMax : PMax) << ")" << std::endl;
      return false;
    }
  }

  std::string buffer = sendCommand(std::string() + type + "p" + std::to_string(count) + " ");

  if (buffer.empty() || buffer[0] != '*')
  {
    std::cerr << "[flir_ptu_driver] Error setting pan-tilt pos" << std::endl;
    return false;
  }

  if (block)
  {
    while (getPosition(type) != pos)
    {
      usleep(1000);
    }
  }

  return true;
}

// get speed in radians/sec
float PTU::getSpeed(char type)
{
  if (!initialized())
  {
    return -1;
  }

  std::string buffer = sendCommand(std::string() + type + "s ");

  if (buffer.length() < 3 || buffer[0] != '*')
  {
    std::cerr << "[flir_ptu_driver] Error getting pan-tilt speed" << std::endl;
    return -1;
  }

  return parseResponse<double>(buffer) * getResolution(type);
}

// set speed in radians/sec
bool PTU::setSpeed(char type, float pos)
{
  if (!initialized())
  {
    return false;
  }

  // get raw encoder speed to move
  int count = static_cast<int>(pos / getResolution(type));

  // Check limits
  if (
    abs(count) < (type == PTU_TILT ? TSMin : PSMin) ||
    abs(count) > (type == PTU_TILT ? TSMax : PSMax))
  {
    std::cerr << "[flir_ptu_driver] Pan Tilt Speed Value out of Range: " << type << " " << pos
              << "(" << count << ") (" << (type == PTU_TILT ? TSMin : PSMin) << "-"
              << (type == PTU_TILT ? TSMax : PSMax) << ")" << std::endl;
    return false;
  }

  std::string buffer = sendCommand(std::string() + type + "s" + std::to_string(count) + " ");

  if (buffer.empty() || buffer[0] != '*')
  {
    std::cerr << "[flir_ptu_driver] Error setting pan-tilt speed" << std::endl;
    return false;
  }

  return true;
}

// set movement mode (position/velocity)
bool PTU::setMode(char type)
{
  if (!initialized())
  {
    return false;
  }

  std::string buffer = sendCommand(std::string("c") + type + " ");

  if (buffer.empty() || buffer[0] != '*')
  {
    std::cerr << "[flir_ptu_driver] Error setting pan-tilt move mode" << std::endl;
    return false;
  }

  return true;
}

// get ptu mode
char PTU::getMode()
{
  if (!initialized())
  {
    return -1;
  }

  std::string buffer = sendCommand("c ");

  if (buffer.length() < 3 || buffer[0] != '*')
  {
    std::cerr << "[flir_ptu_driver] Error getting pan-tilt mode" << std::endl;
    return -1;
  }

  if (buffer[2] == 'p' || buffer[2] == 'P')
  {
    return PTU_VELOCITY;
  }
  else if (buffer[2] == 'i' || buffer[2] == 'I')
  {
    return PTU_POSITION;
  }
  else
  {
    return -1;
  }
}

}  // namespace flir_ptu_driver
