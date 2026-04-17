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

#pragma once

#include <cstddef>
#include <memory>
#include <string>
#include <utility>

#include <flir_ptu_driver/transport.h>

namespace flir_ptu_driver
{

// Common defaults.
constexpr std::size_t PTU_BUFFER_LEN = 255;
constexpr int PTU_DEFAULT_HZ = 10;
constexpr double PTU_DEFAULT_VEL = 0.0;

// Serial defaults.
constexpr int PTU_SERIAL_DEFAULT_BAUD = 9600;
constexpr const char * PTU_SERIAL_DEFAULT_PORT = "/dev/ttyUSB0";

// TCP defaults.
constexpr const char * PTU_DEFAULT_TCP_IP = "192.168.131.70";
constexpr int PTU_DEFAULT_TCP_PORT = 4000;

// PTU command characters.
constexpr char PTU_PAN = 'p';
constexpr char PTU_TILT = 't';
constexpr char PTU_MIN = 'n';
constexpr char PTU_MAX = 'x';
constexpr char PTU_MIN_SPEED = 'l';
constexpr char PTU_MAX_SPEED = 'u';
constexpr char PTU_VELOCITY = 'v';
constexpr char PTU_POSITION = 'i';

/**
 * @brief High-level driver for FLIR Pan-Tilt Units.
 *
 * Wraps a @ref Transport (serial or TCP) and exposes the PTU's ASCII
 * command set as typed C++ methods. All angular quantities are in radians
 * and all rates are in radians per second; conversions to/from raw encoder
 * counts use the per-axis resolutions reported by the PTU on @ref initialize().
 *
 * Pan/tilt axes are selected throughout the API by a single character:
 *   - @c PTU_PAN  ('p') for the pan axis
 *   - @c PTU_TILT ('t') for the tilt axis
 */
class PTU
{
public:
  /**
   * @brief Construct a PTU bound to an already-configured transport.
   *
   * The transport is not opened here; the caller must have called
   * @c transport->open() before @ref initialize().
   *
   * @param transport Transport instance ready to communicate with the device.
   */
  explicit PTU(std::unique_ptr<Transport> transport)
  : transport_(std::move(transport)), initialized_(false)
  {
  }

  /**
   * @brief Query axis resolutions and limits and prepare for commanding.
   * @return @c true if the PTU responded to all initialization queries.
   */
  bool initialize();

  /**
   * @brief Disable the PTU's internal soft motion limits.
   * @return @c true if the command was acknowledged.
   */
  bool disableLimits();

  /// @return @c true if the transport is open and the PTU has been initialized.
  bool initialized();

  /**
   * @brief Read the current position of an axis.
   * @param type @c PTU_PAN or @c PTU_TILT.
   * @return Position in radians.
   */
  float getPosition(char type);

  /**
   * @brief Read the current commanded speed of an axis.
   * @param type @c PTU_PAN or @c PTU_TILT.
   * @return Speed in radians/second.
   */
  float getSpeed(char type);

  /**
   * @brief Per-count angular resolution of an axis.
   * @param type @c PTU_PAN or @c PTU_TILT.
   * @return Resolution in radians/count.
   */
  float getResolution(char type) { return type == PTU_TILT ? tr : pr; }

  /**
   * @brief Minimum reachable position of an axis.
   * @param type @c PTU_PAN or @c PTU_TILT.
   * @return Minimum position in radians.
   */
  float getMin(char type) { return getResolution(type) * (type == PTU_TILT ? TMin : PMin); }

  /**
   * @brief Maximum reachable position of an axis.
   * @param type @c PTU_PAN or @c PTU_TILT.
   * @return Maximum position in radians.
   */
  float getMax(char type) { return getResolution(type) * (type == PTU_TILT ? TMax : PMax); }

  /**
   * @brief Minimum commandable speed of an axis.
   * @param type @c PTU_PAN or @c PTU_TILT.
   * @return Minimum speed in radians/second.
   */
  float getMinSpeed(char type) { return getResolution(type) * (type == PTU_TILT ? TSMin : PSMin); }

  /**
   * @brief Maximum commandable speed of an axis.
   * @param type @c PTU_PAN or @c PTU_TILT.
   * @return Maximum speed in radians/second.
   */
  float getMaxSpeed(char type) { return getResolution(type) * (type == PTU_TILT ? TSMax : PSMax); }

  /**
   * @brief Move an axis to the desired position.
   *
   * If @p Block is @c true the call does not return until the PTU reports
   * that the target has been reached; otherwise it returns as soon as the
   * command is acknowledged.
   *
   * @param type  @c PTU_PAN or @c PTU_TILT.
   * @param pos   Desired position in radians.
   * @param Block Wait for the move to complete before returning.
   * @return @c true if the command was successfully sent.
   */
  bool setPosition(char type, float pos, bool Block = false);

  /**
   * @brief Set the commanded speed of an axis.
   * @param type  @c PTU_PAN or @c PTU_TILT.
   * @param speed Desired speed in radians/second.
   * @return @c true if the command was successfully sent.
   */
  bool setSpeed(char type, float speed);

  /**
   * @brief Switch the PTU between position and velocity control modes.
   * @param type @c PTU_VELOCITY ('v') or @c PTU_POSITION ('i').
   * @return @c true if the command was successfully sent.
   */
  bool setMode(char type);

  /**
   * @brief Query the active control mode.
   * @return @c PTU_VELOCITY ('v') or @c PTU_POSITION ('i').
   */
  char getMode();

  /**
   * @brief Query the PTU firmware/version banner (response to the `v` command).
   *
   * The leading `*` and surrounding whitespace are stripped from the response,
   * e.g. `"Pan-Tilt Controller v3.5.2, (C)2010-2022 Teledyne FLIR ..."`.
   *
   * @return The banner string, or an empty string if the query fails.
   */
  std::string getVersion();

  /**
   * @brief Home (reset) the PTU.
   * @return @c true if the command was acknowledged.
   */
  bool home();

private:
  /// Query the per-count resolution of an axis from the PTU.
  float getRes(char type);

  /// Query a position or speed limit (in raw counts) for an axis.
  int getLimit(char type, char limType);

  // Position Limits
  int TMin;  ///< Min Tilt in Counts
  int TMax;  ///< Max Tilt in Counts
  int PMin;  ///< Min Pan in Counts
  int PMax;  ///< Max Pan in Counts
  bool Lim;  ///< Position Limits enabled

  // Speed Limits
  int TSMin;  ///< Min Tilt Speed in Counts/second
  int TSMax;  ///< Max Tilt Speed in Counts/second
  int PSMin;  ///< Min Pan Speed in Counts/second
  int PSMax;  ///< Max Pan Speed in Counts/second

protected:
  /**
   * @brief Send an ASCII command to the PTU and return its response.
   * @param command Command string excluding any line terminator.
   * @return Raw response from the PTU.
   */
  std::string sendCommand(std::string command);

  std::unique_ptr<Transport> transport_;  ///< Underlying byte-stream link to the PTU.
  bool initialized_;                      ///< @c true after a successful @ref initialize().

  float tr;  ///< tilt resolution (rads/count)
  float pr;  ///< pan resolution (rads/count)
};

}  // namespace flir_ptu_driver
