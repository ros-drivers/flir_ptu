/**
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
 *  TCP Client class
 *  Ref: http://www.binarytides.com/code-a-simple-socket-client-class-in-c/
 */

#pragma once

#include <cstddef>
#include <string>

namespace flir_ptu_driver
{

/**
 * @brief Abstract byte-stream link between @ref PTU and the physical unit.
 *
 * Concrete implementations (@ref SerialTransport, @ref TcpTransport) wrap a
 * file descriptor and expose a minimal blocking read/write interface that the
 * driver uses to issue ASCII commands and parse responses.
 */
class Transport
{
public:
  virtual ~Transport() = default;

  /**
   * @brief Open the underlying connection.
   * @return @c true if the link is ready for I/O, @c false on failure.
   */
  virtual bool open() = 0;

  /// Close the underlying connection. Safe to call when already closed.
  virtual void close() = 0;

  /// @return @c true while the underlying connection is open.
  virtual bool isOpen() const = 0;

  /**
   * @brief Send raw bytes to the device with no framing or escaping.
   * @param data Bytes to write.
   */
  virtual void write(const std::string & data) = 0;

  /**
   * @brief Read a single line terminated by @p eol from the device.
   *
   * Blocks until a terminator is seen, @p max_len bytes have accumulated,
   * or the link reports an error. The terminator is included in the result
   * when one was received.
   *
   * @param max_len Maximum number of bytes to accumulate before returning.
   * @param eol     Line-terminator byte.
   */
  virtual std::string readline(size_t max_len = 255, char eol = '\n') = 0;

  /**
   * @brief Read exactly @p size bytes from the device.
   *
   * Blocks until the requested count has been read or the link is closed.
   */
  virtual std::string read(size_t size) = 0;

  /// @return Number of bytes currently buffered and immediately readable.
  virtual size_t available() = 0;

  /// Discard any bytes pending in the receive buffer.
  virtual void flush() = 0;
};

}  // namespace flir_ptu_driver
