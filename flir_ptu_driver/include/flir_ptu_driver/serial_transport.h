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

#include <flir_ptu_driver/transport.h>

#include <termios.h>
#include <string>

namespace flir_ptu_driver
{

/**
 * @brief Serial (TTY) implementation of the PTU @ref Transport interface.
 *
 * Opens a POSIX serial device in raw 8-N-1 mode at the requested baud rate
 * and exposes the byte stream expected by @ref PTU. The previous termios
 * settings are saved on @ref open() and restored on @ref close().
 */
class SerialTransport : public Transport
{
public:
  /**
   * @brief Construct a serial transport bound to a TTY device.
   *
   * The device is not opened until @ref open() is called.
   *
   * @param port  Path to the serial device, e.g. "/dev/ttyUSB0".
   * @param baud  Symbol rate in bits per second (must be a value accepted
   *              by termios, e.g. 9600, 19200, 38400, 57600, 115200).
   */
  SerialTransport(const std::string & port, int baud);

  /// Closes the device if still open and restores the prior termios state.
  ~SerialTransport() override;

  /**
   * @brief Open the serial device and apply raw 8-N-1 termios settings.
   * @return @c true if the device was opened successfully.
   */
  bool open() override;

  /// Restore the prior termios state and close the file descriptor.
  void close() override;

  /// @return @c true while the underlying file descriptor is open.
  bool isOpen() const override;

  /**
   * @brief Send raw bytes over the serial link.
   * @param data Bytes to write; sent as-is with no framing or escaping.
   */
  void write(const std::string & data) override;

  /**
   * @brief Read a single line terminated by @p eol.
   *
   * Blocks until a terminator is seen, @p max_len bytes have accumulated,
   * or the device reports an error. The terminator is included in the
   * returned string when one was received.
   */
  std::string readline(size_t max_len = 255, char eol = '\n') override;

  /// Read exactly @p size bytes, blocking until satisfied or on error.
  std::string read(size_t size) override;

  /// @return Number of bytes currently buffered and immediately readable.
  size_t available() override;

  /// Discard any bytes pending in the receive buffer.
  void flush() override;

private:
  std::string port_;        ///< Path to the serial device.
  int baud_;                ///< Configured symbol rate (bits/s).
  int fd_;                  ///< File descriptor for the open device; -1 when closed.
  struct termios old_tio_;  ///< Termios state captured on open() and restored on close().

  /// Translate a numeric baud rate to the matching termios @c speed_t constant.
  speed_t baudToSpeed(int baud) const;
};

}  // namespace flir_ptu_driver
