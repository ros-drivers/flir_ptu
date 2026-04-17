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

#include <string>

namespace flir_ptu_driver
{

/**
 * @brief TCP/IP implementation of the PTU @ref Transport interface.
 *
 * Establishes a blocking IPv4 TCP connection to a FLIR PTU's network port
 * (typically 4000) and exposes the line-oriented byte stream expected by
 * @ref PTU. Intended as a drop-in replacement for @ref SerialTransport when
 * the unit is reached over Ethernet rather than RS-232.
 */
class TcpTransport : public Transport
{
public:
  /**
   * @brief Construct a TCP transport for a PTU at @p host : @p port.
   *
   * The socket is not opened until @ref open() is called.
   *
   * @param host  Hostname or dotted-quad IPv4 address of the PTU.
   * @param port  TCP port the PTU is listening on (typically 4000).
   */
  TcpTransport(const std::string & host, int port);

  /// Closes the socket if still open and releases any resources.
  ~TcpTransport() override;

  /**
   * @brief Resolve the host and open a TCP connection to the PTU.
   * @return @c true if the connection was established, @c false on error.
   */
  bool open() override;

  /// Close the socket. Safe to call when already closed.
  void close() override;

  /// @return @c true while the underlying socket is connected.
  bool isOpen() const override;

  /**
   * @brief Send raw bytes to the PTU.
   * @param data Bytes to write; sent as-is with no framing or escaping.
   */
  void write(const std::string & data) override;

  /**
   * @brief Read a single line terminated by @p eol from the PTU.
   *
   * Blocks until either a terminator is seen, @p max_len bytes have been
   * accumulated, or the socket reports an error. The returned string includes
   * the terminator when one was received.
   *
   * @param max_len Maximum number of bytes to accumulate before returning.
   * @param eol     Byte that delimits a line.
   */
  std::string readline(size_t max_len = 255, char eol = '\n') override;

  /**
   * @brief Read exactly @p size bytes from the socket.
   *
   * Blocks until the requested count has been read or the socket is closed.
   */
  std::string read(size_t size) override;

  /// @return Number of bytes currently buffered and immediately readable.
  size_t available() override;

  /// Discard any bytes pending in the receive buffer.
  void flush() override;

  /**
   * @brief Consume the multi-line greeting banner emitted by the PTU on
   *        connect so that subsequent reads start at command responses.
   */
  void drainBanner();

private:
  std::string host_;  ///< Hostname or IPv4 address of the PTU.
  int port_;          ///< TCP port to connect to.
  int fd_;            ///< File descriptor for the connected socket; -1 when closed.
};

}  // namespace flir_ptu_driver
