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

#include <flir_ptu_driver/tcp_transport.h>

#include <arpa/inet.h>
#include <netdb.h>
#include <netinet/tcp.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>
#include <cstring>
#include <iostream>

namespace flir_ptu_driver
{

TcpTransport::TcpTransport(const std::string & host, int port) : host_(host), port_(port), fd_(-1)
{
}

TcpTransport::~TcpTransport() { close(); }

bool TcpTransport::open()
{
  fd_ = ::socket(AF_INET, SOCK_STREAM, 0);
  if (fd_ < 0)
  {
    return false;
  }

  // Disable Nagle for low-latency ASCII commands
  int flag = 1;
  setsockopt(fd_, IPPROTO_TCP, TCP_NODELAY, &flag, sizeof(flag));

  // Set receive timeout to 2 seconds (matching noetic-devel)
  struct timeval tv;
  tv.tv_sec = 2;
  tv.tv_usec = 0;
  setsockopt(fd_, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

  struct sockaddr_in addr;
  std::memset(&addr, 0, sizeof(addr));
  addr.sin_family = AF_INET;
  addr.sin_port = htons(static_cast<uint16_t>(port_));

  if (inet_pton(AF_INET, host_.c_str(), &addr.sin_addr) <= 0)
  {
    // Try hostname resolution
    struct addrinfo hints, *res;
    std::memset(&hints, 0, sizeof(hints));
    hints.ai_family = AF_INET;
    hints.ai_socktype = SOCK_STREAM;

    if (getaddrinfo(host_.c_str(), nullptr, &hints, &res) != 0)
    {
      ::close(fd_);
      fd_ = -1;
      return false;
    }

    auto * sa = reinterpret_cast<struct sockaddr_in *>(res->ai_addr);
    addr.sin_addr = sa->sin_addr;
    freeaddrinfo(res);
  }

  if (::connect(fd_, reinterpret_cast<struct sockaddr *>(&addr), sizeof(addr)) < 0)
  {
    ::close(fd_);
    fd_ = -1;
    return false;
  }

  return true;
}

void TcpTransport::close()
{
  if (fd_ >= 0)
  {
    ::close(fd_);
    fd_ = -1;
  }
}

bool TcpTransport::isOpen() const { return fd_ >= 0; }

void TcpTransport::write(const std::string & data)
{
  if (fd_ < 0)
  {
    return;
  }
  ::send(fd_, data.c_str(), data.size(), MSG_NOSIGNAL);
}

std::string TcpTransport::readline(size_t max_len, char eol)
{
  std::string result;
  result.reserve(max_len);
  char c;
  while (result.size() < max_len)
  {
    ssize_t n = ::recv(fd_, &c, 1, 0);
    if (n <= 0)
    {
      break;
    }
    result += c;
    if (c == eol)
    {
      break;
    }
  }
  // Strip trailing \r\n
  while (!result.empty() && (result.back() == '\r' || result.back() == '\n'))
  {
    result.pop_back();
  }
  return result;
}

std::string TcpTransport::read(size_t size)
{
  std::string result;
  result.resize(size);
  size_t total = 0;
  while (total < size)
  {
    ssize_t n = ::recv(fd_, &result[total], size - total, 0);
    if (n <= 0)
    {
      break;
    }
    total += static_cast<size_t>(n);
  }
  result.resize(total);
  return result;
}

size_t TcpTransport::available()
{
  if (fd_ < 0)
  {
    return 0;
  }
  int bytes_available = 0;
  ioctl(fd_, FIONREAD, &bytes_available);
  return static_cast<size_t>(bytes_available);
}

void TcpTransport::flush()
{
  if (fd_ < 0)
  {
    return;
  }
  // Drain any buffered data
  char buf[256];
  int bytes_available = 0;
  ioctl(fd_, FIONREAD, &bytes_available);
  while (bytes_available > 0)
  {
    ssize_t n = ::recv(fd_, buf, std::min(static_cast<int>(sizeof(buf)), bytes_available), 0);
    if (n <= 0)
    {
      break;
    }
    ioctl(fd_, FIONREAD, &bytes_available);
  }
}

void TcpTransport::drainBanner()
{
  // The FLIR PTU sends several lines of banner text on TCP connect (~120 chars).
  // Read and discard them before issuing commands.
  sleep(2);
  std::cerr << "[flir_ptu_tcp] Draining TCP banner from PTU..." << std::endl;
  for (int i = 0; i < 10; i++)
  {
    std::string line = readline(512);
    if (!line.empty())
    {
      std::cerr << "[flir_ptu_tcp] Banner: " << line << std::endl;
    }
    if (line.find("Initializing...") != std::string::npos)
    {
      break;
    }
  }
}

}  // namespace flir_ptu_driver
