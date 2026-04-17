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

#include <flir_ptu_driver/serial_transport.h>

#include <fcntl.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <cstring>

namespace flir_ptu_driver
{

SerialTransport::SerialTransport(const std::string & port, int baud)
: port_(port), baud_(baud), fd_(-1)
{
  std::memset(&old_tio_, 0, sizeof(old_tio_));
}

SerialTransport::~SerialTransport() { close(); }

speed_t SerialTransport::baudToSpeed(int baud) const
{
  switch (baud)
  {
    case 2400:
      return B2400;
    case 4800:
      return B4800;
    case 9600:
      return B9600;
    case 19200:
      return B19200;
    case 38400:
      return B38400;
    case 57600:
      return B57600;
    case 115200:
      return B115200;
    default:
      return B9600;
  }
}

bool SerialTransport::open()
{
  fd_ = ::open(port_.c_str(), O_RDWR | O_NOCTTY);
  if (fd_ < 0)
  {
    return false;
  }

  // Save old settings
  tcgetattr(fd_, &old_tio_);

  struct termios tio;
  std::memset(&tio, 0, sizeof(tio));

  speed_t speed = baudToSpeed(baud_);
  cfsetispeed(&tio, speed);
  cfsetospeed(&tio, speed);

  tio.c_cflag |= (CLOCAL | CREAD);
  tio.c_cflag &= ~CSIZE;
  tio.c_cflag |= CS8;
  tio.c_cflag &= ~PARENB;
  tio.c_cflag &= ~CSTOPB;

  // Raw input
  tio.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
  tio.c_iflag &= ~(IXON | IXOFF | IXANY | ICRNL | INLCR);
  tio.c_oflag &= ~OPOST;

  // Read timeout: 4 seconds (matching noetic-devel serial::Timeout(4000, 4000, ...))
  tio.c_cc[VMIN] = 0;
  tio.c_cc[VTIME] = 40;

  tcflush(fd_, TCIFLUSH);
  tcsetattr(fd_, TCSANOW, &tio);

  return true;
}

void SerialTransport::close()
{
  if (fd_ >= 0)
  {
    tcsetattr(fd_, TCSANOW, &old_tio_);
    ::close(fd_);
    fd_ = -1;
  }
}

bool SerialTransport::isOpen() const { return fd_ >= 0; }

void SerialTransport::write(const std::string & data)
{
  if (fd_ < 0)
  {
    return;
  }
  size_t total = 0;
  while (total < data.size())
  {
    ssize_t n = ::write(fd_, data.c_str() + total, data.size() - total);
    if (n < 0)
    {
      break;
    }
    total += static_cast<size_t>(n);
  }
}

std::string SerialTransport::readline(size_t max_len, char eol)
{
  std::string result;
  result.reserve(max_len);
  char c;
  while (result.size() < max_len)
  {
    ssize_t n = ::read(fd_, &c, 1);
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
  return result;
}

std::string SerialTransport::read(size_t size)
{
  std::string result;
  result.resize(size);
  size_t total = 0;
  while (total < size)
  {
    ssize_t n = ::read(fd_, &result[total], size - total);
    if (n <= 0)
    {
      break;
    }
    total += static_cast<size_t>(n);
  }
  result.resize(total);
  return result;
}

size_t SerialTransport::available()
{
  if (fd_ < 0)
  {
    return 0;
  }
  int bytes_available = 0;
  ioctl(fd_, FIONREAD, &bytes_available);
  return static_cast<size_t>(bytes_available);
}

void SerialTransport::flush()
{
  if (fd_ >= 0)
  {
    tcflush(fd_, TCIOFLUSH);
  }
}

}  // namespace flir_ptu_driver
