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

#include <boost/lexical_cast.hpp>
#include <boost/algorithm/string/trim.hpp>
#include <flir_ptu_driver/driver.h>
#include <serial/serial.h>
#include <ros/console.h>

#include <math.h>
#include <string>
#include <errno.h>
#include <sys/ioctl.h>

using boost::lexical_cast;
using namespace std;

namespace flir_ptu_driver
{

/**
    TCP Client class
    Ref: http://www.binarytides.com/code-a-simple-socket-client-class-in-c/
*/

TcpClient::TcpClient()
{
    mysock = -1;
    port = 0;
    address = "";
}

/**
    Connect to a host on a certain port number
*/
bool TcpClient::conn(std::string ip_address, int port)
{
    //create socket if it is not already created
    if(mysock == -1)
    {
        //Create socket
        mysock = socket(AF_INET, SOCK_STREAM, 0);
        if (mysock == -1)
        {
            perror("Could not create socket");
        }
    }
    else    {   /* OK, nothing */  }

    //setup address structure
    if(inet_addr(ip_address.c_str()) == -1)
    {
        struct hostent *he;
        struct in_addr **addr_list;

        //resolve the hostname, its not an ip address
        if ( (he = gethostbyname( ip_address.c_str() ) ) == NULL)
        {
            //gethostbyname failed
            herror("gethostbyname");
            cout << "Failed to resolve hostname\n";
            return false;
        }

        //Cast the h_addr_list to in_addr, since h_addr_list also has the ip address in long format only
        addr_list = (struct in_addr **) he->h_addr_list;

        for(int i = 0; addr_list[i] != NULL; i++)
        {
            //strcpy(ip, inet_ntoa(*addr_list[i]) );
            server.sin_addr = *addr_list[i];
            cout << ip_address << " resolved to " << inet_ntoa(*addr_list[i]) << endl;
            break;
        }
    }

    //plain ip address
    else
    {
        server.sin_addr.s_addr = inet_addr( ip_address.c_str() );
    }

    server.sin_family = AF_INET;
    server.sin_port = htons( port );

    //Connect to remote server
    if (connect(mysock, (struct sockaddr *)&server, sizeof(server)) < 0)
    {
        perror("connect failed. Error");
        return 1;
    }

    return true;
}

void TcpClient::setTimeout(int secs, int usecs)
{
	struct timeval tv;

	tv.tv_sec = secs;  //
	tv.tv_usec = usecs;  // Not init'ing this can cause strange errors

	setsockopt(mysock, SOL_SOCKET, SO_RCVTIMEO, (char *)&tv, sizeof(struct timeval));
}

/**
    Send data to the connected host
*/
bool TcpClient::send_data(string data)
{
    //Send some data
    // cout <<"data_send: " << data << "\n";
    if( send(mysock, data.c_str(), strlen( data.c_str() ), 0) < 0)
    {
        perror("Send failed : ");
        return false;
    }

    return true;
}

size_t TcpClient::readline (std::string &buffer, size_t size, std::string eol)
{
	char c;
	int ret;
	buffer = "";

    while (buffer.length() < size)
    {
        ret = read(mysock, &c, 1);  // read a single byte
        if (ret < 1)break;  // error or disconnect
        buffer += c;  // add c to string
        if (c == '\n')break; // was it an end of line?
    }
    // clean input - remove ending \r\n
    string::size_type pos = 0;
    if ( ( pos = buffer.find ("\r\n",pos) ) != string::npos )
    {
    	buffer.erase ( pos, 2 );
    }

    return buffer.length();
}

/**
    Receive data from the connected host
*/
string TcpClient::receive(int size)
{
    char buffer[size];
    string reply;
    int len;

    //Receive a reply from the server
    if( (len = recv(mysock, buffer, sizeof(buffer), 0)) < 0)
    {
        printf ("tcpClient::receive: failed (%d) %s\n", len, strerror(len)); //errno
        buffer[0] = '\0'; //return empty string
    }

    reply = buffer;
    return reply;
}

/** Templated wrapper function on lexical_cast to assist with extracting
 * values from serial response strings.
 */
template<typename T>
T parseResponse(std::string responseBuffer)
{
  T parsed; // declared outside 'try' so gdb debugger can see it

  try
  {
	std::string trimmed = responseBuffer.substr(1); // skip the leading '*'
	boost::trim(trimmed); // remove leading and trailing spaces
	parsed = lexical_cast<T>(trimmed);
	ROS_DEBUG_STREAM("Parsed response value: " << parsed);
  }
  catch (boost::exception& e)
  {
	ROS_ERROR_STREAM("Unable to parse " << responseBuffer);
  }

  return parsed;
}

bool PTU::initialized()
{
  return connected_ && initialized_;
}

bool PTU::initialize()
{
  ptuWrite("ft ");  // terse feedback
  if (connection_type_ == tcp)ptuRead(20); // tcp produces more output (* \r\n) than tty
  ptuWrite("ed ");  // disable echo
  if (connection_type_ == tcp)ptuRead(20);
  ptuWrite("ci ");  // position mode
  ptuRead(20); // read and discard response to above commands

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

bool PTU::connectTCP(std::string ip_addr, int32_t tcp_port) // for tcp connection
{
	if (!tcpClient_) {
		tcpClient_ = new TcpClient();
	}
	tcpClient_->conn(ip_addr, tcp_port);
	tcpClient_->setTimeout(2, 0); //set read timeout to X second
	connected_ = true;

	// for the TCP connection, the FLIR returns several lines of bannerish junk, about 120 chars
	// we need to read this or it will cause trouble later parsing other cmd output
	sleep(2); // maybe need to sleep before ptu sends banner
	ROS_INFO_STREAM("PTU::connectTCP: banner:");
	for (int i=0; i<10; i++)
	{
		std::string s = "";
		int len;
		len = ptuReadline(s);
		if(len > 1)ROS_INFO_STREAM("rcv len " << len << " " << s.c_str());
		if (s.find( "Initializing...") != std::string::npos)return true; // the end of the banner
	}

	return true;
}

bool PTU::connectTTY(std::string port, int32_t baud) // for tty connection
{
	if (!ser_) {
		ser_ = new serial::Serial();
	}
	ser_->setBaudrate(baud);
	//serial::Timeout to = serial::Timeout(200, 200, 0, 200, 0);
	// prevent lexical_cast error when tty slow. A better fix is to permit timeouts but deal with empty responseBuffer
	serial::Timeout to = serial::Timeout(4000, 4000, 0, 200, 0);
	ser_->setPort(port);
	ser_->setTimeout(to);
	ser_->open();
	connected_ = true;
	return true;
}

void PTU::ptuWrite(std::string command){
	if (connection_type_ == tty)
		ser_->write(command);
	else
		tcpClient_->send_data(command);
}

std::string PTU::ptuRead (size_t size) // default size=1
{
	std::string result;
	if (connection_type_ == tty)
		result = ser_->read(size);
	else
		result = tcpClient_->receive(size);
	return result;
}

size_t  PTU::ptuRead (std::string &buffer, size_t size)
{
	if (connection_type_ == tty)
	{
	    ser_->flush();
	}
	return 0;
}

size_t PTU::ptuReadline(std::string &buffer, size_t size, std::string eol)
{
	size_t len = 0;
	// readline (std::string &buffer, size_t size = 65536, std::string eol = "\n");
	if (connection_type_ == tty)
	{
		len = ser_->readline(buffer, PTU_BUFFER_LEN, "\n");
	}
	else
	{
		//buffer = tcpClient_->receive(size);
		//len = buffer.length();
		len = tcpClient_->readline(buffer, PTU_BUFFER_LEN, "\n");
	}
	return len;
}

void PTU::ptuFlush()
{
	if (connection_type_ == tty)
	{
	    ser_->flush();
	}
}

size_t PTU::available () /* Return the number of characters in the buffer. */
{
	size_t len;
	if (connection_type_ == tty)
		len = ser_->available();
	else {
		ioctl(tcpClient_->mysock, FIONREAD, &len);
	}
	return len;
}


std::string PTU::sendCommand(std::string command)
{
  std::string buffer;
  ptuWrite(command);
  ROS_DEBUG_STREAM("TX: " << command);
  int length  = ptuReadline(buffer, PTU_BUFFER_LEN, "\n");
  ROS_DEBUG_STREAM("RX: " << buffer);
  //ROS_INFO_STREAM("[PTU::sendCommand] TX: " << command << " RX (length=" << length << ") ["  << buffer << "]");
  return buffer;
}

bool PTU::home()
{
  ROS_INFO("Sending command to reset PTU.");

  // Issue reset command
  ptuFlush();
  ptuWrite(" r ");

  std::string actual_response, expected_response("!T!T!P!P*");

  // 30 seconds to receive full confirmation of reset action completed.
  for (int i = 0; i < 300; i++)
  {
    usleep(100000);

    if (available() >= expected_response.length())
    {
      ROS_INFO("PTU reset command response received.");
      ptuRead(actual_response, expected_response.length());
      return (actual_response == expected_response);
    }
  }

  ROS_WARN("PTU reset command response not received before timeout.");
  return false;
}

// get radians/count resolution
float PTU::getRes(char type)
{
  if (! connected_ == true)return -1;

  std::string buffer = sendCommand(std::string() + type + "r ");

  if (buffer.length() < 3 || buffer[0] != '*')
  {
    ROS_ERROR("Error getting pan-tilt res");
    return -1;
  }

  double z = parseResponse<double>(buffer);
  z = z / 3600;  // degrees/count
  return z * M_PI / 180;  // radians/count
}

// get position limit
int PTU::getLimit(char type, char limType)
{
  if (! connected_ == true)return -1;

  std::string buffer = sendCommand(std::string() + type + limType + " ");

  if (buffer.length() < 3 || buffer[0] != '*')
  {
    ROS_ERROR("Error getting pan-tilt limit");
    return -1;
  }

  return parseResponse<int>(buffer);
}


// get position in radians
float PTU::getPosition(char type)
{
  if (!initialized()) return -1;

  std::string buffer = sendCommand(std::string() + type + "p ");

  if (buffer.length() < 3 || buffer[0] != '*')
  {
    ROS_ERROR("Error getting pan-tilt pos");
    return -1;
  }

  return parseResponse<double>(buffer) * getResolution(type);
}


// set position in radians
bool PTU::setPosition(char type, float pos, bool block)
{
  if (!initialized()) return false;

  // get raw encoder count to move
  int count = static_cast<int>(pos / getResolution(type));

  // Check limits
  if (count < (type == PTU_TILT ? TMin : PMin) || count > (type == PTU_TILT ? TMax : PMax))
  {
    ROS_ERROR("Pan Tilt Value out of Range: %c %f(%d) (%d-%d)\n",
              type, pos, count, (type == PTU_TILT ? TMin : PMin), (type == PTU_TILT ? TMax : PMax));
    return false;
  }

  std::string buffer = sendCommand(std::string() + type + "p" +
                                   lexical_cast<std::string>(count) + " ");

  if (buffer.empty() || buffer[0] != '*')
  {
    ROS_ERROR("Error setting pan-tilt pos");
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
  if (!initialized()) return -1;

  std::string buffer = sendCommand(std::string() + type + "s ");

  if (buffer.length() < 3 || buffer[0] != '*')
  {
    ROS_ERROR("Error getting pan-tilt speed");
    return -1;
  }

  return parseResponse<double>(buffer) * getResolution(type);
}



// set speed in radians/sec
bool PTU::setSpeed(char type, float pos)
{
  if (!initialized()) return false;

  // get raw encoder speed to move
  int count = static_cast<int>(pos / getResolution(type));

  // Check limits
  if (abs(count) < (type == PTU_TILT ? TSMin : PSMin) || abs(count) > (type == PTU_TILT ? TSMax : PSMax))
  {
    ROS_ERROR("Pan Tilt Speed Value out of Range: %c %f(%d) (%d-%d)\n",
              type, pos, count, (type == PTU_TILT ? TSMin : PSMin), (type == PTU_TILT ? TSMax : PSMax));
    return false;
  }

  std::string buffer = sendCommand(std::string() + type + "s" +
                                   lexical_cast<std::string>(count) + " ");

  if (buffer.empty() || buffer[0] != '*')
  {
    ROS_ERROR("Error setting pan-tilt speed\n");
    return false;
  }

  return true;
}


// set movement mode (position/velocity)
bool PTU::setMode(char type)
{
  if (!initialized()) return false;

  std::string buffer = sendCommand(std::string("c") + type + " ");

  if (buffer.empty() || buffer[0] != '*')
  {
    ROS_ERROR("Error setting pan-tilt move mode");
    return false;
  }

  return true;
}

// get ptu mode
char PTU::getMode()
{
  if (!initialized()) return -1;

  // get pan tilt mode
  std::string buffer = sendCommand("c ");

  if (buffer.length() < 3 || buffer[0] != '*')
  {
    ROS_ERROR("Error getting pan-tilt pos");
    return -1;
  }

  if (buffer[2] == 'p')
    return PTU_VELOCITY;
  else if (buffer[2] == 'i')
    return PTU_POSITION;
  else
    return -1;
}

}  // namespace flir_ptu_driver
