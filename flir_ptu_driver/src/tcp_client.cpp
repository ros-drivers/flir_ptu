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

#include <sys/ioctl.h>
#include <string>
#include <errno.h>
#include <flir_ptu_driver/tcp_client.h>
#include <ros/console.h>

using std::cout;
using std::endl;
using std::string;

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
    // create socket if it is not already created
    if (mysock == -1)
    {
        // Create socket
        mysock = socket(AF_INET, SOCK_STREAM, 0);
        if (mysock == -1)
        {
            perror("Could not create socket");
        }
    }
    else    {   /* OK, nothing */  }

    // setup address structure
    if (inet_addr(ip_address.c_str()) == -1)
    {
        struct hostent *he;
        struct in_addr **addr_list;

        // resolve the hostname, its not an ip address
        if ( (he = gethostbyname( ip_address.c_str() ) ) == NULL)
        {
            // gethostbyname failed
            herror("gethostbyname");
            cout << "Failed to resolve hostname\n";
            return false;
        }

        // Cast the h_addr_list to in_addr, since h_addr_list also has the ip address in long format only
        addr_list = (struct in_addr **) he->h_addr_list;

        for (int i = 0; addr_list[i] != NULL; i++)
        {
            // strcpy(ip, inet_ntoa(*addr_list[i]) );
            server.sin_addr = *addr_list[i];
            cout << ip_address << " resolved to " << inet_ntoa(*addr_list[i]) << endl;
            break;
        }
    }

    // plain ip address
    else
    {
        server.sin_addr.s_addr = inet_addr(ip_address.c_str());
    }

    server.sin_family = AF_INET;
    server.sin_port = htons(port);

    // Connect to remote server
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

  tv.tv_sec = secs;    //
  tv.tv_usec = usecs;  // Not init'ing this can cause strange errors

  setsockopt(mysock, SOL_SOCKET, SO_RCVTIMEO, reinterpret_cast<char *>(&tv), sizeof(struct timeval));
}

/**
    Send data to the connected host
*/
bool TcpClient::send_data(string data)
{
    // Send some data
    // cout <<"data_send: " << data << "\n";
    if (send(mysock, data.c_str(), strlen(data.c_str()), 0) < 0)
    {
        perror("Send failed : ");
        return false;
    }

    return true;
}

size_t TcpClient::readline(std::string &buffer, size_t size, std::string eol)
{
  char c;
  int ret;
  buffer = "";

    while (buffer.length() < size)
    {
        ret = read(mysock, &c, 1);  // read a single byte
        if (ret < 1)break;  // error or disconnect
        buffer += c;  // add c to string
        if (c == '\n')break;  // was it an end of line?
    }
    // clean input - remove ending \r\n
    string::size_type pos = 0;
    if ((pos = buffer.find("\r\n", pos)) != string::npos)
    {
      buffer.erase(pos, 2);
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

    // Receive a reply from the server
    if ((len = recv(mysock, buffer, sizeof(buffer), 0)) < 0)
    {
        printf("tcpClient::receive: failed (%d) %s\n", len, strerror(len));  // errno
        buffer[0] = '\0';  // return empty string
    }

    reply = buffer;
    return reply;
}
