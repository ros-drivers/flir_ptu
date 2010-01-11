/*
 * PTU46 ROS Package
 * Copyright (C) 2009 Erik Karulf (erik@cse.wustl.edu)
 *
 */

/*
 *  Player - One Hell of a Robot Server
 *  Copyright (C) 2000  Brian Gerkey   &  Kasper Stoy
 *                      gerkey@usc.edu    kaspers@robotics.usc.edu
 *
 * $Id: ptu46.cc 7798 2009-06-06 09:00:59Z thjc $
 *
 * Author: Toby Collett (University of Auckland)
 * Date: 2003-02-10
 *
 */

/*
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

#ifndef _PTU46_DRIVER_H_
#define _PTU46_DRIVER_H_

#include <termios.h>

// serial defines
#define PTU46_DEFAULT_BAUD 9600
#define PTU46_BUFFER_LEN 255
#define PTU46_DEFAULT_PORT "/dev/ttyUSB1"
#define PTU46_DEFAULT_HZ 10

// command defines
#define PTU46_PAN 'p'
#define PTU46_TILT 't'
#define PTU46_MIN 'n'
#define PTU46_MAX 'x'
#define PTU46_MIN_SPEED 'l'
#define PTU46_MAX_SPEED 'u'
#define PTU46_VELOCITY 'v'
#define PTU46_POSITION 'i'

namespace PTU46 {

//
// Pan-Tilt Control Class
//
class PTU46 {
    public:
        PTU46(const char* port, int rate);
        ~PTU46();

        // get degree/count resolution
        float GetRes (char type);
        // get position limit
        int GetLimit (char type, char LimType);

        // get/set position in degrees
        float GetPos (char type);
        bool SetPos  (char type, float pos, bool Block = false);

        // get/set speed in degrees/sec
        bool SetSpeed  (char type, float speed);
        float GetSpeed (char type);

        // get/set move mode
        bool SetMode (char type);
        char GetMode ();

        bool Open () {
            return fd >0;
        };

        // Position Limits
        int TMin, TMax, PMin, PMax;
        // Speed Limits
        int TSMin, TSMax, PSMin, PSMax;

    protected:
        // pan and tilt resolution
        float tr,pr;

        // serial port descriptor
        int fd;
        struct termios oldtio;

        // read buffer
        char buffer[PTU46_BUFFER_LEN+1];

        int Write(const char * data, int length = 0);

        // cleanly disconnect
        void Disconnect();
};

}

#endif
