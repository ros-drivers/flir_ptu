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
 * $Id$
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

// class declaration
#include <ptu46/ptu46_driver.h>

// serial includes
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <termios.h>
#include <stdio.h>
#include <strings.h>
#include <unistd.h>
#include <stdlib.h>
#include <errno.h>
#include <string.h>
#include <math.h>

namespace PTU46 {

//
// Pan-Tilt Control Class
//

// Constructor opens the serial port, and read the config info from it
PTU46::PTU46(const char * port, int rate) {
    tr = pr = 1;
    fd = -1;

    // open the serial port

    fd = open(port, O_RDWR | O_NOCTTY | O_NDELAY);
    if ( fd<0 ) {
        fprintf(stderr, "Could not open serial device %s\n",port);
        return;
    }
    fcntl(fd,F_SETFL, 0);

    // save the current io settings
    tcgetattr(fd, &oldtio);

    // rtv - CBAUD is pre-POSIX and doesn't exist on OS X
    // should replace this with ispeed and ospeed instead.

    // set up new settings
    struct termios newtio;
    memset(&newtio, 0,sizeof(newtio));
    newtio.c_cflag = /*(rate & CBAUD) |*/ CS8 | CLOCAL | CREAD;
    newtio.c_iflag = IGNPAR;
    newtio.c_oflag = 0;
    newtio.c_lflag = ICANON;

    speed_t sp = B9600;
    switch (rate) {
    case 0:
        sp = B0;
        break;
    case 50:
        sp = B50;
        break;
    case 75:
        sp = B75;
        break;
    case 110:
        sp = B110;
        break;
    case 134:
        sp = B134;
        break;
    case 150:
        sp = B150;
        break;
    case 200:
        sp = B200;
        break;
    case 300:
        sp = B300;
        break;
    case 600:
        sp = B600;
        break;
    case 1200:
        sp = B1200;
        break;
    case 2400:
        sp = B2400;
        break;
    case 4800:
        sp = B4800;
        break;
    case 9600:
        sp = B9600;
        break;
    case 19200:
        sp = B19200;
        break;
    case 38400:
        sp = B38400;
        break;
    default:
        fprintf(stderr,"Failed to set serial baud rate: %d\n", rate);
        Disconnect();
        return;
    }

    if (cfsetispeed(&newtio, sp) < 0 ||   cfsetospeed(&newtio, sp) < 0) {
        fprintf(stderr,"Failed to set serial baud rate: %d\n", rate);
        Disconnect();
        return;
    }
    // activate new settings
    tcflush(fd, TCIFLUSH);
    tcsetattr(fd, TCSANOW, &newtio);

    // now set up the pan tilt camera
    Write(" "); // terse feedback
    usleep(100000);
    tcflush(fd, TCIFLUSH);

    Write("ft "); // terse feedback
    Write("ed "); // disable echo
    Write("ci "); // position mode

    // delay here so data has arrived at serial port so we can flush it
    usleep(200000);
    tcflush(fd, TCIFLUSH);

    // get pan tilt encoder res
    tr = GetRes(PTU46_TILT);
    pr = GetRes(PTU46_PAN);

    PMin = GetLimit(PTU46_PAN, PTU46_MIN);
    PMax = GetLimit(PTU46_PAN, PTU46_MAX);
    TMin = GetLimit(PTU46_TILT, PTU46_MIN);
    TMax = GetLimit(PTU46_TILT, PTU46_MAX);
    PSMin = GetLimit(PTU46_PAN, PTU46_MIN_SPEED);
    PSMax = GetLimit(PTU46_PAN, PTU46_MAX_SPEED);
    TSMin = GetLimit(PTU46_TILT, PTU46_MIN_SPEED);
    TSMax = GetLimit(PTU46_TILT, PTU46_MAX_SPEED);

    if (tr <= 0 || pr <= 0 || PMin == 0 || PMax == 0 || TMin == 0 || TMax == 0) {
        // if limit request failed try resetting the unit and then getting limits..
        Write(" r "); // reset pan-tilt unit (also clears any bad input on serial port)

        // wait for reset to complete
        int len = 0;
        char temp;
        char response[10] = "!T!T!P!P*";

        for (int i = 0; i < 9; ++i) {
            while ((len = read(fd, &temp, 1 )) == 0) {};
            if ((len != 1) || (temp != response[i])) {
                fprintf(stderr,"Error Resetting Pan Tilt unit\n");
                fprintf(stderr,"Stopping access to pan-tilt unit\n");
                Disconnect();
            }
        }

        // delay here so data has arrived at serial port so we can flush it
        usleep(100000);
        tcflush(fd, TCIFLUSH);


        // get pan tilt encoder res
        tr = GetRes(PTU46_TILT);
        pr = GetRes(PTU46_PAN);

        PMin = GetLimit(PTU46_PAN, PTU46_MIN);
        PMax = GetLimit(PTU46_PAN, PTU46_MAX);
        TMin = GetLimit(PTU46_TILT, PTU46_MIN);
        TMax = GetLimit(PTU46_TILT, PTU46_MAX);
        PSMin = GetLimit(PTU46_PAN, PTU46_MIN_SPEED);
        PSMax = GetLimit(PTU46_PAN, PTU46_MAX_SPEED);
        TSMin = GetLimit(PTU46_TILT, PTU46_MIN_SPEED);
        TSMax = GetLimit(PTU46_TILT, PTU46_MAX_SPEED);

        if (tr <= 0 || pr <= 0 || PMin == 0 || PMax == 0 || TMin == 0 || TMax == 0) {
            // if it really failed give up and disable the driver
            fprintf(stderr,"Error getting pan-tilt resolution...is the serial port correct?\n");
            fprintf(stderr,"Stopping access to pan-tilt unit\n");
            Disconnect();
        }
    }
}


PTU46::~PTU46() {
    Disconnect();
}

void PTU46::Disconnect() {
    if (fd > 0) {
        // restore old port settings
        tcsetattr(fd, TCSANOW, &oldtio);
        // close the connection
        close(fd);
        fd = -1;
    }
}

int PTU46::Write(const char * data, int length) {

    if (fd < 0)
        return -1;

    // autocalculate if using short form
    if (length == 0)
        length = strlen(data);

    // ugly error handling, if write fails then shut down unit
    if (write(fd, data, length) < length) {
        fprintf(stderr,"Error writing to Pan Tilt Unit, disabling\n");
        Disconnect();
        return -1;
    }
    return 0;
}


// get radians/count resolution
float PTU46::GetRes(char type) {
    if (fd < 0)
        return -1;
    char cmd[4] = " r ";
    cmd[0] = type;

    // get pan res
    int len = 0;
    Write(cmd);
    len = read(fd, buffer, PTU46_BUFFER_LEN );

    if (len < 3 || buffer[0] != '*') {
        fprintf(stderr,"Error getting pan-tilt res\n");
        return -1;
    }

    buffer[len] = '\0';
    double z = strtod(&buffer[2],NULL);
    z = z/3600; // degrees/count
    return  z*M_PI/180; //radians/count
}

// get position limit
int PTU46::GetLimit(char type, char LimType) {
    if (fd < 0)
        return -1;
    char cmd[4] = "   ";
    cmd[0] = type;
    cmd[1] = LimType;

    // get limit
    int len = 0;
    Write(cmd);
    len = read(fd, buffer, PTU46_BUFFER_LEN );

    if (len < 3 || buffer[0] != '*') {
        fprintf(stderr,"Error getting pan-tilt limit\n");
        return -1;
    }

    buffer[len] = '\0';
    return strtol(&buffer[2],NULL,0);
}


// get position in radians
float PTU46::GetPosition (char type) {
    if (fd < 0)
        return -1;

    char cmd[4] = " p ";
    cmd[0] = type;

    // get pan pos
    int len = 0;
    Write (cmd);
    len = read (fd, buffer, PTU46_BUFFER_LEN );

    if (len < 3 || buffer[0] != '*') {
        fprintf(stderr,"Error getting pan-tilt pos\n");
        return -1;
    }

    buffer[len] = '\0';

    return strtod (&buffer[2],NULL) * GetResolution(type);
}


// set position in radians
bool PTU46::SetPosition (char type, float pos, bool Block) {
    if (fd < 0)
        return false;

    // get raw encoder count to move
    int Count = static_cast<int> (pos/GetResolution(type));

    // Check limits
    if (Count < (type == PTU46_TILT ? TMin : PMin) || Count > (type == PTU46_TILT ? TMax : PMax)) {
        fprintf (stderr,"Pan Tilt Value out of Range: %c %f(%d) (%d-%d)\n", type, pos, Count, (type == PTU46_TILT ? TMin : PMin),(type == PTU46_TILT ? TMax : PMax));
        return false;
    }

    char cmd[16];
    snprintf (cmd,16,"%cp%d ",type,Count);

    // set pos
    int len = 0;
    Write (cmd);
    len = read (fd, buffer, PTU46_BUFFER_LEN );

    if (len <= 0 || buffer[0] != '*') {
        fprintf(stderr,"Error setting pan-tilt pos\n");
        return false;
    }

    if (Block)
        while (GetPosition (type) != pos) {};

    return true;
}

// get speed in radians/sec
float PTU46::GetSpeed (char type) {
    if (fd < 0)
        return -1;

    char cmd[4] = " s ";
    cmd[0] = type;

    // get speed
    int len = 0;
    Write (cmd);
    len = read (fd, buffer, PTU46_BUFFER_LEN );

    if (len < 3 || buffer[0] != '*') {
        fprintf (stderr,"Error getting pan-tilt speed\n");
        return -1;
    }

    buffer[len] = '\0';

    return strtod(&buffer[2],NULL) * GetResolution(type);
}



// set speed in radians/sec
bool PTU46::SetSpeed (char type, float pos) {
    if (fd < 0)
        return false;

    // get raw encoder speed to move
    int Count = static_cast<int> (pos/GetResolution(type));
    // Check limits
    if (abs(Count) < (type == PTU46_TILT ? TSMin : PSMin) || abs(Count) > (type == PTU46_TILT ? TSMax : PSMax)) {
        fprintf (stderr,"Pan Tilt Speed Value out of Range: %c %f(%d) (%d-%d)\n", type, pos, Count, (type == PTU46_TILT ? TSMin : PSMin),(type == PTU46_TILT ? TSMax : PSMax));
        return false;
    }

    char cmd[16];
    snprintf (cmd,16,"%cs%d ",type,Count);

    // set speed
    int len = 0;
    Write (cmd);
    len = read (fd, buffer, PTU46_BUFFER_LEN );

    if (len <= 0 || buffer[0] != '*') {
        fprintf (stderr,"Error setting pan-tilt speed\n");
        return false;
    }
    return true;
}


// set movement mode (position/velocity)
bool PTU46::SetMode (char type) {
    if (fd < 0)
        return false;

    char cmd[4] = "c  ";
    cmd[1] = type;

    // set mode
    int len = 0;
    Write (cmd);
    len = read (fd, buffer, PTU46_BUFFER_LEN );

    if (len <= 0 || buffer[0] != '*') {
        fprintf (stderr,"Error setting pan-tilt move mode\n");
        return false;
    }
    return true;
}

// get ptu mode
char PTU46::GetMode () {
    if (fd < 0)
        return -1;

    // get pan tilt mode
    int len = 0;
    Write ("c ");
    len = read (fd, buffer, PTU46_BUFFER_LEN );

    if (len < 3 || buffer[0] != '*') {
        fprintf (stderr,"Error getting pan-tilt pos\n");
        return -1;
    }

    if (buffer[2] == 'p')
        return PTU46_VELOCITY;
    else if (buffer[2] == 'i')
        return PTU46_POSITION;
    else
        return -1;
}

}
