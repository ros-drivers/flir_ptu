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

/**
 * \brief PTU46 Pan Tilt Unit Driver
 * Copyright (C) 2009 Erik Karulf (erik@cse.wustl.edu)
 *
 *  Player - One Hell of a Robot Server
 *  Copyright (C) 2000  Brian Gerkey   &  Kasper Stoy
 *                      gerkey@usc.edu    kaspers@robotics.usc.edu
 *
 * $Id: ptu46.cc 7798 2009-06-06 09:00:59Z thjc $
 *
 * Author: Toby Collett (University of Auckland)
 * Date: 2003-02-10
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
class PTU46 {
    public:
        /** Constructor - opens port
         * \param port Filename where PTU is connected
         * \param rate rate Baud rate to use */
        PTU46(const char* port, int rate);
        ~PTU46();

        /** \return true if the ptu is open/ready */
        bool isOpen () {
            return fd > 0;
        };

        /**
         * \param type 'p' or 't'
         * \return position in radians
         */
        float GetPosition (char type);

        /**
         * \param type 'p' or 't'
         * \return speed in radians/second
         */
        float GetSpeed (char type);

        /**
         * \param type 'p' or 't'
         * \return resolution in radians/count
         */
        float GetResolution (char type) {
            return (type == PTU46_TILT ? tr : pr);
        }

        /**
         * \param type 'p' or 't'
         * \return Minimum position in radians
         */
        float GetMin (char type) {
            return GetResolution(type)*(type == PTU46_TILT ? TMin : PMin);
        }
        /**
         * \param type 'p' or 't'
         * \return Maximum position in radians
         */
        float GetMax (char type) {
            return GetResolution(type)*(type == PTU46_TILT ? TMax : PMax);
        }

        /**
         * \param type 'p' or 't'
         * \return Minimum speed in radians/second
         */
        float GetMinSpeed (char type) {
            return GetResolution(type)*(type == PTU46_TILT ? TSMin : PSMin);
        }
        /**
         * \param type 'p' or 't'
         * \return Maximum speed in radians/second
         */
        float GetMaxSpeed (char type) {
            return GetResolution(type)*(type == PTU46_TILT ? TSMax : PSMax);
        }



        /**
         * Moves the PTU to the desired position. If Block is true,
         * the call blocks until the desired position is reached
         * \param type 'p' or 't'
         * \param pos desired position in radians
         * \param Block block until ready
         * \return True if successfully sent command
        */
        bool SetPosition  (char type, float pos, bool Block = false);

        /**
         * Sets the desired speed in radians/second
         * \param type 'p' or 't'
         * \param speed desired speed in radians/second
         * \return True if successfully sent command
        */
        bool SetSpeed  (char type, float speed);

        /**
         * Set the control mode, position or velocity
         * \param type 'v' for velocity, 'i' for position
         * \return True if successfully sent command
         */
        bool SetMode (char type);

        /**
         * Get the control mode, position or velocity
         * \return 'v' for velocity, 'i' for position
         */
        char GetMode ();

    private:
        /** Get radian/count resolution
         * \param type 'p' or 't'
         * \return pan resolution if type=='p', tilt resolution if type=='t'
         */
        float GetRes (char type);

        /** Get limiting position/speed in counts or counts/second
         *
         * \param type 'p' or 't' (pan or tilt)
         * \param LimType {'n', 'x', 'l', 'u'} (min position, max position, min speed, max speed)
         * \return limiting position/speed
         */
        int GetLimit (char type, char LimType);



        // Position Limits
        int TMin;	///< Min Tilt in Counts
        int TMax;	///< Max Tilt in Counts
        int PMin;	///< Min Pan in Counts
        int PMax;	///< Max Pan in Counts

        // Speed Limits
        int TSMin;	///< Min Tilt Speed in Counts/second
        int TSMax;	///< Max Tilt Speed in Counts/second
        int PSMin;	///< Min Pan Speed in Counts/second
        int PSMax;	///< Max Pan Speed in Counts/second

    protected:
        float tr;	///< tilt resolution (rads/count)
        float pr; 	///< pan resolution (rads/count)

        int fd;		///< serial port descriptor
        struct termios oldtio; ///< old terminal settings

        char buffer[PTU46_BUFFER_LEN+1]; ///< read buffer

        /**
         * Write Data to PTU
         * \param data
         * \param length number of chars (default=0)
         * \return 0 if successful, -1 if failure
        */
        int Write(const char * data, int length = 0);

        /** Cleanly disconnect
         */
        void Disconnect();
};

}

#endif
