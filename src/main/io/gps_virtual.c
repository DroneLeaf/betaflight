/*
 * This file is part of Betaflight.
 *
 * Betaflight is free software. You can redistribute this software
 * and/or modify this software under the terms of the GNU General
 * Public License as published by the Free Software Foundation,
 * either version 3 of the License, or (at your option) any later
 * version.
 *
 * Betaflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public
 * License along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdbool.h>
#include <stdint.h>

#include "platform.h"

#ifdef USE_VIRTUAL_GPS

#include <math.h>
#include <pthread.h>

#include "io/gps_virtual.h"

// setVirtualGPS runs on the simulator's FDM thread, getVirtualGPS on the main loop's GPS
// task: the whole solution is swapped under one lock, so a reader never sees lat from one
// FDM packet and lon / velocity from the next.
static gpsSolutionData_t gpsVirtualData;
static pthread_mutex_t gpsVirtualLock = PTHREAD_MUTEX_INITIALIZER;

// Nearest, not truncated toward zero (truncation biased every field by up to one unit).
static int32_t roundToInt32(double value)
{
    return (int32_t)lround(value);
}

static int16_t roundToInt16(double value)
{
    const long rounded = lround(value);
    return (int16_t)(rounded > INT16_MAX ? INT16_MAX : (rounded < INT16_MIN ? INT16_MIN : rounded));
}

static uint16_t roundToUint16(double value)
{
    const long rounded = lround(value);
    return (uint16_t)(rounded > UINT16_MAX ? UINT16_MAX : (rounded < 0 ? 0 : rounded));
}

void setVirtualGPS(double latitude, double longitude, double altiutude, double speed, double speed3D, double course, double velN, double velE, double velD)
{
    gpsSolutionData_t sol = { 0 };
    sol.numSat = 12;    // satellites_in_view
    sol.acc.hAcc = 500; // horizontal_pos_accuracy - convert cm to mm
    sol.acc.vAcc = 500; // vertical_pos_accuracy - convert cm to mm
    sol.acc.sAcc = 400; // horizontal_vel_accuracy - convert cm to mm
    sol.dop.pdop = 10; // hdop in 4.4 and earlier, pdop in 4.5 and above
    sol.llh.lon = roundToInt32(longitude * GPS_DEGREES_DIVIDER);
    sol.llh.lat = roundToInt32(latitude * GPS_DEGREES_DIVIDER);
    sol.llh.altCm = roundToInt32(altiutude * 100.0); // alt, cm
    sol.groundSpeed = roundToUint16(speed * 100.0);  // cm/sec
    sol.speed3d = roundToUint16(speed3D * 100.0);    // cm/sec
    const uint16_t courseDdeg = roundToUint16(course * 10.0);
    sol.groundCourse = courseDdeg >= 3600 ? courseDdeg - 3600 : courseDdeg; // decidegrees, [0, 3600)
    sol.velned.velN = roundToInt16(velN * 100.0);   // cm/sec
    sol.velned.velE = roundToInt16(velE * 100.0);   // cm/sec
    sol.velned.velD = roundToInt16(velD * 100.0);   // cm/sec

    pthread_mutex_lock(&gpsVirtualLock);
    gpsVirtualData = sol;
    pthread_mutex_unlock(&gpsVirtualLock);
}

void getVirtualGPS(gpsSolutionData_t *gpsSolData)
{
    pthread_mutex_lock(&gpsVirtualLock);
    *gpsSolData = gpsVirtualData;
    pthread_mutex_unlock(&gpsVirtualLock);
}
#endif
