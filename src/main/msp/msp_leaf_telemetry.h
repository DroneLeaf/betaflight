/*
 * This file is part of Cleanflight and Betaflight.
 *
 * Cleanflight and Betaflight are free software. You can redistribute
 * this software and/or modify this software under the terms of the
 * GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * Cleanflight and Betaflight are distributed in the hope that they
 * will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

#include <stdint.h>

#include "common/time.h"

// Payload sizes of the pushed LEAF_* frames (see msp_leaf_telemetry.c for layouts).
#define LEAF_IMU_PAYLOAD_SIZE   40
#define LEAF_BARO_PAYLOAD_SIZE  24
#define LEAF_GPS_PAYLOAD_SIZE   62

void leafTelemetryInit(void);
// Highest configured stream rate in Hz (the task rate), 0 when every stream is off.
uint16_t leafTelemetryTaskRateHz(void);
void taskLeafTelemetry(timeUs_t currentTimeUs);
