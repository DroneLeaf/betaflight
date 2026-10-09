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

// DroneLeaf pushed telemetry (MSPv2 native, unsolicited, direction '>').
// The 0x4C00 block is unused by Betaflight, iNav and the MSP common range.
#define MSP2_LEAF_IMU   0x4C01  // gyro, acc, attitude point samples
#define MSP2_LEAF_BARO  0x4C02  // raw pressure/temperature sample with sample timestamp
#define MSP2_LEAF_GPS   0x4C03  // full GPS solution with solution timestamp
