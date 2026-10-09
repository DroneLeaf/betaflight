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

#include "pg/pg.h"

// Values of leaf_messages_msp_port (index into lookupTableLeafMspPort).
// LEAF_MSP_PORT_AUTO selects the first non-VCP MSP port; LEAF_MSP_PORT_UART1 + n
// selects SERIAL_PORT_USART1 + n.
typedef enum {
    LEAF_MSP_PORT_AUTO = 0,
    LEAF_MSP_PORT_UART1,
    LEAF_MSP_PORT_UART2,
    LEAF_MSP_PORT_UART3,
    LEAF_MSP_PORT_UART4,
    LEAF_MSP_PORT_UART5,
    LEAF_MSP_PORT_UART6,
    LEAF_MSP_PORT_UART7,
    LEAF_MSP_PORT_UART8,
    LEAF_MSP_PORT_UART9,
    LEAF_MSP_PORT_UART10,
    LEAF_MSP_PORT_COUNT
} leafMspPort_e;

typedef struct leafTelemetryConfig_s {
    uint16_t imu_rate_hz;   // LEAF_IMU push rate, 0 = off
    uint16_t baro_rate_hz;  // LEAF_BARO max push rate (sent once per new sample), 0 = off
    uint16_t gps_rate_hz;   // LEAF_GPS max push rate (sent on a new solution), 0 = off
    uint8_t msp_port;       // leafMspPort_e
} leafTelemetryConfig_t;

PG_DECLARE(leafTelemetryConfig_t, leafTelemetryConfig);
