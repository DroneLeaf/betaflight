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

#include "platform.h"

#ifdef USE_LEAF_TELEMETRY

#include "pg/pg_ids.h"
#include "pg/leaf_telemetry.h"

PG_REGISTER_WITH_RESET_TEMPLATE(leafTelemetryConfig_t, leafTelemetryConfig, PG_LEAF_TELEMETRY_CONFIG, 0);

PG_RESET_TEMPLATE(leafTelemetryConfig_t, leafTelemetryConfig,
    .imu_rate_hz = 200,
    .baro_rate_hz = 20,     // DPS310 (MICOAIR743) produces a sample every ~50 ms
    .gps_rate_hz = 20,
    .msp_port = LEAF_MSP_PORT_AUTO,
);

#endif // USE_LEAF_TELEMETRY
