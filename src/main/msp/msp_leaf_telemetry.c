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

// DroneLeaf pushed telemetry for the companion computer.
//
// Three unsolicited MSPv2 native frames are pushed to ONE MSP port
// (leaf_messages_msp_port, AUTO = first non-VCP MSP port). All fields are
// little-endian and packed.
//
// LEAF_IMU 0x4C01 (40 B), periodic at leaf_imu_msp_rate:
//   u32 seq, u32 t_fc_us, f32 gyro_dps[3] (gyroADCf), f32 acc_mps2[3] (post-LPF accADC),
//   i16 att_ddeg[3] (attitude.raw roll, pitch, yaw), u8 flags, u8 reserved
//   flags: b0 acc clipped since last push, b1 gyro calibrated, b2 acc calibrated, b3 armed
//
// LEAF_BARO 0x4C02 (24 B), on each new sample, at most leaf_baro_msp_rate:
//   u32 seq, u32 t_fc_us, u32 t_baro_us, u32 baro_sample_count, i32 pressure_pa,
//   i16 temperature_cdeg, u8 flags (b0 present, b1 calibrated), u8 reserved
//
// LEAF_GPS 0x4C03 (62 B), on a new solution at most leaf_gps_msp_rate, else 1 Hz heartbeat:
//   u32 seq, u32 t_fc_us, u32 t_sol_us, u32 sol_count, u32 itow_ms, i32 lat_1e7, i32 lon_1e7,
//   i32 alt_cm, i16 vel_n_cms, i16 vel_e_cms, i16 vel_d_cms, u32 h_acc_mm, u32 v_acc_mm,
//   u32 s_acc_mms, u16 pdop, u16 ground_speed_cms, u16 ground_course_ddeg, u16 nav_interval_ms,
//   u8 fix, u8 num_sat, u8 flags (b0 velned valid, b1 heartbeat), u8 reserved
//
// A frame that does not fit the target port's TX buffer is dropped and counted; its
// sequence number is still consumed so the companion sees the gap.

#include <stdbool.h>
#include <stdint.h>

#include "platform.h"

#ifdef USE_LEAF_TELEMETRY

#include "build/debug.h"

#include "common/streambuf.h"
#include "common/utils.h"

#include "drivers/time.h"

#include "fc/runtime_config.h"

#include "flight/imu.h"

#include "io/displayport_msp.h"
#include "io/gps.h"
#include "io/serial.h"

#include "msp/msp.h"
#include "msp/msp_protocol_v2_leaf.h"
#include "msp/msp_serial.h"

#include "pg/gps.h"
#include "pg/leaf_telemetry.h"

#include "sensors/acceleration.h"
#include "sensors/barometer.h"
#include "sensors/gyro.h"
#include "sensors/sensors.h"

#include "msp/msp_leaf_telemetry.h"

// '$' 'X' '>' + flags u8 + cmd u16 + size u16 + crc u8
#define LEAF_MSP_V2_OVERHEAD        9
#define LEAF_GPS_HEARTBEAT_US       1000000
#define LEAF_GRAVITY_MSS            9.80665f

#define LEAF_IMU_FLAG_ACC_CLIPPED   (1 << 0)
#define LEAF_IMU_FLAG_GYRO_CAL      (1 << 1)
#define LEAF_IMU_FLAG_ACC_CAL       (1 << 2)
#define LEAF_IMU_FLAG_ARMED         (1 << 3)

#define LEAF_BARO_FLAG_PRESENT      (1 << 0)
#define LEAF_BARO_FLAG_CALIBRATED   (1 << 1)

#define LEAF_GPS_FLAG_VELNED_VALID  (1 << 0)
#define LEAF_GPS_FLAG_HEARTBEAT     (1 << 1)

typedef struct leafStream_s {
    uint32_t periodUs;      // 0 = stream off
    timeUs_t nextUs;
    uint32_t seq;
    uint16_t drops;
} leafStream_t;

static leafStream_t imuStream;
static leafStream_t baroStream;
static leafStream_t gpsStream;

static serialPortIdentifier_e pushPort = SERIAL_PORT_NONE;
static bool pushPortFallback = false;
static uint16_t minTxFree = UINT16_MAX;
static uint16_t overruns = 0;

#ifdef USE_ACC
static uint32_t lastAccClipCount = 0;
#endif
#ifdef USE_BARO
static uint32_t lastBaroSampleCount = 0;
static timeUs_t lastBaroPushUs = 0;
#endif
#ifdef USE_GPS
static uint32_t lastGpsSolutionCount = 0;
static timeUs_t lastGpsPushUs = 0;
#endif

static uint32_t rateToPeriodUs(uint16_t rateHz)
{
    if (rateHz == 0) {
        return 0;
    }
    return 1000000 / rateHz;
}

void leafTelemetryInit(void)
{
    imuStream.periodUs = rateToPeriodUs(leafTelemetryConfig()->imu_rate_hz);
    baroStream.periodUs = rateToPeriodUs(leafTelemetryConfig()->baro_rate_hz);
    gpsStream.periodUs = rateToPeriodUs(leafTelemetryConfig()->gps_rate_hz);
}

uint16_t leafTelemetryTaskRateHz(void)
{
    uint16_t rateHz = leafTelemetryConfig()->imu_rate_hz;
    if (leafTelemetryConfig()->baro_rate_hz > rateHz) {
        rateHz = leafTelemetryConfig()->baro_rate_hz;
    }
    if (leafTelemetryConfig()->gps_rate_hz > rateHz) {
        rateHz = leafTelemetryConfig()->gps_rate_hz;
    }
    return rateHz;
}

static serialPortIdentifier_e requestedPushPort(void)
{
    const uint8_t configured = leafTelemetryConfig()->msp_port;
    if (configured == LEAF_MSP_PORT_AUTO || configured >= LEAF_MSP_PORT_COUNT) {
        return SERIAL_PORT_ALL;
    }
    return (serialPortIdentifier_e)(SERIAL_PORT_USART1 + (configured - LEAF_MSP_PORT_UART1));
}

// Periodic deadline: true once per period; resyncs if more than one period behind.
// Due up to half a period early, so scheduler jitter around the task period (stream rate ==
// task rate) never skips a frame.
static bool streamDue(leafStream_t *stream, timeUs_t currentTimeUs)
{
    if (stream->periodUs == 0) {
        return false;
    }
    if (cmpTimeUs(currentTimeUs, stream->nextUs) < -(timeDelta_t)(stream->periodUs / 2)) {
        return false;
    }
    stream->nextUs += stream->periodUs;
    if (cmpTimeUs(currentTimeUs, stream->nextUs) >= 0) {
        stream->nextUs = currentTimeUs + stream->periodUs;
        overruns++;
    }
    return true;
}

// Pushes one frame to pushPort if it fits. Always consumes a sequence number.
static bool leafPush(leafStream_t *stream, uint16_t cmd, uint8_t *data, int dataLen)
{
    stream->seq++;

    const uint32_t txFree = mspSerialTxBytesFreeOnPort(pushPort);
    if (txFree < minTxFree) {
        minTxFree = (uint16_t)MIN(txFree, (uint32_t)UINT16_MAX);
    }
    if (txFree < (uint32_t)(dataLen + LEAF_MSP_V2_OVERHEAD)) {
        stream->drops++;
        return false;
    }

    if (mspSerialPush(pushPort, cmd, data, dataLen, MSP_DIRECTION_REPLY, MSP_V2_NATIVE) <= 0) {
        // Refused (port in CLI mode, released, or buffer filled meanwhile).
        stream->drops++;
        return false;
    }
    return true;
}

static void writeFloat(sbuf_t *dst, float value)
{
    sbufWriteData(dst, &value, sizeof(value));
}

static bool pushImu(timeUs_t currentTimeUs)
{
    uint8_t payload[LEAF_IMU_PAYLOAD_SIZE];
    sbuf_t buf;
    sbuf_t *dst = sbufInit(&buf, payload, payload + sizeof(payload));

    uint8_t flags = 0;
#ifdef USE_ACC
    const uint32_t accClipCount = leafAccClipCount;
    if (accClipCount != lastAccClipCount) {
        flags |= LEAF_IMU_FLAG_ACC_CLIPPED;
    }
    if (accHasBeenCalibrated()) {
        flags |= LEAF_IMU_FLAG_ACC_CAL;
    }
#endif
    if (gyroIsCalibrationComplete()) {
        flags |= LEAF_IMU_FLAG_GYRO_CAL;
    }
    if (ARMING_FLAG(ARMED)) {
        flags |= LEAF_IMU_FLAG_ARMED;
    }

    sbufWriteU32(dst, imuStream.seq);
    sbufWriteU32(dst, (uint32_t)currentTimeUs);
    for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
        writeFloat(dst, gyro.gyroADCf[axis]);
    }
    for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
#ifdef USE_ACC
        writeFloat(dst, acc.accADC.v[axis] * acc.dev.acc_1G_rec * LEAF_GRAVITY_MSS);
#else
        writeFloat(dst, 0.0f);
#endif
    }
    for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
        sbufWriteU16(dst, (uint16_t)attitude.raw[axis]);
    }
    sbufWriteU8(dst, flags);
    sbufWriteU8(dst, 0);

    const bool sent = leafPush(&imuStream, MSP2_LEAF_IMU, payload, sizeof(payload));
#ifdef USE_ACC
    if (sent) {
        // Keep the clip edge until it has actually reached the companion.
        lastAccClipCount = accClipCount;
    }
#endif
    return sent;
}

#ifdef USE_BARO
static bool pushBaro(timeUs_t currentTimeUs, uint32_t sampleCount)
{
    uint8_t payload[LEAF_BARO_PAYLOAD_SIZE];
    sbuf_t buf;
    sbuf_t *dst = sbufInit(&buf, payload, payload + sizeof(payload));

    uint8_t flags = 0;
    if (sensors(SENSOR_BARO)) {
        flags |= LEAF_BARO_FLAG_PRESENT;
    }
    if (baroIsCalibrated()) {
        flags |= LEAF_BARO_FLAG_CALIBRATED;
    }

    int32_t temperature = baro.temperature;
    if (temperature > INT16_MAX) {
        temperature = INT16_MAX;
    } else if (temperature < INT16_MIN) {
        temperature = INT16_MIN;
    }

    sbufWriteU32(dst, baroStream.seq);
    sbufWriteU32(dst, (uint32_t)currentTimeUs);
    sbufWriteU32(dst, leafBaroSampleTimeUs);
    sbufWriteU32(dst, sampleCount);
    sbufWriteU32(dst, (uint32_t)baro.pressure);
    sbufWriteU16(dst, (uint16_t)(int16_t)temperature);
    sbufWriteU8(dst, flags);
    sbufWriteU8(dst, 0);

    return leafPush(&baroStream, MSP2_LEAF_BARO, payload, sizeof(payload));
}
#endif // USE_BARO

#ifdef USE_GPS
static bool pushGps(timeUs_t currentTimeUs, uint32_t solutionCount, bool heartbeat)
{
    uint8_t payload[LEAF_GPS_PAYLOAD_SIZE];
    sbuf_t buf;
    sbuf_t *dst = sbufInit(&buf, payload, payload + sizeof(payload));

    uint8_t flags = 0;
    // UBX fills velned (NAV-PVT or NAV-VELNED); NMEA and MSP sources leave it zero.
    if (gpsConfig()->provider == GPS_UBLOX || gpsConfig()->provider == GPS_VIRTUAL) {
        flags |= LEAF_GPS_FLAG_VELNED_VALID;
    }
    if (heartbeat) {
        flags |= LEAF_GPS_FLAG_HEARTBEAT;
    }

    uint8_t fix = 0;
    if (STATE(GPS_FIX)) {
        fix = 1;
    }

    sbufWriteU32(dst, gpsStream.seq);
    sbufWriteU32(dst, (uint32_t)currentTimeUs);
    sbufWriteU32(dst, leafGpsSolutionTimeUs);
    sbufWriteU32(dst, solutionCount);
    sbufWriteU32(dst, gpsSol.time);
    sbufWriteU32(dst, (uint32_t)gpsSol.llh.lat);
    sbufWriteU32(dst, (uint32_t)gpsSol.llh.lon);
    sbufWriteU32(dst, (uint32_t)gpsSol.llh.altCm);
    sbufWriteU16(dst, (uint16_t)gpsSol.velned.velN);
    sbufWriteU16(dst, (uint16_t)gpsSol.velned.velE);
    sbufWriteU16(dst, (uint16_t)gpsSol.velned.velD);
    sbufWriteU32(dst, gpsSol.acc.hAcc);
    sbufWriteU32(dst, gpsSol.acc.vAcc);
    sbufWriteU32(dst, gpsSol.acc.sAcc);
    sbufWriteU16(dst, gpsSol.dop.pdop);
    sbufWriteU16(dst, gpsSol.groundSpeed);
    sbufWriteU16(dst, gpsSol.groundCourse);
    sbufWriteU16(dst, (uint16_t)MIN(gpsSol.navIntervalMs, (uint32_t)UINT16_MAX));
    sbufWriteU8(dst, fix);
    sbufWriteU8(dst, gpsSol.numSat);
    sbufWriteU8(dst, flags);
    sbufWriteU8(dst, 0);

    return leafPush(&gpsStream, MSP2_LEAF_GPS, payload, sizeof(payload));
}
#endif // USE_GPS

void taskLeafTelemetry(timeUs_t currentTimeUs)
{
    const serialPortIdentifier_e requested = requestedPushPort();
    serialPortIdentifier_e excluded = SERIAL_PORT_NONE;
#ifdef USE_MSP_DISPLAYPORT
    excluded = displayPortMspGetSerial();
#endif
    pushPort = mspSerialResolvePushPort(requested, excluded);
    pushPortFallback = (requested != SERIAL_PORT_ALL) && (pushPort != requested);

    if (pushPort != SERIAL_PORT_NONE) {
        // Order IMU -> GPS -> BARO. Once a frame does not fit, the link is saturated:
        // skip the remaining streams this cycle (they stay due and go next cycle).
        bool linkFull = false;

        if (streamDue(&imuStream, currentTimeUs)) {
            if (!pushImu(currentTimeUs)) {
                linkFull = true;
            }
        }

#ifdef USE_GPS
        if (!linkFull && gpsStream.periodUs > 0) {
            const uint32_t solutionCount = leafGpsSolutionCount;
            const timeDelta_t sinceLastPushUs = cmpTimeUs(currentTimeUs, lastGpsPushUs);
            bool send = false;
            bool heartbeat = false;
            if (solutionCount != lastGpsSolutionCount) {
                // Rate limit with a quarter-period tolerance (scheduler jitter).
                if (sinceLastPushUs >= (timeDelta_t)(gpsStream.periodUs - gpsStream.periodUs / 4)) {
                    send = true;
                }
            } else if (sinceLastPushUs >= LEAF_GPS_HEARTBEAT_US) {
                send = true;
                heartbeat = true;
            }
            if (send) {
                if (pushGps(currentTimeUs, solutionCount, heartbeat)) {
                    lastGpsSolutionCount = solutionCount;
                    lastGpsPushUs = currentTimeUs;
                } else {
                    linkFull = true;
                }
            }
        }
#endif

#ifdef USE_BARO
        // Event-driven: each new pressure sample is pushed once (no duplicates), at most
        // leaf_baro_msp_rate times per second (quarter-period tolerance for scheduler jitter).
        if (!linkFull && baroStream.periodUs > 0) {
            const uint32_t sampleCount = leafBaroSampleCount;
            const timeDelta_t sinceLastPushUs = cmpTimeUs(currentTimeUs, lastBaroPushUs);
            if (sampleCount != lastBaroSampleCount && sinceLastPushUs >= (timeDelta_t)(baroStream.periodUs - baroStream.periodUs / 4)) {
                if (pushBaro(currentTimeUs, sampleCount)) {
                    lastBaroSampleCount = sampleCount;
                    lastBaroPushUs = currentTimeUs;
                }
            }
        }
#endif
    }

    DEBUG_SET(DEBUG_LEAF_TELEMETRY, 0, imuStream.drops);
    DEBUG_SET(DEBUG_LEAF_TELEMETRY, 1, baroStream.drops);
    DEBUG_SET(DEBUG_LEAF_TELEMETRY, 2, gpsStream.drops);
    DEBUG_SET(DEBUG_LEAF_TELEMETRY, 3, MIN(minTxFree, (uint16_t)INT16_MAX));
    DEBUG_SET(DEBUG_LEAF_TELEMETRY, 4, pushPort);
    DEBUG_SET(DEBUG_LEAF_TELEMETRY, 5, overruns);
    DEBUG_SET(DEBUG_LEAF_TELEMETRY, 6, pushPortFallback);
    DEBUG_SET(DEBUG_LEAF_TELEMETRY, 7, (int16_t)(imuStream.seq & 0x7FFF));
}

#endif // USE_LEAF_TELEMETRY
