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
#include <stdbool.h>
#include <stdint.h>

#include "platform.h"

#ifdef USE_LEAF_TELEMETRY

#include "common/time.h"

#include "drivers/time.h"

#include "fc/leaf_arm_clearance.h"

#include "pg/leaf_arm_clearance.h"

static bool lastCleared = false;
static bool received = false;
static timeMs_t lastReceivedMs = 0;

void leafArmClearanceReceive(bool cleared)
{
    lastCleared = cleared;
    received = true;
    lastReceivedMs = millis();
}

bool leafArmClearanceDenies(void)
{
    if (!leafArmClearanceConfig()->required) {
        return false;
    }
    if (!received || !lastCleared) {
        return true;
    }
    return cmpTimeMs(millis(), lastReceivedMs) > (timeDelta_t)leafArmClearanceConfig()->timeout_ms;
}

#endif // USE_LEAF_TELEMETRY
