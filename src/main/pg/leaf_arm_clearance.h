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

typedef struct leafArmClearanceConfig_s {
    uint8_t required;      // OFF lets the board arm without LeafFC (bench use)
    uint16_t timeout_ms;   // a clearance older than this no longer grants arming
} leafArmClearanceConfig_t;

PG_DECLARE(leafArmClearanceConfig_t, leafArmClearanceConfig);
