#!/usr/bin/env bash
# Build Betaflight for a real flight controller the way the Betaflight cloud
# build server does (build.betaflight.com), with DroneLeaf's standard options.
#
#   droneleaf/build_fc.sh <BOARD> [-DUSE_EXTRA_OPTION ...]
#   e.g. droneleaf/build_fc.sh MICOAIR743V2
#
# Output: obj/betaflight_<version>_<MCU>_<BOARD>.hex
#
# Why this and not a plain `make CONFIG=<BOARD>`:
#   A plain local build is a "classic" build: every optional feature comes from
#   common_pre.h, which is read AFTER the board's config.h. Board defaults that
#   depend on an option (GPS_UART needs USE_GPS, MSP_DISPLAYPORT_UART needs
#   USE_OSD_HD) are therefore missing, so a settings reset leaves no GPS and no
#   HD OSD port. The cloud build passes CLOUD_BUILD and each option as -D on the
#   command line, so they exist before config.h is read. This script does the
#   same, so a branch build carries exactly the options a cloud build of the
#   same release would, plus the branch's own code.
#
# STANDARD_OPTIONS is the team's Betaflight App selection (Firmware Flasher ->
# Board & Build): radio CRSF (telemetry automatically included), OSD Digital,
# motor DSHOT, options Acro Trainer, GPS, LED Strip, Optical Flow, Pin IO,
# Range Finder, VTX. The list below is how build.betaflight.com normalises that
# selection for 2025.12.5 (POST /api/builds -> request.options); its build log
# runs `make <BOARD> EXTRA_FLAGS="-DCLOUD_BUILD -D..."`, as below. If the team
# selection changes, update this list from a new build request's response.
set -euo pipefail

cd "$(dirname "$0")/.."

if [ $# -lt 1 ]; then
    echo "usage: $0 <BOARD> [-DUSE_EXTRA_OPTION ...]" >&2
    exit 2
fi
board=$1
shift

STANDARD_OPTIONS=(
    CLOUD_BUILD
    USE_ACRO_TRAINER
    USE_DSHOT
    USE_GPS
    USE_GPS_PLUS_CODES
    USE_LED_STRIP
    USE_OPTICALFLOW
    USE_OSD_HD
    USE_PINIO
    USE_RANGEFINDER
    USE_SERIALRX
    USE_SERIALRX_CRSF
    USE_TELEMETRY
    USE_TELEMETRY_CRSF
    USE_VTX
)
flags="$(printf -- '-D%s ' "${STANDARD_OPTIONS[@]}")$*"

make arm_sdk_install           # the ARM compiler, into tools/ (no-op once installed)
make configs                   # board definitions (the src/config submodule)

if [ ! -f "src/config/configs/$board/config.h" ]; then
    echo "unknown board '$board' (not in src/config/configs)" >&2
    exit 1
fi

echo "building $board at $(git rev-parse --short=9 HEAD) with: $flags"
make -j"$(nproc)" "$board" EXTRA_FLAGS="$flags"
ls -l obj/betaflight_*_"$board".hex
