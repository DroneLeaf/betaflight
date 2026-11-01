#!/usr/bin/env bash
# Build Betaflight for a real flight controller the way the Betaflight cloud
# build server does (build.betaflight.com), with DroneLeaf's standard options.
#
#   droneleaf/build_fc.sh [BOARD] [-DUSE_EXTRA_OPTION ...]
#   droneleaf/build_fc.sh                     # MICOAIR743V2, our racer FC
#   droneleaf/build_fc.sh SKYSTARSF7HDPRO     # any board in src/config/configs
#
# Output: obj/betaflight_<version>_<MCU>_<BOARD>.hex
#
# Environment:
#   BETAFLIGHT_DIR=<dir>  build that checkout instead of this script's own (CI
#                         uses it to build any commit with this script)
#   REV=yes               put the commit in the file name, as upstream's release
#                         builds do: ..._<BOARD>_<commit>.hex
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
# Board & Build, release 2025.12.5, no custom defines):
#   Radio protocol      CRSF
#   Telemetry protocol  (CRSF telemetry, included automatically)
#   OSD protocol        Digital
#   Motor protocol      DSHOT
#   Other options       Acro Trainer, GPS, LED Strip, Optical Flow, Pin IO,
#                       Range Finder, VTX
# The list below is how build.betaflight.com normalises that selection
# (POST /api/builds -> request.options): it adds USE_SERIALRX, USE_TELEMETRY,
# USE_TELEMETRY_CRSF and USE_GPS_PLUS_CODES itself, and its build log runs
# `make <BOARD> EXTRA_FLAGS="-DCLOUD_BUILD -D..."`, as below. Everything else
# (ESC sensor, DShot telemetry, blackbox, ...) comes from common_pre.h, as in
# a cloud build. If the team selection changes, update this list from a new
# build request's response.
set -euo pipefail

cd "${BETAFLIGHT_DIR:-$(dirname "$0")/..}"

board=MICOAIR743V2
if [ $# -gt 0 ] && [[ $1 != -D* ]]; then
    board=$1
    shift
fi

STANDARD_OPTIONS=(
    CLOUD_BUILD
    USE_ACRO_TRAINER       # Acro Trainer
    USE_DSHOT              # Motor protocol: DSHOT
    USE_GPS                # GPS
    USE_GPS_PLUS_CODES     # added by the server with GPS
    USE_LED_STRIP          # LED Strip
    USE_OPTICALFLOW        # Optical Flow
    USE_OSD_HD             # OSD protocol: Digital
    USE_PINIO              # Pin IO
    USE_RANGEFINDER        # Range Finder
    USE_SERIALRX           # added by the server with the radio protocol
    USE_SERIALRX_CRSF      # Radio protocol: CRSF
    USE_TELEMETRY          # added by the server: CRSF telemetry
    USE_TELEMETRY_CRSF
    USE_VTX                # VTX
)
flags="$(printf -- '-D%s ' "${STANDARD_OPTIONS[@]}")$*"

# Board definitions first: until src/config is there the Makefile refuses
# every other target.
make configs
# The ARM compiler, into tools/, only when it isn't there: arm_sdk_install
# downloads it again whenever its archive in downloads/ is gone (a CI cache
# holds tools/ only).
make -s arm_sdk_version > /dev/null 2>&1 || make arm_sdk_install

if [ ! -f "src/config/configs/$board/config.h" ]; then
    echo "unknown board '$board' (not in src/config/configs)" >&2
    exit 1
fi

# Betaflight stamps the firmware with the commit only from a clean tree; with
# uncommitted changes `version` on the FC reports "norevision" instead.
commit=$(git rev-parse --short=9 HEAD)
if [ -n "$(git diff --shortstat)" ]; then
    echo "WARNING: uncommitted changes: the FC will report 'norevision', not $commit" >&2
fi

# Always from scratch: make doesn't recompile when only the commit changed, so
# an incremental build can carry the previous commit's stamp, or objects left
# over from another branch.
make "${board}_clean" > /dev/null

echo "building $board at $(git rev-parse --abbrev-ref HEAD) $commit with: $flags"
make -j"$(nproc)" "$board" EXTRA_FLAGS="$flags" ${REV:+REV=$REV}
ls -lt obj/betaflight_*_"$board"*.hex | head -1
