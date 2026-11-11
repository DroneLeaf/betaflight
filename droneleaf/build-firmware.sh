#!/usr/bin/env bash
# DroneLeaf firmware builds, shared by this repo's release workflow and software-stack CI.
#
#   build-firmware.sh targets          JSON array of board targets (CI matrix)
#   build-firmware.sh build <TARGET>   build one board, or SITL
#   build-firmware.sh smoke-sitl <bin> run the SITL binary briefly and fail on a crash
#   build-firmware.sh release-info     values for config-betaflight_manager_requirements
set -euo pipefail

BETAFLIGHT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "${BETAFLIGHT_DIR}"

TARGETS="MICOAIR743V2 MICOAIR743 KAKUTEH7V2"

# Equivalent of the Betaflight App cloud build: CRSF, telemetry auto, digital OSD, DSHOT,
# Acro Trainer, GPS, LED Strip, Optical Flow, Pin IO, Range Finder, VTX.
# CLOUD_BUILD drops the local-build default feature set; USE_SERIALRX and USE_TELEMETRY
# are required with it or common_post.h removes the CRSF receiver and telemetry.
OPTIONS="CLOUD_BUILD \
USE_SERIALRX USE_SERIALRX_CRSF \
USE_TELEMETRY USE_TELEMETRY_CRSF \
USE_OSD_HD \
USE_DSHOT \
USE_ACRO_TRAINER USE_GPS USE_LED_STRIP USE_PINIO USE_VTX \
USE_RANGEFINDER_TF USE_OPTICALFLOW_MT"

FORKNAME="droneleaf"
SITL_SMOKE_DURATION_S=8

usage() {
    sed -n '2,8p' "${BASH_SOURCE[0]}" >&2
    exit 2
}

print_targets() {
    local json=""
    local target
    for target in ${TARGETS}; do
        if [ -n "${json}" ]; then
            json="${json},"
        fi
        json="${json}\"${target}\""
    done
    echo "[${json}]"
}

build_target() {
    local target="$1"
    if [ "${target}" = "SITL" ]; then
        make TARGET=SITL FORKNAME="${FORKNAME}" EXTRA_FLAGS=-Werror
        return
    fi
    case " ${TARGETS} " in
        *" ${target} "*) ;;
        *)
            echo "Unknown target '${target}', expected one of: ${TARGETS} SITL" >&2
            exit 2
            ;;
    esac
    make configs
    make "${target}_rev" FORKNAME="${FORKNAME}" EXTRA_FLAGS=-Werror OPTIONS="${OPTIONS}"
}

smoke_sitl() {
    local sitl_bin="$1"
    local log_file="sitl.log"
    chmod +x "${sitl_bin}"
    local exit_code=0
    timeout "${SITL_SMOKE_DURATION_S}s" "${sitl_bin}" > "${log_file}" 2>&1 || exit_code=$?
    cat "${log_file}"
    # 124: still running when timeout stopped it, which is the healthy outcome.
    if [ "${exit_code}" -ne 124 ]; then
        echo "SITL exited early with code ${exit_code}" >&2
        exit 1
    fi
    if grep -iE "segmentation fault|core dumped|panic|fatal" "${log_file}"; then
        echo "SITL crash detected" >&2
        exit 1
    fi
}

release_info() {
    echo "fc_version: $(make --no-print-directory version)"
    echo "required_build_date: $(date -u +%F)"
    echo "required_git_revision: $(git rev-parse --short=9 HEAD)"
    echo "targets: ${TARGETS} SITL"
    echo "options: ${OPTIONS}"
}

if [ $# -lt 1 ]; then
    usage
fi

case "$1" in
    targets)
        print_targets
        ;;
    build)
        if [ $# -ne 2 ]; then
            usage
        fi
        build_target "$2"
        ;;
    smoke-sitl)
        if [ $# -ne 2 ]; then
            usage
        fi
        smoke_sitl "$2"
        ;;
    release-info)
        release_info
        ;;
    *)
        usage
        ;;
esac
