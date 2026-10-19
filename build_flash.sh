#!/usr/bin/env bash
# Build, flash and monitor the A2DP sink firmware on Linux.
#
# Usage: ./build_flash.sh [options] [idf.py actions...]
#
#   -b, --board 8mb|4mb   Flash size / partition layout (default: 8mb)
#   -H, --headless        No LED matrix, rotary encoder or beat LED
#   -p, --port PORT       Serial port, e.g. /dev/ttyUSB0 (default: auto-detect)
#   -B, --build-dir DIR   Build directory, relative to bt_audio_sink/
#                         (default: build, build_4mb, build_headless, ...)
#   -r, --recovery        Build and flash the WiFi recovery firmware instead
#                         (8MB boards only; flash the main firmware first)
#   -h, --help            Show this help
#
# Actions default to "build flash monitor". Examples:
#   ./build_flash.sh                          # 8MB board, build + flash + monitor
#   ./build_flash.sh -b 4mb -H -p /dev/ttyUSB0 build app-flash
#   ./build_flash.sh -r -p /dev/ttyUSB0       # recovery partition
#   ./build_flash.sh -b 4mb menuconfig

set -euo pipefail

ROOT=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd -P)
PROJECT="$ROOT/bt_audio_sink"

board=8mb
headless=0
port=
build_dir=
recovery=0
actions=()

usage() { sed -n '2,/^$/{s/^# \{0,1\}//;p}' "${BASH_SOURCE[0]}"; }

while [ $# -gt 0 ]; do
    case "$1" in
        -b|--board)     board=${2:?missing board}; shift 2 ;;
        -H|--headless)  headless=1; shift ;;
        -p|--port)      port=${2:?missing port}; shift 2 ;;
        -B|--build-dir) build_dir=${2:?missing build dir}; shift 2 ;;
        -r|--recovery)  recovery=1; shift ;;
        -h|--help)      usage; exit 0 ;;
        -*)             echo "Unknown option: $1" >&2; usage >&2; exit 2 ;;
        *)              actions+=("$1"); shift ;;
    esac
done

case "$board" in
    8mb|4mb) ;;
    *) echo "Unknown board '$board' (use 8mb or 4mb)" >&2; exit 2 ;;
esac

# Set up the ESP-IDF environment from the patched tree in this repository,
# unless it is already active.
if [ "${IDF_PATH:-}" != "$ROOT/esp-idf" ] || ! command -v idf.py >/dev/null 2>&1; then
    if ! . "$ROOT/esp-idf/export.sh" >/dev/null; then
        echo "ESP-IDF environment setup failed. Install the tools first:" >&2
        echo "  cd esp-idf && ./install.sh esp32" >&2
        exit 1
    fi
fi

port_args=()
[ -n "$port" ] && port_args=(-p "$port")

if [ "$recovery" = 1 ]; then
    if [ "$board" != 8mb ]; then
        echo "The 4MB layout has no recovery partition." >&2
        exit 2
    fi
    cd "$PROJECT/recovery"
    idf.py build
    # Write only the recovery app to its partition (ota_2 at 0x10000). The
    # bootloader and partition table come from the main firmware flash, so
    # the main firmware's flash settings are kept.
    python -m esptool --chip esp32 "${port_args[@]}" -b 460800 \
        --before default_reset --after hard_reset \
        write_flash 0x10000 build/recovery.bin
    exit 0
fi

defaults="sdkconfig.defaults"
suffix=
if [ "$board" = 4mb ]; then
    defaults="$defaults;sdkconfig.defaults.4mb"
    suffix=_4mb
fi
if [ "$headless" = 1 ]; then
    defaults="$defaults;sdkconfig.defaults.headless"
    suffix=${suffix}_headless
fi

# The default 8MB build uses the checked-in bt_audio_sink/sdkconfig. Other
# variants keep their own sdkconfig inside their build directory, generated
# from the layered defaults on the first build.
idf_args=()
if [ -n "$suffix" ] || [ -n "$build_dir" ]; then
    build_dir=${build_dir:-build$suffix}
    idf_args=(-B "$build_dir" -D SDKCONFIG="$build_dir/sdkconfig"
              -D SDKCONFIG_DEFAULTS="$defaults")
fi

[ ${#actions[@]} -eq 0 ] && actions=(build flash monitor)

cd "$PROJECT"
exec idf.py "${idf_args[@]}" "${port_args[@]}" "${actions[@]}"
