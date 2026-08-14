#!/usr/bin/env bash
# ==============================================================================
# build.sh - Build & Upload helper script for ELT-Remote dual-device system
# ==============================================================================
set -e

REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$REPO_DIR"

usage() {
    echo "Usage: $0 [beacon|remote|all|test|clean] [--upload]"
    echo ""
    echo "Targets:"
    echo "  beacon    Build RAK4631 WisBlock Beacon firmware"
    echo "  remote    Build Heltec WiFi LoRa 32 V3 Remote firmware"
    echo "  all       Build firmware for both devices (default)"
    echo "  test      Run C++ Native & JavaScript Unit Test Suites"
    echo "  clean     Clean build artifacts"
    echo ""
    echo "Options:"
    echo "  --upload  Upload firmware to connected hardware after build"
    echo ""
    echo "Examples:"
    echo "  $0 test               # Run all C++ and JS unit tests"
    echo "  $0 beacon --upload    # Build and upload to RAK4631"
    echo "  $0 remote --upload    # Build and upload to Heltec V3"
    echo "  $0 all                # Compile both environments"
    exit 1
}

TARGET="all"
DO_UPLOAD=false

for arg in "$@"; do
    case "$arg" in
        beacon)   TARGET="beacon" ;;
        remote)   TARGET="remote" ;;
        all)      TARGET="all" ;;
        test)     TARGET="test" ;;
        clean)    TARGET="clean" ;;
        --upload) DO_UPLOAD=true ;;
        -h|--help) usage ;;
        *) echo "Unknown argument: $arg"; usage ;;
    esac
done

build_beacon() {
    echo "==> [1/2] Building RAK4631 Beacon Firmware..."
    if [ "$DO_UPLOAD" = true ]; then
        pio run -e wiscore_rak4631 --target upload
    else
        pio run -e wiscore_rak4631
    fi
}

build_remote() {
    echo "==> [2/2] Building Heltec WiFi LoRa 32 V3 Remote Firmware..."
    if [ "$DO_UPLOAD" = true ]; then
        pio run -e heltec_wifi_lora_32_V3 --target upload
    else
        pio run -e heltec_wifi_lora_32_V3
    fi
}

run_tests() {
    echo "==> [1/2] Running C++ Native Unity Unit Tests..."
    pio test -e native
    echo ""
    echo "==> [2/2] Running JavaScript Unit Tests..."
    node test/dashboard_test.js
}

clean_builds() {
    echo "==> Cleaning build directories..."
    pio run --target clean
}

case "$TARGET" in
    beacon)
        build_beacon
        ;;
    remote)
        build_remote
        ;;
    all)
        build_beacon
        build_remote
        ;;
    test)
        run_tests
        ;;
    clean)
        clean_builds
        ;;
esac

echo "==> Operation completed successfully!"
