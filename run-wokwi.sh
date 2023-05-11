#!/usr/bin/env bash

set -e

BUILD_MODE=""
case "$1" in
    ""|"release")
        bash build.sh
        BUILD_MODE="release"
        ;;
    "debug")
        bash build.sh debug
        BUILD_MODE="debug"
        ;;
    *)
        echo "Wrong argument. Only \"debug\"/\"release\" arguments are supported"
        exit 1;;
esac

# TODO: Update ESP_BOARD and ESP_ELF
export ESP_BOARD="esp32"
export ESP_ELF="esp32_74hc595"

export WOKWI_PROJECT_ID="332616143815574099"

export WOKWI_PROJECT_ID="332616143815574099"
export ESP_ARCH="xtensa-esp32-none-elf"

wokwi-server --chip ${ESP_BOARD} --id ${WOKWI_PROJECT_ID} target/${ESP_ARCH}/${BUILD_MODE}/${ESP_ELF}
