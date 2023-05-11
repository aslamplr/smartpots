#!/bin/bash

set -e

export ESP_BOARD="esp32"
export ESP_ARCH="xtensa-esp32-none-elf"

case "$1" in
    ""|"release")
        cargo build --target ${ESP_ARCH} --release
        ;;
    "debug")
        cargo build --target ${ESP_ARCH}
        ;;
    *)
        echo "Wrong argument. Only \"debug\"/\"release\" arguments are supported"
        exit 1;;
esac
