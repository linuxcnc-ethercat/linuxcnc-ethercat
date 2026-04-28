#!/bin/sh

set -e

cd "$(dirname "$0")"

DEVICEDIR=../documentation/devices

./devicelist.py --esi=esi.yml --src=../src --device_directory="$DEVICEDIR" --lcec_devices=../src/lcec_devices
