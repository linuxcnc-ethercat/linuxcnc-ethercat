#!/bin/sh

set -e

cd "$(dirname "$0")"

DEVICEDIR=../documentation/devices
DEVICESMD=../documentation/DEVICES.md
DEVICESNEW=$DEVICESMD-new

if ./devicetable.py --path="$DEVICEDIR" > "$DEVICESNEW"; then
    mv "$DEVICESNEW" "$DEVICESMD"
else
    rm -f "$DEVICESNEW"
fi
