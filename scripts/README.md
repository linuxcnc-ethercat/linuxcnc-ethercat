# Scripts

This directory contains tools used to maintain this package; nothing
in here should be needed to build or use LinuxCNC-Ethercat.

## `update-esi.sh`

This updates `scripts/esi.yml` using data from manufacturer websites.
This should provide details on all known Ethercat devices.  This data
is used by `update-devicelist.sh`, below.

Currently, we fetch ESI data from Beckhoff and Omron.  Additional
manufacturers can be added fairly easily.

The XML decoding is implemented in `scripts/esidecoder.py`.

## `update-devicelist.sh`

This shell script generates a list of all Ethercat PIDs supported by
all drivers included in this package and creates files in
`documentation/devices/*.yml` that describe the supported devices.
This uses ESI data from `update-esi.sh`, above.

The cross-reference + YAML emit is implemented in
`scripts/devicelist.py`.

## `update-devicetable.sh`

This updates `documentation/DEVICES.md`, using the data in
`documentation/devices/*.yml`, to show a table of all supported
devices.

The Markdown rendering is implemented in `scripts/devicetable.py`.

## Requirements

The maintainer scripts require Python 3 only (stdlib). No extra
packages are needed.
