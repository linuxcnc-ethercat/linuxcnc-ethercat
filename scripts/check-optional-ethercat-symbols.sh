#!/bin/sh
# Fork-only APIs must be resolved with dlsym so binaries built against our
# libethercat headers still load with vanilla libethercat at runtime.
set -eu

cd "$(dirname "$0")/.."
status=0
for binary in src/lcec.so src/lcec_conf src/lcec_devices src/lcec_configgen; do
    if [ ! -f "${binary}" ]; then
        echo "error: missing build output: ${binary}" >&2
        status=1
        continue
    fi
    if objdump -T "${binary}" | grep 'LIBETHERCAT_1\.6\.13'; then
        echo "error: ${binary} has a hard LIBETHERCAT_1.6.13 dependency" >&2
        status=1
    fi
done
exit "${status}"
