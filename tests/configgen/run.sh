#!/bin/sh
# Regression test for src/lcec_configgen.
#
# Stubs out `ethercat` with a shim that returns canned bus data, runs
# lcec_configgen against each scenario, and diffs the output against
# the golden XML captured when the C port was first verified
# byte-identical to the original Go implementation.
#
# Run: `tests/configgen/run.sh [/path/to/lcec_configgen]`
# Default binary: ../../src/lcec_configgen relative to this script.

set -e

cd "$(dirname "$0")"
HERE=$(pwd)
BIN=${1:-$HERE/../../src/lcec_configgen}

if [ ! -x "$BIN" ]; then
    echo "lcec_configgen not built at $BIN — run 'make user' in src/ first." >&2
    exit 1
fi

PATH="$HERE/shim:$PATH"
export PATH

fail=0
for scenario in canned/*/; do
    name=$(basename "$scenario")
    golden="golden/$name.xml"
    if [ ! -f "$golden" ]; then
        echo "SKIP: no golden for $name"
        continue
    fi
    actual=$(CANNED_DIR="$HERE/$scenario" "$BIN")
    if printf '%s\n' "$actual" | diff -u "$golden" - > /tmp/configgen-diff.$$.txt; then
        echo "OK: $name"
    else
        echo "FAIL: $name"
        cat /tmp/configgen-diff.$$.txt
        fail=1
    fi
    rm -f /tmp/configgen-diff.$$.txt
done

exit $fail
