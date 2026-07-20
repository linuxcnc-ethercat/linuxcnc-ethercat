#!/bin/sh
# Verify that the committed ESI-generated coupler headers match what
# scripts/esi2coupler.py produces from the ESI files in scripts/esi/.
# Run by CI; run manually after editing the generator or updating an ESI.
set -eu

cd "$(dirname "$0")/.."

fail=0
check() {
    esi="$1"; family="$2"; header="$3"
    if ! python3 scripts/esi2coupler.py --esi "$esi" --family "$family" 2>/dev/null | diff -u "$header" - > /tmp/gen_diff.$$ 2>&1; then
        echo "ERROR: $header is stale; regenerate with:"
        echo "  ./scripts/esi2coupler.py --esi $esi --family $family > $header"
        sed -n 1,20p /tmp/gen_diff.$$
        fail=1
    else
        echo "OK: $header matches $esi"
    fi
    rm -f /tmp/gen_diff.$$
}

check scripts/esi/UTRIO-UC20-RTU-EC-A_20251104.xml uc20 src/devices/lcec_mdp_uc20.h
check scripts/esi/INOVANCE_GL20_RTU_ECT32_3.1.2.0.xml gl20 src/devices/lcec_mdp_gl20.h

exit $fail
