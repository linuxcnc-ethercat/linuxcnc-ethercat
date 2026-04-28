# `lcec_configgen` regression tests

Snapshot-based test harness for [`src/lcec_configgen`](../../src/lcec_configgen.c).

## Layout

- `shim/ethercat` — fake `ethercat` CLI that serves canned text from
  `$CANNED_DIR` (slaves, sdos, pdos, upload). Used in place of the real
  master by prepending its parent directory to `$PATH`.
- `canned/scenario1/` — synthetic 4-slave bus exercising all three
  driver-inference paths (known driver, basic_cia402, generic).
- `canned/realbus/` — capture from a real 20-slave LS Mecapion/Inovance
  bus. SDOs are empty (slaves did not expose the CoE Object Dictionary
  list service), which exercises the SDO-less generic path with
  `halType=BLANK`.
- `golden/<scenario>.xml` — expected `lcec_configgen` output for each
  scenario.

## Running

```
tests/configgen/run.sh
```

Builds nothing; expects `src/lcec_configgen` to already exist.

## Refreshing the golden after an intentional change

```
for d in tests/configgen/canned/*/; do
    name=$(basename "$d")
    CANNED_DIR="$d" PATH="tests/configgen/shim:$PATH" \
        src/lcec_configgen > "tests/configgen/golden/$name.xml"
done
```

Review the diff, commit if intended.
