#!/usr/bin/env python3
"""Generate per-device YAML stubs in documentation/devices/*.yml.

Replaces scripts/devicelist/devicelist.go. Walks the device list emitted by
src/lcec_devices, cross-references each VID:PID against scripts/esi.yml, and
writes a YAML stub for any device that doesn't already have one. Existing
files are left untouched.
"""

import argparse
import io
import os
import subprocess
import sys

from _yaml_mini import dump_mapping, load

FIELD_ORDER = [
    "Device",
    "VendorID",
    "VendorName",
    "PID",
    "Description",
    "DocumentationURL",
    "DeviceType",
    "Notes",
    "SrcFile",
    "TestingStatus",
]


def render_device(d):
    buf = io.StringIO()
    dump_mapping({k: d.get(k, "") for k in FIELD_ORDER}, buf, doc_start=True)
    return buf.getvalue()


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--esi", default="esi.yml")
    p.add_argument("--src", default="src/")
    p.add_argument("--device_directory", default="")
    p.add_argument("--lcec_devices", default="../src/lcec_devices",
                   help="Path to the lcec_devices binary")
    args = p.parse_args()

    print(f"Reading ESI file from {args.esi}")
    with open(args.esi) as f:
        esi_entries = load(f.read()) or []
    print(f"Found {len(esi_entries)} entries")

    esi_map = {}
    for e in esi_entries:
        key = f"{e.get('VendorID', '')}:{e.get('ProductCode', '')}"
        esi_map.setdefault(key, e)

    out = subprocess.check_output([args.lcec_devices], text=True)

    defs = []
    for line in out.splitlines():
        parts = line.split("\t")
        if len(parts) < 4:
            continue
        defs.append({
            "Device": parts[0],
            "VendorID": parts[1],
            "PID": parts[2],
            "SrcFile": parts[3],
            "VendorName": "",
            "Description": "",
            "DocumentationURL": "",
            "DeviceType": "",
            "Notes": "",
            "TestingStatus": "",
        })

    for d in defs:
        key = f"{d['VendorID']}:{d['PID']}"
        esi = esi_map.get(key)
        if not esi:
            d["Description"] = "UNKNOWN"
        else:
            d["VendorName"] = esi.get("Vendor", "") or ""
            vendor_short = d["VendorName"].split(" ", 1)[0] if d["VendorName"] else ""
            name = esi.get("Name", "") or ""
            d["Description"] = f"{vendor_short} {name}".strip()
            d["DocumentationURL"] = esi.get("URL", "") or ""
            d["DeviceType"] = esi.get("DeviceGroup", "") or ""

    for d in defs:
        body = render_device(d)
        if not args.device_directory:
            print(body)
            continue
        fn = os.path.join(args.device_directory, f"{d['Device']}.yml")
        if os.path.exists(fn):
            continue
        with open(fn, "w") as f:
            f.write(body)


if __name__ == "__main__":
    main()
