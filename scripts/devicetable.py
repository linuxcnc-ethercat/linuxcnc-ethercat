#!/usr/bin/env python3
"""Render documentation/devices/*.yml as the table in documentation/DEVICES.md.

Replaces scripts/devicetable/devicetable.go. Reads every YAML stub, drops
ones tagged Description: UNKNOWN, prints a Markdown table of the rest.
"""

import argparse
import os
import sys

from _yaml_mini import load


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--path", default="../../documentation/devices/")
    args = p.parse_args()

    entries = []
    other = 0
    for name in sorted(os.listdir(args.path)):
        if not name.endswith(".yml") or name.startswith("."):
            continue
        with open(os.path.join(args.path, name)) as f:
            entry = load(f.read()) or {}
        if (entry.get("Description") or "").strip() == "UNKNOWN":
            other += 1
            continue
        entries.append(entry)

    print("# Devices Supported by LinuxCNC-Ethercat")
    print()
    print("*This is a work in progress, listing all of the devices that LinuxCNC-Ethercat")
    print("has code to support today.  Not all of these are well-tested.*")
    print()
    print("Description | Driver | EtherCAT VID:PID | Device Type | Testing Status | Notes")
    print("----------- | ------ | ---------------- | ----------- | -------------- | ------")

    for e in entries:
        desc = e.get("Description", "") or ""
        url = e.get("DocumentationURL", "") or ""
        if url.startswith("http"):
            name = f"[{desc}]({url})"
        else:
            name = desc

        src = e.get("SrcFile", "") or ""
        if src:
            shortname = os.path.basename(src).replace(".c", "").replace("lcec_", "")
            src_link = f"[{shortname}](../{src})"
        else:
            src_link = "???"

        try:
            vid_int = int(e.get("VendorID", "") or "0", 0)
        except ValueError:
            vid_int = 0
        pid = e.get("PID", "") or ""
        vidpid = f"0x{vid_int:x}:{pid}"

        device_type = e.get("DeviceType", "") or ""
        testing = e.get("TestingStatus", "") or ""
        notes = e.get("Notes", "") or ""

        print(f"{name} | {src_link} | {vidpid} | {device_type} | {testing} | {notes}")

    print()
    if other > 0:
        print(f"There are an additional {other} device(s) supported that do not have enough")
        print("documentation to display here.  Please look at the `documentation/devices/` files")
        print("and update them if you're able.")


if __name__ == "__main__":
    main()
