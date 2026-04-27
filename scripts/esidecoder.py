#!/usr/bin/env python3
"""Decode EtherCAT ESI XML files into a single YAML index.

Replaces scripts/esidecoder/esidecoder.go. Reads every *.xml in --esi-directory,
extracts per-device metadata, emits the consolidated list to --output.
"""

import argparse
import os
import sys
import xml.etree.ElementTree as ET

from _yaml_mini import dump_sequence

EN_LCID = "1033"


def fix_hex(value, nibbles):
    """Normalize ESI hex/int to "0x"-prefixed lowercase, padded to nibbles."""
    if not value:
        return ""
    if value[0] == "#":
        # ESI uses #xXXXX; strip the '#' and reparse.
        n = int("0x" + value[2:], 16)
    else:
        n = int(value, 0)
    return f"0x{n:0{nibbles}x}"


def english_text(elements):
    """Pick the English (LcId=1033) text from a list of ESI <Name>/<URL> nodes,
    falling back to the first one if there's no LCID match."""
    if not elements:
        return ""
    for el in elements:
        if el.get("LcId") == EN_LCID:
            return (el.text or "").strip()
    return (elements[0].text or "").strip()


def parse_esi_file(path):
    tree = ET.parse(path)
    root = tree.getroot()

    vendor_id = fix_hex((root.findtext("Vendor/Id") or "").strip(), 8)

    vendor_names = root.findall("Vendor/Name")
    vendor = "Unknown"
    if vendor_names:
        vendor = (vendor_names[0].text or "").strip() or vendor
        for v in vendor_names:
            if v.get("LcId") == EN_LCID:
                vendor = (v.text or "").strip() or vendor

    group_map = {}
    for g in root.findall("Descriptions/Groups/Group"):
        gtype = g.findtext("Type") or ""
        gname = english_text(g.findall("Name"))
        if gtype:
            group_map[gtype] = gname

    devices = []
    for d in root.findall("Descriptions/Devices/Device"):
        type_el = d.find("Type")
        if type_el is None:
            continue

        product_code = fix_hex(type_el.get("ProductCode", ""), 8)
        revision_no = fix_hex(type_el.get("RevisionNo", ""), 8)
        short_type = (type_el.text or "").strip()

        name = english_text(d.findall("Name"))
        url = english_text(d.findall("URL"))
        group_type = (d.findtext("GroupType") or "").strip()
        group_name = group_map.get(group_type, "")

        # Pick the first OpMode with AssignActivate >= 0x300 for DC sync info.
        assign_activate = ""
        sync_cycle0 = sync_cycle1 = ""
        sync_factor0 = sync_factor1 = ""
        sync_shift0 = sync_shift1 = ""
        for op in d.findall("Dc/OpMode"):
            raw = op.findtext("AssignActivate") or ""
            if not raw:
                continue
            normalized = fix_hex(raw, 3)
            try:
                aa = int(normalized, 0)
            except ValueError:
                continue
            if aa >= 0x300:
                assign_activate = normalized
                cs0 = op.find("CycleTimeSync0")
                cs1 = op.find("CycleTimeSync1")
                sync_shift0 = (op.findtext("ShiftTimeSync0") or "").strip()
                sync_shift1 = (op.findtext("ShiftTimeSync1") or "").strip()
                if cs0 is not None:
                    sync_cycle0 = (cs0.text or "").strip()
                    sync_factor0 = cs0.get("Factor", "").strip()
                if cs1 is not None:
                    sync_cycle1 = (cs1.text or "").strip()
                    sync_factor1 = cs1.get("Factor", "").strip()
                break

        entry = {
            "Type": short_type,
            "ProductCode": product_code,
            "RevisionNo": revision_no,
            "URL": url,
            "Name": name,
            "DeviceGroup": group_name,
            "Vendor": vendor,
            "VendorID": vendor_id,
            "AssignActivate": assign_activate,
        }
        # Optional sync fields are emitted only when populated, matching the
        # Go ",omitempty" tags.
        if sync_cycle0:
            entry["SyncCycle0"] = sync_cycle0
        if sync_cycle1:
            entry["SyncCycle1"] = sync_cycle1
        if sync_factor0:
            entry["SyncFactor0"] = sync_factor0
        if sync_factor1:
            entry["SyncFactor1"] = sync_factor1
        if sync_shift0:
            entry["SyncShift0"] = sync_shift0
        if sync_shift1:
            entry["SyncShift1"] = sync_shift1

        devices.append(entry)

    return devices


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--esi_directory", default="/tmp/esi")
    p.add_argument("--output", default="")
    args = p.parse_args()

    devices = []
    for fname in sorted(os.listdir(args.esi_directory)):
        if not fname.endswith(".xml"):
            continue
        path = os.path.join(args.esi_directory, fname)
        sys.stderr.write(f"Parsing {fname}... ")
        try:
            ds = parse_esi_file(path)
        except ET.ParseError as e:
            sys.stderr.write(f"PARSE ERROR: {e}\n")
            continue
        sys.stderr.write(f"{len(ds)} devices\n")
        devices.extend(ds)

    if args.output:
        with open(args.output, "w") as f:
            dump_sequence(devices, f)
    else:
        dump_sequence(devices, sys.stdout)


if __name__ == "__main__":
    main()
