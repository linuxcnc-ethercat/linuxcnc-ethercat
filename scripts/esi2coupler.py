#!/usr/bin/env python3
"""Generate an lcec MDP-coupler module table from a vendor ESI file.

EtherCAT modular bus couplers (ETG.5001 MDP) differ only in data: slot
geometry, module ident lists, and per-module PDO layouts.  All of it is
declared in the vendor's ESI XML.  This tool extracts that data and emits
a C header consumed by the generic MDP coupler driver
(src/devices/lcec_mdp_coupler.c), so supporting a new coupler brand is:

    ./scripts/esi2coupler.py --esi VENDOR.xml --family mybrand \
        > src/devices/lcec_mdp_mybrand.h

plus one typelist entry and (if needed) a quirk flag in the driver.

Nothing in the emitted table is hand-maintained; re-run against a newer
ESI to update.  The source ESI path and its SHA-256 are recorded in the
header so drift is auditable.

Only information present in the ESI is emitted.  Behavioral quirks that
ESI cannot express (e.g. "refuses PDO reassignment") live in the driver's
family registration, not here.
"""

import argparse
import hashlib
import re
import sys
import xml.etree.ElementTree as ET
from dataclasses import dataclass, field


def parse_ecnum(text):
    """ESI numbers are decimal or '#x' hex."""
    if text is None:
        return None
    text = text.strip()
    if text.lower().startswith("#x"):
        return int(text[2:], 16)
    return int(text)


def parse_bool(text, default=False):
    if text is None:
        return default
    return text.strip().lower() in ("1", "true")


def c_ident(name):
    return re.sub(r"[^A-Za-z0-9]", "_", name).strip("_").lower()


@dataclass
class PdoEntry:
    index: int          # object index base (slot 0)
    index_dos: bool     # index is DependOnSlot
    subindex: int
    bitlen: int
    name: str
    datatype: str

    def is_padding(self):
        if self.index == 0:
            return True
        n = (self.name or "").lower()
        return self.datatype != "BOOL" and any(k in n for k in ("align", "reserve", "pad", "gap"))


@dataclass
class Pdo:
    index: int          # PDO index base (slot 0)
    index_dos: bool
    name: str
    entries: list = field(default_factory=list)


@dataclass
class Module:
    ident: int
    type_name: str
    module_class: str
    txpdos: list = field(default_factory=list)  # alternatives; first = default
    rxpdos: list = field(default_factory=list)


def parse_pdo(el):
    idx_el = el.find("Index")
    pdo = Pdo(
        index=parse_ecnum(idx_el.text),
        index_dos=parse_bool(idx_el.get("DependOnSlot")),
        name=(el.findtext("Name") or "").strip(),
    )
    for entry_el in el.findall("Entry"):
        e_idx = entry_el.find("Index")
        pdo.entries.append(PdoEntry(
            index=parse_ecnum(e_idx.text) if e_idx is not None else 0,
            index_dos=parse_bool(e_idx.get("DependOnSlot")) if e_idx is not None else False,
            subindex=parse_ecnum(entry_el.findtext("SubIndex") or "0"),
            bitlen=parse_ecnum(entry_el.findtext("BitLen") or "0"),
            name=(entry_el.findtext("Name") or "").strip(),
            datatype=(entry_el.findtext("DataType") or "").strip(),
        ))
    return pdo


def parse_modules(root):
    modules = []
    for mod_el in root.iter("Module"):
        type_el = mod_el.find("Type")
        if type_el is None or type_el.get("ModuleIdent") is None:
            continue
        mod = Module(
            ident=parse_ecnum(type_el.get("ModuleIdent")),
            type_name=(type_el.text or "").strip(),
            module_class=(type_el.get("ModuleClass") or "").strip(),
        )
        for pdo_el in mod_el.findall("TxPdo"):
            mod.txpdos.append(parse_pdo(pdo_el))
        for pdo_el in mod_el.findall("RxPdo"):
            mod.rxpdos.append(parse_pdo(pdo_el))
        modules.append(mod)
    return modules


def find_coupler_device(root, name_filter):
    """Return (device_el, slots_el) for the first Device carrying <Slots>."""
    for dev in root.iter("Device"):
        slots = dev.find("Slots")
        if slots is None:
            continue
        dev_name = ""
        type_el = dev.find("Type")
        if type_el is not None:
            dev_name = (type_el.text or "")
        if name_filter and name_filter not in dev_name:
            continue
        return dev, slots
    raise SystemExit("no <Device> with <Slots> matched" + (f" filter {name_filter!r}" if name_filter else ""))


def classify(module_class):
    mc = module_class.lower()
    if "digital" in mc and "in" in mc and "out" not in mc:
        return "LCEC_MDP_MOD_DIN"
    if "digital" in mc and "out" in mc:
        return "LCEC_MDP_MOD_DOUT"
    if "analog" in mc and "in" in mc:
        return "LCEC_MDP_MOD_AIN"
    if "analog" in mc and "out" in mc:
        return "LCEC_MDP_MOD_AOUT"
    if "count" in mc or "encoder" in mc:
        return "LCEC_MDP_MOD_ENC"
    return "LCEC_MDP_MOD_OTHER"


def emit_entries(out, prefix, pdo):
    out.append(f"static const lcec_mdp_pdo_entry_t {prefix}_entries[] = {{")
    for e in pdo.entries:
        pad = 1 if e.is_padding() else 0
        out.append(
            f'    {{0x{e.index:04x}, {int(e.index_dos)}, {e.subindex}, {e.bitlen}, {pad}, "{e.name}"}},')
    out.append("};")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--esi", required=True)
    ap.add_argument("--family", required=True, help="C identifier for this coupler family, e.g. uc20")
    ap.add_argument("--device", default=None, help="substring filter for the coupler Device name")
    args = ap.parse_args()

    raw = open(args.esi, "rb").read()
    sha = hashlib.sha256(raw).hexdigest()
    root = ET.fromstring(raw)

    vendor_id = parse_ecnum(root.findtext("Vendor/Id"))
    dev, slots = find_coupler_device(root, args.device)
    type_el = dev.find("Type")
    product_code = parse_ecnum(type_el.get("ProductCode"))
    revision = parse_ecnum(type_el.get("RevisionNo") or "0")
    dev_name = (type_el.text or "").strip()

    slot_el = slots.find("Slot")
    max_slots = parse_ecnum(slot_el.get("MaxInstances") or "0") if slot_el is not None else 0
    fam = {
        "slot_index_incr": parse_ecnum(slots.get("SlotIndexIncrement") or "0"),
        "slot_pdo_incr": parse_ecnum(slots.get("SlotPdoIncrement") or "0"),
        "download_ident_list": parse_bool(slots.get("DownloadModuleIdentList")),
        "max_slots": max_slots,
    }

    modules = parse_modules(root)
    if not modules:
        raise SystemExit("no <Module> elements with ModuleIdent found")

    f = args.family
    out = []
    out.append("// SPDX-License-Identifier: GPL-2.0-or-later")
    out.append(f"// GENERATED by scripts/esi2coupler.py -- DO NOT EDIT BY HAND")
    out.append(f"//   source ESI : {args.esi.split('/')[-1]}")
    out.append(f"//   sha256     : {sha}")
    out.append(f"//   device     : {dev_name}")
    out.append(f"// Regenerate with:")
    out.append(f"//   ./scripts/esi2coupler.py --esi <esi> --family {f}" + (f" --device '{args.device}'" if args.device else ""))
    out.append("")
    out.append(f"#ifndef _LCEC_MDP_{f.upper()}_H_")
    out.append(f"#define _LCEC_MDP_{f.upper()}_H_")
    out.append("")
    out.append('#include "lcec_mdp_coupler.h"')
    out.append("")

    mod_refs = []
    for m in modules:
        mid = c_ident(m.type_name) or f"ident_{m.ident:08x}"
        kind = classify(m.module_class)
        # first declared PDO of each direction is the vendor default mapping
        tx = m.txpdos[0] if m.txpdos else None
        rx = m.rxpdos[0] if m.rxpdos else None
        if tx:
            emit_entries(out, f"{f}_{mid}_tx", tx)
        if rx:
            emit_entries(out, f"{f}_{mid}_rx", rx)
        alt_note = ""
        if len(m.txpdos) > 1 or len(m.rxpdos) > 1:
            alt_note = f"  // NOTE: ESI offers {len(m.txpdos)}tx/{len(m.rxpdos)}rx alternative mappings; default (first) emitted"
        mod_refs.append((m, mid, kind, tx, rx, alt_note))
        out.append("")

    out.append(f"static const lcec_mdp_module_t {f}_modules[] = {{")
    for m, mid, kind, tx, rx, alt_note in mod_refs:
        tx_pdo = f"0x{tx.index:04x}" if tx else "0"
        tx_dos = int(tx.index_dos) if tx else 0
        rx_pdo = f"0x{rx.index:04x}" if rx else "0"
        rx_dos = int(rx.index_dos) if rx else 0
        tx_ref = f"{f}_{mid}_tx_entries" if tx else "NULL"
        tx_cnt = len(tx.entries) if tx else 0
        rx_ref = f"{f}_{mid}_rx_entries" if rx else "NULL"
        rx_cnt = len(rx.entries) if rx else 0
        out.append(
            f'    {{0x{m.ident:08x}, "{m.type_name}", {kind}, {tx_pdo}, {tx_dos}, {tx_ref}, {tx_cnt}, '
            f"{rx_pdo}, {rx_dos}, {rx_ref}, {rx_cnt}}},{alt_note}")
    out.append("    {0, NULL, LCEC_MDP_MOD_OTHER, 0, 0, NULL, 0, 0, 0, NULL, 0},")
    out.append("};")
    out.append("")
    out.append(f"static const lcec_mdp_family_t {f}_family = {{")
    out.append(f'    .name = "{f}",')
    out.append(f"    .vid = 0x{vendor_id:08x},")
    out.append(f"    .pid = 0x{product_code:08x},")
    out.append(f"    .revision = 0x{revision:08x},")
    out.append(f"    .slot_index_incr = 0x{fam['slot_index_incr']:x},")
    out.append(f"    .slot_pdo_incr = {fam['slot_pdo_incr']},")
    out.append(f"    .max_slots = {fam['max_slots']},")
    out.append(f"    .download_ident_list = {int(fam['download_ident_list'])},")
    out.append(f"    .modules = {f}_modules,")
    out.append("};")
    out.append("")
    out.append("#endif")
    print("\n".join(out))
    print(f"generated {len(modules)} modules for {dev_name} (vid 0x{vendor_id:x} pid 0x{product_code:x})", file=sys.stderr)


if __name__ == "__main__":
    main()
