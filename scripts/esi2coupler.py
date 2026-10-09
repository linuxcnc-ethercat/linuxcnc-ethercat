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
    excludes: list = field(default_factory=list)  # PDO indices this one excludes

    def entry_base(self):
        """Lowest object index referenced by this mapping (0 if empty)."""
        return min((e.index for e in self.entries), default=0)


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
        excludes=[parse_ecnum(x.text) for x in el.findall("Exclude") if x.text],
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


def refine_digital_kind(kind, tx, rx):
    """Promote a digital module to DIO when it maps I/O in both directions.

    Inovance labels its combined modules ModuleClass="Digital Out" (0404ETP-5V,
    0808ETN, 3232ETN), which would leave their input half mapped but pinless.
    Only DIN/DOUT are ever promoted, so analog, serial and counter modules keep
    the class the ESI declares.
    """
    if kind not in ("LCEC_MDP_MOD_DIN", "LCEC_MDP_MOD_DOUT"):
        return kind
    if any(pdo_is_io(p) for p in tx) and any(pdo_is_io(p) for p in rx):
        return "LCEC_MDP_MOD_DIO"
    return kind


def variant_index_shift(pdo, siblings):
    """Offset to subtract from a mapping's entry object indices.

    Mutually exclusive mappings (<Exclude>) are alternative views of one and
    the same process data -- bitwise, 8-bit packed, 16-bit.  Vendors encode
    the variant number in the *entry* object index as well as in the PDO
    index, but only one object exists: the one the module's <Objects> section
    declares, which is the lowest entry index used across the exclusion
    group.  Inovance GL20 1600END offers TxPdo 0x1A00/0x6000 (bitwise),
    0x1A01/0x6001 (8-bit) and 0x1A02/0x6002 (16-bit), yet its dictionary
    declares only object 0x6000, and that object's two 8-bit subindexes are
    the 8-bit view; hardware agrees, PDO 0x1A19 (0x1A01 at slot 3) maps
    0x60C0:01/:02 and 0x60C1 does not exist.  Emitting any variant other
    than the group's lowest therefore has to normalize its entry indices
    back onto the base object.

    Only single-object variants are normalized: a mapping spanning several
    objects (per-channel objects, as on GL20 4LC-PID) is not a repacking of
    one object, so its indices are real and left alone.
    """
    if not pdo.excludes:
        return 0
    group = [p for p in siblings if p.index in pdo.excludes] + [pdo]
    shift = pdo.entry_base() - min(p.entry_base() for p in group)
    if shift == 0 or len({e.index for e in pdo.entries}) != 1:
        return 0
    return shift


def select_pdos(pdos):
    """Mappings the coupler activates: every non-exclusive one, plus the first
    member of each mutual-exclusion group, in ESI document order.

    Verified against an Inovance GL20 carrying 2SCOM-MDB + 2x 0016ETP + 2x
    1600END: this reproduces the coupler's self-assembled 0x1C12/0x1C13
    exactly, 6 RxPDOs and 11 TxPDOs, index for index and in order.  Emitting
    only the first mapping per direction understates the image the slave
    produces, which shifts every process-data offset past the first omission.
    """
    chosen, superseded = [], set()
    for p in pdos:
        if p.index in superseded:
            continue
        chosen.append(p)
        superseded.update(p.excludes)
    return chosen


def drop_repeat_data_sets(pdos):
    """Drop repeat buffer sets that exist only so a user can scale data size up.

    A few modules map the same process data several times over, at object
    indices differing only by set number.  GL20-2SCOM-MDB and 2S485-MDB offer
    four identical sets per direction (0x1B00-0x1B03 in, 0x1700-0x1703 out).
    Per the GL20-2SCOM/2SCOM-MDB Equipment Guide (PS00021928) the default is
    ONE 120-byte set each way, and the per-module budget shrinks as more
    communication modules are fitted:

        communication modules fitted:  1     2     3     4
        max PDO bytes per module:      480   240   120   120

    Assigning all four sets spends a lone module's entire 480-byte budget, and
    the guide warns that exceeding the budget makes the module report an error
    and PDO exchange fail.  So emit the vendor default and let a user who needs
    bigger buffers say so explicitly.

    Identical entry *names* are what marks a repeat set, not identical shape:
    GL20-2CAN and GL20-2HC also map two same-shaped sets per direction, but
    theirs are named CAN0_/CAN1_ and "2HC CH0"/"2HC CH1" because they are two
    physical channels, and dropping the second would lose real data.
    """
    seen, chosen = set(), []
    for p in pdos:
        sig = tuple((e.subindex, e.bitlen, e.name) for e in p.entries)
        if sig in seen:
            continue
        seen.add(sig)
        chosen.append(p)
    return chosen


def pdo_is_io(pdo):
    """Does this mapping carry process I/O, or status/diagnosis data?

    ETG.5001 partitions modular-device objects by role: 0x6000-0x6FFF inputs,
    0x7000-0x7FFF outputs, 0xA000-0xAFFF diagnosis.  Only I/O may become HAL
    pins.  Diagnosis still has to be in the process image, so the driver needs
    the distinction flagged rather than inferred from bit widths -- a 16-bit
    diagnosis word is indistinguishable from a packed 16-channel digital
    register otherwise.
    """
    idx = [e.index for e in pdo.entries if not e.is_padding()]
    return bool(idx) and all(0x6000 <= i <= 0x7FFF for i in idx)


def emit_entries(out, prefix, pdo, shift=0):
    if shift:
        out.append(f"// {pdo.name}: exclusive-variant entry base normalized "
                   f"0x{pdo.entry_base():04x} -> 0x{pdo.entry_base() - shift:04x}")
    out.append(f"static const lcec_mdp_pdo_entry_t {prefix}_entries[] = {{")
    for e in pdo.entries:
        pad = 1 if e.is_padding() else 0
        out.append(
            f'    {{0x{e.index - shift:04x}, {int(e.index_dos)}, {e.subindex}, {e.bitlen}, {pad}, "{e.name}"}},')
    out.append("};")


def emit_pdos(out, prefix, pdos):
    """Emit the per-direction PDO list, referencing the entry arrays."""
    out.append(f"static const lcec_mdp_pdo_t {prefix}_pdos[] = {{")
    for n, p in enumerate(pdos):
        out.append(f"    {{0x{p.index:04x}, {int(p.index_dos)}, {int(pdo_is_io(p))}, "
                   f"{prefix}{n}_entries, {len(p.entries)}}},  // {p.name}")
    out.append("};")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--esi", required=True)
    ap.add_argument("--family", required=True, help="C identifier for this coupler family, e.g. uc20; "
                    "its uppercase form (UC20) is the <slave type=...> name")
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
        tx_sel, rx_sel = select_pdos(m.txpdos), select_pdos(m.rxpdos)
        tx, rx = drop_repeat_data_sets(tx_sel), drop_repeat_data_sets(rx_sel)
        kind = refine_digital_kind(classify(m.module_class), tx, rx)
        for n, p in enumerate(tx):
            emit_entries(out, f"{f}_{mid}_tx{n}", p, variant_index_shift(p, m.txpdos))
        if tx:
            emit_pdos(out, f"{f}_{mid}_tx", tx)
        for n, p in enumerate(rx):
            emit_entries(out, f"{f}_{mid}_rx{n}", p, variant_index_shift(p, m.rxpdos))
        if rx:
            emit_pdos(out, f"{f}_{mid}_rx", rx)
        excl = (len(m.txpdos) - len(tx_sel)) + (len(m.rxpdos) - len(rx_sel))
        sets = (len(tx_sel) - len(tx)) + (len(rx_sel) - len(rx))
        alt_note = f"  // NOTE: {excl} mutually exclusive mapping variant(s) not emitted" if excl else ""
        if sets:
            alt_note += f"  // NOTE: {sets} repeat data set(s) not emitted; vendor default is one per direction"
        mod_refs.append((m, mid, kind, tx, rx, alt_note))
        out.append("")

    out.append(f"static const lcec_mdp_module_t {f}_modules[] = {{")
    for m, mid, kind, tx, rx, alt_note in mod_refs:
        tx_ref = f"{f}_{mid}_tx_pdos" if tx else "NULL"
        rx_ref = f"{f}_{mid}_rx_pdos" if rx else "NULL"
        out.append(
            f'    {{0x{m.ident:08x}, "{m.type_name}", {kind}, '
            f"{tx_ref}, {len(tx)}, {rx_ref}, {len(rx)}}},{alt_note}")
    out.append("    {0, NULL, LCEC_MDP_MOD_OTHER, NULL, 0, NULL, 0},")
    out.append("};")
    out.append("")
    out.append(f"static const lcec_mdp_family_t {f}_family = {{")
    # uppercase, like every other driver's type name (lcec_findslavetype is case-sensitive)
    out.append(f'    .name = "{f.upper()}",')
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
