# Leadshine R2EC / R3EC modular bus couplers

The `lcec_leadshine_ec` driver supports the Leadshine R2EC and R3EC
EtherCAT bus couplers.  A coupler is a single EtherCAT slave that hosts
up to 32 (R2EC) or 64 (R3EC) pluggable modules on a passive backplane:
digital input, digital output, mixed I/O, analog input/output, and
encoder modules.

Each populated slot is described with a
[`<subModule>`](configuration-reference.md#submodule) tag inside the
`<slave>`, identifying the backplane slot (`id`) and the module type
(`ident`):

```xml
<slave idx="0" type="R2EC" name="io">
  <subModule id="0" ident="0x61100025" name="in0">
    <modParam name="inputFilter0" value="20"/>
  </subModule>
  <subModule id="1" ident="0x61100205" name="out0">
    <modParam name="safeState" value="0"/>
  </subModule>
</slave>
```

For every slot the driver builds the dynamic PDO layout, exports HAL
pins through the standard `lcec_class_*` helpers, applies the module's
`<modParam>`s, and writes the configured module ident list (0xF030) so
the coupler accepts the PDO mapping.

## HAL pins

Pin names are `<master>.<slave>.<submodule-name>-<kind>-<n>`, where
`<submodule-name>` is the subModule `name` (default: the slot `id`):

- digital input modules: `...-din-<n>` (bit)
- digital output modules: `...-dout-<n>` (bit)
- analog modules: `...-ain-<n>-value` / `...-aout-<n>-value` (s32)
- encoder modules: the standard `lcec_class_enc` pins (`...-enc-<n>-count`, ...)

## Module idents

The `ident` is the module's type identifier as listed in the vendor ESI
(R3EC v2.4, available from Leadshine).  The driver validates every
`ident` at parse time against its module table
(`leadshine_ec_module_table` in
[src/devices/lcec_leadshine_ec.h](../src/devices/lcec_leadshine_ec.h)),
which is the source of truth.  Common modules:

| ident | module | type | channels |
|---|---|---|---|
| 0x61100025 | PM-1600 | digital in | 16 |
| 0x61100045 | PM-3200 | digital in | 32 |
| 0x61100205 | PM-0016-N | digital out (NPN) | 16 |
| 0x61110205 | PM-0016-P | digital out (PNP) | 16 |
| 0x61100405 | PM-0032-N | digital out (NPN) | 32 |
| 0x61100225 | PM-1616-N | mixed I/O | 16+16 |
| 0x61000025 | PM-A0400-IV | analog in | 4 |
| 0x61000205 | PM-A0004-IV | analog out | 4 |
| 0x61300025 | PM-E0200-S | encoder (single-ended) | 2 |
| 0x61300125 | PM-E0200-D | encoder (differential) | 2 |

`R3-*` (R3EC backplane) and `R3-*-V20` variants of the same modules have
their own idents; see the table in the header.

## Module modParams

Set inside the `<subModule>` tag; writes go to the module's config
object (0x8000 + slot) at startup.

Digital modules:

- `safeState` (U32, default 0): output value when the link is lost.
  Bits 0-15 at sub 1, bits 16-31 (32-channel modules) at sub 2.
- `inputFilter0` .. `inputFilter3` (U32, default 10): input filter time
  in ms, one per group of 8 channels.

Analog modules:

- `ch0Config` .. `ch3Config` (U32, default 0): per-channel range/mode
  (AD/DA config), per the ESI DT8000 layout.

Encoder modules (applied to every channel of the module):

- `encoderMode` (U32, default 0): operation mode.
- `abPhase` (U32, default 0): AB phase configuration.
- `minValue` / `maxValue` (S32, default -100000 / 100000).
- `countMode` (U32, default 0).
- `encoderFilter` (U32, default 2).

## Notes

- Slot ids are validated at startup: an out-of-range slot, a duplicate
  slot id, or a duplicate module name is a configuration error.
- A sparse slot layout (e.g. modules in slots 0 and 3 only) is
  supported; gap entries in the coupler's module list are cleared
  explicitly so the configured-vs-detected check still passes.
- The driver writes the SM2/SM3 PDO assignment (0x1C12/0x1C13)
  explicitly, matching the built sync layout; the master's automatic
  assignment does not enable this coupler's input sync manager.
