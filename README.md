# RTL8821CEwifi.kext
**Wi-Fi driver for macOS (RTL8821CE)**

Project under development, for testing.
Based on macOS 15.5 Sequoia and later, although it may work on versions like Sonoma.

- Based on the Linux driver for rtw88 in its RTL8821CE variant, but may be compatible with similar variants.
- Initially designed for PCIe, additional lines of code and the usb.h and usb.c files could be added to support USB variants.
- It inherits Linux compatibility and the macOS framework from Itlwm, but focuses on Realtek and the Linux rtw88 driver.

You can contribute ideas, code, and improvements for driver stability and performance, as well as communication with macOS. After releasing the first stable versions for RTL8821CE PCIe, more support could be added. Some parts of the code may be reviewed by AI to inspect for any details or human error.

**Project STILL UNDER DEVELOPMENT, test and build at your own risk.**

## Safe bring-up target

`make probe` builds `RTL8821CEProbe.kext`, a constrained hardware bring-up probe.
It only matches the observed `10ec:c821` board and hard-gates the exact stable
off-state snapshot collected during earlier bring-up. It reads the standard PCIe
and PCI power-management capability blocks to determine whether endpoint Function
Level Reset is advertised, and publishes device/link control and status values.
Version 0.7.0 also builds a software-only dry-run of all 30 PCIe power commands
across five phases and projects their effects over the 20 captured registers.
Projection continues conditionally along the path where each poll reaches its
target; each poll record remains marked as not software-guaranteed because that
hardware-owned transition cannot be inferred from register writes. The upstream
worst-case poll budget is recorded, but its timeout-recovery register toggle is
forbidden.

Version 0.7.1 additionally samples the three poll conditions 64 times over a
bounded 3.15 ms window while the MAC remains off. It publishes hit counts,
transitions, min/max values, and the raw read-only sample pairs. Baseline targets
being stable before any power write does not predict hardware-owned state after a
future write, so this observation never authorizes execution.

Version 0.8.0 contains a fail-stop one-shot power-on executor scaffold. The
executor is compile-time disabled while its post-sequence system configuration
and firmware handoff are incomplete, so even both authorization boot arguments
cannot arm it. Disarmed mode maps BAR2 read-only. The scaffold verifies each
planned write and would stop permanently at the first failed write or poll. It
does not use the upstream timeout-recovery toggle or automatic power-off as
rollback. Firmware, DDMA, bus mastering, interrupts, and network registration
remain unavailable. Any future armed failure will require a cold shutdown with
power removal.

Version 0.8.1 makes every byte in the serialized register snapshot and
projection records explicit. Reserved bytes are always zero rather than compiler
padding, making IORegistry blobs deterministic without changing hardware access.

Version 0.8.2 embeds the exact RTL8821C firmware as an immutable Mach-O section
and validates its 64-byte header without touching the device. It checks the
DMEM/IMEM/EMEM size equation and destination addresses, then publishes a
software-only 35-chunk transfer plan. The required PCIe beacon TX descriptor
ring, bus mastering, and IDDMA remain unimplemented and disabled.

Version 0.8.3 transiently allocates zero-filled 16-byte descriptor and
4,144-byte firmware-staging buffers. Each allocation must produce exactly one
32-bit I/O virtual segment through `IODMACommand`; the command is completed,
detached, and released immediately after inspection. Published addresses are
diagnostic stale values only. They are never retained or written to the device,
and PCI bus mastering remains disabled.

Version 0.8.4 serializes and decodes a software-only beacon transport template
for the first 4 KiB DMEM chunk while two transient DMA mappings are alive. It
validates the 48-byte TX descriptor, RTL8821C 32-byte XOR checksum, two 8-byte
PCI buffer elements, beacon OWN/PSB fields, payload offset, and firmware bytes.
The template is never made device-visible: no queue base, index, doorbell, or
DMA address is written to hardware, and both mappings are released immediately.

Version 0.8.5 retains the validated descriptor ring and staging mapping for the
lifetime of the probe service. Ownership moves to explicit service members only
after both mappings and the serialized template pass validation. A failed start
and `stop()` both complete, detach, and release every mapping in reverse order.
The persistent ring remains software-only: queue base/index/doorbell registers,
bus mastering, and device DMA configuration are still untouched.
No reset is initiated. BAR2 remains read-only, the MAC must remain off, and the
exact PCI command must be restored before the service is published. It does not
run the RTL8821C power FSM, touch firmware/RF/PCI-DMA controls, probe BAR sizes,
enable bus mastering, register interrupts, expose DMA to the device, or upload firmware. This
reset-capability stage is for isolated OC-TEST bring-up only.

## Reset containment decision

The observed card is PCI function `02:00.0`, the only child below root port
`GPP4` (`00:02.2`, secondary/subordinate bus `2:2`). Its PCIe capability at
`0x70` does not advertise Function Level Reset. Its power-management capability
at `0x40` reports PMCSR `No_Soft_Reset`, so a D3hot-to-D0 transition cannot be
treated as an internal reset. A secondary-bus reset is not used because macOS
does not expose it as a documented endpoint recovery contract and topology
isolation alone cannot prove recovery from a partial vendor power transition.

Consequently, the probe must not execute the RTL8821C power FSM automatically.
If a future explicitly authorized power experiment fails after its first write,
the device is quarantined for that boot and recovery requires a cold shutdown
with power removal. A normal reboot is not considered sufficient containment.

The dry-run therefore always publishes `PowerFSMExecutionAuthorized=false` and
`PowerFSMExecutionReady=false`. It is a planner and audit artifact, not an opt-in
path to real power sequencing.

Special thanks to:

- [OpenIntelWireless](https://github.com/OpenIntelWireless) for [Itlwm](https://github.com/OpenIntelWireless/itlwm/tree/53c51c2cdd6e4b69beb91f310d74c53422b0f8bd)
- [Linux](https://www.kernel.org) for rtw88 driver
