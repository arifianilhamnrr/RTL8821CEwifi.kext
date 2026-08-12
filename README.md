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

Version 0.8.6 reads stable baselines for the PCI control byte, beacon ring base,
global ring-pointer clear register, and beacon work flag. It serializes the exact
four-command queue setup projection without executing it. The upstream global
pointer reset affects every TX/RX queue, so execution remains blocked until all
TRX rings exist; configuring only the beacon ring would not be contained.

Version 0.8.7 serializes the complete upstream PCI resource layout: eight TX
rings and two RX rings, their descriptor counts/sizes, base/count/index
registers, and RX payload requirements. It validates unique registers, 12-bit
entry limits, and exact memory totals without allocating them. Upstream allocates
an RX C2H ring but only programs RX MPDU, so the roughly 11.8 MB allocation and
C2H ownership semantics remain blockers before a global pointer reset is safe.

Version 0.8.8 distinguishes that upstream allocation inventory from the hardware
resource layout. The second RX C2H ring is allocated only by a generic loop: it
has no registers, consumer, refill, or index advancement. C2H packets instead
arrive through RX MPDU and are identified by the RX descriptor C2H bit. The
hardware-required planner therefore contains eight TX rings and one RX MPDU ring
(nine records, 324 bytes), while retaining explicit audit properties for the dead
C2H allocation artifact. Hardware-required allocation is still not attempted.

Version 0.8.9 transiently tests the host-side allocation shape required by that
planner: nine separate descriptor-ring mappings and 512 separate RX MPDU payload
mappings. Every mapping must be a single 32-bit segment. The probe records exact
counts and byte totals, then completes and releases all mappings in reverse order
before publishing the service. Allocation failure is diagnostic and fully cleaned
up; it does not enable bus mastering, expose an address to the device, or execute
queue configuration.

Version 0.9.0 retains the same proven 521 mappings for the service lifetime
instead of releasing and reallocating them. Ownership transfers only after the
complete nine-ring and 512-payload allocation succeeds. Failed start and
`stop()` release payloads and rings in reverse order before releasing the two
firmware-template mappings. The retained TRX resources remain software-only:
their addresses are not programmed into the device and bus mastering stays off.

Version 0.9.1 materializes the retained RX MPDU ring entirely in host memory.
Each of its 512 eight-byte descriptors contains the little-endian 11,478-byte
buffer size, a zero initial RX tag, and the corresponding independent 32-bit
payload IOVM address. A second pass validates every field and address range before
ownership transfer. RX has no per-descriptor OWN or EOR bit. The ring is not
synchronized for device access and no queue, DMA-control, or pointer register is
written.

Version 0.9.2 validates the initial host-only state of all eight retained TX
rings. All 1,025 sixteen-byte PCI TX buffer descriptors must initially be zero-filled,
and each complete ring mapping must have a valid nonzero 32-bit IOVM range.
This matches upstream initialization, where TX descriptors stay idle until a
packet payload is mapped and queued. No TX payload is allocated, no descriptor
is synchronized for device access, and no TX queue, DMA-control, or pointer
register is written.

Version 0.10.0 batches the next host-only milestones into one runtime test. It
copies the already validated firmware beacon descriptor into the retained beacon
ring and verifies the copy without synchronizing either mapping. The resulting
state has 1,024 zero idle descriptors and one host-only beacon template. It then builds a
21-record dynamic device plan from all nine retained 32-bit ring addresses: PCI
control projection, eight count-before-base pairs plus the beacon base, the global pointer
reset, RTL8821C H2C index reset, and TRX DMA-control projection. Separately, it
expands the firmware manifest into a 35-record RTL8821C staging and IDDMA contract,
including each firmware offset, rebuilt packet and PSB lengths, TX-buffer OCP
source, section destination, exact transfer length, checksum reset/continue and
validation boundaries, and both bounded OWN polls per transaction. Both plans are serialized for
inspection only. Their execution is unconditionally unauthorized; no descriptor
is synchronized, no MMIO queue or IDDMA register is written, and PCI bus
mastering remains disabled. Any future queue executor must keep bus mastering
disabled until the complete ring plan has been programmed and validated.
The same boot also captures two read-only samples of twelve firmware-path
registers, including MCU control, beacon/FIFO state, platform reset, FW debug,
and IDDMA source/destination/control. All samples must remain stable before the
service is published, providing the next execution stage with a single-boot
baseline without performing any firmware-path write.

Version 0.11.0 begins the next offline-only batch and is intentionally not
deployed yet. It serializes the canonical 23-stage RTL8821C lifecycle, including
the probe-time firmware/efuse cycle, its power-off boundary, and the separate
operational firmware cycle through final FIFO, MAC, PHY/RF, and interrupt stages.
It also records the five-operation post-power system configuration required to
enable the 3081 platform and IDDMA, plus the 17-operation temporary firmware
setup, platform reset, CPU transition, restore obligations, and common failure
cleanup. These records are symbolic contracts only; execution remains
unconditionally unauthorized while the remaining reserved-page, final FIFO,
efuse, and interrupt planners are completed in the same batch. The batch now
also contains a fourteen-operation per-chunk reserved-page contract, exact
RTL8821C PCI FIFO arithmetic (512 total pages, 52 reserved, boundary 460, 397
public), the H2C address range `0xFA00` through `0xFE00`, and a 27-operation
FIFO/RQPN/H2C plan. Synthetic efuse fixtures independently validate one-byte and
extended headers, logical bounds, physical truncation, and extended-header
truncation without reading hardware; the model intentionally corrects upstream's
edge bound from `>` to `>=`. A sixteen-operation
interrupt lifecycle records disable, unmask, status recognition, write-one-clear,
conditional RX-masked re-enable, and final NAPI-completion re-enable while
requiring firmware, RX ownership, and a NAPI equivalent. Serialized register
plans use a versioned operation schema that distinguishes reads, writes, masked
updates, polls, saved-value restores, W1C, DMA submission, and host comparisons.
The H2C byte range is `[0xFA00, 0xFE00)`. None of these new plans has been
deployed or executed.

The same undeployed 0.11.0 batch now includes a versioned DMA publication and
fail-stop model. Seventeen states and twenty-one transitions separate host-only
allocation, BME-off ring programming, the first power transition, BME exposure,
beacon ownership/kick, IDDMA, partial firmware, CPU release, firmware-ready, and
operational state from identity, DMA, firmware, and power quarantine. An
eighteen-record publication plan treats `IODMACommand::synchronize()` only as a
whole-prepared-range copy contract when command-private or bounce buffers are
present, never as a memory fence. TX staging and descriptors require Out synchronization, an explicit CPU
release fence, ownership publication last, a second Out synchronization, a
doorbell, and MMIO readback. RX reclaim requires descriptor In synchronization,
an acquire fence, payload In synchronization, and another acquire fence;
RTL8821C RX descriptors are explicitly modeled without a per-descriptor OWN bit.

Seven rollback policies distinguish rollback from containment. Host-only and
verified BME-off state may release mappings; once BME, beacon work, IDDMA,
firmware memory, or a power transition is exposed, visible mappings cannot be
released. Clearing BME is containment only, never proof of reset or quiescence.
Beacon/IDDMA/power failures prohibit pointer reset, ownership rewrite, firmware
retry, nominal power-off rollback, and visible-mapping release, and terminate in
quarantine until confirmed cold power removal. This remains serialized planning
data only: no synchronize call, fence, MMIO write, BME change, or mapping release
behavior has been executed.

Version 0.12.0 remains offline-only and adds the execution-safety layer required
before any hardware executor can be considered. A six-record serialized-command
contract requires one logical owner, rejects reentry, models `BeginStop` solely
as admission closure before containment, and requires a write-ahead intent for every modeled state
mutation. This is a symbolic contract only: no `IOWorkLoop`, `IOCommandGate`,
callback, or executor is instantiated. Serialization orders host intentions; it
does not stop autonomous DMA, firmware, interrupts, or posted PCIe transactions.

A five-phase volatile execution-journal schema distinguishes intent, success,
known no-effect failure, unknown-effect failure, and containment. Intent records
carry the conservative post-attempt hazard vector before the modeled effect;
unknown outcomes preserve all worst-case power, programmed-address, BME, beacon,
IDDMA, firmware-memory, CPU-release, and interrupt hazards. The journal is a
synthetic ordering and validation artifact, not durable recovery evidence across
a panic, reboot, or power loss. State may advance only after modeled success;
ambiguous effects require containment.

The mapping-exposure ledger covers all 523 retained mappings as five resource
classes: firmware descriptor, firmware staging, eight TX rings, one RX ring, and
512 RX payloads. Seven allowed exposure transitions prohibit a direct path from
DMA-reachable or quarantined memory to release. Host-only memory may be released
as never exposed; programmed BME-off mappings require verified address restore;
possibly device-visible mappings require confirmed cold power removal. Clearing
BME alone is never release evidence. Mapping generations are mandatory so stale
descriptor synchronization cannot authorize a later ownership generation.

Static failure-boundary classification now covers all 23 lifecycle, 25
DMA-transition, and 18 publication boundaries with pre-intent rejection, known
no-effect failure, unknown-effect failure, and begin-stop-after-intent cases. It
conservatively accumulates hazards and classifies cleanup versus containment, but
does not claim to be a full contract interpreter or scenario replay. The DMA graph also gains
explicit containment routes from every nonterminal state, RX publication now
states its BME prerequisite directly, and TX validation rejects stale
synchronization generations. All classifications are pure host-side arithmetic: no MMIO
write, DMA synchronization, fence execution, BME change, power transition, or
mapping release is authorized or attempted.

Version 0.13.0 adds an offline symbolic interpreter over the version-two
execution-safety schema. It evaluates five outcomes at each of the 23 lifecycle,
25 DMA-transition, and 18 publication boundaries: reentrant admission rejection,
success, known no-effect failure, unknown-effect failure, and begin-stop after a
write-ahead intent. The resulting 330 traces preserve independent accumulated
hazards, require the same gate ticket and transaction identity from intent to
resolution, block new transactions while an unknown effect awaits containment,
and exercise command admission, journal phases, declared result flags, failure
dispositions, and mapping retention. DMA boundary fixtures explicitly include
prerequisites supplied by their publication subplans rather than claiming that a
source state alone establishes them.

Publication success prefixes now feed synchronization generations before TX or
RX doorbells can match the current generation. Positive controls require a
non-stale publication sequence and a cold-removal release path. Seven negative
controls reject reentry, post-stop ordinary work, unresolved unknown-effect work,
missing transition prerequisites, release without evidence, and stale TX/RX
doorbells. Release validation is failure-atomic through a tentative state copy.
Confirmed cold removal is a separate symbolic event that establishes quiescence,
clears historical device-reachability hazards for the new power epoch, and only
then permits quarantined mappings to become release-authorized and released.

This interpreter validates the serialized model, not hardware behavior. Mapping
exposure remains conservatively class-based, lifecycle hazards are symbolic, and
no real command gate, persistent journal, cold-removal detector, synchronization,
MMIO write, BME transition, DMA operation, or power executor is instantiated.

Version 0.14.0 adds a separately gated power-only executor scaffold for the first
future hardware experiment. Its immutable contract contains eight pre-system
writes followed by the eighteen-command power-on FSM: twenty-four verified writes
and two bounded polls across twenty-six total steps. Seventy-eight synthetic cases
cover success, rejection before intent, write effect unknown, and poll timeout;
retry and nominal power-off recovery counts must remain zero. The executor emits
a fixed-capacity write-ahead journal record before every MMIO attempt and a second
record only after verified success or terminal failure. A write mismatch is
effect-unknown and a poll timeout is quarantined because the device may have
changed autonomously while the host waited.

The legacy experiment and version-two executor use independent compile-time
gates, both currently `false`, and mutually exclusive runtime arming. Version two
also requires the `oc-test=1` marker, a dedicated enable argument, and the exact
confirmation value. It runs synchronously inside `start()` before service
publication, so no command gate or concurrent entry point exists yet. Once any
journal intent is recorded, further TRX allocation and hardware work are blocked
for that boot; no queue address, pointer, BME, DMA, firmware, interrupt, retry, or
power-off operation follows. Existing firmware-template mappings remain releasable
because their addresses were never programmed and bus mastering remained off;
quarantine applies to the partially transitioned device, not unexposed host-only
memory. The exact PCI command is still restored after the one-shot invocation.

This v0.14.0 scaffold is not armed or deployed. BAR2 remains read-only in the
current build and the executor cannot run regardless of boot arguments. A future
enablement change requires another review and an isolated OC-TEST deployment;
failure after the first write still requires cold shutdown with power removal.
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
