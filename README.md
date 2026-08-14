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

Version 0.15.0 is a read-only runtime diagnostic follow-up. The v0.14.0 binary
loaded successfully on OC-TEST but returned from `start()` before publishing its
child service, so its service-local properties were unavailable. Version 0.15.0
publishes a versioned 64-bit failure mask and selected validator results on the
provider `IOPCIDevice` before any failed-start return. Individual bits distinguish
BAR mapping and decode, power-plan validation, chip and MAC identity, pre-system
stability and expected values, queue/template planning, firmware-register
stability, PCI command restoration, and TRX post-checks. The provider telemetry
survives child attach failure and therefore identifies the exact guard on the
next boot. Both power build gates remain `false`; this change adds properties
only and does not authorize power, BME, DMA, firmware, interrupt, retry, or
power-off execution.

Versions 0.16.0 and 0.17.0 completed the read-only diagnosis. Runtime telemetry
showed that the cold-boot `SYS_CFG1` status bit and several mutable power-path
registers can change while all samples remain stable, so identity validation now
masks only that status bit and treats mutable values as observations rather than
immutable straps. The symbolic interpreter's positive cold-removal fixture now
starts in `ColdRemovalRequired`, producing all 330 traces and seven expected
negative controls. Version 0.17.0 published successfully with a zero diagnostic
failure mask, restored PCI command zero, bus mastering disabled, MAC off, and no
power attempt.

Version 0.18.0 is the final read-only preflight batch. It corrects two firmware-
stage planner validators: the `0x022c` firmware setup record is backup-before,
not a synthetic write restore, and a saved-register restore mask may be a full-
width superset of the mutation mask. An aggregate `PowerExecutorV2PreflightReady`
property now requires every lifecycle, publication, rollback, journal, mapping,
firmware, interrupt, identity, and runtime containment check simultaneously.
Preflight readiness is explicitly telemetry, never execution authorization; in
the v0.18.0 validation build both power gates remain `false` and no power boot
arguments are installed.

Version 0.19.0 is the isolated first power-on experiment build. Only the version-
two power executor compile-time gate is enabled; the legacy executor remains
disabled. Invocation additionally requires the OC-TEST marker, dedicated enable
and exact confirmation boot arguments, every static lifecycle/firmware/DMA/
rollback/journal/interpreter validator, and the complete runtime MMIO identity
baseline. It performs only the twenty-six-step pre-system and power-on contract
with write-ahead journaling, masked readback verification, and bounded polls.
Bus mastering, queue programming, DMA publication, firmware upload, interrupts,
retry, and automatic power-off remain forbidden. Any result after the first
intent is quarantined and requires cold shutdown with physical power removal.

The v0.19.0 OC-TEST experiment completed all twenty-four writes and both bounded
polls with fifty-two journal records, no failed step, no retry, and no timeout.
MAC control changed from off-state `0xea` to `0x00`; PCI command was restored to
zero and bus mastering, DMA, firmware, and interrupts remained disabled. No panic,
IOMMU, or DMAR fault was observed. OC-TEST power arguments were removed immediately
after collecting the result so subsequent boots cannot repeat the sequence.

Version 0.20.0 returns the power executor build gate to `false` and adds the next
offline-only contract: a fifteen-register read-only snapshot collected only after
a successful power-on result. Two samples cover system function/power state, MCU
firmware control, MAC control, reserved-page/FIFO state, PCI queue control,
platform/IDDMA enable, and IDDMA status. No post-power setup write, queue address,
BME, DMA, firmware upload, interrupt, retry, or power-off is introduced.

Version 0.21.0 serializes the five upstream post-power system operations as a
separate offline execution contract. The first three operations enable 3081
platform reset/DDMA support, chip-specific system-function bits, and the required
`CR_EXT+3` low nibble; two conditional operations disable boot-from-flash state
when advertised. Seventeen synthetic paths cover five successes, five pre-intent
rejections, five unknown-effect quarantines, and two conditional skips. Every
step requires an already powered MAC, explicitly forbids BME, and prohibits
rollback, retry, nominal power-off, DMA, firmware, or interrupt execution. This
planner remains unauthorized and unattempted.

Version 0.22.0 is a snapshot-only runtime build with a separate OC-TEST marker,
enable argument, and confirmation. The power executor gate remains `false` and
BAR2 is mapped read-only. If MAC control is already powered and stable, it records
the fifteen-register post-power snapshot without journaling or writes; if reboot
returns the card to `0xea`, it reports the snapshot path ineligible and still does
nothing. Power and snapshot arming are mutually exclusive, preventing an
already-powered card from receiving the power sequence twice.

The v0.22.0 OC-TEST boot confirmed that reboot returns MAC control to `0xea`.
Version 0.23.0 therefore re-enables only the previously successful journaled
power executor and immediately collects the same fifteen-register read-only
snapshot after all twenty-four writes and both polls complete in that boot. It
also treats `SYS_CFG1[1:0]` and `PAD_CTRL1[29:28]` as mutable status bits observed
across cold boots while preserving every other identity and pre-system guard.
Post-power setup writes, BME, queue programming, DMA, firmware upload, interrupts,
retry, and automatic power-off remain forbidden.

Version 0.24.0 adds a separately armed post-power executor after the proven
power-on and stable fifteen-register snapshot. It journals all five upstream
system-configuration records before acting, verifies every masked write, and
stops permanently on the first mismatch. The observed firmware-control snapshot
does not advertise boot from flash, so the two flash-disable records are expected
to be journaled conditional skips; only the platform/DDMA enable, system-function,
and `CR_EXT+3` records write hardware. BME, queue addresses, DMA submission,
firmware upload, CPU release, interrupts, retry, and automatic power-off remain
forbidden.

Version 0.25.0 batches the host-only prerequisite for queue and firmware work.
All nine descriptor rings and 512 RX payload mappings are allocated, validated,
and retained before BAR mapping or any optional power transition. The existing
helper also validates all 1,025 zero TX descriptors, materializes and validates
all 512 RX descriptors, copies the beacon template with OWN clear, and builds the
dynamic 21-record device plan from the retained IOVM addresses. PCI command must
remain exactly unchanged with BME clear across allocation. This runtime remains
unarmed and performs no queue/MMIO write, synchronization, BME change, DMA,
firmware upload, IDDMA, CPU release, or interrupt registration.

Version 0.26.0 adds a separately armed full 21-record TRX publication executor.
After the proven power and post-power stages, it synchronizes all nine descriptor
rings Out, applies the exact count-before-base ordering, resets all ring pointers
and H2C indices, and enables the modeled TRX DMA-control bits while PCI BME remains
clear. Every record has write-ahead intent, masked readback verification, and a
BME check before and after its MMIO write. The 512 RX payload mappings are retained
but not synchronized because they contain no CPU-produced device input. BME,
beacon OWN/doorbell, firmware upload, IDDMA, CPU release, and interrupts remain
forbidden.

Version 0.27.0 narrows the queue verifier correction to the global TX descriptor
read/write-pointer clear command at `0x039c`. Hardware accepted the v0.26 write
of `0xffffffff` but immediately read back zero, matching command-register
self-clear behavior. The v0.27 executor therefore requires a zero terminal
readback only for plan operation 3 at that exact offset, width, mask, and value;
all other records retain strict masked projected-value verification. Telemetry
counts accepted command readbacks, and a new `queue-v2` confirmation token keeps
this contract distinct from v0.26. Containment remains unchanged: BME, beacon
OWN/doorbell, firmware upload, IDDMA, CPU release, and interrupts are forbidden.

Version 0.28.0 incorporates the next runtime observation without broadening the
verifier generally. The upstream `0x1330` H2C queue CSR write explicitly requests
host-index and hardware-index clear with bits 16 and 8; v0.27 observed those bits
read back zero immediately. The executor now accepts zero only for that exact
operation-4 tuple and the previously proven `0x039c` tuple. A host-side contract
validator requires exactly those two command records and requires the final
operation-5 `0x0300` DMA-control record to remain strict. Separate counters prove
each command readback, and final DMA-control telemetry captures its observed
value. A new `queue-v3` token prevents older queue contracts from arming this
path. BME, descriptor ownership, firmware transport, and interrupts remain
forbidden.

Version 0.29.0 batches the complete reversible pre-firmware transport transaction
behind a new `fw-setup-v1` token. After the proven power, post-power, and 21-record
queue sequence, it stops the WLAN CPU and MCU I/O interface, backs up the six
upstream temporary download registers, applies HIQ/TXDMA/page/beacon setup,
performs the platform and CPU-clock reset sequence, enters firmware-download
mode, verifies DDMA channel 0 is idle, exits firmware-download mode, and restores
the six temporary registers in reverse order. All 21 steps have write-ahead and
terminal journal records and enforce BME clear before and after every mutation.
The upstream synthetic restore semantics are explicit: H2CQ CSR restores to the
FULL marker and RQPN_CTRL_2 restores to its saved value with LD_RQPN set. A
successful endpoint intentionally leaves WLAN CPU and MCU I/O disabled. Beacon
OWN, beacon doorbell, IDDMA source/destination/control writes, firmware-memory
mutation, CPU release, retry, power-off, and interrupts remain forbidden.

Version 0.30.0 advances that entire 21-step transaction behind a new
`fw-setup-v2` token. Hardware evidence from 0.29.0 established that
`RQPN_CTRL_2.BIT_LD_RQPN` is an exact self-clearing load command. Both its setup
write and synthetic reverse-restore now require terminal readback with bit 31
clear while every other bit remains strict. The contract validator permits this
only for the two exact `0x022c` records, separate setup and restore counters must
each reach one, the DDMA-idle poll must complete, all six reverse restores must
complete, and final FWDL and RQPN state are audited before success. No firmware
DMA or additional hardware boundary is enabled by this change.

Version 0.31.0 adds one aggressive, separately armed first-DMEM transaction.
Only `rtl8821ce-dmem-v1=1` with exact confirmation `0x3100c821` and the OC-TEST
marker can select it; the older `fw-setup-v2` token is explicitly exclusive and
cannot trigger firmware transport. The executor inserts the transaction after
setup step 13 confirms DDMA channel 0 idle and before step 14 clears FWDL mode,
matching the upstream requirement that IDDMA firmware download occur while FWDL
is enabled. It strictly validates only plan record section 0, chunk 0, first
chunk, 4096 bytes, destination `0x200000`, source `0x18780030`, and control
`OWN|checksum-enable|4096` against the embedded first DMEM payload and prepared
descriptor.

The staging mapping is synchronized Out; retained beacon ring resource 4 gets
OWN as its final host mutation, is synchronized Out, and is release-fenced before
temporarily enabling exact PCI BME with memory decode preserved. The already
programmed beacon base is used, `0x0383[4]` is rung once, and completion is bounded
on the beacon descriptor OWN bit clearing after In synchronization. Upstream does
not define the doorbell bit as a completion or self-clear contract, so its
readback is captured but not required to clear. The exact PCI command is restored
and read back before IDDMA. The executor then pre-polls DDMA OWN clear, resets
checksum status as upstream does, writes SA/DA/CTRL, polls OWN clear, and captures
checksum status. It deliberately neither requires a section-final checksum nor
sets DMEM download/checksum OK bits because seven DMEM chunks remain. On success,
step 14 clears FWDL and all six reverse restores run. A separate fixed write-ahead
transport journal and detailed synchronization, poll, PCI-command, doorbell,
IDDMA, checksum, and hazard properties record the boundary. Any failure after
OWN, BME, doorbell, or IDDMA fail-stops without retry, nominal power-off, CPU
release, or interrupts; PCI command restoration is best-effort and an uncertain
readback is classified as an unknown effect requiring cold power removal.

Version 0.32.0 corrects the beacon completion contract from the exact 0.31.0
hardware evidence. The beacon doorbell command at `0x0383[4]` was accepted and
self-cleared (`0x12` to `0x02`) while the host descriptor OWN bit was not written
back during 1000 coherent In polls. The new exclusive `dmem-v2` token therefore
requires the exact doorbell bit to clear and records descriptor OWN as a
non-writeback ownership publication marker, matching the upstream path which
never waits for descriptor writeback. All staging/ring synchronization, temporary
BME enable, exact PCI-command restore, bounded DDMA OWN polls, first 4096-byte
DMEM IDDMA, checksum capture, FWDL exit, and six reverse restores remain in the
same one-shot transaction. No retry, CPU release, interrupt, or nominal power-off
is added.
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
