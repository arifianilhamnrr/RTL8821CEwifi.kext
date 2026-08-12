#include "RTL8821CEProbe.hpp"

#include <IOKit/IOBufferMemoryDescriptor.h>
#include <IOKit/IODMACommand.h>
#include <IOKit/IOLib.h>
#include <IOKit/IOMemoryDescriptor.h>
#include <IOKit/pci/IOPCIFamilyDefinitions.h>
#include <kern/task.h>
#include <libkern/OSByteOrder.h>
#include <libkern/c++/OSData.h>
#include <pexpert/pexpert.h>

namespace {
constexpr IOByteCount kRegisterSysCfg1 = 0x00f0;
constexpr IOByteCount kRegisterCr = 0x0100;
constexpr UInt16 kCommandDecodeMask = kIOPCICommandMemorySpace | kIOPCICommandBusMaster;
constexpr UInt32 kExpectedSysCfg1 = 0x00494d39;
constexpr UInt32 kPowerPollIntervalMicroseconds = 50;
constexpr UInt32 kPowerPollIterations = 20000;
constexpr UInt8 kPowerPollMaximumAttempts = 2;
constexpr UInt8 kBaselineSampleCount = 64;
constexpr UInt32 kBaselineSampleIntervalMicroseconds = 50;
constexpr UInt32 kPowerExperimentConfirmation = 0x8821c821U;
constexpr bool kPowerExperimentBuildEnabled = false;
constexpr UInt32 kPowerExecutorContractConfirmation = 0x1400c821U;
constexpr bool kPowerExecutorBuildEnabled = false;
constexpr UInt32 kPowerExecutorSchemaVersion = 1;
constexpr size_t kPowerOnlyStepCount = 26;
constexpr size_t kPowerExecutionJournalCapacity = kPowerOnlyStepCount * 2;
constexpr UInt32 kFirmwareHeaderSize = 64;
constexpr UInt32 kFirmwareChecksumSize = 8;
constexpr UInt32 kFirmwareChunkSize = 0x1000;
constexpr UInt32 kExpectedFirmwareSize = 139472;
constexpr size_t kFirmwareTransferPlanCapacity = 35;
constexpr UInt32 kBeaconDescriptorRingSize = 16;
constexpr UInt32 kFirmwareStagingPacketSize = 48 + kFirmwareChunkSize;
constexpr UInt32 kDMAAlignment = 16;
constexpr UInt32 kTXPacketDescriptorSize = 48;
constexpr UInt16 kBeaconQueueOwn = 1U << 15;
constexpr UInt8 kBeaconQueueSelect = 16;
constexpr IOByteCount kRegisterPCICtrl3 = 0x0303;
constexpr IOByteCount kRegisterBeaconRingBase = 0x0308;
constexpr IOByteCount kRegisterBeaconWork = 0x0383;
constexpr IOByteCount kRegisterRWPTRClear = 0x039c;
constexpr UInt32 kTXRingDescriptorSize = 16;
constexpr UInt32 kRXRingDescriptorSize = 8;
constexpr UInt32 kDefaultTXRingEntries = 128;
constexpr UInt32 kBestEffortTXRingEntries = 256;
constexpr UInt32 kRXRingEntries = 512;
constexpr UInt32 kRXBufferSize = 11454 + 24;
constexpr size_t kTRXResourceCount = 9;
constexpr size_t kTRXDevicePlanCapacity = 21;
constexpr UInt32 kIDDMAFirmwareSource = 0x18780000U + kTXPacketDescriptorSize;
constexpr UInt32 kIDDMAOwn = 1U << 31;
constexpr UInt32 kIDDMAChecksumEnable = 1U << 29;
constexpr UInt32 kIDDMAChecksumContinue = 1U << 24;
constexpr UInt32 kIDDMALengthMask = 0x0003ffffU;
constexpr UInt32 kIDDMAPollIterations = 1000;
constexpr UInt32 kIDDMAPollIntervalMicroseconds = 10;
constexpr size_t kFirmwareRegisterBaselineCount = 12;
constexpr size_t kLifecyclePlanCount = 23;
constexpr size_t kPostPowerPlanCount = 5;
constexpr size_t kFirmwareSetupPlanCount = 17;
constexpr size_t kReservedPagePlanCount = 14;
constexpr size_t kFinalFIFOPlanCount = 27;
constexpr size_t kInterruptPlanCount = 16;
constexpr UInt32 kRegisterPlanSchemaVersion = 1;
constexpr UInt32 kDMAStatePlanSchemaVersion = 1;
constexpr UInt32 kExecutionSafetySchemaVersion = 2;
constexpr size_t kDMAStateCount = 17;
constexpr size_t kDMATransitionCount = 25;
constexpr size_t kDMAPublicationPlanCount = 18;
constexpr size_t kRollbackPolicyCount = 7;
constexpr size_t kMMIOPublicationContractCount = 2;
constexpr size_t kSerializedCommandContractCount = 6;
constexpr size_t kExecutionJournalContractCount = 5;
constexpr size_t kMappingExposureClassCount = 5;
constexpr size_t kMappingExposureTransitionCount = 8;
constexpr size_t kSynchronizationGenerationContractCount = 6;
constexpr UInt32 kRetainedMappingCount = 523;
constexpr UInt16 kTXFIFOPageCount = 512;
constexpr UInt16 kReservedPageCount = 52;
constexpr UInt16 kACQueuePageCount = 460;
constexpr UInt16 kReservedBoundary = 460;
constexpr UInt16 kReservedH2CInfoAddress = 468;
constexpr UInt16 kReservedH2CStaticAddress = 492;
constexpr UInt16 kReservedH2CQueueAddress = 500;
constexpr UInt16 kReservedFirmwareTXAddress = 508;
constexpr UInt16 kPublicQueuePageCount = 397;
constexpr UInt32 kH2CQueueAddress = 0x0000fa00U;
constexpr UInt32 kH2CQueueEnd = 0x0000fe00U;

extern "C" const UInt8 firmwareSectionStart[]
    __asm("section$start$__DATA_CONST$__firmware");
extern "C" const UInt8 firmwareSectionEnd[]
    __asm("section$end$__DATA_CONST$__firmware");

enum class PowerOp : UInt8 {
    Write8,
    Poll8,
};

struct PowerCommand {
    PowerOp op;
    UInt8 phase;
    UInt16 offset;
    UInt8 mask;
    UInt8 value;
};

struct RegisterSnapshot {
    UInt16 offset;
    UInt8 value;
    UInt8 reserved;
};

struct DryRunCommand {
    UInt8 index;
    UInt8 phase;
    UInt8 operation;
    UInt8 stateKnown;
    UInt16 offset;
    UInt8 mask;
    UInt8 value;
    UInt8 valueBefore;
    UInt8 valueAfter;
};

struct ProjectedRegister {
    UInt16 offset;
    UInt8 initialValue;
    UInt8 afterPowerOn;
    UInt8 afterPowerOff;
    UInt8 powerOnWriteMask;
    UInt8 powerOffWriteMask;
    UInt8 reserved;
};

struct PollBaselineSample {
    UInt8 sysPowerState;
    UInt8 systemPowerControl;
};

enum class ExperimentResult : UInt8 {
    Disarmed,
    Completed,
    WriteVerificationFailed,
    PollTimedOut,
    MACStillOff,
};

struct PreSystemCommand {
    UInt16 offset;
    UInt32 mask;
    UInt32 value;
    UInt8 width;
};

enum PowerOnlySource : UInt8 {
    kPowerOnlyPreSystem,
    kPowerOnlyFSM,
};

enum PowerJournalOutcome : UInt8 {
    kPowerJournalIntent,
    kPowerJournalSucceeded,
    kPowerJournalWriteEffectUnknown,
    kPowerJournalPollTimedOut,
};

struct PowerOnlyStepRecord {
    UInt8 sequence;
    UInt8 source;
    UInt8 operation;
    UInt8 width;
    UInt16 offset;
    UInt8 phase;
    UInt8 reserved;
    UInt32 mask;
    UInt32 value;
};

struct PowerExecutionJournalRecord {
    UInt8 sequence;
    UInt8 step;
    UInt8 outcome;
    UInt8 width;
    UInt16 offset;
    UInt16 reserved;
    UInt32 before;
    UInt32 after;
    UInt32 cumulativeHazards;
    UInt32 status;
};

struct PowerExecutorSimulationSummary {
    UInt16 stepCount;
    UInt16 writeCount;
    UInt16 pollCount;
    UInt16 scenarioCount;
    UInt32 successScenarioCount;
    UInt32 preIntentRejectionCount;
    UInt32 unknownEffectQuarantineCount;
    UInt32 timeoutQuarantineCount;
    UInt32 retryAuthorizedCount;
    UInt32 powerOffAuthorizedCount;
};

enum class PowerExecutorResult : UInt8 {
    Disarmed,
    Completed,
    ContractInvalid,
    WriteEffectUnknown,
    PollTimedOut,
    MACStillOff,
};

struct FirmwareHeader {
    UInt16 signature;
    UInt8 category;
    UInt8 function;
    UInt16 version;
    UInt8 subversion;
    UInt8 subindex;
    UInt32 reserved0;
    UInt32 feature;
    UInt8 month;
    UInt8 day;
    UInt8 hour;
    UInt8 minute;
    UInt16 year;
    UInt16 reserved1;
    UInt8 memoryUsage;
    UInt8 reserved2[3];
    UInt16 h2cFormatVersion;
    UInt16 reserved3;
    UInt32 dmemAddress;
    UInt32 dmemSize;
    UInt32 reserved4;
    UInt32 reserved5;
    UInt32 imemSize;
    UInt32 ememSize;
    UInt32 ememAddress;
    UInt32 imemAddress;
} __attribute__((packed));

struct FirmwareTransferPlan {
    UInt8 section;
    UInt8 firstChunk;
    UInt16 reserved;
    UInt32 firmwareOffset;
    UInt32 destination;
    UInt32 size;
};

struct PreSystemSnapshot {
    UInt8 reservedControl;
    UInt32 hciOptionControl;
    UInt32 padControl1;
    UInt32 ledConfig;
    UInt32 gpioMuxConfig;
    UInt8 systemFunctionEnable;
    UInt8 rfControl;
    UInt32 wlrf1;
    UInt8 systemPowerControl;
    UInt16 firmwareControl;
    UInt8 rpwm;
};

enum class DMAProbeStage : UInt8 {
    NotAttempted,
    BufferAllocation,
    VirtualAddress,
    CommandAllocation,
    SetMemoryDescriptor,
    Prepare,
    GenerateSegments,
    ValidateSegment,
    Complete,
    ClearMemoryDescriptor,
    Completed,
};

struct DMAProbeResult {
    DMAProbeStage stage;
    IOReturn status;
    UInt32 requestedLength;
    UInt32 segmentCount;
    UInt32 address;
    UInt32 length;
    bool zeroFilled;
};

struct DMATemplateResult {
    IOReturn status;
    UInt32 descriptorAddress;
    UInt32 stagingAddress;
    UInt16 packetBufferSize;
    UInt16 packetPSBLength;
    UInt32 packetAddress;
    UInt16 payloadBufferSize;
    UInt32 payloadAddress;
    UInt32 txWord0;
    UInt32 txWord1;
    UInt32 txWord3;
    UInt32 txWord8;
    UInt16 txChecksum;
    bool roundTripValid;
    bool payloadValid;
    bool resourcesReleased;
};

struct PreparedDMA {
    IOBufferMemoryDescriptor *buffer;
    IODMACommand *command;
    UInt32 address;
    UInt32 length;
    bool descriptorSet;
    bool prepared;
};

struct PCITXBufferElement {
    UInt16 bufferSize;
    UInt16 psbLength;
    UInt32 address;
} __attribute__((packed));

struct PCIRXBufferDescriptor {
    UInt16 bufferSize;
    UInt16 totalPacketSize;
    UInt32 address;
} __attribute__((packed));

struct QueueRegisterSnapshot {
    UInt8 pciControl3;
    UInt8 beaconWork;
    UInt16 reserved;
    UInt32 beaconRingBase;
    UInt32 rwPointerClear;
};

struct QueueProjectedCommand {
    UInt8 phase;
    UInt8 width;
    UInt16 offset;
    UInt32 original;
    UInt32 mask;
    UInt32 value;
    UInt32 projected;
};

struct TRXResourcePlan {
    UInt32 type;
    UInt32 queue;
    UInt32 entries;
    UInt32 descriptorSize;
    UInt32 baseRegister;
    UInt32 countRegister;
    UInt32 indexRegister;
    UInt32 ringBytes;
    UInt32 payloadBytes;
};

struct TRXDevicePlanRecord {
    UInt8 phase;
    UInt8 width;
    UInt8 resource;
    UInt8 operation;
    UInt16 offset;
    UInt16 reserved;
    UInt32 mask;
    UInt32 value;
};

struct FirmwareIDDMAPlanRecord {
    UInt8 section;
    UInt8 chunk;
    UInt8 firstChunk;
    UInt8 lastChunk;
    UInt32 firmwareOffset;
    UInt32 source;
    UInt32 destination;
    UInt32 length;
    UInt32 control;
    UInt16 stagingPacketLength;
    UInt16 stagingPSBLength;
    UInt8 preOwnPoll;
    UInt8 postOwnPoll;
    UInt8 checksumResetBefore;
    UInt8 checksumValidateAfter;
};

struct FirmwareRegisterBaseline {
    UInt16 offset;
    UInt8 width;
    UInt8 stable;
    UInt32 first;
    UInt32 second;
};

struct LifecyclePlanRecord {
    UInt8 sequence;
    UInt8 cycle;
    UInt8 operation;
    UInt8 requiredState;
    UInt16 prerequisite;
    UInt16 failureBoundary;
    UInt32 count;
};

struct RegisterPlanRecord {
    UInt8 phase;
    UInt8 width;
    UInt8 operation;
    UInt8 flags;
    UInt16 offset;
    UInt16 restoreGroup;
    UInt32 mask;
    UInt32 value;
};

enum RegisterPlanOperation : UInt8 {
    kPlanMaskSet = 1,
    kPlanMaskClear = 2,
    kPlanWrite = 3,
    kPlanMaskReplace = 4,
    kPlanFirmwareMode = 5,
    kPlanHostDMASubmit = 6,
    kPlanPoll = 7,
    kPlanRestoreSaved = 8,
    kPlanPollOwnership = 9,
    kPlanChecksumStatus = 10,
    kPlanRead = 11,
    kPlanWriteOneClear = 12,
    kPlanHostCompare = 13,
};

enum RegisterPlanFlags : UInt8 {
    kPlanFlagConditional = 1U << 0,
    kPlanFlagBackupBefore = 1U << 1,
    kPlanFlagSyntheticRestore = 1U << 2,
    kPlanFlagCPUTransition = 1U << 3,
    kPlanFlagFailureCleanup = 1U << 4,
    kPlanFlagAfterRXHandler = 1U << 5,
    kPlanFlagAfterNAPICompletion = 1U << 6,
};
constexpr UInt8 kRegisterPlanKnownFlags =
    kPlanFlagConditional | kPlanFlagBackupBefore |
    kPlanFlagSyntheticRestore | kPlanFlagCPUTransition |
    kPlanFlagFailureCleanup | kPlanFlagAfterRXHandler |
    kPlanFlagAfterNAPICompletion;

struct EfuseParserValidationResult {
    bool oneByteHeader;
    bool twoByteHeader;
    bool logicalBoundsRejection;
    bool physicalTruncationRejection;
    bool extendedHeaderTruncationRejection;
};

struct DMAStateRecord {
    UInt8 state;
    UInt8 exposure;
    UInt8 rollbackClass;
    UInt8 terminal;
    UInt32 requiredFlags;
    UInt32 forbiddenFlags;
    UInt32 releasePolicy;
};

struct DMATransitionRecord {
    UInt8 from;
    UInt8 to;
    UInt8 operation;
    UInt8 failureState;
    UInt32 prerequisiteFlags;
    UInt32 resultFlags;
    UInt32 sequence;
};

struct DMAPublicationRecord {
    UInt8 sequence;
    UInt8 resource;
    UInt8 operation;
    UInt8 direction;
    UInt32 prerequisiteFlags;
    UInt32 resultFlags;
    UInt32 failureState;
};

struct RollbackPolicyRecord {
    UInt8 exposure;
    UInt8 clearBusMasterAllowed;
    UInt8 restoreRegistersAllowed;
    UInt8 releaseVisibleMappingsAllowed;
    UInt32 permittedActions;
    UInt32 forbiddenActions;
    UInt32 failureState;
};

struct MMIOPublicationContract {
    UInt8 resource;
    UInt8 width;
    UInt16 offset;
    UInt32 mask;
    UInt32 value;
    UInt32 readbackMask;
};

struct SerializedCommandContractRecord {
    UInt8 command;
    UInt8 admissionBefore;
    UInt8 admissionAfter;
    UInt8 flags;
    UInt32 requiredJournalPhases;
    UInt32 forbiddenAdmissionMask;
    UInt32 failureDisposition;
    UInt32 requiredReleaseEvidence;
};

struct ExecutionJournalContractRecord {
    UInt8 phase;
    UInt8 terminal;
    UInt8 stateMayAdvance;
    UInt8 preservesWorstCaseHazards;
    UInt32 requiredPriorPhaseMask;
    UInt32 permittedSuccessorMask;
    UInt32 requiredFields;
};

struct MappingExposureClassRecord {
    UInt8 resource;
    UInt8 direction;
    UInt8 initialExposure;
    UInt8 generationRequired;
    UInt32 mappingCount;
    UInt32 deviceReference;
    UInt32 releaseEvidenceMask;
};

struct SynchronizationGenerationContractRecord {
    UInt8 sequence;
    UInt8 resource;
    UInt8 event;
    UInt8 advancesGeneration;
    UInt32 requiredPriorEventMask;
    UInt32 capturesCurrentGeneration;
    UInt32 requiresCurrentGenerationMatch;
};

struct MappingExposureTransitionRecord {
    UInt8 from;
    UInt8 to;
    UInt8 requiresDeviceQuiescence;
    UInt8 retainsMapping;
    UInt32 requiredEvidence;
    UInt32 forbiddenEvidence;
};

struct FailureInjectionSummary {
    UInt16 lifecycleBoundaries;
    UInt16 transitionBoundaries;
    UInt16 publicationBoundaries;
    UInt16 totalBoundaries;
    UInt32 scenarioCount;
    UInt32 rejectedUnsafeScenarioCount;
    UInt32 containmentScenarioCount;
    UInt32 hostCleanupScenarioCount;
};

struct SymbolicInterpreterSummary {
    UInt32 lifecycleTraceCount;
    UInt32 transitionTraceCount;
    UInt32 publicationTraceCount;
    UInt32 totalTraceCount;
    UInt32 successfulTraceCount;
    UInt32 containmentTraceCount;
    UInt32 rejectedMutationCount;
    UInt32 expectedRejectedMutationCount;
};

struct SymbolicExecutionState {
    UInt8 admission;
    UInt8 dmaState;
    UInt8 journalPhase;
    UInt8 activeCommand;
    UInt8 deviceQuiescent;
    UInt8 reserved[3];
    UInt32 hazards;
    UInt32 hazardsBeforeIntent;
    UInt32 dmaFlags;
    UInt32 releaseEvidence;
    UInt32 gateTicket;
    UInt32 transaction;
    UInt32 intentGateTicket;
    UInt32 intentTransaction;
    UInt32 journalSequence;
    UInt32 generationEvents;
    UInt32 txGeneration;
    UInt32 txSynchronizedGeneration;
    UInt32 rxGeneration;
    UInt32 rxSynchronizedGeneration;
    UInt8 mappingExposure[kMappingExposureClassCount];
};

struct FIFOPlanSummary {
    UInt16 txPages;
    UInt16 reservedPages;
    UInt16 acQueuePages;
    UInt16 reservedBoundary;
    UInt16 driverAddress;
    UInt16 h2cInfoAddress;
    UInt16 h2cStaticAddress;
    UInt16 h2cQueueAddress;
    UInt16 firmwareTXAddress;
    UInt16 publicQueuePages;
    UInt32 h2cByteAddress;
    UInt32 h2cByteEnd;
    UInt16 txdmaQueueMap;
    UInt16 reserved;
};

struct EfuseDependencyPlan {
    UInt16 physicalSize;
    UInt16 logicalSize;
    UInt16 protectSize;
    UInt16 readableSize;
    UInt32 physicalPollLimit;
    UInt32 physicalPollDelayMicroseconds;
    UInt32 requiredFieldMask;
    UInt32 phyDependencyMask;
};

enum EfuseDependency : UInt32 {
    kEfuseRFEOption = 1U << 0,
    kEfuseRFBoardOption = 1U << 1,
    kEfuseCrystalCap = 1U << 2,
    kEfusePAType2G = 1U << 3,
    kEfusePAType5G = 1U << 4,
    kEfuseLNAType2G = 1U << 5,
    kEfuseLNAType5G = 1U << 6,
    kEfuseChannelPlan = 1U << 7,
    kEfuseCountryCode = 1U << 8,
    kEfuseBTSetting = 1U << 9,
    kEfuseThermalMeter = 1U << 10,
    kEfuseTXSwing2G = 1U << 11,
    kEfuseTXSwing5G = 1U << 12,
    kEfusePackageType = 1U << 13,
    kEfuseTXPowerTables = 1U << 14,
    kEfuseMACAddress = 1U << 15,
    kEfuseRegulatory = 1U << 16,
};

constexpr UInt32 kEfuseRequiredDependencies =
    kEfuseRFEOption | kEfuseRFBoardOption | kEfuseCrystalCap |
    kEfusePAType2G | kEfusePAType5G | kEfuseLNAType2G |
    kEfuseLNAType5G | kEfuseChannelPlan | kEfuseCountryCode |
    kEfuseBTSetting | kEfuseThermalMeter | kEfuseTXSwing2G |
    kEfuseTXSwing5G | kEfusePackageType | kEfuseTXPowerTables |
    kEfuseMACAddress | kEfuseRegulatory;
constexpr UInt32 kEfusePHYDependencies =
    kEfuseRFEOption | kEfuseCrystalCap | kEfusePAType2G |
    kEfusePAType5G | kEfuseLNAType2G | kEfuseLNAType5G |
    kEfuseThermalMeter | kEfuseTXPowerTables;

struct TRXAllocationProbeResult {
    DMAProbeStage stage;
    IOReturn status;
    UInt32 failedResourceIndex;
    UInt32 failedPayloadIndex;
    UInt32 ringMappingCount;
    UInt32 payloadMappingCount;
    UInt32 peakMappingCount;
    UInt64 ringBytes;
    UInt64 payloadBytes;
    UInt32 rxDescriptorValidCount;
    UInt32 rxDescriptorZeroTagCount;
    UInt32 rxDescriptorAddressMatchCount;
    UInt32 rxDescriptorAddressRangeValidCount;
    UInt32 txRingValidCount;
    UInt32 txDescriptorZeroCount;
    UInt32 txRingAddressRangeValidCount;
    bool attempted;
    bool allocationComplete;
    bool resourcesReleased;
    bool rxDescriptorMaterialized;
    bool rxDescriptorFormatValid;
    bool txDescriptorFormatValid;
};

static_assert(sizeof(PCITXBufferElement) == 8,
              "unexpected PCI TX buffer element layout");
static_assert(sizeof(PCIRXBufferDescriptor) == 8,
              "unexpected PCI RX buffer descriptor layout");
static_assert(sizeof(QueueRegisterSnapshot) == 12,
              "unexpected queue snapshot layout");
static_assert(sizeof(QueueProjectedCommand) == 20,
              "unexpected queue command layout");
static_assert(sizeof(TRXResourcePlan) == 36,
              "unexpected TRX resource layout");
static_assert(sizeof(TRXDevicePlanRecord) == 16,
              "unexpected TRX device-plan layout");
static_assert(sizeof(FirmwareIDDMAPlanRecord) == 32,
              "unexpected firmware IDDMA-plan layout");
static_assert(sizeof(FirmwareRegisterBaseline) == 12,
              "unexpected firmware register-baseline layout");
static_assert(sizeof(LifecyclePlanRecord) == 12,
              "unexpected lifecycle-plan layout");
static_assert(sizeof(RegisterPlanRecord) == 16,
              "unexpected register-plan layout");
static_assert(sizeof(FIFOPlanSummary) == 32,
              "unexpected FIFO-plan summary layout");
static_assert(sizeof(EfuseDependencyPlan) == 24,
              "unexpected efuse-dependency layout");
static_assert(sizeof(DMAStateRecord) == 16,
              "unexpected DMA-state layout");
static_assert(sizeof(DMATransitionRecord) == 16,
              "unexpected DMA-transition layout");
static_assert(sizeof(DMAPublicationRecord) == 16,
              "unexpected DMA-publication layout");
static_assert(sizeof(RollbackPolicyRecord) == 16,
              "unexpected rollback-policy layout");
static_assert(sizeof(MMIOPublicationContract) == 16,
              "unexpected MMIO-publication layout");
static_assert(sizeof(SerializedCommandContractRecord) == 20,
              "unexpected serialized-command layout");
static_assert(sizeof(ExecutionJournalContractRecord) == 16,
              "unexpected execution-journal layout");
static_assert(sizeof(MappingExposureClassRecord) == 16,
              "unexpected mapping-exposure layout");
static_assert(sizeof(MappingExposureTransitionRecord) == 12,
              "unexpected mapping-exposure transition layout");
static_assert(sizeof(SynchronizationGenerationContractRecord) == 16,
              "unexpected synchronization-generation layout");
static_assert(sizeof(FailureInjectionSummary) == 24,
              "unexpected failure-injection summary layout");
static_assert(sizeof(SymbolicInterpreterSummary) == 32,
              "unexpected symbolic-interpreter summary layout");

enum DMAState : UInt8 {
    kDMAColdBaseline,
    kDMAHostPrepared,
    kDMADevicePlanBMEOff,
    kDMABMEArmedPreKick,
    kDMABeaconInFlight,
    kDMAIDDMAInFlight,
    kDMAFirmwarePartial,
    kDMAFirmwareCopiedCPUStopped,
    kDMACPUReleasedFWWait,
    kDMAOperationalPending,
    kDMAOperational,
    kDMAQuarantineIdentity,
    kDMAQuarantineDMA,
    kDMAQuarantineFirmware,
    kDMAQuarantinePower,
    kDMAColdRemovalRequired,
    kDMAPoweredBMEOff,
};

enum DMAStateFlag : UInt32 {
    kDMAFlagMappingsPrepared = 1U << 0,
    kDMAFlagDescriptorsMaterialized = 1U << 1,
    kDMAFlagDescriptorsSynchronized = 1U << 2,
    kDMAFlagRingsProgrammed = 1U << 3,
    kDMAFlagBMEEnabled = 1U << 4,
    kDMAFlagBeaconOwned = 1U << 5,
    kDMAFlagBeaconKicked = 1U << 6,
    kDMAFlagIDDMAOwned = 1U << 7,
    kDMAFlagFirmwareMemoryDirty = 1U << 8,
    kDMAFlagCPUReleased = 1U << 9,
    kDMAFlagFirmwareRunning = 1U << 10,
    kDMAFlagInterruptsEnabled = 1U << 11,
    kDMAFlagMappingsReleaseAuthorized = 1U << 12,
    kDMAFlagPowerTransitionStarted = 1U << 13,
    kDMAFlagStagingSynchronized = 1U << 14,
    kDMAFlagTXRingMaterialized = 1U << 15,
    kDMAFlagTXRingSynchronized = 1U << 16,
    kDMAFlagRXRingMaterialized = 1U << 17,
    kDMAFlagRXRingSynchronized = 1U << 18,
    kDMAFlagRXPayloadSynchronized = 1U << 19,
};

enum DMAPlanOperation : UInt8 {
    kDMAAllocate,
    kDMAProgramRings,
    kDMAEnableBME,
    kDMAPublishBeacon,
    kDMAStartIDDMA,
    kDMACompleteChunk,
    kDMACompleteFirmwareCopy,
    kDMAReleaseCPU,
    kDMAObserveFirmwareReady,
    kDMAFinishOperationalSetup,
    kDMAEnterQuarantine,
    kDMAStartPowerTransition,
    kDMASynchronizeOut,
    kDMASynchronizeIn,
    kDMAReleaseFence,
    kDMAAcquireFence,
    kDMASetOwnershipLast,
    kDMAMMIODoorbell,
    kDMAMMIOReadback,
    kDMANoOwnershipContract,
};

enum DMAResource : UInt8 {
    kDMAResourceStaging,
    kDMAResourceTXRing,
    kDMAResourceRXRing,
    kDMAResourceRXPayload,
    kDMAResourceDevice,
};

enum DMAExposure : UInt8 {
    kDMAExposureHostOnly,
    kDMAExposureProgrammedBMEOff,
    kDMAExposureBME,
    kDMAExposureBeacon,
    kDMAExposureIDDMA,
    kDMAExposurePower,
    kDMAExposureOperational,
};

enum SerializedCommand : UInt8 {
    kSerializedOpen,
    kSerializedAdvance,
    kSerializedPublish,
    kSerializedContain,
    kSerializedBeginStop,
    kSerializedReleaseMappings,
};

enum CommandAdmission : UInt8 {
    kAdmissionOpen,
    kAdmissionClosing,
    kAdmissionContained,
    kAdmissionClosed,
};

enum SerializedCommandFlag : UInt8 {
    kCommandMutatesState = 1U << 0,
    kCommandRequiresWriteAheadIntent = 1U << 1,
    kCommandMayIncreaseHazard = 1U << 2,
    kCommandContainmentOnly = 1U << 3,
    kCommandRejectReentry = 1U << 4,
};

enum JournalPhase : UInt8 {
    kJournalIntent,
    kJournalSucceeded,
    kJournalFailedNoEffect,
    kJournalFailedEffectUnknown,
    kJournalContained,
};

enum JournalField : UInt32 {
    kJournalFieldSequence = 1U << 0,
    kJournalFieldTransaction = 1U << 1,
    kJournalFieldGateTicket = 1U << 2,
    kJournalFieldPlanSequence = 1U << 3,
    kJournalFieldState = 1U << 4,
    kJournalFieldHazards = 1U << 5,
    kJournalFieldMappingGeneration = 1U << 6,
    kJournalFieldStatus = 1U << 7,
};

constexpr UInt32 kJournalIdentityFields =
    kJournalFieldSequence | kJournalFieldTransaction |
    kJournalFieldGateTicket | kJournalFieldPlanSequence |
    kJournalFieldState | kJournalFieldHazards |
    kJournalFieldMappingGeneration;

enum ExecutionHazard : UInt32 {
    kHazardPowerWritePossible = 1U << 0,
    kHazardAddressProgrammed = 1U << 1,
    kHazardBMEPossible = 1U << 2,
    kHazardBeaconPossible = 1U << 3,
    kHazardIDDMAPossible = 1U << 4,
    kHazardFirmwareMemoryDirty = 1U << 5,
    kHazardCPUReleased = 1U << 6,
    kHazardInterruptsPossible = 1U << 7,
};

constexpr UInt32 kKnownExecutionHazards =
    kHazardPowerWritePossible | kHazardAddressProgrammed |
    kHazardBMEPossible | kHazardBeaconPossible |
    kHazardIDDMAPossible | kHazardFirmwareMemoryDirty |
    kHazardCPUReleased | kHazardInterruptsPossible;

enum MappingExposureState : UInt8 {
    kMappingHostOnly,
    kMappingAddressProgrammedBMEOff,
    kMappingDMAReachable,
    kMappingQuarantinedRetained,
    kMappingReleaseAuthorized,
    kMappingReleased,
};

enum MappingResourceClass : UInt8 {
    kMappingFirmwareDescriptor,
    kMappingFirmwareStaging,
    kMappingTXRing,
    kMappingRXRing,
    kMappingRXPayload,
};

enum MappingReleaseEvidence : UInt32 {
    kReleaseNeverExposed = 1U << 0,
    kReleaseVerifiedBMEOffRestore = 1U << 1,
    kReleaseConfirmedColdPowerRemoval = 1U << 2,
};

enum SynchronizationGenerationEvent : UInt8 {
    kGenerationInitialTXSynchronize,
    kGenerationOwnershipMutation,
    kGenerationFinalTXSynchronize,
    kGenerationTXDoorbell,
    kGenerationRXRebuild,
    kGenerationRXDoorbell,
};

constexpr DMAStateRecord kDMAStates[kDMAStateCount] = {
    {0, 0, 0, 0, 0, kDMAFlagBMEEnabled, kDMAFlagMappingsReleaseAuthorized},
    {1, 0, 0, 0, kDMAFlagMappingsPrepared, kDMAFlagBMEEnabled,
        kDMAFlagMappingsReleaseAuthorized},
    {2, 1, 1, 0, kDMAFlagMappingsPrepared | kDMAFlagRingsProgrammed,
        kDMAFlagBMEEnabled, kDMAFlagMappingsReleaseAuthorized},
    {3, 5, 5, 0, kDMAFlagBMEEnabled | kDMAFlagRingsProgrammed |
        kDMAFlagPowerTransitionStarted,
        kDMAFlagBeaconKicked, 0},
    {4, 5, 5, 0, kDMAFlagBeaconOwned | kDMAFlagBeaconKicked |
        kDMAFlagPowerTransitionStarted, 0, 0},
    {5, 5, 5, 0, kDMAFlagIDDMAOwned | kDMAFlagPowerTransitionStarted, 0, 0},
    {6, 5, 5, 0, kDMAFlagFirmwareMemoryDirty |
        kDMAFlagPowerTransitionStarted, 0, 0},
    {7, 5, 5, 0, kDMAFlagFirmwareMemoryDirty |
        kDMAFlagPowerTransitionStarted, kDMAFlagCPUReleased, 0},
    {8, 5, 5, 0, kDMAFlagCPUReleased | kDMAFlagPowerTransitionStarted,
        kDMAFlagFirmwareRunning, 0},
    {9, 5, 5, 0, kDMAFlagFirmwareRunning |
        kDMAFlagPowerTransitionStarted, kDMAFlagInterruptsEnabled, 0},
    {10, 6, 6, 0, kDMAFlagFirmwareRunning | kDMAFlagInterruptsEnabled |
        kDMAFlagPowerTransitionStarted, 0, 0},
    {11, 0, 0, 1, 0, 0, kDMAFlagMappingsReleaseAuthorized},
    {12, 2, 2, 1, 0, 0, 0},
    {13, 4, 4, 1, 0, 0, 0},
    {14, 5, 5, 1, 0, 0, 0},
    {15, 5, 5, 1, 0, 0, 0},
    {16, 5, 5, 0, kDMAFlagPowerTransitionStarted,
        kDMAFlagBMEEnabled, 0},
};

constexpr DMATransitionRecord kDMATransitions[kDMATransitionCount] = {
    {0, 1, kDMAAllocate, 11, 0,
        kDMAFlagMappingsPrepared | kDMAFlagDescriptorsMaterialized |
            kDMAFlagTXRingMaterialized | kDMAFlagRXRingMaterialized, 0},
    {1, 2, kDMAProgramRings, 11, kDMAFlagMappingsPrepared,
        kDMAFlagRingsProgrammed, 1},
    {2, 16, kDMAStartPowerTransition, 14, kDMAFlagRingsProgrammed,
        kDMAFlagPowerTransitionStarted, 2},
    {16, 3, kDMAEnableBME, 14,
        kDMAFlagRingsProgrammed | kDMAFlagPowerTransitionStarted,
        kDMAFlagBMEEnabled, 3},
    {3, 4, kDMAPublishBeacon, 14, kDMAFlagTXRingSynchronized,
        kDMAFlagBeaconOwned | kDMAFlagBeaconKicked, 4},
    {4, 5, kDMAStartIDDMA, 14, kDMAFlagBeaconKicked,
        kDMAFlagIDDMAOwned | kDMAFlagFirmwareMemoryDirty, 5},
    {5, 6, kDMACompleteChunk, 14, kDMAFlagIDDMAOwned,
        kDMAFlagFirmwareMemoryDirty, 6},
    {6, 4, kDMAPublishBeacon, 14, kDMAFlagFirmwareMemoryDirty,
        kDMAFlagBeaconOwned | kDMAFlagBeaconKicked, 7},
    {6, 7, kDMACompleteFirmwareCopy, 14, kDMAFlagFirmwareMemoryDirty,
        kDMAFlagFirmwareMemoryDirty, 8},
    {7, 8, kDMAReleaseCPU, 14, kDMAFlagFirmwareMemoryDirty,
        kDMAFlagCPUReleased, 9},
    {8, 9, kDMAObserveFirmwareReady, 14, kDMAFlagCPUReleased,
        kDMAFlagFirmwareRunning, 10},
    {9, 10, kDMAFinishOperationalSetup, 14, kDMAFlagFirmwareRunning,
        kDMAFlagInterruptsEnabled, 11},
    {0, 11, kDMAEnterQuarantine, 11, 0, 0, 12},
    {1, 11, kDMAEnterQuarantine, 11, 0, 0, 13},
    {2, 11, kDMAEnterQuarantine, 11, 0, 0, 14},
    {3, 14, kDMAEnterQuarantine, 14, 0, 0, 15},
    {4, 14, kDMAEnterQuarantine, 14, 0, 0, 16},
    {5, 14, kDMAEnterQuarantine, 14, 0, 0, 17},
    {6, 14, kDMAEnterQuarantine, 14, 0, 0, 18},
    {8, 14, kDMAEnterQuarantine, 14, 0, 0, 19},
    {14, 15, kDMAEnterQuarantine, 15, 0, 0, 20},
    {7, 14, kDMAEnterQuarantine, 14, 0, 0, 21},
    {9, 14, kDMAEnterQuarantine, 14, 0, 0, 22},
    {10, 14, kDMAEnterQuarantine, 14, 0, 0, 23},
    {16, 14, kDMAEnterQuarantine, 14, 0, 0, 24},
};

constexpr DMAPublicationRecord
kDMAPublicationPlan[kDMAPublicationPlanCount] = {
    {0, kDMAResourceStaging, kDMASynchronizeOut, kIODirectionOut,
        kDMAFlagMappingsPrepared, kDMAFlagStagingSynchronized, 11},
    {1, kDMAResourceStaging, kDMAReleaseFence, 0,
        kDMAFlagStagingSynchronized, kDMAFlagStagingSynchronized, 11},
    {2, kDMAResourceTXRing, kDMASynchronizeOut, kIODirectionOut,
        kDMAFlagTXRingMaterialized | kDMAFlagStagingSynchronized,
        kDMAFlagTXRingSynchronized, 11},
    {3, kDMAResourceTXRing, kDMAReleaseFence, 0,
        kDMAFlagTXRingSynchronized, kDMAFlagTXRingSynchronized, 11},
    {4, kDMAResourceTXRing, kDMASetOwnershipLast, 0,
        kDMAFlagTXRingSynchronized, kDMAFlagBeaconOwned, 14},
    {5, kDMAResourceTXRing, kDMASynchronizeOut, kIODirectionOut,
        kDMAFlagBeaconOwned, kDMAFlagTXRingSynchronized, 14},
    {6, kDMAResourceTXRing, kDMAReleaseFence, 0,
        kDMAFlagTXRingSynchronized, kDMAFlagTXRingSynchronized, 14},
    {7, kDMAResourceDevice, kDMAMMIODoorbell, 0,
        kDMAFlagBMEEnabled | kDMAFlagBeaconOwned,
        kDMAFlagBeaconKicked, 14},
    {8, kDMAResourceDevice, kDMAMMIOReadback, 0,
        kDMAFlagBeaconKicked, kDMAFlagBeaconKicked, 14},
    {9, kDMAResourceRXRing, kDMASynchronizeIn, kIODirectionIn,
        kDMAFlagBMEEnabled, kDMAFlagRXRingSynchronized, 14},
    {10, kDMAResourceRXRing, kDMAAcquireFence, 0,
        kDMAFlagRXRingSynchronized, kDMAFlagRXRingSynchronized, 14},
    {11, kDMAResourceRXPayload, kDMASynchronizeIn, kIODirectionIn,
        kDMAFlagRXRingSynchronized, kDMAFlagRXPayloadSynchronized, 14},
    {12, kDMAResourceRXPayload, kDMAAcquireFence, 0,
        kDMAFlagRXPayloadSynchronized, kDMAFlagRXPayloadSynchronized, 14},
    {13, kDMAResourceRXRing, kDMASynchronizeOut, kIODirectionOut,
        kDMAFlagRXRingMaterialized, kDMAFlagRXRingSynchronized, 14},
    {14, kDMAResourceRXRing, kDMAReleaseFence, 0,
        kDMAFlagRXRingSynchronized, kDMAFlagRXRingSynchronized, 14},
    {15, kDMAResourceRXRing, kDMANoOwnershipContract, 0,
        kDMAFlagRXRingSynchronized, kDMAFlagRXRingSynchronized, 14},
    {16, kDMAResourceDevice, kDMAMMIODoorbell, 0,
        kDMAFlagBMEEnabled | kDMAFlagRXRingSynchronized,
        kDMAFlagRXRingSynchronized, 14},
    {17, kDMAResourceDevice, kDMAMMIOReadback, 0,
        kDMAFlagRXRingSynchronized, kDMAFlagRXRingSynchronized, 14},
};

enum RollbackAction : UInt32 {
    kRollbackReleaseHostOnly = 1U << 0,
    kRollbackRestoreBMEOffRegisters = 1U << 1,
    kRollbackStopProducers = 1U << 2,
    kRollbackMaskInterrupts = 1U << 3,
    kRollbackClearBMEContainment = 1U << 4,
    kRollbackRetainVisibleMappings = 1U << 5,
    kRollbackCollectReadOnlyDiagnostics = 1U << 6,
    kRollbackColdPowerRemoval = 1U << 7,
    kRollbackResetPointers = 1U << 8,
    kRollbackRewriteOwnership = 1U << 9,
    kRollbackRetryFirmware = 1U << 10,
    kRollbackRunPowerOff = 1U << 11,
    kRollbackReleaseVisibleMappings = 1U << 12,
};

constexpr RollbackPolicyRecord kRollbackPolicies[kRollbackPolicyCount] = {
    {kDMAExposureHostOnly, 0, 0, 1,
        kRollbackReleaseHostOnly,
        kRollbackRunPowerOff, kDMAQuarantineIdentity},
    {kDMAExposureProgrammedBMEOff, 0, 1, 1,
        kRollbackRestoreBMEOffRegisters | kRollbackReleaseHostOnly,
        kRollbackRunPowerOff, kDMAQuarantineIdentity},
    {kDMAExposureBME, 1, 0, 0,
        kRollbackStopProducers | kRollbackClearBMEContainment |
            kRollbackRetainVisibleMappings | kRollbackCollectReadOnlyDiagnostics,
        kRollbackResetPointers | kRollbackReleaseVisibleMappings |
            kRollbackRunPowerOff, kDMAQuarantineDMA},
    {kDMAExposureBeacon, 1, 0, 0,
        kRollbackStopProducers | kRollbackClearBMEContainment |
            kRollbackRetainVisibleMappings | kRollbackCollectReadOnlyDiagnostics |
            kRollbackColdPowerRemoval,
        kRollbackRewriteOwnership | kRollbackResetPointers |
            kRollbackRetryFirmware | kRollbackReleaseVisibleMappings |
            kRollbackRunPowerOff, kDMAQuarantineDMA},
    {kDMAExposureIDDMA, 1, 0, 0,
        kRollbackStopProducers | kRollbackClearBMEContainment |
            kRollbackRetainVisibleMappings | kRollbackCollectReadOnlyDiagnostics |
            kRollbackColdPowerRemoval,
        kRollbackRewriteOwnership | kRollbackResetPointers |
            kRollbackRetryFirmware | kRollbackReleaseVisibleMappings |
            kRollbackRunPowerOff, kDMAQuarantineFirmware},
    {kDMAExposurePower, 1, 0, 0,
        kRollbackStopProducers | kRollbackClearBMEContainment |
            kRollbackRetainVisibleMappings |
            kRollbackCollectReadOnlyDiagnostics | kRollbackColdPowerRemoval,
        kRollbackResetPointers | kRollbackRetryFirmware |
            kRollbackReleaseVisibleMappings | kRollbackRunPowerOff,
        kDMAQuarantinePower},
    {kDMAExposureOperational, 1, 0, 0,
        kRollbackStopProducers | kRollbackMaskInterrupts |
            kRollbackClearBMEContainment | kRollbackRetainVisibleMappings |
            kRollbackCollectReadOnlyDiagnostics | kRollbackColdPowerRemoval,
        kRollbackRetryFirmware | kRollbackReleaseVisibleMappings |
            kRollbackRunPowerOff, kDMAQuarantineFirmware},
};

constexpr MMIOPublicationContract
kMMIOPublicationContracts[kMMIOPublicationContractCount] = {
    {kDMAResourceTXRing, 1, 0x0383, 0x00000010U,
        0x00000010U, 0x00000010U},
    {kDMAResourceRXRing, 2, 0x03b4, 0x00000fffU,
        0x00000000U, 0x00000fffU},
};

constexpr UInt32 admissionMask(CommandAdmission admission)
{
    return 1U << admission;
}

constexpr UInt32 journalPhaseMask(JournalPhase phase)
{
    return 1U << phase;
}

constexpr SerializedCommandContractRecord
kSerializedCommandContracts[kSerializedCommandContractCount] = {
    {kSerializedOpen, kAdmissionClosed, kAdmissionOpen,
        kCommandMutatesState | kCommandRequiresWriteAheadIntent |
            kCommandRejectReentry,
        journalPhaseMask(kJournalIntent),
        admissionMask(kAdmissionOpen) | admissionMask(kAdmissionClosing) |
            admissionMask(kAdmissionContained),
        kDMAQuarantineIdentity, 0},
    {kSerializedAdvance, kAdmissionOpen, kAdmissionOpen,
        kCommandMutatesState | kCommandRequiresWriteAheadIntent |
            kCommandMayIncreaseHazard | kCommandRejectReentry,
        journalPhaseMask(kJournalIntent),
        admissionMask(kAdmissionClosing) | admissionMask(kAdmissionContained) |
            admissionMask(kAdmissionClosed),
        kDMAQuarantinePower, 0},
    {kSerializedPublish, kAdmissionOpen, kAdmissionOpen,
        kCommandMutatesState | kCommandRequiresWriteAheadIntent |
            kCommandMayIncreaseHazard | kCommandRejectReentry,
        journalPhaseMask(kJournalIntent),
        admissionMask(kAdmissionClosing) | admissionMask(kAdmissionContained) |
            admissionMask(kAdmissionClosed),
        kDMAQuarantinePower, 0},
    {kSerializedContain, kAdmissionClosing, kAdmissionContained,
        kCommandMutatesState | kCommandRequiresWriteAheadIntent |
            kCommandContainmentOnly | kCommandRejectReentry,
        journalPhaseMask(kJournalIntent), admissionMask(kAdmissionClosed),
        kDMAColdRemovalRequired, 0},
    {kSerializedBeginStop, kAdmissionOpen, kAdmissionClosing,
        kCommandMutatesState | kCommandRequiresWriteAheadIntent |
            kCommandRejectReentry,
        journalPhaseMask(kJournalIntent),
        admissionMask(kAdmissionContained) | admissionMask(kAdmissionClosed),
        kDMAQuarantinePower, 0},
    {kSerializedReleaseMappings, kAdmissionContained, kAdmissionClosed,
        kCommandMutatesState | kCommandRequiresWriteAheadIntent |
            kCommandContainmentOnly | kCommandRejectReentry,
        journalPhaseMask(kJournalIntent),
        admissionMask(kAdmissionOpen) | admissionMask(kAdmissionClosing) |
            admissionMask(kAdmissionClosed),
        kDMAColdRemovalRequired, kReleaseConfirmedColdPowerRemoval},
};

constexpr ExecutionJournalContractRecord
kExecutionJournalContracts[kExecutionJournalContractCount] = {
    {kJournalIntent, 0, 0, 1, 0,
        journalPhaseMask(kJournalSucceeded) |
            journalPhaseMask(kJournalFailedNoEffect) |
            journalPhaseMask(kJournalFailedEffectUnknown),
        kJournalIdentityFields},
    {kJournalSucceeded, 1, 1, 1, journalPhaseMask(kJournalIntent), 0,
        kJournalIdentityFields | kJournalFieldStatus},
    {kJournalFailedNoEffect, 1, 0, 0, journalPhaseMask(kJournalIntent), 0,
        kJournalIdentityFields | kJournalFieldStatus},
    {kJournalFailedEffectUnknown, 0, 0, 1,
        journalPhaseMask(kJournalIntent), journalPhaseMask(kJournalContained),
        kJournalIdentityFields | kJournalFieldStatus},
    {kJournalContained, 1, 0, 1,
        journalPhaseMask(kJournalFailedEffectUnknown), 0,
        kJournalIdentityFields | kJournalFieldStatus},
};

constexpr MappingExposureClassRecord
kMappingExposureClasses[kMappingExposureClassCount] = {
    {kMappingFirmwareDescriptor, kIODirectionOut, kMappingHostOnly, 1,
        1, 1, kReleaseNeverExposed | kReleaseVerifiedBMEOffRestore |
            kReleaseConfirmedColdPowerRemoval},
    {kMappingFirmwareStaging, kIODirectionOut, kMappingHostOnly, 1,
        1, 1, kReleaseNeverExposed | kReleaseVerifiedBMEOffRestore |
            kReleaseConfirmedColdPowerRemoval},
    {kMappingTXRing, kIODirectionOut, kMappingHostOnly, 1,
        8, 1, kReleaseNeverExposed | kReleaseVerifiedBMEOffRestore |
            kReleaseConfirmedColdPowerRemoval},
    {kMappingRXRing, kIODirectionInOut, kMappingHostOnly, 1,
        1, 1, kReleaseNeverExposed | kReleaseVerifiedBMEOffRestore |
            kReleaseConfirmedColdPowerRemoval},
    {kMappingRXPayload, kIODirectionIn, kMappingHostOnly, 2,
        512, 1, kReleaseNeverExposed |
            kReleaseConfirmedColdPowerRemoval},
};

constexpr MappingExposureTransitionRecord
kMappingExposureTransitions[kMappingExposureTransitionCount] = {
    {kMappingHostOnly, kMappingAddressProgrammedBMEOff, 0, 1, 0, 0},
    {kMappingHostOnly, kMappingReleaseAuthorized, 0, 1,
        kReleaseNeverExposed, 0},
    {kMappingAddressProgrammedBMEOff, kMappingDMAReachable, 0, 1, 0, 0},
    {kMappingAddressProgrammedBMEOff, kMappingReleaseAuthorized, 1, 1,
        kReleaseVerifiedBMEOffRestore, 0},
    {kMappingAddressProgrammedBMEOff, kMappingQuarantinedRetained, 0, 1,
        0, 0},
    {kMappingDMAReachable, kMappingQuarantinedRetained, 0, 1, 0, 0},
    {kMappingQuarantinedRetained, kMappingReleaseAuthorized, 1, 1,
        kReleaseConfirmedColdPowerRemoval,
        kReleaseVerifiedBMEOffRestore},
    {kMappingReleaseAuthorized, kMappingReleased, 1, 0, 0, 0},
};

constexpr UInt32 generationEventMask(SynchronizationGenerationEvent event)
{
    return 1U << event;
}

constexpr SynchronizationGenerationContractRecord
kSynchronizationGenerationContracts[kSynchronizationGenerationContractCount] = {
    {0, kDMAResourceTXRing, kGenerationInitialTXSynchronize, 0, 0, 1, 0},
    {1, kDMAResourceTXRing, kGenerationOwnershipMutation, 1,
        generationEventMask(kGenerationInitialTXSynchronize), 0, 0},
    {2, kDMAResourceTXRing, kGenerationFinalTXSynchronize, 0,
        generationEventMask(kGenerationOwnershipMutation), 1, 0},
    {3, kDMAResourceDevice, kGenerationTXDoorbell, 0,
        generationEventMask(kGenerationFinalTXSynchronize), 0, 1},
    {4, kDMAResourceRXRing, kGenerationRXRebuild, 1, 0, 1, 0},
    {5, kDMAResourceDevice, kGenerationRXDoorbell, 0,
        generationEventMask(kGenerationRXRebuild), 0, 1},
};

enum : UInt8 {
    kLifecycleHostResources = 0,
    kLifecycleTRXSetup,
    kLifecyclePreSystem,
    kLifecyclePowerFSM,
    kLifecyclePostPower,
    kLifecycleFirmwareSetup,
    kLifecycleReservedPageIDDMA,
    kLifecycleFirmwareReady,
    kLifecycleTRXSetupAgain,
    kLifecycleEfuseFeature,
    kLifecyclePowerOff,
    kLifecycleFinalFIFO,
    kLifecycleMACInit,
    kLifecyclePHYRF,
    kLifecycleInterrupts,
};

constexpr LifecyclePlanRecord kLifecyclePlan[kLifecyclePlanCount] = {
    {0, 0, kLifecycleHostResources, 0, 0xffff, 0, 523},
    {1, 1, kLifecycleTRXSetup, 1, 0, 1, 21},
    {2, 1, kLifecyclePreSystem, 1, 1, 2, 8},
    {3, 1, kLifecyclePowerFSM, 1, 2, 3, 18},
    {4, 1, kLifecyclePostPower, 2, 3, 4, 5},
    {5, 1, kLifecycleFirmwareSetup, 2, 4, 5, 17},
    {6, 1, kLifecycleReservedPageIDDMA, 3, 5, 6, 35},
    {7, 1, kLifecycleFirmwareReady, 3, 6, 7, 1},
    {8, 1, kLifecycleTRXSetupAgain, 4, 7, 8, 21},
    {9, 1, kLifecycleEfuseFeature, 4, 8, 9, 512},
    {10, 1, kLifecyclePowerOff, 4, 9, 10, 12},
    {11, 2, kLifecycleTRXSetup, 1, 10, 11, 21},
    {12, 2, kLifecyclePreSystem, 1, 11, 12, 8},
    {13, 2, kLifecyclePowerFSM, 1, 12, 13, 18},
    {14, 2, kLifecyclePostPower, 2, 13, 14, 5},
    {15, 2, kLifecycleFirmwareSetup, 2, 14, 15, 17},
    {16, 2, kLifecycleReservedPageIDDMA, 3, 15, 16, 35},
    {17, 2, kLifecycleFirmwareReady, 3, 16, 17, 1},
    {18, 2, kLifecycleTRXSetupAgain, 4, 17, 18, 21},
    {19, 2, kLifecycleFinalFIFO, 4, 18, 19, 1},
    {20, 2, kLifecycleMACInit, 4, 19, 20, 1},
    {21, 2, kLifecyclePHYRF, 4, 20, 21, 1},
    {22, 2, kLifecycleInterrupts, 5, 21, 22, 1},
};

constexpr RegisterPlanRecord kPostPowerPlan[kPostPowerPlanCount] = {
    {0, 4, 1, 0, 0x1080, 0, 0x00010100U, 0x00010100U},
    {1, 1, 1, 0, 0x0003, 0, 0x000000d8U, 0x000000d8U},
    {2, 1, 3, 0, 0x1103, 0, 0x0000000fU, 0x0000000cU},
    {3, 4, 2, 1, 0x0080, 0, 0x00100000U, 0x00000000U},
    {4, 4, 2, 1, 0x0040, 0, 0x00080000U, 0x00000000U},
};

constexpr RegisterPlanRecord
kFirmwareSetupPlan[kFirmwareSetupPlanCount] = {
    {0, 1, 2, 0, 0x0003, 0, 0x00000004U, 0},
    {0, 1, 2, 0, 0x001d, 0, 0x00000001U, 0},
    {1, 1, 3, 2, 0x010d, 1, 0x000000ffU, 0x000000c0U},
    {1, 1, 3, 2, 0x0100, 1, 0x000000ffU, 0x00000005U},
    {1, 4, 3, 4, 0x1330, 1, 0xffffffffU, 0x80000000U},
    {1, 2, 3, 2, 0x0230, 1, 0x0000ffffU, 0x00000200U},
    {1, 4, 1, 6, 0x022c, 1, 0x80000000U, 0x80000000U},
    {1, 1, 4, 2, 0x0550, 1, 0x00000018U, 0x00000010U},
    {2, 1, 2, 0, 0x1082, 0, 0x00000001U, 0},
    {2, 1, 2, 0, 0x0009, 0, 0x00000040U, 0},
    {2, 1, 1, 0, 0x1082, 0, 0x00000001U, 1},
    {2, 1, 1, 0, 0x0009, 0, 0x00000040U, 0x40},
    {3, 2, 5, 0, 0x0080, 0, 0x0000c7ffU, 0x00000001U},
    {4, 1, 1, 8, 0x001d, 0, 0x00000001U, 1},
    {4, 1, 1, 8, 0x0003, 0, 0x00000004U, 4},
    {5, 1, 2, 16, 0x0080, 0, 0x00000001U, 0},
    {5, 1, 1, 16, 0x0003, 0, 0x00000004U, 4},
};

constexpr RegisterPlanRecord
kReservedPagePlan[kReservedPagePlanCount] = {
    {0, 1, kPlanRead, kPlanFlagBackupBefore, 0x0550, 2,
        0x000000ffU, 0},
    {1, 2, kPlanWrite, 0, 0x0204, 0, 0x0000ffffU, 0x00008000U},
    {2, 1, kPlanMaskSet, kPlanFlagBackupBefore, 0x0101, 2,
        0x00000001U, 0x00000001U},
    {3, 1, kPlanMaskReplace, 0, 0x0550, 0,
        0x00000018U, 0x00000010U},
    {4, 1, kPlanMaskClear, kPlanFlagBackupBefore, 0x0422, 2,
        0x00000040U, 0},
    {5, 4, kPlanHostDMASubmit, 0, 0x0000, 0, 0xffffffffU, 0},
    {6, 1, kPlanMaskSet, 0, 0x0383, 0, 0x00000010U, 0x00000010U},
    {7, 4, kPlanPoll, 0, 0x0204, 0, 0x00008000U, 0x00008000U},
    {8, 2, kPlanWrite, kPlanFlagSyntheticRestore, 0x0204, 0,
        0x0000ffffU,
        kReservedBoundary | 0x8000U},
    {9, 1, kPlanRestoreSaved, 0, 0x0550, 2, 0x000000ffU, 0},
    {10, 1, kPlanRestoreSaved, 0, 0x0422, 2, 0x000000ffU, 0},
    {11, 1, kPlanRestoreSaved, 0, 0x0101, 2, 0x000000ffU, 0},
    {12, 4, kPlanPollOwnership, 0, 0x1208, 0, 0x80000000U, 0},
    {13, 4, kPlanChecksumStatus, 0, 0x1208, 0, 0x08000000U, 0},
};

constexpr RegisterPlanRecord kFinalFIFOPlan[kFinalFIFOPlanCount] = {
    {0, 2, 3, 0, 0x010c, 0, 0x0000ffffU, 0x0000c5a0U},
    {1, 1, 3, 0, 0x0100, 0, 0x000000ffU, 0},
    {2, 1, 3, 0, 0x0100, 0, 0x000000ffU, 0x000000ffU},
    {3, 4, 3, 0, 0x1330, 0, 0xffffffffU, 0x80000000U},
    {4, 2, 3, 0, 0x0230, 0, 0x0000ffffU, 16},
    {5, 2, 3, 0, 0x0234, 0, 0x0000ffffU, 16},
    {6, 2, 3, 0, 0x0238, 0, 0x0000ffffU, 16},
    {7, 2, 3, 0, 0x023c, 0, 0x0000ffffU, 14},
    {8, 2, 3, 0, 0x0240, 0, 0x0000ffffU, kPublicQueuePageCount},
    {9, 4, 1, 0, 0x022c, 0, 0x80000000U, 0x80000000U},
    {10, 2, 3, 0, 0x0204, 0, 0x0000ffffU, kReservedBoundary},
    {11, 1, 1, 0, 0x0422, 0, 0x00000010U, 0x00000010U},
    {12, 2, 3, 0, 0x0424, 0, 0x0000ffffU, kReservedBoundary},
    {13, 2, 3, 0, 0x0206, 0, 0x0000ffffU, kReservedBoundary},
    {14, 2, 3, 0, 0x0456, 0, 0x0000ffffU, kReservedBoundary},
    {15, 4, 3, 0, 0x011c, 0, 0xffffffffU, 0x00003effU},
    {16, 1, 1, 0, 0x0208, 0, 0x00000001U, 0x00000001U},
    {17, 4, 7, 0, 0x0208, 0, 0x00000001U, 0},
    {18, 4, 3, 0, 0x0244, 0, 0x0003ffffU, kH2CQueueAddress},
    {19, 4, 3, 0, 0x024c, 0, 0x0003ffffU, kH2CQueueAddress},
    {20, 4, 3, 0, 0x0248, 0, 0x0003ffffU, kH2CQueueEnd},
    {21, 1, 4, 0, 0x0254, 0, 0x00000003U, 0x00000001U},
    {22, 1, 4, 0, 0x0254, 0, 0x00000004U, 0x00000004U},
    {23, 1, 4, 0, 0x020d, 0, 0x00000080U, 0x00000080U},
    {24, 4, 11, 0, 0x10d4, 0, 0x0003ffffU, 0},
    {25, 4, 11, 0, 0x10d0, 0, 0x0003ffffU, 0},
    {26, 4, 13, 0, 0x0000, 0, 0x00000400U, 0x00000400U},
};

constexpr RegisterPlanRecord kInterruptPlan[kInterruptPlanCount] = {
    {0, 4, kPlanWrite, 0, 0x00b0, 0, 0xffffffffU, 0},
    {0, 4, kPlanWrite, 0, 0x00b8, 0, 0xffffffffU, 0},
    {0, 4, kPlanWrite, 0, 0x10b8, 0, 0xffffffffU, 0},
    {1, 4, kPlanWrite, 0, 0x00b0, 0, 0xffffffffU, 0x000044fdU},
    {1, 4, kPlanWrite, 0, 0x00b8, 0, 0xffffffffU, 0x00000200U},
    {1, 4, kPlanWrite, 0, 0x10b8, 0, 0xffffffffU, 0x00010000U},
    {2, 4, kPlanRead, 0, 0x00b4, 0, 0x000044fdU, 0},
    {2, 4, kPlanRead, 0, 0x00bc, 0, 0x00000200U, 0},
    {2, 4, kPlanRead, 0, 0x10bc, 0, 0x00010000U, 0},
    {3, 4, kPlanWriteOneClear, 0, 0x00b4, 0,
        0x000044fdU, 0x000044fdU},
    {3, 4, kPlanWriteOneClear, 0, 0x00bc, 0,
        0x00000200U, 0x00000200U},
    {3, 4, kPlanWriteOneClear, 0, 0x10bc, 0,
        0x00010000U, 0x00010000U},
    {4, 4, kPlanWrite,
        kPlanFlagConditional | kPlanFlagAfterRXHandler, 0x00b0, 0,
        0xffffffffU, 0x000044fcU},
    {5, 4, kPlanWrite,
        kPlanFlagConditional | kPlanFlagAfterNAPICompletion, 0x00b0, 0,
        0xffffffffU, 0x000044fdU},
    {5, 4, kPlanWrite, kPlanFlagAfterNAPICompletion, 0x00b8, 0,
        0xffffffffU, 0x00000200U},
    {5, 4, kPlanWrite, kPlanFlagAfterNAPICompletion, 0x10b8, 0,
        0xffffffffU, 0x00010000U},
};

constexpr FIFOPlanSummary kFIFOPlanSummary = {
    kTXFIFOPageCount, kReservedPageCount, kACQueuePageCount,
    kReservedBoundary, kReservedBoundary, kReservedH2CInfoAddress,
    kReservedH2CStaticAddress, kReservedH2CQueueAddress,
    kReservedFirmwareTXAddress, kPublicQueuePageCount,
    kH2CQueueAddress, kH2CQueueEnd, 0xc5a0U, 0,
};

constexpr EfuseDependencyPlan kEfuseDependencyPlan = {
    512, 512, 96, 416, 1000000, 1,
    kEfuseRequiredDependencies, kEfusePHYDependencies,
};

constexpr FirmwareRegisterBaseline
kFirmwareRegisterBaselineTemplate[kFirmwareRegisterBaselineCount] = {
    {0x0080, 4, 0, 0, 0},
    {0x0100, 4, 0, 0, 0},
    {0x0204, 2, 0, 0, 0},
    {0x0230, 2, 0, 0, 0},
    {0x022c, 4, 0, 0, 0},
    {0x0420, 4, 0, 0, 0},
    {0x0550, 1, 0, 0, 0},
    {0x1082, 1, 0, 0, 0},
    {0x10fc, 4, 0, 0, 0},
    {0x1200, 4, 0, 0, 0},
    {0x1204, 4, 0, 0, 0},
    {0x1208, 4, 0, 0, 0},
};

constexpr TRXResourcePlan kTRXResources[kTRXResourceCount] = {
    {0, 0, kDefaultTXRingEntries, kTXRingDescriptorSize,
     0x0330, 0x038a, 0x03ac, kDefaultTXRingEntries * kTXRingDescriptorSize, 0},
    {0, 1, kBestEffortTXRingEntries, kTXRingDescriptorSize,
     0x0328, 0x0388, 0x03a8, kBestEffortTXRingEntries * kTXRingDescriptorSize, 0},
    {0, 2, kDefaultTXRingEntries, kTXRingDescriptorSize,
     0x0320, 0x0386, 0x03a4, kDefaultTXRingEntries * kTXRingDescriptorSize, 0},
    {0, 3, kDefaultTXRingEntries, kTXRingDescriptorSize,
     0x0318, 0x0384, 0x03a0, kDefaultTXRingEntries * kTXRingDescriptorSize, 0},
    {0, 4, 1, kTXRingDescriptorSize,
     0x0308, 0, 0, kTXRingDescriptorSize, 0},
    {0, 5, kDefaultTXRingEntries, kTXRingDescriptorSize,
     0x0310, 0x0380, 0x03b0, kDefaultTXRingEntries * kTXRingDescriptorSize, 0},
    {0, 6, kDefaultTXRingEntries, kTXRingDescriptorSize,
     0x0340, 0x038c, 0x03b8, kDefaultTXRingEntries * kTXRingDescriptorSize, 0},
    {0, 7, kDefaultTXRingEntries, kTXRingDescriptorSize,
     0x1320, 0x1328, 0x132c, kDefaultTXRingEntries * kTXRingDescriptorSize, 0},
    {1, 0, kRXRingEntries, kRXRingDescriptorSize,
     0x0338, 0x0382, 0x03b4, kRXRingEntries * kRXRingDescriptorSize,
     kRXRingEntries * kRXBufferSize},
};

bool validateTRXResourcePlan(UInt32 &txRingBytes,
                             UInt32 &rxRingBytes,
                             UInt64 &rxPayloadBytes,
                             UInt8 &deviceConfiguredCount)
{
    txRingBytes = 0;
    rxRingBytes = 0;
    rxPayloadBytes = 0;
    deviceConfiguredCount = 0;

    for (size_t index = 0; index < kTRXResourceCount; index++) {
        const TRXResourcePlan &resource = kTRXResources[index];
        if (resource.entries == 0 || resource.entries > 0x0fffU ||
            resource.ringBytes != resource.entries * resource.descriptorSize)
            return false;

        if (resource.type == 0) {
            if (resource.queue > 7 ||
                resource.descriptorSize != kTXRingDescriptorSize ||
                resource.payloadBytes != 0 || resource.baseRegister == 0)
                return false;
            txRingBytes += resource.ringBytes;
        } else {
            if (resource.type != 1 || resource.queue > 1 ||
                resource.descriptorSize != kRXRingDescriptorSize ||
                resource.payloadBytes != resource.entries * kRXBufferSize)
                return false;
            rxRingBytes += resource.ringBytes;
            rxPayloadBytes += resource.payloadBytes;
        }

        if (resource.baseRegister != 0)
            deviceConfiguredCount++;

        for (size_t otherIndex = index + 1;
             otherIndex < kTRXResourceCount; otherIndex++) {
            const TRXResourcePlan &other = kTRXResources[otherIndex];
            const UInt32 registers[] = {
                resource.baseRegister, resource.countRegister,
                resource.indexRegister,
            };
            const UInt32 otherRegisters[] = {
                other.baseRegister, other.countRegister, other.indexRegister,
            };
            for (UInt8 left = 0; left < 3; left++)
                for (UInt8 right = 0; right < 3; right++)
                    if (registers[left] != 0 &&
                        registers[left] == otherRegisters[right])
                        return false;
        }
    }

    return txRingBytes == 16400 && rxRingBytes == 4096 &&
           rxPayloadBytes == 5876736 && deviceConfiguredCount == 9;
}

bool buildTRXDevicePlan(const PreparedDMA *rings,
                        UInt32 ringCount,
                        TRXDevicePlanRecord *plan,
                        size_t capacity,
                        size_t &planCount,
                        UInt8 &baseCount,
                        UInt8 &entryCount)
{
    planCount = 0;
    baseCount = 0;
    entryCount = 0;
    if (!rings || !plan || ringCount != kTRXResourceCount ||
        capacity < kTRXDevicePlanCapacity)
        return false;

    plan[planCount++] = {
        0, 1, 0xff, 0, 0x0303, 0, 0x000000f7U, 0x000000f7U,
    };
    constexpr UInt8 programmingOrder[kTRXResourceCount] = {
        4, 7, 0, 1, 3, 2, 5, 6, 8,
    };
    for (UInt8 orderIndex = 0; orderIndex < kTRXResourceCount; orderIndex++) {
        const UInt8 index = programmingOrder[orderIndex];
        const TRXResourcePlan &resource = kTRXResources[index];
        const PreparedDMA &ring = rings[index];
        const UInt64 addressEnd = static_cast<UInt64>(ring.address) +
            resource.ringBytes;
        if (!ring.prepared || ring.length != resource.ringBytes ||
            ring.address == 0 || addressEnd > 0x100000000ULL ||
            resource.baseRegister > 0xffffU)
            return false;

        if (resource.countRegister != 0) {
            if (resource.countRegister > 0xffffU ||
                resource.entries > 0x0fffU)
                return false;
            plan[planCount++] = {
                1, 2, index, 2,
                static_cast<UInt16>(resource.countRegister), 0,
                0x00000fffU, resource.entries,
            };
            entryCount++;
        }
        plan[planCount++] = {
            2, 4, index, 1,
            static_cast<UInt16>(resource.baseRegister), 0,
            0xffffffffU, ring.address,
        };
        baseCount++;
    }

    plan[planCount++] = {
        3, 4, 0xff, 3, 0x039c, 0, 0xffffffffU, 0xffffffffU,
    };
    plan[planCount++] = {
        4, 4, 7, 4, 0x1330, 0, 0x00010100U, 0x00010100U,
    };
    plan[planCount++] = {
        5, 4, 0xff, 5, 0x0300, 0, 0x00108000U, 0x00108000U,
    };

    return planCount == kTRXDevicePlanCapacity && baseCount == 9 &&
           entryCount == 8;
}

bool buildFirmwareIDDMAPlan(const FirmwareTransferPlan *transferPlan,
                            size_t transferCount,
                            FirmwareIDDMAPlanRecord *iddmaPlan,
                            size_t capacity,
                            size_t &iddmaCount)
{
    iddmaCount = 0;
    if (!transferPlan || !iddmaPlan ||
        transferCount != kFirmwareTransferPlanCapacity ||
        capacity < transferCount)
        return false;

    UInt8 sectionChunks[3] = {};
    for (size_t index = 0; index < transferCount; index++) {
        const FirmwareTransferPlan &transfer = transferPlan[index];
        const bool lastChunk = index + 1 == transferCount ||
            transferPlan[index + 1].section != transfer.section;
        if (transfer.section > 2 || transfer.size == 0 ||
            transfer.size > kIDDMALengthMask ||
            transfer.firstChunk != (sectionChunks[transfer.section] == 0))
            return false;

        const UInt32 stagingPacketLength = kTXPacketDescriptorSize +
            transfer.size;
        const UInt32 stagingPSBLength =
            ((stagingPacketLength - 1) / 128 + 1) | kBeaconQueueOwn;
        if (stagingPacketLength > kFirmwareStagingPacketSize ||
            stagingPSBLength > 0xffffU)
            return false;
        const UInt32 control = kIDDMAOwn | kIDDMAChecksumEnable |
            transfer.size |
            (transfer.firstChunk ? 0 : kIDDMAChecksumContinue);
        iddmaPlan[iddmaCount++] = {
            transfer.section,
            sectionChunks[transfer.section]++,
            transfer.firstChunk,
            static_cast<UInt8>(lastChunk),
            transfer.firmwareOffset,
            kIDDMAFirmwareSource,
            transfer.destination,
            transfer.size,
            control,
            static_cast<UInt16>(stagingPacketLength),
            static_cast<UInt16>(stagingPSBLength),
            1,
            1,
            transfer.firstChunk,
            static_cast<UInt8>(lastChunk),
        };
    }

    return iddmaCount == transferCount && sectionChunks[0] == 8 &&
           sectionChunks[1] == 16 && sectionChunks[2] == 11;
}

bool validateRegisterPlanStructure(const RegisterPlanRecord *plan,
                                   size_t count);

bool validateLifecyclePlans()
{
    for (size_t index = 0; index < kLifecyclePlanCount; index++) {
        const LifecyclePlanRecord &record = kLifecyclePlan[index];
        if (record.sequence != index || record.cycle > 2 ||
            record.operation > kLifecycleInterrupts || record.count == 0 ||
            (index == 0 ? record.prerequisite != 0xffffU :
                          record.prerequisite != index - 1) ||
            record.failureBoundary != index)
            return false;
    }
    return true;
}

bool validatePostPowerPlan()
{
    if (!validateRegisterPlanStructure(kPostPowerPlan, kPostPowerPlanCount))
        return false;
    for (size_t index = 0; index < kPostPowerPlanCount; index++) {
        const RegisterPlanRecord &record = kPostPowerPlan[index];
        if (record.phase != index ||
            (record.width != 1 && record.width != 4))
            return false;
    }
    return kPostPowerPlan[0].offset == 0x1080 &&
           kPostPowerPlan[0].value == 0x00010100U &&
           kPostPowerPlan[1].offset == 0x0003 &&
           kPostPowerPlan[1].value == 0x000000d8U &&
           kPostPowerPlan[2].offset == 0x1103 &&
           kPostPowerPlan[2].value == 0x0000000cU;
}

bool validateFirmwareSetupPlan()
{
    if (!validateRegisterPlanStructure(
            kFirmwareSetupPlan, kFirmwareSetupPlanCount))
        return false;
    UInt8 restoreRecordCount = 0;
    UInt8 failureRecordCount = 0;
    for (size_t index = 0; index < kFirmwareSetupPlanCount; index++) {
        const RegisterPlanRecord &record = kFirmwareSetupPlan[index];
        if ((record.width != 1 && record.width != 2 && record.width != 4) ||
            record.mask == 0)
            return false;
        if (record.restoreGroup == 1)
            restoreRecordCount++;
        if ((record.flags & kPlanFlagFailureCleanup) != 0)
            failureRecordCount++;
    }

    return restoreRecordCount == 6 && failureRecordCount == 2 &&
           kFirmwareSetupPlan[2].value == 0x000000c0U &&
           kFirmwareSetupPlan[3].value == 0x00000005U &&
           kFirmwareSetupPlan[12].value == 0x00000001U;
}

bool decodeSyntheticEfuse(const UInt8 *physical,
                          UInt32 physicalSize,
                          UInt32 protectSize,
                          UInt8 *logical,
                          UInt32 logicalSize)
{
    if (!physical || !logical || physicalSize <= protectSize)
        return false;

    UInt32 physicalIndex = 0;
    const UInt32 readableSize = physicalSize - protectSize;
    while (physicalIndex < readableSize) {
        const UInt8 header1 = physical[physicalIndex];
        if ((header1 & 0x1fU) == 0x0fU &&
            physicalIndex + 1 >= readableSize)
            return false;
        const UInt8 header2 = physicalIndex + 1 < readableSize ?
            physical[physicalIndex + 1] : 0xffU;
        if (header1 == 0xffU ||
            ((header1 & 0x1fU) == 0x0fU && header2 == 0xffU))
            break;

        UInt32 blockIndex = 0;
        UInt8 wordEnable = 0;
        if ((header1 & 0x1fU) == 0x0fU) {
            blockIndex = ((header2 & 0xf0U) >> 1) |
                ((header1 >> 5) & 0x07U);
            wordEnable = header2 & 0x0fU;
            physicalIndex += 2;
        } else {
            blockIndex = (header1 & 0xf0U) >> 4;
            wordEnable = header1 & 0x0fU;
            physicalIndex++;
        }

        for (UInt8 word = 0; word < 4; word++) {
            if ((wordEnable & (1U << word)) != 0)
                continue;
            const UInt32 logicalIndex = (blockIndex << 3) +
                (static_cast<UInt32>(word) << 1);
            if (physicalIndex + 1 >= readableSize ||
                logicalIndex + 1 >= logicalSize)
                return false;
            logical[logicalIndex] = physical[physicalIndex++];
            logical[logicalIndex + 1] = physical[physicalIndex++];
        }
    }
    return true;
}

EfuseParserValidationResult validateEfuseParserModel()
{
    EfuseParserValidationResult result = {};
    UInt8 physical[32];
    UInt8 logical[64];
    memset(physical, 0xff, sizeof(physical));
    memset(logical, 0xff, sizeof(logical));

    physical[0] = 0x2e;
    physical[1] = 0x12;
    physical[2] = 0x34;
    result.oneByteHeader = decodeSyntheticEfuse(
        physical, sizeof(physical), 8, logical, sizeof(logical));
    result.oneByteHeader = result.oneByteHeader &&
        logical[16] == 0x12 && logical[17] == 0x34;

    memset(physical, 0xff, sizeof(physical));
    memset(logical, 0xff, sizeof(logical));
    physical[0] = 0x6f;
    physical[1] = 0x0e;
    physical[2] = 0x56;
    physical[3] = 0x78;
    result.twoByteHeader = decodeSyntheticEfuse(
        physical, sizeof(physical), 8, logical, sizeof(logical));
    result.twoByteHeader = result.twoByteHeader &&
        logical[24] == 0x56 && logical[25] == 0x78;

    UInt8 invalidPhysical[4] = {0xee, 0x11, 0x22, 0xff};
    UInt8 smallLogical[4];
    memset(smallLogical, 0xff, sizeof(smallLogical));
    result.logicalBoundsRejection = !decodeSyntheticEfuse(
        invalidPhysical, sizeof(invalidPhysical), 0,
        smallLogical, sizeof(smallLogical));

    UInt8 truncatedPhysical[2] = {0x0e, 0x11};
    UInt8 truncationLogical[16];
    memset(truncationLogical, 0xff, sizeof(truncationLogical));
    result.physicalTruncationRejection = !decodeSyntheticEfuse(
        truncatedPhysical, sizeof(truncatedPhysical), 0,
        truncationLogical, sizeof(truncationLogical));

    UInt8 extendedTruncated[1] = {0x6f};
    memset(truncationLogical, 0xff, sizeof(truncationLogical));
    result.extendedHeaderTruncationRejection = !decodeSyntheticEfuse(
        extendedTruncated, sizeof(extendedTruncated), 0,
        truncationLogical, sizeof(truncationLogical));
    return result;
}

bool validateRegisterPlanStructure(const RegisterPlanRecord *plan,
                                   size_t count)
{
    for (size_t index = 0; index < count; index++) {
        const RegisterPlanRecord &record = plan[index];
        if ((record.width != 1 && record.width != 2 &&
            record.width != 4) || record.mask == 0 ||
            record.operation < kPlanMaskSet ||
            record.operation > kPlanHostCompare ||
            (record.flags & ~kRegisterPlanKnownFlags) != 0 ||
            ((record.flags & kPlanFlagAfterRXHandler) != 0 &&
             (record.flags & kPlanFlagAfterNAPICompletion) != 0) ||
            (record.operation == kPlanRestoreSaved &&
             record.restoreGroup == 0) ||
            ((record.flags & kPlanFlagSyntheticRestore) != 0 &&
             record.operation != kPlanWrite))
            return false;
    }
    return true;
}

bool validateReservedPagePlan()
{
    if (!validateRegisterPlanStructure(
            kReservedPagePlan, kReservedPagePlanCount))
        return false;

    UInt8 backupCount = 0;
    UInt8 restoreCount = 0;
    for (size_t index = 0; index < kReservedPagePlanCount; index++) {
        const RegisterPlanRecord &record = kReservedPagePlan[index];
        if ((record.flags & kPlanFlagBackupBefore) != 0) {
            backupCount++;
            UInt8 matches = 0;
            for (size_t restoreIndex = 0;
                 restoreIndex < kReservedPagePlanCount; restoreIndex++) {
                const RegisterPlanRecord &restore =
                    kReservedPagePlan[restoreIndex];
                if (restore.operation == kPlanRestoreSaved &&
                    restore.restoreGroup == record.restoreGroup &&
                    restore.offset == record.offset &&
                    restore.width == record.width &&
                    restore.mask == record.mask)
                    matches++;
            }
            if (matches != 1)
                return false;
        }
        if (record.operation == kPlanRestoreSaved) {
            restoreCount++;
            UInt8 matches = 0;
            for (size_t backupIndex = 0;
                 backupIndex < kReservedPagePlanCount; backupIndex++) {
                const RegisterPlanRecord &backup =
                    kReservedPagePlan[backupIndex];
                if ((backup.flags & kPlanFlagBackupBefore) != 0 &&
                    backup.restoreGroup == record.restoreGroup &&
                    backup.offset == record.offset &&
                    backup.width == record.width &&
                    backup.mask == record.mask)
                    matches++;
            }
            if (matches != 1)
                return false;
        }
    }

    return backupCount == 3 && restoreCount == 3 &&
        kReservedPagePlan[0].offset == 0x0550 &&
        kReservedPagePlan[0].operation == kPlanRead &&
        (kReservedPagePlan[0].flags & kPlanFlagBackupBefore) != 0 &&
        kReservedPagePlan[0].restoreGroup == 2 &&
        kReservedPagePlan[2].offset == 0x0101 &&
        (kReservedPagePlan[2].flags & kPlanFlagBackupBefore) != 0 &&
        kReservedPagePlan[2].restoreGroup == 2 &&
        kReservedPagePlan[4].offset == 0x0422 &&
        (kReservedPagePlan[4].flags & kPlanFlagBackupBefore) != 0 &&
        kReservedPagePlan[4].restoreGroup == 2 &&
        kReservedPagePlan[8].operation == kPlanWrite &&
        (kReservedPagePlan[8].flags & kPlanFlagSyntheticRestore) != 0 &&
        kReservedPagePlan[8].value == (kReservedBoundary | 0x8000U) &&
        kReservedPagePlan[9].operation == kPlanRestoreSaved &&
        kReservedPagePlan[9].offset == 0x0550 &&
        kReservedPagePlan[9].restoreGroup == 2 &&
        kReservedPagePlan[10].offset == 0x0422 &&
        kReservedPagePlan[10].operation == kPlanRestoreSaved &&
        kReservedPagePlan[10].restoreGroup == 2 &&
        kReservedPagePlan[11].offset == 0x0101 &&
        kReservedPagePlan[11].operation == kPlanRestoreSaved &&
        kReservedPagePlan[11].restoreGroup == 2;
}

bool validateFinalFIFOPlan()
{
    return validateRegisterPlanStructure(kFinalFIFOPlan, kFinalFIFOPlanCount) &&
        kFIFOPlanSummary.txPages == 65536 / 128 &&
        kFIFOPlanSummary.reservedPages == 8 + 24 + 8 + 8 + 0 + 4 + 0 &&
        kFIFOPlanSummary.acQueuePages ==
            kFIFOPlanSummary.txPages - kFIFOPlanSummary.reservedPages &&
        kFIFOPlanSummary.reservedBoundary ==
            kFIFOPlanSummary.driverAddress &&
        kFIFOPlanSummary.publicQueuePages ==
            kFIFOPlanSummary.acQueuePages - 16 - 16 - 16 - 14 - 1 &&
        kFIFOPlanSummary.h2cByteEnd - kFIFOPlanSummary.h2cByteAddress ==
            8 * 128 && kFIFOPlanSummary.txdmaQueueMap == 0xc5a0U;
}

bool validateInterruptPlan()
{
    return validateRegisterPlanStructure(kInterruptPlan, kInterruptPlanCount) &&
        kInterruptPlan[3].value == 0x44fdU &&
        kInterruptPlan[9].operation == kPlanWriteOneClear &&
        kInterruptPlan[12].value == 0x44fcU &&
        kInterruptPlan[12].phase == 4 &&
        kInterruptPlan[12].flags ==
            (kPlanFlagConditional | kPlanFlagAfterRXHandler) &&
        kInterruptPlan[13].value == 0x44fdU &&
        kInterruptPlan[13].phase == 5 &&
        kInterruptPlan[13].flags ==
            (kPlanFlagConditional | kPlanFlagAfterNAPICompletion) &&
        kInterruptPlan[14].phase == 5 &&
        kInterruptPlan[14].flags == kPlanFlagAfterNAPICompletion &&
        kInterruptPlan[15].phase == 5 &&
        kInterruptPlan[15].flags == kPlanFlagAfterNAPICompletion;
}

bool validateDMAStateMachine()
{
    bool containmentRoute[kDMAStateCount] = {};
    for (size_t index = 0; index < kDMAStateCount; index++) {
        const DMAStateRecord &state = kDMAStates[index];
        if (state.state != index || state.exposure > kDMAExposureOperational ||
            (state.requiredFlags & state.forbiddenFlags) != 0 ||
            (state.exposure >= kDMAExposureBME &&
             (state.releasePolicy & kDMAFlagMappingsReleaseAuthorized) != 0))
            return false;
    }

    for (size_t index = 0; index < kDMATransitionCount; index++) {
        const DMATransitionRecord &transition = kDMATransitions[index];
        if (transition.sequence != index ||
            transition.from >= kDMAStateCount ||
            transition.to >= kDMAStateCount ||
            transition.failureState >= kDMAStateCount ||
            transition.operation > kDMAStartPowerTransition)
            return false;
        const DMAStateRecord &from = kDMAStates[transition.from];
        const DMAStateRecord &to = kDMAStates[transition.to];
        if (to.exposure < from.exposure && !to.terminal)
            return false;
        if (transition.operation == kDMAEnterQuarantine && to.terminal)
            containmentRoute[transition.from] = true;
    }

    for (size_t index = 0; index < kDMAStateCount; index++)
        if (!kDMAStates[index].terminal && !containmentRoute[index])
            return false;

    return kDMATransitions[2].to == kDMAPoweredBMEOff &&
        kDMATransitions[2].failureState == kDMAQuarantinePower &&
        kDMATransitions[3].from == kDMAPoweredBMEOff &&
        kDMATransitions[4].operation == kDMAPublishBeacon;
}

bool validateDMAPublicationPlan()
{
    UInt32 flags = kDMAFlagMappingsPrepared |
        kDMAFlagDescriptorsMaterialized |
        kDMAFlagTXRingMaterialized |
        kDMAFlagRXRingMaterialized |
        kDMAFlagBMEEnabled;
    UInt32 txGeneration = 0;
    UInt32 txSynchronizedGeneration = 0;
    bool rxSynchronized = false;
    for (size_t index = 0; index < kDMAPublicationPlanCount; index++) {
        const DMAPublicationRecord &record = kDMAPublicationPlan[index];
        if (record.sequence != index || record.resource > kDMAResourceDevice ||
            record.operation < kDMASynchronizeOut ||
            record.operation > kDMANoOwnershipContract ||
            record.failureState >= kDMAStateCount)
            return false;
        if (record.operation == kDMASynchronizeOut &&
            record.direction != kIODirectionOut)
            return false;
        if (record.operation == kDMASynchronizeIn &&
            record.direction != kIODirectionIn)
            return false;
        if ((flags & record.prerequisiteFlags) != record.prerequisiteFlags)
            return false;
        if (record.resource == kDMAResourceTXRing &&
            record.operation == kDMASynchronizeOut)
            txSynchronizedGeneration = txGeneration;
        if (record.operation == kDMASetOwnershipLast) {
            txGeneration++;
            if (txSynchronizedGeneration == txGeneration)
                return false;
        }
        if (index == 7 && record.operation == kDMAMMIODoorbell &&
            txSynchronizedGeneration != txGeneration)
            return false;
        if (record.resource == kDMAResourceRXRing &&
            record.operation == kDMASynchronizeOut)
            rxSynchronized = true;
        if (index == 16 && (!rxSynchronized ||
            (record.prerequisiteFlags & kDMAFlagBMEEnabled) == 0))
            return false;
        flags |= record.resultFlags;
    }

    return kDMAPublicationPlan[4].operation == kDMASetOwnershipLast &&
        kDMAPublicationPlan[7].operation == kDMAMMIODoorbell &&
        kDMAPublicationPlan[8].operation == kDMAMMIOReadback &&
        kDMAPublicationPlan[15].operation == kDMANoOwnershipContract &&
        kDMAPublicationPlan[16].operation == kDMAMMIODoorbell &&
        kDMAPublicationPlan[17].operation == kDMAMMIOReadback &&
        kMMIOPublicationContracts[0].offset == 0x0383 &&
        kMMIOPublicationContracts[0].width == 1 &&
        kMMIOPublicationContracts[1].offset == 0x03b4 &&
        kMMIOPublicationContracts[1].width == 2;
}

bool validateRollbackPolicies()
{
    for (size_t index = 0; index < kRollbackPolicyCount; index++) {
        const RollbackPolicyRecord &policy = kRollbackPolicies[index];
        if (policy.exposure != index ||
            policy.failureState >= kDMAStateCount ||
            (policy.exposure >= kDMAExposureBME &&
             policy.releaseVisibleMappingsAllowed) ||
            (policy.exposure >= kDMAExposureBeacon &&
             policy.restoreRegistersAllowed) ||
            ((policy.permittedActions & policy.forbiddenActions) != 0))
            return false;
        const bool actionAllowsClearBME =
            (policy.permittedActions & kRollbackClearBMEContainment) != 0;
        const bool actionAllowsRestore =
            (policy.permittedActions & kRollbackRestoreBMEOffRegisters) != 0;
        const bool actionAllowsRelease =
            (policy.permittedActions & kRollbackReleaseHostOnly) != 0;
        if (policy.clearBusMasterAllowed != actionAllowsClearBME ||
            policy.restoreRegistersAllowed != actionAllowsRestore ||
            policy.releaseVisibleMappingsAllowed != actionAllowsRelease)
            return false;
    }
    return kRollbackPolicies[kDMAExposureHostOnly]
               .releaseVisibleMappingsAllowed == 1 &&
        kRollbackPolicies[kDMAExposureProgrammedBMEOff]
               .restoreRegistersAllowed == 1 &&
        kRollbackPolicies[kDMAExposureIDDMA]
               .failureState == kDMAQuarantineFirmware &&
        (kRollbackPolicies[kDMAExposurePower].forbiddenActions &
         kRollbackRunPowerOff) != 0;
}

bool validateSerializedCommandContracts()
{
    UInt32 seenCommands = 0;
    for (size_t index = 0; index < kSerializedCommandContractCount; index++) {
        const SerializedCommandContractRecord &record =
            kSerializedCommandContracts[index];
        if (record.command != index || record.admissionBefore > kAdmissionClosed ||
            record.admissionAfter > kAdmissionClosed ||
            record.failureDisposition >= kDMAStateCount ||
            (record.command == kSerializedReleaseMappings &&
             record.requiredReleaseEvidence !=
                kReleaseConfirmedColdPowerRemoval) ||
            (seenCommands & (1U << record.command)) != 0 ||
            (record.forbiddenAdmissionMask &
             admissionMask(static_cast<CommandAdmission>(record.admissionBefore))) != 0)
            return false;
        if ((record.flags & kCommandMutatesState) != 0 &&
            (record.flags & kCommandRequiresWriteAheadIntent) == 0)
            return false;
        if ((record.flags & kCommandRequiresWriteAheadIntent) != 0 &&
            (record.requiredJournalPhases & journalPhaseMask(kJournalIntent)) == 0)
            return false;
        seenCommands |= 1U << record.command;
    }

    return seenCommands == (1U << kSerializedCommandContractCount) - 1 &&
        kSerializedCommandContracts[kSerializedBeginStop].admissionAfter ==
            kAdmissionClosing &&
        kSerializedCommandContracts[kSerializedContain].admissionBefore ==
            kAdmissionClosing &&
        kSerializedCommandContracts[kSerializedReleaseMappings].admissionBefore ==
            kAdmissionContained;
}

bool validateExecutionJournalContracts()
{
    UInt32 seenPhases = 0;
    for (size_t index = 0; index < kExecutionJournalContractCount; index++) {
        const ExecutionJournalContractRecord &record =
            kExecutionJournalContracts[index];
        if (record.phase != index || (seenPhases & (1U << record.phase)) != 0 ||
            (record.requiredFields & kJournalIdentityFields) !=
                kJournalIdentityFields)
            return false;
        if (record.phase != kJournalIntent &&
            (record.requiredPriorPhaseMask & journalPhaseMask(kJournalIntent)) == 0 &&
            record.phase != kJournalContained)
            return false;
        if (record.stateMayAdvance && record.phase != kJournalSucceeded)
            return false;
        if (record.terminal && record.permittedSuccessorMask != 0)
            return false;
        seenPhases |= 1U << record.phase;
    }

    return seenPhases == (1U << kExecutionJournalContractCount) - 1 &&
        kExecutionJournalContracts[kJournalIntent].preservesWorstCaseHazards &&
        kExecutionJournalContracts[kJournalFailedEffectUnknown]
            .preservesWorstCaseHazards &&
        kExecutionJournalContracts[kJournalFailedEffectUnknown]
            .permittedSuccessorMask == journalPhaseMask(kJournalContained);
}

bool validateMappingExposureLedger()
{
    UInt32 mappingCount = 0;
    UInt32 seenResources = 0;
    for (size_t index = 0; index < kMappingExposureClassCount; index++) {
        const MappingExposureClassRecord &record = kMappingExposureClasses[index];
        if (record.resource != index || record.mappingCount == 0 ||
            record.initialExposure != kMappingHostOnly ||
            !record.generationRequired ||
            record.deviceReference == 0 ||
            (seenResources & (1U << record.resource)) != 0 ||
            (record.releaseEvidenceMask & kReleaseNeverExposed) == 0 ||
            (record.releaseEvidenceMask & kReleaseConfirmedColdPowerRemoval) == 0)
            return false;
        if (record.resource == kMappingRXPayload &&
            (record.releaseEvidenceMask & kReleaseVerifiedBMEOffRestore) != 0)
            return false;
        mappingCount += record.mappingCount;
        seenResources |= 1U << record.resource;
    }

    bool directReachableRelease = false;
    bool quarantineColdRemovalRoute = false;
    for (size_t index = 0; index < kMappingExposureTransitionCount; index++) {
        const MappingExposureTransitionRecord &transition =
            kMappingExposureTransitions[index];
        if (transition.from > kMappingReleased ||
            transition.to > kMappingReleased || transition.from == transition.to ||
            (transition.requiredEvidence & transition.forbiddenEvidence) != 0)
            return false;
        if (transition.from == kMappingDMAReachable &&
            transition.to == kMappingReleased)
            directReachableRelease = true;
        if (transition.from == kMappingQuarantinedRetained &&
            transition.to == kMappingReleaseAuthorized &&
            transition.requiredEvidence == kReleaseConfirmedColdPowerRemoval &&
            transition.retainsMapping)
            quarantineColdRemovalRoute = true;
        if (transition.to == kMappingReleased &&
            (transition.from != kMappingReleaseAuthorized ||
             transition.retainsMapping))
            return false;
    }

    return mappingCount == kRetainedMappingCount &&
        seenResources == (1U << kMappingExposureClassCount) - 1 &&
        !directReachableRelease && quarantineColdRemovalRoute;
}

bool validateSynchronizationGenerationContracts()
{
    UInt32 seenEvents = 0;
    UInt32 generation = 0;
    UInt32 synchronizedGeneration = 0;
    for (size_t index = 0;
         index < kSynchronizationGenerationContractCount; index++) {
        const SynchronizationGenerationContractRecord &record =
            kSynchronizationGenerationContracts[index];
        if (record.sequence != index ||
            record.event != index || record.resource > kDMAResourceDevice ||
            (record.requiredPriorEventMask & seenEvents) !=
                record.requiredPriorEventMask)
            return false;
        if (record.advancesGeneration)
            generation++;
        if (record.capturesCurrentGeneration)
            synchronizedGeneration = generation;
        if (record.requiresCurrentGenerationMatch &&
            synchronizedGeneration != generation)
            return false;
        seenEvents |= generationEventMask(
            static_cast<SynchronizationGenerationEvent>(record.event));
    }
    return seenEvents ==
        (1U << kSynchronizationGenerationContractCount) - 1;
}

UInt32 transitionWorstCaseHazards(const DMATransitionRecord &transition)
{
    UInt32 hazards = 0;
    if (transition.operation >= kDMAProgramRings)
        hazards |= kHazardAddressProgrammed;
    if (transition.operation == kDMAStartPowerTransition ||
        transition.from == kDMAPoweredBMEOff ||
        kDMAStates[transition.from].exposure >= kDMAExposurePower)
        hazards |= kHazardPowerWritePossible;
    if (transition.operation == kDMAEnableBME ||
        kDMAStates[transition.from].requiredFlags & kDMAFlagBMEEnabled)
        hazards |= kHazardBMEPossible;
    if (transition.operation == kDMAPublishBeacon ||
        kDMAStates[transition.from].requiredFlags & kDMAFlagBeaconKicked)
        hazards |= kHazardBeaconPossible;
    if (transition.operation == kDMAStartIDDMA ||
        transition.operation == kDMACompleteChunk ||
        transition.operation == kDMACompleteFirmwareCopy)
        hazards |= kHazardIDDMAPossible | kHazardFirmwareMemoryDirty;
    if (transition.operation == kDMAReleaseCPU ||
        transition.operation == kDMAObserveFirmwareReady ||
        transition.operation == kDMAFinishOperationalSetup)
        hazards |= kHazardCPUReleased;
    if (transition.operation == kDMAFinishOperationalSetup)
        hazards |= kHazardInterruptsPossible;
    return hazards & kKnownExecutionHazards;
}

UInt32 stateAccumulatedHazards(UInt8 state)
{
    switch (state) {
    case kDMAColdBaseline:
    case kDMAHostPrepared:
    case kDMAQuarantineIdentity:
        return 0;
    case kDMADevicePlanBMEOff:
        return kHazardAddressProgrammed;
    case kDMAPoweredBMEOff:
        return kHazardAddressProgrammed | kHazardPowerWritePossible;
    case kDMABMEArmedPreKick:
        return kHazardAddressProgrammed | kHazardPowerWritePossible |
            kHazardBMEPossible;
    case kDMABeaconInFlight:
        return kHazardAddressProgrammed | kHazardPowerWritePossible |
            kHazardBMEPossible | kHazardBeaconPossible;
    case kDMAIDDMAInFlight:
    case kDMAFirmwarePartial:
    case kDMAFirmwareCopiedCPUStopped:
        return kHazardAddressProgrammed | kHazardPowerWritePossible |
            kHazardBMEPossible | kHazardBeaconPossible |
            kHazardIDDMAPossible | kHazardFirmwareMemoryDirty;
    case kDMACPUReleasedFWWait:
    case kDMAOperationalPending:
        return kHazardAddressProgrammed | kHazardPowerWritePossible |
            kHazardBMEPossible | kHazardBeaconPossible |
            kHazardIDDMAPossible | kHazardFirmwareMemoryDirty |
            kHazardCPUReleased;
    case kDMAOperational:
        return kKnownExecutionHazards;
    case kDMAQuarantineDMA:
        return kHazardAddressProgrammed | kHazardBMEPossible;
    case kDMAQuarantineFirmware:
        return kHazardAddressProgrammed | kHazardPowerWritePossible |
            kHazardBMEPossible | kHazardFirmwareMemoryDirty;
    case kDMAQuarantinePower:
    case kDMAColdRemovalRequired:
        return kHazardAddressProgrammed | kHazardPowerWritePossible |
            kHazardBMEPossible;
    }
    return kKnownExecutionHazards;
}

const MappingExposureTransitionRecord *mappingTransition(
    UInt8 from, UInt8 to, UInt32 evidence, bool deviceQuiescent)
{
    for (size_t index = 0; index < kMappingExposureTransitionCount; index++) {
        const MappingExposureTransitionRecord &transition =
            kMappingExposureTransitions[index];
        if (transition.from == from && transition.to == to &&
            (evidence & transition.requiredEvidence) ==
                transition.requiredEvidence &&
            (evidence & transition.forbiddenEvidence) == 0 &&
            (!transition.requiresDeviceQuiescence || deviceQuiescent))
            return &transition;
    }
    return nullptr;
}

bool transitionAllMappings(SymbolicExecutionState &state, UInt8 to)
{
    for (size_t index = 0; index < kMappingExposureClassCount; index++) {
        const UInt8 from = state.mappingExposure[index];
        if (from == to)
            continue;
        const MappingExposureTransitionRecord *transition =
            mappingTransition(from, to, state.releaseEvidence,
                state.deviceQuiescent);
        if (!transition ||
            (transition->requiredEvidence &
             kMappingExposureClasses[index].releaseEvidenceMask) !=
                transition->requiredEvidence)
            return false;
    }
    for (size_t index = 0; index < kMappingExposureClassCount; index++)
        state.mappingExposure[index] = to;
    return true;
}

bool updateMappingExposureForHazards(SymbolicExecutionState &state)
{
    if ((state.hazards & kHazardAddressProgrammed) != 0) {
        for (size_t index = 0; index < kMappingExposureClassCount; index++) {
            if (state.mappingExposure[index] == kMappingHostOnly &&
                !mappingTransition(kMappingHostOnly,
                    kMappingAddressProgrammedBMEOff, state.releaseEvidence,
                    state.deviceQuiescent))
                return false;
            if (state.mappingExposure[index] == kMappingHostOnly)
                state.mappingExposure[index] = kMappingAddressProgrammedBMEOff;
        }
    }
    if ((state.hazards & kHazardBMEPossible) != 0)
        return transitionAllMappings(state, kMappingDMAReachable);
    return true;
}

void initializeSymbolicState(SymbolicExecutionState &state,
                             UInt8 dmaState, UInt32 hazards)
{
    state = {};
    state.admission = kAdmissionOpen;
    state.dmaState = dmaState;
    state.journalPhase = kJournalSucceeded;
    state.activeCommand = 0xffU;
    state.deviceQuiescent = hazards == 0;
    state.hazards = hazards & kKnownExecutionHazards;
    state.dmaFlags = kDMAStates[dmaState].requiredFlags;
    for (size_t index = 0; index < kMappingExposureClassCount; index++)
        state.mappingExposure[index] = kMappingHostOnly;
    updateMappingExposureForHazards(state);
}

bool admitSymbolicCommand(SymbolicExecutionState &state, UInt8 command)
{
    if (command >= kSerializedCommandContractCount ||
        state.activeCommand != 0xffU ||
        state.journalPhase == kJournalFailedEffectUnknown)
        return false;
    const SerializedCommandContractRecord &contract =
        kSerializedCommandContracts[command];
    if (state.admission != contract.admissionBefore ||
        (contract.forbiddenAdmissionMask &
         admissionMask(static_cast<CommandAdmission>(state.admission))) != 0 ||
        (state.releaseEvidence & contract.requiredReleaseEvidence) !=
            contract.requiredReleaseEvidence)
        return false;
    state.activeCommand = command;
    state.gateTicket++;
    state.transaction++;
    state.intentGateTicket = state.gateTicket;
    state.intentTransaction = state.transaction;
    state.journalSequence++;
    state.journalPhase = kJournalIntent;
    state.hazardsBeforeIntent = state.hazards;
    return true;
}

bool resolveSymbolicCommand(SymbolicExecutionState &state, UInt8 journalPhase,
                            UInt8 successState, UInt32 attemptedHazards,
                            UInt32 resultFlags = 0)
{
    if (state.activeCommand >= kSerializedCommandContractCount ||
        state.journalPhase != kJournalIntent ||
        state.intentGateTicket != state.gateTicket ||
        state.intentTransaction != state.transaction ||
        journalPhase < kJournalSucceeded ||
        journalPhase > kJournalFailedEffectUnknown)
        return false;
    const SerializedCommandContractRecord &contract =
        kSerializedCommandContracts[state.activeCommand];
    const ExecutionJournalContractRecord &journal =
        kExecutionJournalContracts[journalPhase];
    if ((journal.requiredPriorPhaseMask & journalPhaseMask(kJournalIntent)) == 0)
        return false;
    if (journalPhase == kJournalSucceeded) {
        state.hazards |= attemptedHazards;
        state.dmaState = successState;
        state.dmaFlags |= resultFlags;
        state.admission = contract.admissionAfter;
    } else if (journalPhase == kJournalFailedNoEffect) {
        if (state.hazards != state.hazardsBeforeIntent)
            return false;
    } else {
        state.hazards |= attemptedHazards;
    }
    state.hazards &= kKnownExecutionHazards;
    state.journalPhase = journalPhase;
    state.journalSequence++;
    state.activeCommand = 0xffU;
    return updateMappingExposureForHazards(state);
}

bool containSymbolicState(SymbolicExecutionState &state, UInt8 failureState)
{
    if (state.journalPhase == kJournalFailedEffectUnknown) {
        if ((kExecutionJournalContracts[kJournalFailedEffectUnknown]
             .permittedSuccessorMask & journalPhaseMask(kJournalContained)) == 0)
            return false;
        state.journalPhase = kJournalContained;
        state.journalSequence++;
    }
    if (state.admission == kAdmissionOpen) {
        if (!admitSymbolicCommand(state, kSerializedBeginStop) ||
            !resolveSymbolicCommand(state, kJournalSucceeded, state.dmaState, 0))
            return false;
    }
    if (state.admission != kAdmissionClosing ||
        !admitSymbolicCommand(state, kSerializedContain) ||
        !resolveSymbolicCommand(state, kJournalSucceeded,
            failureState, 0))
        return false;
    for (size_t index = 0; index < kMappingExposureClassCount; index++) {
        const UInt8 from = state.mappingExposure[index];
        if (from == kMappingHostOnly || from == kMappingQuarantinedRetained)
            continue;
        if (!mappingTransition(from, kMappingQuarantinedRetained,
            state.releaseEvidence, state.deviceQuiescent))
            return false;
        state.mappingExposure[index] = kMappingQuarantinedRetained;
    }
    return state.admission == kAdmissionContained;
}

bool releaseSymbolicMappings(SymbolicExecutionState &state, UInt32 evidence)
{
    SymbolicExecutionState candidate = state;
    candidate.releaseEvidence = evidence;
    if (!admitSymbolicCommand(candidate, kSerializedReleaseMappings))
        return false;
    if (!transitionAllMappings(candidate, kMappingReleaseAuthorized) ||
        !transitionAllMappings(candidate, kMappingReleased))
        return false;
    if (!resolveSymbolicCommand(candidate, kJournalSucceeded,
        candidate.dmaState, 0))
        return false;
    for (size_t index = 0; index < kMappingExposureClassCount; index++)
        if (candidate.mappingExposure[index] != kMappingReleased)
            return false;
    if (candidate.admission != kAdmissionClosed || !candidate.deviceQuiescent)
        return false;
    state = candidate;
    return true;
}

bool observeSymbolicColdRemoval(SymbolicExecutionState &state)
{
    if (state.admission != kAdmissionContained ||
        state.dmaState != kDMAColdRemovalRequired)
        return false;
    for (size_t index = 0; index < kMappingExposureClassCount; index++)
        if (state.mappingExposure[index] != kMappingQuarantinedRetained &&
            state.mappingExposure[index] != kMappingHostOnly)
            return false;
    state.deviceQuiescent = true;
    state.releaseEvidence |= kReleaseConfirmedColdPowerRemoval;
    state.hazards = 0;
    state.dmaFlags &= ~kDMAFlagBMEEnabled;
    return true;
}

bool replayGenerationContracts(bool injectStaleTX, bool injectStaleRX)
{
    SymbolicExecutionState state = {};
    for (size_t index = 0;
         index < kSynchronizationGenerationContractCount; index++) {
        const SynchronizationGenerationContractRecord &record =
            kSynchronizationGenerationContracts[index];
        if ((record.requiredPriorEventMask & state.generationEvents) !=
            record.requiredPriorEventMask)
            return false;
        UInt32 *generation = record.event >= kGenerationRXRebuild ?
            &state.rxGeneration : &state.txGeneration;
        UInt32 *synchronizedGeneration = record.event >= kGenerationRXRebuild ?
            &state.rxSynchronizedGeneration : &state.txSynchronizedGeneration;
        if (record.advancesGeneration)
            (*generation)++;
        if (record.capturesCurrentGeneration)
            *synchronizedGeneration = *generation;
        if (record.event == kGenerationTXDoorbell && injectStaleTX)
            (*generation)++;
        if (record.event == kGenerationRXDoorbell && injectStaleRX)
            (*generation)++;
        if (record.requiresCurrentGenerationMatch &&
            *synchronizedGeneration != *generation)
            return false;
        state.generationEvents |= generationEventMask(
            static_cast<SynchronizationGenerationEvent>(record.event));
    }
    return true;
}

bool applyPublicationGeneration(SymbolicExecutionState &state, size_t sequence,
                                bool injectStale)
{
    SynchronizationGenerationEvent event;
    if (sequence == 2)
        event = kGenerationInitialTXSynchronize;
    else if (sequence == 4)
        event = kGenerationOwnershipMutation;
    else if (sequence == 5)
        event = kGenerationFinalTXSynchronize;
    else if (sequence == 7)
        event = kGenerationTXDoorbell;
    else if (sequence == 13)
        event = kGenerationRXRebuild;
    else if (sequence == 16)
        event = kGenerationRXDoorbell;
    else
        return true;

    const SynchronizationGenerationContractRecord &record =
        kSynchronizationGenerationContracts[event];
    if ((record.requiredPriorEventMask & state.generationEvents) !=
        record.requiredPriorEventMask)
        return false;
    UInt32 *generation = event >= kGenerationRXRebuild ?
        &state.rxGeneration : &state.txGeneration;
    UInt32 *synchronizedGeneration = event >= kGenerationRXRebuild ?
        &state.rxSynchronizedGeneration : &state.txSynchronizedGeneration;
    if (record.advancesGeneration)
        (*generation)++;
    if (record.capturesCurrentGeneration)
        *synchronizedGeneration = *generation;
    if (injectStale && record.requiresCurrentGenerationMatch)
        (*generation)++;
    if (record.requiresCurrentGenerationMatch &&
        *synchronizedGeneration != *generation)
        return false;
    state.generationEvents |= generationEventMask(event);
    return true;
}

bool replayFailureBoundary(UInt8 command, UInt8 initialState,
                           UInt8 successState, UInt32 initialHazards,
                           UInt32 attemptedHazards, UInt32 initialFlags,
                           UInt32 prerequisiteFlags, UInt32 resultFlags,
                           UInt8 failureState, UInt8 outcome,
                           bool &contained, bool &safeCleanup)
{
    SymbolicExecutionState state = {};
    initializeSymbolicState(state, initialState, initialHazards);
    state.dmaFlags = initialFlags;
    contained = false;
    safeCleanup = false;

    if (outcome == 0) {
        state.activeCommand = command;
        const bool rejected = !admitSymbolicCommand(state, command);
        state.activeCommand = 0xffU;
        safeCleanup = rejected;
        return rejected;
    }
    if (!admitSymbolicCommand(state, command))
        return false;
    if (outcome == 1) {
        if ((state.dmaFlags & prerequisiteFlags) != prerequisiteFlags ||
            !resolveSymbolicCommand(state, kJournalSucceeded,
                successState, attemptedHazards, resultFlags) ||
            (state.dmaFlags & resultFlags) != resultFlags)
            return false;
        safeCleanup = true;
        return true;
    }
    if (outcome == 2) {
        if (!resolveSymbolicCommand(state, kJournalFailedNoEffect,
            initialState, attemptedHazards))
            return false;
        if (state.hazards == 0) {
            state.releaseEvidence = kReleaseNeverExposed;
            safeCleanup = transitionAllMappings(state, kMappingReleaseAuthorized) &&
                transitionAllMappings(state, kMappingReleased);
            return safeCleanup;
        }
        contained = containSymbolicState(state, failureState);
        return contained;
    }
    if (outcome == 3) {
        if (!resolveSymbolicCommand(state, kJournalFailedEffectUnknown,
            initialState, attemptedHazards))
            return false;
        contained = containSymbolicState(state, failureState);
        return contained &&
            (state.hazards & attemptedHazards) == attemptedHazards;
    }

    // A stop arriving after intent makes the pending effect ambiguous first.
    if (!resolveSymbolicCommand(state, kJournalFailedEffectUnknown,
        initialState, attemptedHazards))
        return false;
    contained = containSymbolicState(state, failureState);
    return contained && state.admission == kAdmissionContained;
}

bool validateSymbolicInterpreter(SymbolicInterpreterSummary &summary)
{
    summary = {};
    for (size_t index = 0; index < kLifecyclePlanCount; index++) {
        UInt32 initialHazards = index == 0 ? 0 : kHazardAddressProgrammed;
        UInt32 attemptedHazards = initialHazards;
        if (index >= 3)
            attemptedHazards |= kHazardPowerWritePossible;
        if (index >= 6)
            attemptedHazards |= kHazardBMEPossible | kHazardBeaconPossible |
                kHazardIDDMAPossible | kHazardFirmwareMemoryDirty;
        if (index >= 7)
            attemptedHazards |= kHazardCPUReleased;
        if (index == kLifecyclePlanCount - 1)
            attemptedHazards |= kHazardInterruptsPossible;
        const LifecyclePlanRecord &record = kLifecyclePlan[index];
        if (record.sequence != index || record.operation > kLifecycleInterrupts ||
            record.failureBoundary != index)
            return false;
        for (UInt8 outcome = 0; outcome < 5; outcome++) {
            bool contained = false;
            bool safeCleanup = false;
            if (!replayFailureBoundary(kSerializedAdvance, kDMAHostPrepared,
                kDMAHostPrepared, initialHazards, attemptedHazards, 0, 0,
                0,
                static_cast<UInt8>(attemptedHazards == 0 ?
                    kDMAQuarantineIdentity : kDMAQuarantinePower),
                outcome, contained, safeCleanup))
                return false;
            summary.lifecycleTraceCount++;
            summary.successfulTraceCount += outcome == 1;
            summary.containmentTraceCount += contained;
        }
    }
    for (size_t index = 0; index < kDMATransitionCount; index++) {
        const DMATransitionRecord &transition = kDMATransitions[index];
        const UInt32 initialHazards = stateAccumulatedHazards(transition.from);
        const UInt32 attemptedHazards = initialHazards |
            transitionWorstCaseHazards(transition);
        const UInt32 boundaryFixtureFlags =
            kDMAStates[transition.from].requiredFlags |
            transition.prerequisiteFlags;
        for (UInt8 outcome = 0; outcome < 5; outcome++) {
            bool contained = false;
            bool safeCleanup = false;
            if (!replayFailureBoundary(kSerializedAdvance, transition.from,
                transition.to, initialHazards, attemptedHazards,
                boundaryFixtureFlags,
                transition.prerequisiteFlags, transition.resultFlags,
                transition.failureState, outcome, contained, safeCleanup))
                return false;
            summary.transitionTraceCount++;
            summary.successfulTraceCount += outcome == 1;
            summary.containmentTraceCount += contained;
        }
    }
    UInt32 publicationPrefixFlags = kDMAFlagMappingsPrepared |
        kDMAFlagDescriptorsMaterialized | kDMAFlagTXRingMaterialized |
        kDMAFlagRXRingMaterialized | kDMAFlagBMEEnabled;
    SymbolicExecutionState publicationGeneration = {};
    for (size_t index = 0; index < kDMAPublicationPlanCount; index++) {
        const DMAPublicationRecord &record = kDMAPublicationPlan[index];
        if (record.sequence != index ||
            (publicationPrefixFlags & record.prerequisiteFlags) !=
                record.prerequisiteFlags)
            return false;
        UInt32 attemptedHazards = kHazardPowerWritePossible |
            kHazardAddressProgrammed | kHazardBMEPossible;
        if (index >= 4)
            attemptedHazards |= kHazardBeaconPossible;
        for (UInt8 outcome = 0; outcome < 5; outcome++) {
            bool contained = false;
            bool safeCleanup = false;
            if (!replayFailureBoundary(kSerializedPublish,
                kDMABMEArmedPreKick, kDMABeaconInFlight,
                stateAccumulatedHazards(kDMABMEArmedPreKick),
                attemptedHazards,
                kDMAStates[kDMABMEArmedPreKick].requiredFlags |
                    publicationPrefixFlags,
                record.prerequisiteFlags,
                record.resultFlags, static_cast<UInt8>(record.failureState), outcome,
                contained, safeCleanup))
                return false;
            if (outcome == 1) {
                SymbolicExecutionState generationCandidate =
                    publicationGeneration;
                if (!applyPublicationGeneration(generationCandidate, index, false))
                    return false;
            }
            summary.publicationTraceCount++;
            summary.successfulTraceCount += outcome == 1;
            summary.containmentTraceCount += contained;
        }
        publicationPrefixFlags |= record.resultFlags;
        if (!applyPublicationGeneration(publicationGeneration, index, false))
            return false;
    }
    summary.totalTraceCount = summary.lifecycleTraceCount +
        summary.transitionTraceCount + summary.publicationTraceCount;

    SymbolicExecutionState mutation = {};
    initializeSymbolicState(mutation, kDMAHostPrepared, 0);
    if (admitSymbolicCommand(mutation, kSerializedAdvance) &&
        !admitSymbolicCommand(mutation, kSerializedPublish))
        summary.rejectedMutationCount++;
    initializeSymbolicState(mutation, kDMAHostPrepared, 0);
    if (admitSymbolicCommand(mutation, kSerializedBeginStop) &&
        resolveSymbolicCommand(mutation, kJournalSucceeded,
            mutation.dmaState, 0) &&
        !admitSymbolicCommand(mutation, kSerializedAdvance))
        summary.rejectedMutationCount++;
    initializeSymbolicState(mutation, kDMAQuarantinePower,
        kHazardAddressProgrammed | kHazardPowerWritePossible |
            kHazardBMEPossible);
    mutation.admission = kAdmissionContained;
    transitionAllMappings(mutation, kMappingQuarantinedRetained);
    const SymbolicExecutionState releaseBefore = mutation;
    if (!releaseSymbolicMappings(mutation, 0) &&
        memcmp(&mutation, &releaseBefore, sizeof(mutation)) == 0)
        summary.rejectedMutationCount++;
    initializeSymbolicState(mutation, kDMAHostPrepared, 0);
    if (admitSymbolicCommand(mutation, kSerializedAdvance) &&
        resolveSymbolicCommand(mutation, kJournalFailedEffectUnknown,
            mutation.dmaState, kHazardPowerWritePossible) &&
        !admitSymbolicCommand(mutation, kSerializedAdvance))
        summary.rejectedMutationCount++;
    initializeSymbolicState(mutation, kDMABMEArmedPreKick,
        stateAccumulatedHazards(kDMABMEArmedPreKick));
    mutation.dmaFlags &= ~kDMAFlagBMEEnabled;
    bool ignoredContained = false;
    bool ignoredCleanup = false;
    if (!replayFailureBoundary(kSerializedPublish, kDMABMEArmedPreKick,
        kDMABeaconInFlight,
        stateAccumulatedHazards(kDMABMEArmedPreKick),
        stateAccumulatedHazards(kDMABMEArmedPreKick),
        0, kDMAFlagBMEEnabled, 0, kDMAQuarantinePower, 1,
        ignoredContained, ignoredCleanup))
        summary.rejectedMutationCount++;
    if (!replayGenerationContracts(true, false))
        summary.rejectedMutationCount++;
    if (!replayGenerationContracts(false, true))
        summary.rejectedMutationCount++;
    summary.expectedRejectedMutationCount = 7;

    SymbolicExecutionState releasable = {};
    initializeSymbolicState(releasable, kDMAQuarantinePower,
        kHazardAddressProgrammed | kHazardPowerWritePossible |
            kHazardBMEPossible);
    releasable.admission = kAdmissionContained;
    if (!transitionAllMappings(releasable, kMappingQuarantinedRetained) ||
        !observeSymbolicColdRemoval(releasable) ||
        !releaseSymbolicMappings(releasable,
            releasable.releaseEvidence) ||
        !replayGenerationContracts(false, false))
        return false;

    return summary.totalTraceCount ==
            (kLifecyclePlanCount + kDMATransitionCount +
             kDMAPublicationPlanCount) * 5 &&
        summary.rejectedMutationCount ==
            summary.expectedRejectedMutationCount;
}

bool validateFailureInjectionModel(FailureInjectionSummary &summary)
{
    summary = {};
    summary.lifecycleBoundaries = kLifecyclePlanCount;
    summary.transitionBoundaries = kDMATransitionCount;
    summary.publicationBoundaries = kDMAPublicationPlanCount;
    summary.totalBoundaries = summary.lifecycleBoundaries +
        summary.transitionBoundaries + summary.publicationBoundaries;

    // Four synthetic outcomes per boundary: pre-intent rejection, known no-effect
    // failure, unknown-effect failure, and stop after write-ahead intent.
    summary.scenarioCount = static_cast<UInt32>(summary.totalBoundaries) * 4;
    for (size_t index = 0; index < kLifecyclePlanCount; index++) {
        UInt32 hazards = 0;
        const UInt8 operation = kLifecyclePlan[index].operation;
        if (index != 0)
            hazards |= kHazardAddressProgrammed;
        if (operation == kLifecyclePowerFSM || index > 3)
            hazards |= kHazardPowerWritePossible;
        if (operation == kLifecycleReservedPageIDDMA ||
            operation == kLifecycleFirmwareReady || index > 7)
            hazards |= kHazardBMEPossible | kHazardBeaconPossible |
                kHazardIDDMAPossible | kHazardFirmwareMemoryDirty;
        if (operation == kLifecycleFirmwareReady || index > 7)
            hazards |= kHazardCPUReleased;
        if (operation == kLifecycleInterrupts)
            hazards |= kHazardInterruptsPossible;
        summary.rejectedUnsafeScenarioCount++;
        summary.hostCleanupScenarioCount++;
        if (hazards & (kKnownExecutionHazards & ~kHazardAddressProgrammed))
            summary.containmentScenarioCount += 2;
        else
            summary.hostCleanupScenarioCount += 2;
    }
    for (size_t index = 0; index < kDMATransitionCount; index++) {
        const UInt32 hazards = stateAccumulatedHazards(
            kDMATransitions[index].from) |
            transitionWorstCaseHazards(kDMATransitions[index]);
        if ((hazards & stateAccumulatedHazards(kDMATransitions[index].from)) !=
            stateAccumulatedHazards(kDMATransitions[index].from))
            return false;
        summary.rejectedUnsafeScenarioCount++;
        summary.hostCleanupScenarioCount++;
        if (hazards & (kKnownExecutionHazards & ~kHazardAddressProgrammed))
            summary.containmentScenarioCount += 2;
        else
            summary.hostCleanupScenarioCount += 2;
    }
    for (size_t index = 0; index < kDMAPublicationPlanCount; index++) {
        const DMAPublicationRecord &record = kDMAPublicationPlan[index];
        UInt32 hazards = kHazardPowerWritePossible | kHazardAddressProgrammed |
            kHazardBMEPossible;
        if (record.operation == kDMASetOwnershipLast ||
            record.operation == kDMAMMIODoorbell ||
            record.operation == kDMAMMIOReadback)
            hazards |= kHazardBeaconPossible;
        if ((hazards & kKnownExecutionHazards) == 0)
            return false;
        summary.rejectedUnsafeScenarioCount++;
        summary.hostCleanupScenarioCount++;
        summary.containmentScenarioCount += 2;
    }

    return summary.totalBoundaries ==
            kLifecyclePlanCount + kDMATransitionCount +
                kDMAPublicationPlanCount &&
        summary.scenarioCount == static_cast<UInt32>(summary.totalBoundaries) * 4 &&
        summary.rejectedUnsafeScenarioCount == summary.totalBoundaries &&
        summary.containmentScenarioCount + summary.hostCleanupScenarioCount ==
            static_cast<UInt32>(summary.totalBoundaries) * 3;
}

bool prepareSingleSegmentDMA(UInt32 length,
                             IODirection direction,
                             bool physicallyContiguous,
                             PreparedDMA &dma,
                             DMAProbeStage &stage,
                             IOReturn &resultStatus)
{
    dma = {};
    stage = DMAProbeStage::BufferAllocation;
    resultStatus = kIOReturnNoMemory;
    const mach_vm_address_t physicalMask =
        0x00000000ffffffffULL &
        ~(static_cast<mach_vm_address_t>(kDMAAlignment) - 1);
    dma.buffer = IOBufferMemoryDescriptor::inTaskWithPhysicalMask(
        kernel_task, direction |
            (physicallyContiguous ? kIOMemoryPhysicallyContiguous : 0),
        length, physicalMask);
    if (!dma.buffer)
        return false;

    void *bytes = dma.buffer->getBytesNoCopy();
    if (!bytes) {
        stage = DMAProbeStage::VirtualAddress;
        resultStatus = kIOReturnVMError;
        return false;
    }
    bzero(bytes, length);

    stage = DMAProbeStage::CommandAllocation;
    resultStatus = IODMACommand::weakWithSpecification(
        &dma.command, kIODMACommandOutputHost32, 32, length,
        IODMACommand::kMapped, length, kDMAAlignment);
    if (resultStatus != kIOReturnSuccess || !dma.command)
        return false;

    stage = DMAProbeStage::SetMemoryDescriptor;
    resultStatus = dma.command->setMemoryDescriptor(dma.buffer, false);
    if (resultStatus != kIOReturnSuccess)
        return false;
    dma.descriptorSet = true;

    stage = DMAProbeStage::Prepare;
    resultStatus = dma.command->prepare(0, length);
    if (resultStatus != kIOReturnSuccess)
        return false;
    dma.prepared = true;

    IODMACommand::Segment32 segments[2] = {};
    UInt64 offset = 0;
    UInt32 segmentCount = 2;
    stage = DMAProbeStage::GenerateSegments;
    resultStatus = dma.command->gen32IOVMSegments(
        &offset, segments, &segmentCount);
    if (resultStatus != kIOReturnSuccess)
        return false;
    if (segmentCount != 1 || offset != length ||
        segments[0].fLength != length ||
        static_cast<UInt64>(segments[0].fIOVMAddr) + length > 0x100000000ULL) {
        stage = DMAProbeStage::ValidateSegment;
        resultStatus = kIOReturnMessageTooLarge;
        return false;
    }

    dma.address = segments[0].fIOVMAddr;
    dma.length = segments[0].fLength;
    stage = DMAProbeStage::Completed;
    resultStatus = kIOReturnSuccess;
    return true;
}

bool releasePreparedDMA(PreparedDMA &dma)
{
    bool released = true;
    if (dma.prepared) {
        released = dma.command->complete() == kIOReturnSuccess;
        dma.prepared = false;
    }
    if (dma.descriptorSet) {
        released = dma.command->clearMemoryDescriptor(false) == kIOReturnSuccess &&
                   released;
        dma.descriptorSet = false;
    }
    if (dma.command) {
        dma.command->release();
        dma.command = nullptr;
    }
    if (dma.buffer) {
        dma.buffer->release();
        dma.buffer = nullptr;
    }
    return released;
}

bool prepareDMATemplate(const UInt8 *payload,
                        DMATemplateResult &result,
                        PreparedDMA &descriptor,
                        PreparedDMA &staging)
{
    result = {kIOReturnError};
    DMAProbeStage ignoredStage = DMAProbeStage::NotAttempted;
    IOReturn ignoredStatus = kIOReturnError;
    bool valid = prepareSingleSegmentDMA(
                     kBeaconDescriptorRingSize, kIODirectionOut, true, descriptor,
                     ignoredStage, ignoredStatus) &&
                 prepareSingleSegmentDMA(
                     kFirmwareStagingPacketSize, kIODirectionOut, true, staging,
                     ignoredStage, ignoredStatus);

    if (valid) {
        UInt8 *packet = static_cast<UInt8 *>(staging.buffer->getBytesNoCopy());
        UInt8 *ringBytes = static_cast<UInt8 *>(descriptor.buffer->getBytesNoCopy());
        UInt32 *txWords = reinterpret_cast<UInt32 *>(packet);
        PCITXBufferElement *ring =
            reinterpret_cast<PCITXBufferElement *>(ringBytes);

        txWords[0] = OSSwapHostToLittleInt32(
            kFirmwareChunkSize | (kTXPacketDescriptorSize << 16) |
            (1U << 26) | (1U << 31));
        txWords[1] = OSSwapHostToLittleInt32(
            (static_cast<UInt32>(kBeaconQueueSelect) << 8) | (8U << 16));
        txWords[3] = OSSwapHostToLittleInt32((1U << 8) | (1U << 10));
        txWords[8] = OSSwapHostToLittleInt32(1U << 15);

        UInt16 checksum = 0;
        UInt16 *checksumWords = reinterpret_cast<UInt16 *>(packet);
        for (UInt8 index = 0; index < 16; index++)
            checksum ^= OSSwapLittleToHostInt16(checksumWords[index]);
        checksumWords[14] = OSSwapHostToLittleInt16(checksum);
        memcpy(packet + kTXPacketDescriptorSize, payload, kFirmwareChunkSize);

        const UInt16 psbLength = static_cast<UInt16>(
            ((kFirmwareStagingPacketSize - 1) / 128 + 1) | kBeaconQueueOwn);
        ring[0] = {
            OSSwapHostToLittleInt16(kTXPacketDescriptorSize),
            OSSwapHostToLittleInt16(psbLength),
            OSSwapHostToLittleInt32(staging.address),
        };
        ring[1] = {
            OSSwapHostToLittleInt16(kFirmwareChunkSize),
            0,
            OSSwapHostToLittleInt32(staging.address + kTXPacketDescriptorSize),
        };

        UInt16 checksumValidation = 0;
        for (UInt8 index = 0; index < 16; index++)
            checksumValidation ^= OSSwapLittleToHostInt16(checksumWords[index]);

        result.status = kIOReturnSuccess;
        result.descriptorAddress = descriptor.address;
        result.stagingAddress = staging.address;
        result.packetBufferSize = OSSwapLittleToHostInt16(ring[0].bufferSize);
        result.packetPSBLength = OSSwapLittleToHostInt16(ring[0].psbLength);
        result.packetAddress = OSSwapLittleToHostInt32(ring[0].address);
        result.payloadBufferSize = OSSwapLittleToHostInt16(ring[1].bufferSize);
        result.payloadAddress = OSSwapLittleToHostInt32(ring[1].address);
        result.txWord0 = OSSwapLittleToHostInt32(txWords[0]);
        result.txWord1 = OSSwapLittleToHostInt32(txWords[1]);
        result.txWord3 = OSSwapLittleToHostInt32(txWords[3]);
        result.txWord8 = OSSwapLittleToHostInt32(txWords[8]);
        result.txChecksum = OSSwapLittleToHostInt16(checksumWords[14]);
        result.payloadValid = memcmp(packet + kTXPacketDescriptorSize, payload,
                                     kFirmwareChunkSize) == 0;
        result.roundTripValid =
            result.packetBufferSize == kTXPacketDescriptorSize &&
            result.packetPSBLength == psbLength &&
            result.packetAddress == staging.address &&
            result.payloadBufferSize == kFirmwareChunkSize &&
            result.payloadAddress == staging.address + kTXPacketDescriptorSize &&
            (result.txWord0 & 0x0000ffffU) == kFirmwareChunkSize &&
            ((result.txWord0 >> 16) & 0xffU) == kTXPacketDescriptorSize &&
            (result.txWord0 & (1U << 26)) != 0 &&
            (result.txWord0 & (1U << 31)) != 0 &&
            ((result.txWord1 >> 8) & 0x1fU) == kBeaconQueueSelect &&
            checksumValidation == 0 && result.payloadValid;
        valid = result.roundTripValid;
    }

    return valid;
}

bool materializeRetainedBeaconRing(PreparedDMA *rings,
                                   UInt32 ringCount,
                                   IOBufferMemoryDescriptor *source,
                                   UInt32 &ringAddress)
{
    ringAddress = 0;
    if (!rings || ringCount != kTRXResourceCount || !source)
        return false;

    constexpr UInt32 beaconResourceIndex = 4;
    const TRXResourcePlan &resource = kTRXResources[beaconResourceIndex];
    PreparedDMA &ring = rings[beaconResourceIndex];
    const void *sourceBytes = source->getBytesNoCopy();
    void *ringBytes = ring.buffer ? ring.buffer->getBytesNoCopy() : nullptr;
    if (resource.type != 0 || resource.queue != 4 || resource.entries != 1 ||
        resource.ringBytes != kBeaconDescriptorRingSize ||
        !ring.prepared || ring.length != kBeaconDescriptorRingSize ||
        !sourceBytes || !ringBytes)
        return false;

    memcpy(ringBytes, sourceBytes, kBeaconDescriptorRingSize);
    if (memcmp(ringBytes, sourceBytes, kBeaconDescriptorRingSize) != 0)
        return false;

    PCITXBufferElement *elements =
        static_cast<PCITXBufferElement *>(ringBytes);
    const UInt16 sourcePSB = OSSwapLittleToHostInt16(elements[0].psbLength);
    elements[0].psbLength = OSSwapHostToLittleInt16(
        static_cast<UInt16>(sourcePSB & ~kBeaconQueueOwn));
    if (OSSwapLittleToHostInt16(elements[0].bufferSize) !=
            kTXPacketDescriptorSize ||
        (OSSwapLittleToHostInt16(elements[0].psbLength) & kBeaconQueueOwn) != 0 ||
        OSSwapLittleToHostInt16(elements[1].bufferSize) != kFirmwareChunkSize)
        return false;

    ringAddress = ring.address;
    return ringAddress != 0 &&
        static_cast<UInt64>(ringAddress) + kBeaconDescriptorRingSize <=
            0x100000000ULL;
}

void preparePersistentTRXAllocation(TRXAllocationProbeResult &result,
                                    PreparedDMA *&retainedRings,
                                    PreparedDMA *&retainedPayloads)
{
    retainedRings = nullptr;
    retainedPayloads = nullptr;
    result = {};
    result.stage = DMAProbeStage::BufferAllocation;
    result.status = kIOReturnNoMemory;
    result.failedResourceIndex = 0xffffffffU;
    result.failedPayloadIndex = 0xffffffffU;
    result.attempted = true;

    const vm_size_t ringBookkeepingSize =
        sizeof(PreparedDMA) * kTRXResourceCount;
    const vm_size_t payloadBookkeepingSize =
        sizeof(PreparedDMA) * kRXRingEntries;
    PreparedDMA *rings = static_cast<PreparedDMA *>(
        IOMallocZero(ringBookkeepingSize));
    PreparedDMA *payloads = static_cast<PreparedDMA *>(
        IOMallocZero(payloadBookkeepingSize));
    if (!rings || !payloads) {
        if (payloads)
            IOFree(payloads, payloadBookkeepingSize);
        if (rings)
            IOFree(rings, ringBookkeepingSize);
        return;
    }

    bool allocationValid = true;
    bool releaseValid = true;
    for (UInt32 index = 0; index < kTRXResourceCount; index++) {
        const TRXResourcePlan &resource = kTRXResources[index];
        if (!prepareSingleSegmentDMA(
                resource.ringBytes,
                resource.type == 1 ? kIODirectionInOut : kIODirectionOut,
                false, rings[index],
                result.stage, result.status)) {
            result.failedResourceIndex = index;
            releaseValid = releasePreparedDMA(rings[index]) && releaseValid;
            allocationValid = false;
            break;
        }
        result.ringMappingCount++;
        result.ringBytes += resource.ringBytes;
        result.peakMappingCount++;
    }

    if (allocationValid) {
        for (UInt32 index = 0; index < kRXRingEntries; index++) {
            if (!prepareSingleSegmentDMA(
                    kRXBufferSize, kIODirectionIn, false, payloads[index],
                result.stage, result.status)) {
                result.failedPayloadIndex = index;
                releaseValid = releasePreparedDMA(payloads[index]) && releaseValid;
                allocationValid = false;
                break;
            }
            result.payloadMappingCount++;
            result.payloadBytes += kRXBufferSize;
            result.peakMappingCount++;
        }
    }

    if (allocationValid) {
        for (UInt32 resourceIndex = 0;
             resourceIndex < kTRXResourceCount - 1; resourceIndex++) {
            const TRXResourcePlan &resource = kTRXResources[resourceIndex];
            const UInt8 *descriptors = static_cast<const UInt8 *>(
                rings[resourceIndex].buffer->getBytesNoCopy());
            const UInt64 addressEnd =
                static_cast<UInt64>(rings[resourceIndex].address) +
                resource.ringBytes;
            const bool ringValid = resource.type == 0 &&
                resource.descriptorSize == kTXRingDescriptorSize &&
                resource.ringBytes == resource.entries *
                    kTXRingDescriptorSize &&
                descriptors && rings[resourceIndex].prepared &&
                rings[resourceIndex].length == resource.ringBytes;
            if (!ringValid) {
                allocationValid = false;
                break;
            }

            result.txRingValidCount++;
            if (rings[resourceIndex].address != 0 &&
                addressEnd <= 0x100000000ULL)
                result.txRingAddressRangeValidCount++;

            for (UInt32 descriptorIndex = 0;
                 descriptorIndex < resource.entries; descriptorIndex++) {
                const UInt8 *descriptor = descriptors +
                    descriptorIndex * kTXRingDescriptorSize;
                bool zero = true;
                for (UInt32 byteIndex = 0;
                     byteIndex < kTXRingDescriptorSize; byteIndex++)
                    zero = zero && descriptor[byteIndex] == 0;
                if (zero)
                    result.txDescriptorZeroCount++;
            }
        }

        result.txDescriptorFormatValid = allocationValid &&
            result.txRingValidCount == 8 &&
            result.txDescriptorZeroCount == 1025 &&
            result.txRingAddressRangeValidCount == 8;
        allocationValid = result.txDescriptorFormatValid;
        if (!allocationValid) {
            result.stage = DMAProbeStage::ValidateSegment;
            result.status = kIOReturnError;
        }
    }

    if (allocationValid) {
        const UInt32 rxResourceIndex = kTRXResourceCount - 1;
        const TRXResourcePlan &rxResource = kTRXResources[rxResourceIndex];
        PCIRXBufferDescriptor *descriptors =
            static_cast<PCIRXBufferDescriptor *>(
                rings[rxResourceIndex].buffer->getBytesNoCopy());
        allocationValid = descriptors && rxResource.type == 1 &&
            rxResource.queue == 0 && rxResource.entries == kRXRingEntries &&
            rxResource.descriptorSize == sizeof(PCIRXBufferDescriptor) &&
            rxResource.ringBytes == kRXRingEntries *
                sizeof(PCIRXBufferDescriptor) &&
            rings[rxResourceIndex].prepared &&
            rings[rxResourceIndex].length == rxResource.ringBytes;

        if (allocationValid) {
            for (UInt32 index = 0; index < kRXRingEntries; index++) {
                descriptors[index] = {
                    OSSwapHostToLittleInt16(
                        static_cast<UInt16>(kRXBufferSize)),
                    0,
                    OSSwapHostToLittleInt32(payloads[index].address),
                };
            }
            result.rxDescriptorMaterialized = true;

            for (UInt32 index = 0; index < kRXRingEntries; index++) {
                const UInt16 bufferSize = OSSwapLittleToHostInt16(
                    descriptors[index].bufferSize);
                const UInt16 totalPacketSize = OSSwapLittleToHostInt16(
                    descriptors[index].totalPacketSize);
                const UInt32 address = OSSwapLittleToHostInt32(
                    descriptors[index].address);
                const UInt64 addressEnd = static_cast<UInt64>(address) +
                    kRXBufferSize;
                if (bufferSize == kRXBufferSize)
                    result.rxDescriptorValidCount++;
                if (totalPacketSize == 0)
                    result.rxDescriptorZeroTagCount++;
                if (address == payloads[index].address)
                    result.rxDescriptorAddressMatchCount++;
                if (address != 0 && addressEnd <= 0x100000000ULL)
                    result.rxDescriptorAddressRangeValidCount++;
            }

            result.rxDescriptorFormatValid =
                result.rxDescriptorValidCount == kRXRingEntries &&
                result.rxDescriptorZeroTagCount == kRXRingEntries &&
                result.rxDescriptorAddressMatchCount == kRXRingEntries &&
                result.rxDescriptorAddressRangeValidCount == kRXRingEntries;
            allocationValid = result.rxDescriptorFormatValid;
        }

        if (!allocationValid) {
            result.stage = DMAProbeStage::ValidateSegment;
            result.status = kIOReturnError;
        }
    }

    result.allocationComplete = allocationValid &&
        result.ringMappingCount == kTRXResourceCount &&
        result.payloadMappingCount == kRXRingEntries &&
        result.ringBytes == 20496 && result.payloadBytes == 5876736 &&
        result.txDescriptorFormatValid && result.rxDescriptorFormatValid;

    if (result.allocationComplete) {
        retainedRings = rings;
        retainedPayloads = payloads;
        result.resourcesReleased = false;
        result.stage = DMAProbeStage::Completed;
        result.status = kIOReturnSuccess;
        return;
    }

    for (UInt32 index = result.payloadMappingCount; index > 0; index--)
        releaseValid = releasePreparedDMA(payloads[index - 1]) && releaseValid;
    for (UInt32 index = result.ringMappingCount; index > 0; index--)
        releaseValid = releasePreparedDMA(rings[index - 1]) && releaseValid;
    IOFree(payloads, payloadBookkeepingSize);
    IOFree(rings, ringBookkeepingSize);
    result.resourcesReleased = releaseValid;
}

const char *dmaProbeStageName(DMAProbeStage stage)
{
    switch (stage) {
    case DMAProbeStage::NotAttempted:
        return "NotAttempted";
    case DMAProbeStage::BufferAllocation:
        return "BufferAllocation";
    case DMAProbeStage::VirtualAddress:
        return "VirtualAddress";
    case DMAProbeStage::CommandAllocation:
        return "CommandAllocation";
    case DMAProbeStage::SetMemoryDescriptor:
        return "SetMemoryDescriptor";
    case DMAProbeStage::Prepare:
        return "Prepare";
    case DMAProbeStage::GenerateSegments:
        return "GenerateSegments";
    case DMAProbeStage::ValidateSegment:
        return "ValidateSegment";
    case DMAProbeStage::Complete:
        return "Complete";
    case DMAProbeStage::ClearMemoryDescriptor:
        return "ClearMemoryDescriptor";
    case DMAProbeStage::Completed:
        return "Completed";
    }
    return "Unknown";
}

bool probeSingleSegmentDMA(UInt32 requestedLength, DMAProbeResult &result)
{
    result = {
        DMAProbeStage::BufferAllocation,
        kIOReturnNoMemory,
        requestedLength,
        0,
        0,
        0,
        false,
    };

    const mach_vm_address_t physicalMask =
        0x00000000ffffffffULL &
        ~(static_cast<mach_vm_address_t>(kDMAAlignment) - 1);
    IOBufferMemoryDescriptor *buffer =
        IOBufferMemoryDescriptor::inTaskWithPhysicalMask(
            kernel_task, kIODirectionOut | kIOMemoryPhysicallyContiguous,
            requestedLength, physicalMask);
    if (!buffer)
        return false;

    void *bytes = buffer->getBytesNoCopy();
    if (!bytes) {
        result.stage = DMAProbeStage::VirtualAddress;
        result.status = kIOReturnVMError;
        buffer->release();
        return false;
    }
    bzero(bytes, requestedLength);
    result.zeroFilled = true;

    IODMACommand *command = nullptr;
    result.stage = DMAProbeStage::CommandAllocation;
    result.status = IODMACommand::weakWithSpecification(
        &command, kIODMACommandOutputHost32, 32, requestedLength,
        IODMACommand::kMapped, requestedLength, kDMAAlignment);
    if (result.status != kIOReturnSuccess || !command) {
        buffer->release();
        return false;
    }

    bool descriptorSet = false;
    bool prepared = false;
    result.stage = DMAProbeStage::SetMemoryDescriptor;
    result.status = command->setMemoryDescriptor(buffer, false);
    if (result.status == kIOReturnSuccess) {
        descriptorSet = true;
        result.stage = DMAProbeStage::Prepare;
        result.status = command->prepare(0, requestedLength);
    }

    if (result.status == kIOReturnSuccess) {
        prepared = true;
        IODMACommand::Segment32 segments[2] = {};
        UInt64 offset = 0;
        UInt32 segmentCount = 2;
        result.stage = DMAProbeStage::GenerateSegments;
        result.status = command->gen32IOVMSegments(
            &offset, segments, &segmentCount);
        result.segmentCount = segmentCount;
        if (result.status == kIOReturnSuccess) {
            result.address = segments[0].fIOVMAddr;
            result.length = segments[0].fLength;
            const UInt64 segmentEnd = static_cast<UInt64>(result.address) +
                                      result.length;
            if (segmentCount != 1 || offset != requestedLength ||
                result.length != requestedLength || segmentEnd > 0x100000000ULL) {
                result.stage = DMAProbeStage::ValidateSegment;
                result.status = kIOReturnMessageTooLarge;
            }
        }
    }

    const bool mappingValid = result.status == kIOReturnSuccess;
    if (prepared) {
        const IOReturn completeStatus = command->complete();
        if (completeStatus != kIOReturnSuccess) {
            result.stage = DMAProbeStage::Complete;
            result.status = completeStatus;
        }
    }
    if (descriptorSet) {
        const IOReturn clearStatus = command->clearMemoryDescriptor(false);
        if (clearStatus != kIOReturnSuccess) {
            result.stage = DMAProbeStage::ClearMemoryDescriptor;
            result.status = clearStatus;
        }
    }

    command->release();
    buffer->release();
    if (mappingValid && result.status == kIOReturnSuccess)
        result.stage = DMAProbeStage::Completed;
    return result.stage == DMAProbeStage::Completed;
}

bool equalPreSystemSnapshot(const PreSystemSnapshot &left,
                            const PreSystemSnapshot &right)
{
    return left.reservedControl == right.reservedControl &&
           left.hciOptionControl == right.hciOptionControl &&
           left.padControl1 == right.padControl1 &&
           left.ledConfig == right.ledConfig &&
           left.gpioMuxConfig == right.gpioMuxConfig &&
           left.systemFunctionEnable == right.systemFunctionEnable &&
           left.rfControl == right.rfControl &&
           left.wlrf1 == right.wlrf1 &&
           left.systemPowerControl == right.systemPowerControl &&
           left.firmwareControl == right.firmwareControl &&
           left.rpwm == right.rpwm;
}

constexpr PowerCommand kPowerOnSequence[] = {
    {PowerOp::Write8, 0, 0x0005, 0x98, 0x00},
    {PowerOp::Write8, 0, 0x0300, 0xff, 0x00},
    {PowerOp::Write8, 0, 0x0301, 0xff, 0x00},
    {PowerOp::Write8, 1, 0x0005, 0x1c, 0x00},
    {PowerOp::Write8, 1, 0x0075, 0x01, 0x01},
    {PowerOp::Poll8,  1, 0x0006, 0x02, 0x02},
    {PowerOp::Write8, 1, 0x0075, 0x01, 0x00},
    {PowerOp::Write8, 2, 0x0006, 0x01, 0x01},
    {PowerOp::Write8, 2, 0x0005, 0x80, 0x00},
    {PowerOp::Write8, 2, 0x0005, 0x18, 0x00},
    {PowerOp::Write8, 2, 0x0005, 0x01, 0x01},
    {PowerOp::Poll8,  2, 0x0005, 0x01, 0x00},
    {PowerOp::Write8, 3, 0x0020, 0x08, 0x08},
    {PowerOp::Write8, 3, 0x0074, 0x20, 0x20},
    {PowerOp::Write8, 3, 0x0022, 0x02, 0x00},
    {PowerOp::Write8, 3, 0x0062, 0xe0, 0xe0},
    {PowerOp::Write8, 3, 0x0061, 0xe0, 0x00},
    {PowerOp::Write8, 3, 0x007c, 0x02, 0x00},
};

constexpr PowerCommand kPowerOffSequence[] = {
    {PowerOp::Write8, 4, 0x0093, 0x08, 0x00},
    {PowerOp::Write8, 4, 0x001f, 0xff, 0x00},
    {PowerOp::Write8, 4, 0x0049, 0x02, 0x00},
    {PowerOp::Write8, 4, 0x0006, 0x01, 0x01},
    {PowerOp::Write8, 4, 0x0002, 0x02, 0x00},
    {PowerOp::Write8, 4, 0x0005, 0x02, 0x02},
    {PowerOp::Poll8,  4, 0x0005, 0x02, 0x00},
    {PowerOp::Write8, 4, 0x0020, 0x08, 0x00},
    {PowerOp::Write8, 4, 0x0067, 0x20, 0x00},
    {PowerOp::Write8, 4, 0x0005, 0x04, 0x04},
    {PowerOp::Write8, 4, 0x0081, 0xc0, 0x00},
    {PowerOp::Write8, 4, 0x0090, 0x02, 0x00},
};

constexpr PreSystemCommand kPreSystemSequence[] = {
    {0x001c, 0x000000ffU, 0x00000000U, 1},
    {0x0074, 0x00000100U, 0x00000100U, 4},
    {0x0064, 0x30000000U, 0x30000000U, 4},
    {0x004c, 0x06000000U, 0x00000000U, 4},
    {0x0040, 0x00000004U, 0x00000004U, 4},
    {0x0002, 0x00000003U, 0x00000000U, 1},
    {0x001f, 0x00000007U, 0x00000000U, 1},
    {0x00ec, 0x07000000U, 0x00000000U, 4},
};

constexpr UInt16 kSnapshotOffsets[] = {
    0x0002, 0x0005, 0x0006, 0x001c, 0x001f, 0x0020, 0x0022,
    0x0049, 0x0061, 0x0062, 0x0067, 0x0074, 0x0075, 0x007c,
    0x0081, 0x0090, 0x0093, 0x00ec, 0x0300, 0x0301,
};

constexpr size_t kPowerOnCommandCount =
    sizeof(kPowerOnSequence) / sizeof(kPowerOnSequence[0]);
constexpr size_t kPowerOffCommandCount =
    sizeof(kPowerOffSequence) / sizeof(kPowerOffSequence[0]);
constexpr size_t kPowerCommandCount = kPowerOnCommandCount + kPowerOffCommandCount;
constexpr size_t kProjectedRegisterCount =
    sizeof(kSnapshotOffsets) / sizeof(kSnapshotOffsets[0]);
static_assert(kPowerCommandCount == 30, "unexpected PCIe power command count");
static_assert(kProjectedRegisterCount == 20, "unexpected power register count");
static_assert(sizeof(RegisterSnapshot) == 4, "unexpected snapshot record layout");
static_assert(sizeof(ProjectedRegister) == 8, "unexpected projection record layout");
static_assert(sizeof(FirmwareHeader) == kFirmwareHeaderSize,
              "unexpected firmware header layout");
static_assert(sizeof(FirmwareTransferPlan) == 16,
              "unexpected firmware transfer record layout");
static_assert(sizeof(PowerOnlyStepRecord) == 16,
              "unexpected power-only step layout");
static_assert(sizeof(PowerExecutionJournalRecord) == 24,
              "unexpected power journal layout");
static_assert(sizeof(PowerExecutorSimulationSummary) == 32,
              "unexpected power simulation summary layout");

bool buildPowerOnlyContract(PowerOnlyStepRecord *steps, size_t capacity,
                            size_t &stepCount)
{
    stepCount = 0;
    if (!steps || capacity < kPowerOnlyStepCount)
        return false;
    for (size_t index = 0;
         index < sizeof(kPreSystemSequence) / sizeof(kPreSystemSequence[0]);
         index++) {
        const PreSystemCommand &command = kPreSystemSequence[index];
        steps[stepCount] = {
            static_cast<UInt8>(stepCount), kPowerOnlyPreSystem,
            static_cast<UInt8>(PowerOp::Write8), command.width,
            command.offset, 0, 0, command.mask, command.value,
        };
        stepCount++;
    }
    for (size_t index = 0; index < kPowerOnCommandCount; index++) {
        const PowerCommand &command = kPowerOnSequence[index];
        steps[stepCount] = {
            static_cast<UInt8>(stepCount), kPowerOnlyFSM,
            static_cast<UInt8>(command.op), 1, command.offset,
            command.phase, 0, command.mask, command.value,
        };
        stepCount++;
    }
    return stepCount == kPowerOnlyStepCount;
}

bool validatePowerOnlyContract(const PowerOnlyStepRecord *steps,
                               size_t stepCount, IOByteCount barLength,
                               UInt16 &writeCount, UInt16 &pollCount)
{
    writeCount = 0;
    pollCount = 0;
    if (!steps || stepCount != kPowerOnlyStepCount || barLength != 0x10000)
        return false;
    for (size_t index = 0; index < stepCount; index++) {
        const PowerOnlyStepRecord &step = steps[index];
        if (step.sequence != index || step.source > kPowerOnlyFSM ||
            (step.operation != static_cast<UInt8>(PowerOp::Write8) &&
             step.operation != static_cast<UInt8>(PowerOp::Poll8)) ||
            (step.width != 1 && step.width != 4) ||
            step.offset >= barLength || step.mask == 0 ||
            (step.value & ~step.mask) != 0 ||
            (step.width == 1 && (step.mask & 0xffffff00U) != 0) ||
            (step.source == kPowerOnlyPreSystem &&
             step.operation != static_cast<UInt8>(PowerOp::Write8)))
            return false;
        if (step.operation == static_cast<UInt8>(PowerOp::Write8))
            writeCount++;
        else
            pollCount++;
    }
    return writeCount == 24 && pollCount == 2 &&
        steps[0].offset == 0x001c &&
        steps[7].offset == 0x00ec &&
        steps[8].offset == 0x0005 &&
        steps[13].operation == static_cast<UInt8>(PowerOp::Poll8) &&
        steps[19].operation == static_cast<UInt8>(PowerOp::Poll8) &&
        steps[25].offset == 0x007c;
}

bool simulatePowerOnlyExecutor(const PowerOnlyStepRecord *steps,
                               size_t stepCount,
                               PowerExecutorSimulationSummary &summary)
{
    summary = {};
    UInt16 writeCount = 0;
    UInt16 pollCount = 0;
    if (!validatePowerOnlyContract(
            steps, stepCount, 0x10000, writeCount, pollCount))
        return false;
    summary.stepCount = static_cast<UInt16>(stepCount);
    summary.writeCount = writeCount;
    summary.pollCount = pollCount;
    // Success, pre-intent rejection, and post-intent failure per step.
    summary.scenarioCount = static_cast<UInt16>(stepCount * 3);
    for (size_t index = 0; index < stepCount; index++) {
        const PowerOnlyStepRecord &step = steps[index];
        summary.successScenarioCount++;
        summary.preIntentRejectionCount++;
        if (step.operation == static_cast<UInt8>(PowerOp::Write8))
            summary.unknownEffectQuarantineCount++;
        else
            summary.timeoutQuarantineCount++;
    }
    const size_t maximumSuccessJournalCount = stepCount * 2;
    const size_t maximumFailureJournalCount = stepCount * 2;
    return summary.stepCount == kPowerOnlyStepCount &&
        summary.scenarioCount == kPowerOnlyStepCount * 3 &&
        summary.successScenarioCount == kPowerOnlyStepCount &&
        summary.preIntentRejectionCount == kPowerOnlyStepCount &&
        summary.unknownEffectQuarantineCount == writeCount &&
        summary.timeoutQuarantineCount == pollCount &&
        kPowerExecutionJournalCapacity >= maximumSuccessJournalCount &&
        kPowerExecutionJournalCapacity >= maximumFailureJournalCount &&
        summary.retryAuthorizedCount == 0 &&
        summary.powerOffAuthorizedCount == 0;
}

UInt32 firmwareChunkCount(UInt32 size)
{
    return (size + kFirmwareChunkSize - 1) / kFirmwareChunkSize;
}

bool buildFirmwareTransferPlan(const FirmwareHeader &header,
                               UInt32 firmwareSize,
                               FirmwareTransferPlan *plan,
                               size_t planCapacity,
                               size_t &planCount,
                               UInt32 &dmemTransferSize,
                               UInt32 &imemTransferSize,
                               UInt32 &ememTransferSize)
{
    dmemTransferSize = OSSwapLittleToHostInt32(header.dmemSize) +
                       kFirmwareChecksumSize;
    imemTransferSize = OSSwapLittleToHostInt32(header.imemSize) +
                       kFirmwareChecksumSize;
    const UInt32 rawEmemSize = OSSwapLittleToHostInt32(header.ememSize);
    ememTransferSize = (header.memoryUsage & 0x10U) != 0 ?
                       rawEmemSize + kFirmwareChecksumSize : 0;

    const UInt64 calculatedSize = static_cast<UInt64>(kFirmwareHeaderSize) +
                                  dmemTransferSize + imemTransferSize +
                                  ememTransferSize;
    if (calculatedSize != firmwareSize)
        return false;

    struct Section {
        UInt8 id;
        UInt32 offset;
        UInt32 destination;
        UInt32 size;
    };
    const Section sections[] = {
        {0, kFirmwareHeaderSize,
         OSSwapLittleToHostInt32(header.dmemAddress) & 0x7fffffffU,
         dmemTransferSize},
        {1, kFirmwareHeaderSize + dmemTransferSize,
         OSSwapLittleToHostInt32(header.imemAddress) & 0x7fffffffU,
         imemTransferSize},
        {2, kFirmwareHeaderSize + dmemTransferSize + imemTransferSize,
         OSSwapLittleToHostInt32(header.ememAddress) & 0x7fffffffU,
         ememTransferSize},
    };

    for (const Section &section : sections) {
        UInt32 consumed = 0;
        while (consumed < section.size) {
            if (planCount >= planCapacity)
                return false;
            const UInt32 remaining = section.size - consumed;
            const UInt32 chunkSize = remaining < kFirmwareChunkSize ?
                                     remaining : kFirmwareChunkSize;
            plan[planCount++] = {
                section.id,
                static_cast<UInt8>(consumed == 0),
                0,
                section.offset + consumed,
                section.destination + consumed,
                chunkSize,
            };
            consumed += chunkSize;
        }
    }

    return planCount == firmwareChunkCount(dmemTransferSize) +
                        firmwareChunkCount(imemTransferSize) +
                        firmwareChunkCount(ememTransferSize);
}

template <size_t Count>
bool validatePowerSequence(IOByteCount length,
                           const PowerCommand (&sequence)[Count],
                           UInt8 &writeCount)
{
    UInt8 previousPhase = sequence[0].phase;

    for (size_t index = 0; index < Count; index++) {
        const PowerCommand &command = sequence[index];
        if (command.offset >= length || command.mask == 0 ||
            (command.value & ~command.mask) != 0 ||
            command.phase < previousPhase || command.phase > 4)
            return false;

        previousPhase = command.phase;

        if (command.op == PowerOp::Write8 && ++writeCount > 32)
            return false;
    }

    return true;
}

template <size_t Count>
bool planPowerSequence(const PowerCommand (&sequence)[Count],
                       ProjectedRegister *registers,
                       size_t registerCount,
                       DryRunCommand *plan,
                       size_t planCapacity,
                       size_t &planCount,
                       bool powerOn)
{
    for (size_t commandIndex = 0; commandIndex < Count; commandIndex++) {
        const PowerCommand &command = sequence[commandIndex];
        ProjectedRegister *projected = nullptr;

        for (size_t registerIndex = 0; registerIndex < registerCount; registerIndex++) {
            if (registers[registerIndex].offset == command.offset) {
                projected = &registers[registerIndex];
                break;
            }
        }

        if (!projected || planCount >= planCapacity)
            return false;

        UInt8 &current = powerOn ? projected->afterPowerOn : projected->afterPowerOff;
        const UInt8 before = current;
        UInt8 after = before;
        const bool write = command.op == PowerOp::Write8;

        if (write) {
            after = static_cast<UInt8>((before & ~command.mask) |
                                      (command.value & command.mask));
            if (powerOn)
                projected->powerOnWriteMask |= command.mask;
            else
                projected->powerOffWriteMask |= command.mask;
        } else {
            // Continue the dry-run along the path where this poll reaches its target.
            after = static_cast<UInt8>((before & ~command.mask) |
                                      (command.value & command.mask));
        }
        current = after;

        plan[planCount] = {
            static_cast<UInt8>(planCount),
            command.phase,
            static_cast<UInt8>(command.op),
            static_cast<UInt8>(write),
            command.offset,
            command.mask,
            command.value,
            before,
            after,
        };
        planCount++;
    }

    return true;
}

UInt32 readRegister(const volatile UInt8 *base, const PreSystemCommand &command)
{
    return command.width == 4 ? OSReadLittleInt32(base, command.offset) :
                                base[command.offset];
}

UInt32 readRegisterWidth(const volatile UInt8 *base,
                         UInt16 offset,
                         UInt8 width)
{
    if (width == 4)
        return OSReadLittleInt32(base, offset);
    if (width == 2)
        return OSReadLittleInt16(base, offset);
    return base[offset];
}

void writeRegister(volatile UInt8 *base,
                   const PreSystemCommand &command,
                   UInt32 value)
{
    if (command.width == 4)
        OSWriteLittleInt32(base, command.offset, value);
    else
        base[command.offset] = static_cast<UInt8>(value);
    OSSynchronizeIO();
}

ExperimentResult executePowerOnExperiment(volatile UInt8 *base,
                                          UInt8 &completedWrites,
                                          UInt8 &completedPolls,
                                          UInt8 &failedCommand,
                                          UInt8 &macAfter)
{
    for (size_t index = 0;
         index < sizeof(kPreSystemSequence) / sizeof(kPreSystemSequence[0]);
         index++) {
        const PreSystemCommand &command = kPreSystemSequence[index];
        const UInt32 before = readRegister(base, command);
        const UInt32 after = (before & ~command.mask) | (command.value & command.mask);
        writeRegister(base, command, after);
        if ((readRegister(base, command) & command.mask) != (after & command.mask)) {
            failedCommand = static_cast<UInt8>(index);
            return ExperimentResult::WriteVerificationFailed;
        }
        completedWrites++;
    }

    for (size_t index = 0; index < kPowerOnCommandCount; index++) {
        const PowerCommand &command = kPowerOnSequence[index];
        if (command.op == PowerOp::Write8) {
            const UInt8 before = base[command.offset];
            const UInt8 after = static_cast<UInt8>((before & ~command.mask) |
                                                   (command.value & command.mask));
            base[command.offset] = after;
            OSSynchronizeIO();
            if ((base[command.offset] & command.mask) != (after & command.mask)) {
                failedCommand = static_cast<UInt8>(
                    sizeof(kPreSystemSequence) / sizeof(kPreSystemSequence[0]) + index);
                return ExperimentResult::WriteVerificationFailed;
            }
            completedWrites++;
            continue;
        }

        bool matched = false;
        for (UInt32 attempt = 0; attempt < kPowerPollIterations; attempt++) {
            if ((base[command.offset] & command.mask) == command.value) {
                matched = true;
                break;
            }
            IODelay(kPowerPollIntervalMicroseconds);
        }
        if (!matched) {
            failedCommand = static_cast<UInt8>(
                sizeof(kPreSystemSequence) / sizeof(kPreSystemSequence[0]) + index);
            return ExperimentResult::PollTimedOut;
        }
        completedPolls++;
    }

    macAfter = base[kRegisterCr];
    return macAfter == 0xeaU ? ExperimentResult::MACStillOff :
                               ExperimentResult::Completed;
}

bool appendPowerJournal(PowerExecutionJournalRecord *journal,
                        size_t capacity, size_t &count,
                        UInt8 step, PowerJournalOutcome outcome,
                        const PowerOnlyStepRecord &contract,
                        UInt32 before, UInt32 after,
                        UInt32 cumulativeHazards, UInt32 status)
{
    if (!journal || count >= capacity || count > 0xffU)
        return false;
    journal[count] = {
        static_cast<UInt8>(count), step, static_cast<UInt8>(outcome),
        contract.width, contract.offset, 0, before, after,
        cumulativeHazards, status,
    };
    __sync_synchronize();
    count++;
    return true;
}

PowerExecutorResult executePowerOnlyContract(
    volatile UInt8 *base, const PowerOnlyStepRecord *steps, size_t stepCount,
    PowerExecutionJournalRecord *journal, size_t journalCapacity,
    size_t &journalCount, UInt8 &completedWrites, UInt8 &completedPolls,
    UInt8 &failedStep, UInt8 &macAfter)
{
    journalCount = 0;
    completedWrites = 0;
    completedPolls = 0;
    failedStep = 0xffU;
    macAfter = 0xffU;
    UInt16 writeCount = 0;
    UInt16 pollCount = 0;
    if (!base || !journal ||
        !validatePowerOnlyContract(
            steps, stepCount, 0x10000, writeCount, pollCount) ||
        journalCapacity < stepCount * 2)
        return PowerExecutorResult::ContractInvalid;

    UInt32 cumulativeHazards = 0;
    for (size_t index = 0; index < stepCount; index++) {
        const PowerOnlyStepRecord &step = steps[index];
        const UInt32 before = readRegisterWidth(base, step.offset, step.width);
        const UInt32 projected = step.operation ==
            static_cast<UInt8>(PowerOp::Write8) ?
            (before & ~step.mask) | (step.value & step.mask) : before;
        const UInt32 attemptedHazards = cumulativeHazards |
            kHazardPowerWritePossible;
        if (!appendPowerJournal(journal, journalCapacity, journalCount,
            static_cast<UInt8>(index), kPowerJournalIntent, step,
            before, projected, attemptedHazards, kIOReturnSuccess)) {
            failedStep = static_cast<UInt8>(index);
            return PowerExecutorResult::ContractInvalid;
        }

        if (step.operation == static_cast<UInt8>(PowerOp::Write8)) {
            cumulativeHazards = attemptedHazards;
            if (step.width == 4)
                OSWriteLittleInt32(base, step.offset, projected);
            else
                base[step.offset] = static_cast<UInt8>(projected);
            OSSynchronizeIO();
            const UInt32 observed = readRegisterWidth(
                base, step.offset, step.width);
            if ((observed & step.mask) != (projected & step.mask)) {
                appendPowerJournal(journal, journalCapacity, journalCount,
                    static_cast<UInt8>(index),
                    kPowerJournalWriteEffectUnknown, step, before, observed,
                    cumulativeHazards,
                    static_cast<UInt32>(kIOReturnIOError));
                failedStep = static_cast<UInt8>(index);
                return PowerExecutorResult::WriteEffectUnknown;
            }
            if (!appendPowerJournal(journal, journalCapacity, journalCount,
                static_cast<UInt8>(index), kPowerJournalSucceeded, step,
                before, observed, cumulativeHazards, kIOReturnSuccess)) {
                failedStep = static_cast<UInt8>(index);
                return PowerExecutorResult::WriteEffectUnknown;
            }
            completedWrites++;
            continue;
        }

        bool matched = false;
        UInt32 observed = before;
        for (UInt32 attempt = 0; attempt < kPowerPollIterations; attempt++) {
            observed = readRegisterWidth(base, step.offset, step.width);
            if ((observed & step.mask) == step.value) {
                matched = true;
                break;
            }
            IODelay(kPowerPollIntervalMicroseconds);
        }
        if (!matched) {
            cumulativeHazards = attemptedHazards;
            appendPowerJournal(journal, journalCapacity, journalCount,
                static_cast<UInt8>(index), kPowerJournalPollTimedOut, step,
                before, observed, cumulativeHazards,
                static_cast<UInt32>(kIOReturnTimeout));
            failedStep = static_cast<UInt8>(index);
            return PowerExecutorResult::PollTimedOut;
        }
        if (!appendPowerJournal(journal, journalCapacity, journalCount,
            static_cast<UInt8>(index), kPowerJournalSucceeded, step,
            before, observed, cumulativeHazards, kIOReturnSuccess)) {
            failedStep = static_cast<UInt8>(index);
            return PowerExecutorResult::WriteEffectUnknown;
        }
        completedPolls++;
    }

    macAfter = base[kRegisterCr];
    return macAfter == 0xeaU ? PowerExecutorResult::MACStillOff :
                              PowerExecutorResult::Completed;
}

const char *powerExecutorResultName(PowerExecutorResult result)
{
    switch (result) {
    case PowerExecutorResult::Disarmed:
        return "Disarmed";
    case PowerExecutorResult::Completed:
        return "Completed";
    case PowerExecutorResult::ContractInvalid:
        return "ContractInvalid";
    case PowerExecutorResult::WriteEffectUnknown:
        return "WriteEffectUnknown";
    case PowerExecutorResult::PollTimedOut:
        return "PollTimedOut";
    case PowerExecutorResult::MACStillOff:
        return "MACStillOff";
    }
    return "Unknown";
}

const char *experimentResultName(ExperimentResult result)
{
    switch (result) {
    case ExperimentResult::Disarmed:
        return "Disarmed";
    case ExperimentResult::Completed:
        return "Completed";
    case ExperimentResult::WriteVerificationFailed:
        return "WriteVerificationFailed";
    case ExperimentResult::PollTimedOut:
        return "PollTimedOut";
    case ExperimentResult::MACStillOff:
        return "MACStillOff";
    }
    return "Unknown";
}
}

#define super IOService
OSDefineMetaClassAndStructors(RTL8821CEProbe, IOService)

bool RTL8821CEProbe::start(IOService *provider)
{
    if (!super::start(provider))
        return false;

    IOPCIDevice *device = OSDynamicCast(IOPCIDevice, provider);
    if (!device) {
        IOLog("RTL8821CEProbe: provider is not an IOPCIDevice\n");
        super::stop(provider);
        return false;
    }

    const UInt16 vendor = device->configRead16(kIOPCIConfigVendorID);
    const UInt16 product = device->configRead16(kIOPCIConfigDeviceID);
    const UInt16 subsystemVendor = device->configRead16(kIOPCIConfigSubSystemVendorID);
    const UInt16 subsystem = device->configRead16(kIOPCIConfigSubSystemID);
    const UInt8 revision = device->configRead8(kIOPCIConfigRevisionID);
    UInt16 command = device->configRead16(kIOPCIConfigCommand);
    const UInt8 headerType = device->configRead8(kIOPCIConfigHeaderType);

    if (vendor != 0x10ec || product != 0xc821) {
        IOLog("RTL8821CEProbe: refusing unexpected PCI device %04x:%04x\n",
              vendor, product);
        super::stop(provider);
        return false;
    }

    if ((headerType & 0x7fU) != 0) {
        IOLog("RTL8821CEProbe: refusing non-endpoint PCI header type %02x\n", headerType);
        super::stop(provider);
        return false;
    }

    if ((command & kIOPCICommandBusMaster) != 0) {
        IOLog("RTL8821CEProbe: refusing MMIO while PCI bus mastering is already enabled (command 0x%04x)\n",
              command);
        super::stop(provider);
        return false;
    }

    setProperty("ProbeMode", kOSBooleanTrue);
    setProperty("VendorID", vendor, 16);
    setProperty("DeviceID", product, 16);
    setProperty("SubsystemVendorID", subsystemVendor, 16);
    setProperty("SubsystemID", subsystem, 16);
    setProperty("RevisionID", revision, 8);
    setProperty("PCICommand", command, 16);
    setProperty("PCIHeaderType", headerType, 8);
    setProperty("HardwareAccess", "TemporaryMemoryDecode+PowerFSMDryRunReadOnly");

    IOLog("RTL8821CEProbe: detected %04x:%04x subsystem %04x:%04x revision %02x; reset capability probe active\n",
          vendor, product, subsystemVendor, subsystem, revision);

    static const char *const rawProperties[] = {
        "BAR0Raw", "BAR1Raw", "BAR2Raw", "BAR3Raw", "BAR4Raw", "BAR5Raw"
    };
    static const char *const addressProperties[] = {
        "BAR0Address", "BAR1Address", "BAR2Address", "BAR3Address", "BAR4Address", "BAR5Address"
    };
    static const char *const typeProperties[] = {
        "BAR0Type", "BAR1Type", "BAR2Type", "BAR3Type", "BAR4Type", "BAR5Type"
    };
    UInt32 bars[6];

    for (UInt8 index = 0; index < 6; index++) {
        const UInt8 offset = static_cast<UInt8>(kIOPCIConfigBaseAddress0 + index * sizeof(UInt32));
        bars[index] = device->configRead32(offset);
        setProperty(rawProperties[index], bars[index], 32);
    }

    for (UInt8 index = 0; index < 6; index++) {
        const UInt32 raw = bars[index];

        if (raw == 0 || raw == 0xffffffffU) {
            setProperty(typeProperties[index], "Unassigned");
            IOLog("RTL8821CEProbe: BAR%u unassigned (raw 0x%08x)\n", index, raw);
            continue;
        }

        if (raw & 0x1U) {
            const UInt32 address = raw & ~0x3U;
            setProperty(typeProperties[index], "IO");
            setProperty(addressProperties[index], address, 32);
            IOLog("RTL8821CEProbe: BAR%u I/O address 0x%08x (raw 0x%08x)\n",
                  index, address, raw);
            continue;
        }

        const UInt32 type = (raw >> 1) & 0x3U;
        if (type == 0x2U && index < 5) {
            const UInt32 high = bars[index + 1];
            const UInt64 address = (static_cast<UInt64>(high) << 32) | (raw & ~0xfU);
            setProperty(typeProperties[index], "Memory64");
            setProperty(addressProperties[index], address, 64);
            IOLog("RTL8821CEProbe: BAR%u 64-bit memory address 0x%016llx (raw 0x%08x:%08x)\n",
                  index, address, high, raw);
            index++;
            continue;
        }

        if (type == 0x3U) {
            setProperty(typeProperties[index], "ReservedMemoryType");
            IOLog("RTL8821CEProbe: BAR%u uses reserved memory type (raw 0x%08x)\n",
                  index, raw);
            continue;
        }

        const UInt32 address = raw & ~0xfU;
        setProperty(typeProperties[index], type == 0x1U ? "MemoryBelow1MB" : "Memory32");
        setProperty(addressProperties[index], address, 32);
        IOLog("RTL8821CEProbe: BAR%u %s memory address 0x%08x (raw 0x%08x)\n",
              index, type == 0x1U ? "below-1MB" : "32-bit", address, raw);
    }

    const UInt32 bar2 = bars[2];
    const UInt32 bar2Type = (bar2 >> 1) & 0x3U;
    if (bar2 == 0 || bar2 == 0xffffffffU || (bar2 & 0x1U) != 0 || bar2Type != 0x2U) {
        IOLog("RTL8821CEProbe: refusing invalid BAR2 MMIO encoding 0x%08x\n", bar2);
        super::stop(provider);
        return false;
    }

    UInt8 powerCapability = 0;
    if (device->findPCICapability(kIOPCIPowerManagementCapability, &powerCapability) != 0) {
        const UInt16 powerStatus = device->configRead16(powerCapability + 4);
        setProperty("PCIPowerState", powerStatus & kPCIPMCSPowerStateMask, 8);
        if ((powerStatus & kPCIPMCSPowerStateMask) != kPCIPMCSPowerStateD0) {
            IOLog("RTL8821CEProbe: refusing MMIO while PCI function is not in D0 (PMCSR 0x%04x)\n",
                  powerStatus);
            super::stop(provider);
            return false;
        }
    }

    if (!device->open(this)) {
        IOLog("RTL8821CEProbe: could not exclusively open PCI provider\n");
        super::stop(provider);
        return false;
    }

    command = device->configRead16(kIOPCIConfigCommand);
    setProperty("PCICommand", command, 16);
    if ((command & kIOPCICommandBusMaster) != 0) {
        IOLog("RTL8821CEProbe: bus mastering became active after provider open (command 0x%04x)\n",
              command);
        device->close(this);
        super::stop(provider);
        return false;
    }

    if (powerCapability != 0) {
        const UInt16 powerStatus = device->configRead16(powerCapability + 4);
        if ((powerStatus & kPCIPMCSPowerStateMask) != kPCIPMCSPowerStateD0) {
            IOLog("RTL8821CEProbe: PCI function left D0 after provider open (PMCSR 0x%04x)\n",
                  powerStatus);
            device->close(this);
            super::stop(provider);
            return false;
        }
    }

    UInt8 pcieCapability = 0;
    const UInt32 pcieHeader = device->findPCICapability(
        kIOPCIPCIExpressCapability, &pcieCapability);
    if (pcieHeader == 0 || pcieCapability < 0x40) {
        IOLog("RTL8821CEProbe: PCI Express capability not found\n");
        device->close(this);
        super::stop(provider);
        return false;
    }

    const UInt16 pcieCapabilities = device->configRead16(pcieCapability + 2);
    const UInt8 pcieDeviceType = static_cast<UInt8>((pcieCapabilities >> 4) & 0xfU);
    if (pcieDeviceType != 0) {
        IOLog("RTL8821CEProbe: refusing non-native-endpoint PCIe capability type %u\n",
              pcieDeviceType);
        device->close(this);
        super::stop(provider);
        return false;
    }

    const UInt32 deviceCapabilities = device->configRead32(pcieCapability + 4);
    const UInt16 deviceControl = device->configRead16(pcieCapability + 8);
    const UInt16 deviceStatus = device->configRead16(pcieCapability + 10);
    const UInt32 linkCapabilities = device->configRead32(pcieCapability + 12);
    const UInt16 linkControl = device->configRead16(pcieCapability + 16);
    const UInt16 linkStatus = device->configRead16(pcieCapability + 18);
    const bool functionLevelResetSupported = (deviceCapabilities & (1U << 28)) != 0;
    const UInt8 negotiatedLinkSpeed = static_cast<UInt8>(linkStatus & 0xfU);
    const UInt8 negotiatedLinkWidth = static_cast<UInt8>((linkStatus >> 4) & 0x3fU);

    setProperty("PCIBusNumber", device->getBusNumber(), 8);
    setProperty("PCIDeviceNumber", device->getDeviceNumber(), 8);
    setProperty("PCIFunctionNumber", device->getFunctionNumber(), 8);
    setProperty("PCIExpressCapabilityOffset", pcieCapability, 8);
    setProperty("PCIExpressCapabilities", pcieCapabilities, 16);
    setProperty("PCIExpressDeviceType", pcieDeviceType, 8);
    setProperty("PCIExpressDeviceCapabilities", deviceCapabilities, 32);
    setProperty("PCIExpressDeviceControl", deviceControl, 16);
    setProperty("PCIExpressDeviceStatus", deviceStatus, 16);
    setProperty("PCIExpressLinkCapabilities", linkCapabilities, 32);
    setProperty("PCIExpressLinkControl", linkControl, 16);
    setProperty("PCIExpressLinkStatus", linkStatus, 16);
    setProperty("PCIExpressNegotiatedLinkSpeed", negotiatedLinkSpeed, 8);
    setProperty("PCIExpressNegotiatedLinkWidth", negotiatedLinkWidth, 8);
    setProperty("FunctionLevelResetSupported", functionLevelResetSupported);
    setProperty("FunctionLevelResetAttempted", false);

    if (powerCapability != 0) {
        const UInt16 powerCapabilities = device->configRead16(powerCapability + 2);
        const UInt16 powerControlStatus = device->configRead16(powerCapability + 4);
        const bool noSoftReset = (powerControlStatus & (1U << 3)) != 0;

        setProperty("PCIPowerCapabilityOffset", powerCapability, 8);
        setProperty("PCIPowerCapabilities", powerCapabilities, 16);
        setProperty("PCIPowerControlStatus", powerControlStatus, 16);
        setProperty("PCIPowerNoSoftReset", noSoftReset);
        setProperty("D3HotToD0ResetAvailable", !noSoftReset);
    }

    setProperty("ResetContainmentPolicy", functionLevelResetSupported ?
                "FLRAdvertisedButNotAttempted" :
                "ColdPowerRemovalRequiredAfterPartialPowerTransition");
    setProperty("SecondaryBusResetAttempted", false);

    UInt32 experimentEnable = 0;
    UInt32 experimentConfirmation = 0;
    const bool experimentEnablePresent = PE_parse_boot_argn(
        "rtl8821ce-power-experiment", &experimentEnable, sizeof(experimentEnable));
    const bool experimentConfirmationPresent = PE_parse_boot_argn(
        "rtl8821ce-power-confirm", &experimentConfirmation,
        sizeof(experimentConfirmation));
    const bool experimentRequested = experimentEnablePresent && experimentEnable == 1 &&
                                 experimentConfirmationPresent &&
                                 experimentConfirmation == kPowerExperimentConfirmation;
    UInt32 ocTestMarker = 0;
    UInt32 executorEnable = 0;
    UInt32 executorConfirmation = 0;
    const bool ocTestMarkerPresent = PE_parse_boot_argn(
        "oc-test", &ocTestMarker, sizeof(ocTestMarker));
    const bool executorEnablePresent = PE_parse_boot_argn(
        "rtl8821ce-power-v2", &executorEnable, sizeof(executorEnable));
    const bool executorConfirmationPresent = PE_parse_boot_argn(
        "rtl8821ce-power-v2-confirm", &executorConfirmation,
        sizeof(executorConfirmation));
    const bool powerExecutorRequested = ocTestMarkerPresent && ocTestMarker == 1 &&
        executorEnablePresent && executorEnable == 1 &&
        executorConfirmationPresent &&
        executorConfirmation == kPowerExecutorContractConfirmation;
    const bool powerExecutorArmingConflict =
        experimentRequested && powerExecutorRequested;
    const bool experimentArmed = kPowerExperimentBuildEnabled &&
        experimentRequested && !powerExecutorArmingConflict;
    const bool powerExecutorArmed = kPowerExecutorBuildEnabled &&
        powerExecutorRequested && !powerExecutorArmingConflict;

    setProperty("PowerExperimentEnableArgumentPresent", experimentEnablePresent);
    setProperty("PowerExperimentConfirmationArgumentPresent",
                experimentConfirmationPresent);
    setProperty("PowerExperimentRequested", experimentRequested);
    setProperty("PowerExperimentBuildEnabled", kPowerExperimentBuildEnabled);
    setProperty("PowerExperimentArmed", experimentArmed);
    setProperty("PowerExperimentOneShot", true);
    setProperty("PowerExperimentAutomaticRecovery", false);
    setProperty("PowerExperimentFailureRecovery", "ColdPowerRemoval");
    setProperty("PowerExecutorV2OCMarkerPresent", ocTestMarkerPresent);
    setProperty("PowerExecutorV2EnableArgumentPresent", executorEnablePresent);
    setProperty("PowerExecutorV2ConfirmationArgumentPresent",
                executorConfirmationPresent);
    setProperty("PowerExecutorV2Requested", powerExecutorRequested);
    setProperty("PowerExecutorV2BuildEnabled", kPowerExecutorBuildEnabled);
    setProperty("PowerExecutorV2Armed", powerExecutorArmed);
    setProperty("PowerExecutorV2ArmingConflict", powerExecutorArmingConflict);
    setProperty("PowerExecutorV2RequiresOCMarker", true);
    setProperty("PowerExecutorV2OneShot", true);
    setProperty("PowerExecutorV2RetryAuthorized", false);
    setProperty("PowerExecutorV2PowerOffRecoveryAuthorized", false);

    const UInt64 firmwareSectionSize = static_cast<UInt64>(
        firmwareSectionEnd - firmwareSectionStart);
    FirmwareHeader firmwareHeader = {};
    FirmwareTransferPlan firmwareTransferPlan[kFirmwareTransferPlanCapacity] = {};
    FirmwareIDDMAPlanRecord firmwareIDDMAPlan[kFirmwareTransferPlanCapacity] = {};
    size_t firmwareTransferPlanCount = 0;
    size_t firmwareIDDMAPlanCount = 0;
    UInt32 dmemTransferSize = 0;
    UInt32 imemTransferSize = 0;
    UInt32 ememTransferSize = 0;
    bool firmwareManifestValid = firmwareSectionSize == kExpectedFirmwareSize;
    if (firmwareManifestValid) {
        memcpy(&firmwareHeader, firmwareSectionStart, sizeof(firmwareHeader));
        firmwareManifestValid =
            OSSwapLittleToHostInt16(firmwareHeader.signature) == 0x8821U &&
            (firmwareHeader.memoryUsage & 0x10U) != 0 &&
            (OSSwapLittleToHostInt32(firmwareHeader.dmemAddress) & 0x7fffffffU) ==
                0x00200000U &&
            (OSSwapLittleToHostInt32(firmwareHeader.imemAddress) & 0x7fffffffU) ==
                0x00030000U &&
            (OSSwapLittleToHostInt32(firmwareHeader.ememAddress) & 0x7fffffffU) ==
                0x00100000U &&
            buildFirmwareTransferPlan(
                firmwareHeader, static_cast<UInt32>(firmwareSectionSize),
                firmwareTransferPlan, kFirmwareTransferPlanCapacity,
                firmwareTransferPlanCount, dmemTransferSize,
                imemTransferSize, ememTransferSize);
    }
    const bool firmwareIDDMAPlanValid = firmwareManifestValid &&
        buildFirmwareIDDMAPlan(
            firmwareTransferPlan, firmwareTransferPlanCount,
            firmwareIDDMAPlan, kFirmwareTransferPlanCapacity,
            firmwareIDDMAPlanCount);
    const bool lifecyclePlanValid = validateLifecyclePlans();
    const bool postPowerPlanValid = validatePostPowerPlan();
    const bool firmwareSetupPlanValid = validateFirmwareSetupPlan();
    const bool reservedPagePlanValid = validateReservedPagePlan();
    const bool finalFIFOPlanValid = validateFinalFIFOPlan();
    const EfuseParserValidationResult efuseParserValidation =
        validateEfuseParserModel();
    const bool efuseParserModelValid =
        efuseParserValidation.oneByteHeader &&
        efuseParserValidation.twoByteHeader &&
        efuseParserValidation.logicalBoundsRejection &&
        efuseParserValidation.physicalTruncationRejection &&
        efuseParserValidation.extendedHeaderTruncationRejection;
    const bool interruptPlanValid = validateInterruptPlan();
    const bool dmaStateMachineValid = validateDMAStateMachine();
    const bool dmaPublicationPlanValid = validateDMAPublicationPlan();
    const bool rollbackPoliciesValid = validateRollbackPolicies();
    const bool serializedCommandContractsValid =
        validateSerializedCommandContracts();
    const bool executionJournalContractsValid =
        validateExecutionJournalContracts();
    const bool mappingExposureLedgerValid = validateMappingExposureLedger();
    const bool synchronizationGenerationContractsValid =
        validateSynchronizationGenerationContracts();
    FailureInjectionSummary failureInjectionSummary = {};
    const bool failureBoundaryClassificationValid =
        validateFailureInjectionModel(failureInjectionSummary);
    SymbolicInterpreterSummary symbolicInterpreterSummary = {};
    const bool symbolicInterpreterValid =
        validateSymbolicInterpreter(symbolicInterpreterSummary);
    PowerOnlyStepRecord powerOnlyContract[kPowerOnlyStepCount] = {};
    size_t powerOnlyContractCount = 0;
    UInt16 powerOnlyWriteCount = 0;
    UInt16 powerOnlyPollCount = 0;
    const bool powerOnlyContractValid = buildPowerOnlyContract(
        powerOnlyContract, kPowerOnlyStepCount, powerOnlyContractCount) &&
        validatePowerOnlyContract(powerOnlyContract, powerOnlyContractCount,
            0x10000, powerOnlyWriteCount, powerOnlyPollCount);
    PowerExecutorSimulationSummary powerExecutorSimulation = {};
    const bool powerExecutorSimulationValid = powerOnlyContractValid &&
        simulatePowerOnlyExecutor(powerOnlyContract, powerOnlyContractCount,
            powerExecutorSimulation);

    setProperty("FirmwareEmbedded", firmwareSectionSize != 0);
    setProperty("FirmwareManifestValid", firmwareManifestValid);
    setProperty("FirmwareSectionSize", firmwareSectionSize, 64);
    setProperty("FirmwareExpectedSize",
                static_cast<UInt64>(kExpectedFirmwareSize), 32);
    setProperty("FirmwareSignature",
                OSSwapLittleToHostInt16(firmwareHeader.signature), 16);
    setProperty("FirmwareVersion",
                OSSwapLittleToHostInt16(firmwareHeader.version), 16);
    setProperty("FirmwareSubversion", firmwareHeader.subversion, 8);
    setProperty("FirmwareSubindex", firmwareHeader.subindex, 8);
    setProperty("FirmwareFeature",
                OSSwapLittleToHostInt32(firmwareHeader.feature), 32);
    setProperty("FirmwareH2CFormatVersion",
                OSSwapLittleToHostInt16(firmwareHeader.h2cFormatVersion), 16);
    setProperty("FirmwareDMEMAddress",
                OSSwapLittleToHostInt32(firmwareHeader.dmemAddress) & 0x7fffffffU,
                32);
    setProperty("FirmwareIMEMAddress",
                OSSwapLittleToHostInt32(firmwareHeader.imemAddress) & 0x7fffffffU,
                32);
    setProperty("FirmwareEMEMAddress",
                OSSwapLittleToHostInt32(firmwareHeader.ememAddress) & 0x7fffffffU,
                32);
    setProperty("FirmwareDMEMTransferSize", dmemTransferSize, 32);
    setProperty("FirmwareIMEMTransferSize", imemTransferSize, 32);
    setProperty("FirmwareEMEMTransferSize", ememTransferSize, 32);
    setProperty("FirmwareTransferChunkSize",
                static_cast<UInt64>(kFirmwareChunkSize), 32);
    setProperty("FirmwareTransferChunkCount",
                static_cast<UInt64>(firmwareTransferPlanCount), 8);
    setProperty("FirmwareTransport", "PCIeBeaconQueue+IDDMA");
    setProperty("FirmwareTransportPlanned", firmwareIDDMAPlanValid);
    setProperty("FirmwareTransportReady", false);
    setProperty("FirmwareTransportBlocker", "NoIOKitCoherentBeaconTXRing");
    setProperty("FirmwareRequiresBusMastering", true);
    setProperty("FirmwareRequiresBeaconTXQueue", true);
    setProperty("FirmwareRequiresIDDMA", true);
    setProperty("CanonicalLifecyclePlanValid", lifecyclePlanValid);
    setProperty("CanonicalLifecyclePlanRecordCount",
                static_cast<UInt64>(kLifecyclePlanCount), 8);
    setProperty("CanonicalLifecycleCycleCount", static_cast<UInt64>(2), 8);
    setProperty("CanonicalLifecycleProbeFirmwareCycleRequired", true);
    setProperty("CanonicalLifecycleOperationalFirmwareCycleRequired", true);
    setProperty("CanonicalLifecycleExecutionAuthorized", false);
    setProperty("PostPowerSystemPlanValid", postPowerPlanValid);
    setProperty("PostPowerSystemPlanRecordCount",
                static_cast<UInt64>(kPostPowerPlanCount), 8);
    setProperty("PostPowerSystemExecutionAuthorized", false);
    setProperty("FirmwareTemporarySetupPlanValid", firmwareSetupPlanValid);
    setProperty("FirmwareTemporarySetupPlanRecordCount",
                static_cast<UInt64>(kFirmwareSetupPlanCount), 8);
    setProperty("FirmwareTemporarySetupRestoreRecordCount",
                static_cast<UInt64>(6), 8);
    setProperty("FirmwareTemporarySetupFailureCleanupRecordCount",
                static_cast<UInt64>(2), 8);
    setProperty("FirmwareTemporarySetupExecutionAuthorized", false);
    setProperty("RegisterPlanSchemaVersion", kRegisterPlanSchemaVersion, 8);
    setProperty("ReservedPageTransactionPlanValid", reservedPagePlanValid);
    setProperty("ReservedPageTransactionPlanRecordCount",
                static_cast<UInt64>(kReservedPagePlanCount), 8);
    setProperty("ReservedPageTransactionCount",
                static_cast<UInt64>(firmwareIDDMAPlanCount), 8);
    setProperty("ReservedPageBeaconValidPollCount",
                static_cast<UInt64>(firmwareIDDMAPlanCount), 8);
    setProperty("ReservedPageRestoreWriteCount",
                static_cast<UInt64>(firmwareIDDMAPlanCount) * 4, 16);
    setProperty("ReservedPageExecutionAuthorized", false);
    setProperty("FinalFIFOPlanValid", finalFIFOPlanValid);
    setProperty("FinalFIFOPlanRecordCount",
                static_cast<UInt64>(kFinalFIFOPlanCount), 8);
    setProperty("FinalFIFOExecutionAuthorized", false);
    setProperty("FinalFIFOTXPageCount", kFIFOPlanSummary.txPages, 16);
    setProperty("FinalFIFOReservedPageCount",
                kFIFOPlanSummary.reservedPages, 16);
    setProperty("FinalFIFOACQueuePageCount",
                kFIFOPlanSummary.acQueuePages, 16);
    setProperty("FinalFIFOReservedBoundary",
                kFIFOPlanSummary.reservedBoundary, 16);
    setProperty("FinalFIFOPublicQueuePageCount",
                kFIFOPlanSummary.publicQueuePages, 16);
    setProperty("FinalFIFOH2CByteAddress",
                kFIFOPlanSummary.h2cByteAddress, 32);
    setProperty("FinalFIFOH2CByteEnd", kFIFOPlanSummary.h2cByteEnd, 32);
    setProperty("EfuseParserModelValid", efuseParserModelValid);
    setProperty("EfuseParserModelBoundsCorrectedFromUpstream", true);
    setProperty("EfusePhysicalSize",
                kEfuseDependencyPlan.physicalSize, 16);
    setProperty("EfuseLogicalSize", kEfuseDependencyPlan.logicalSize, 16);
    setProperty("EfuseProtectSize", kEfuseDependencyPlan.protectSize, 16);
    setProperty("EfuseReadableSize", kEfuseDependencyPlan.readableSize, 16);
    setProperty("EfuseSyntheticOneByteHeaderValid",
                efuseParserValidation.oneByteHeader);
    setProperty("EfuseSyntheticTwoByteHeaderValid",
                efuseParserValidation.twoByteHeader);
    setProperty("EfuseSyntheticLogicalBoundsRejectionValid",
                efuseParserValidation.logicalBoundsRejection);
    setProperty("EfuseSyntheticPhysicalTruncationRejectionValid",
                efuseParserValidation.physicalTruncationRejection);
    setProperty("EfuseSyntheticExtendedHeaderTruncationRejectionValid",
                efuseParserValidation.extendedHeaderTruncationRejection);
    setProperty("EfuseHardwareReadAttempted", false);
    setProperty("EfuseExecutionAuthorized", false);
    setProperty("InterruptLifecyclePlanValid", interruptPlanValid);
    setProperty("InterruptLifecyclePlanRecordCount",
                static_cast<UInt64>(kInterruptPlanCount), 8);
    setProperty("InterruptLifecycleRequiresFirmwareRunning", true);
    setProperty("InterruptLifecycleRequiresRXDeviceOwnership", true);
    setProperty("InterruptLifecycleRequiresNAPIEquivalent", true);
    setProperty("InterruptLifecycleExecutionAuthorized", false);
    setProperty("DMAStatePlanSchemaVersion", kDMAStatePlanSchemaVersion, 8);
    setProperty("DMAStateMachineValid", dmaStateMachineValid);
    setProperty("DMAStateCount", static_cast<UInt64>(kDMAStateCount), 8);
    setProperty("DMATransitionCount",
                static_cast<UInt64>(kDMATransitionCount), 8);
    setProperty("DMAStateMachineExecutionAuthorized", false);
    setProperty("DMAStateMachineExecutionAttempted", false);
    setProperty("DMAPublicationPlanValid", dmaPublicationPlanValid);
    setProperty("DMAPublicationPlanRecordCount",
                static_cast<UInt64>(kDMAPublicationPlanCount), 8);
    setProperty("DMAPublicationUsesIODMACommandSynchronize", true);
    setProperty("DMAPublicationSynchronizeIsMemoryFence", false);
    setProperty("DMAPublicationRequiresExplicitReleaseFence", true);
    setProperty("DMAReclaimRequiresExplicitAcquireFence", true);
    setProperty("DMAPublicationRequiresMMIOReadback", true);
    setProperty("DMAPublicationSynchronizeRange", "EntirePreparedCommandRange");
    setProperty("DMAPublicationTXOwnershipRule", "OwnershipLast");
    setProperty("DMAPublicationRXOwnershipRule", "NoPerDescriptorOWN");
    setProperty("DMAPublicationExecutionAuthorized", false);
    setProperty("DMAPublicationExecutionAttempted", false);
    setProperty("RollbackPolicyValid", rollbackPoliciesValid);
    setProperty("RollbackPolicyRecordCount",
                static_cast<UInt64>(kRollbackPolicyCount), 8);
    setProperty("RollbackClearBMEIsContainmentOnly", true);
    setProperty("RollbackHardwareResetAvailable", false);
    setProperty("RollbackDeviceVisibleMappingsReleaseAuthorized", false);
    setProperty("RollbackPartialPowerAllowsPowerOff", false);
    setProperty("RollbackPartialFirmwareAllowsRetry", false);
    setProperty("RollbackQuarantineTerminalUntilColdPowerRemoval", true);
    setProperty("ExecutionSafetySchemaVersion", kExecutionSafetySchemaVersion, 8);
    setProperty("SerializedCommandContractValid",
                serializedCommandContractsValid);
    setProperty("SerializedCommandContractRecordCount",
                static_cast<UInt64>(kSerializedCommandContractCount), 8);
    setProperty("SerializedCommandGateRequired", true);
    setProperty("SerializedCommandGateInstantiated", false);
    setProperty("SerializedCommandGateRejectsReentry", true);
    setProperty("SerializedCommandExecutionAuthorized", false);
    setProperty("SerializedCommandExecutionAttempted", false);
    setProperty("ExecutionJournalContractValid",
                executionJournalContractsValid);
    setProperty("ExecutionJournalContractRecordCount",
                static_cast<UInt64>(kExecutionJournalContractCount), 8);
    setProperty("ExecutionJournalWriteAheadRequired", true);
    setProperty("ExecutionJournalPersistentAcrossReboot", false);
    setProperty("ExecutionJournalUnknownEffectPreservesHazards", true);
    setProperty("ExecutionHazardsIndependentBitVector", true);
    setProperty("MappingExposureLedgerValid", mappingExposureLedgerValid);
    setProperty("MappingExposureClassCount",
                static_cast<UInt64>(kMappingExposureClassCount), 8);
    setProperty("MappingExposureTransitionCount",
                static_cast<UInt64>(kMappingExposureTransitionCount), 8);
    setProperty("MappingExposureLedgerMappingCount",
                static_cast<UInt64>(kRetainedMappingCount), 32);
    setProperty("MappingExposureGenerationRequired", true);
    setProperty("MappingExposureReleaseRequiresEvidence", true);
    setProperty("MappingExposureBMEClearIsReleaseEvidence", false);
    setProperty("SynchronizationGenerationContractValid",
                synchronizationGenerationContractsValid);
    setProperty("SynchronizationGenerationContractRecordCount",
                static_cast<UInt64>(kSynchronizationGenerationContractCount), 8);
    setProperty("FailureBoundaryClassificationValid",
                failureBoundaryClassificationValid);
    setProperty("FailureBoundaryClassificationCount",
                failureInjectionSummary.totalBoundaries, 16);
    setProperty("FailureBoundaryOutcomeCount",
                failureInjectionSummary.scenarioCount, 32);
    setProperty("FailureBoundaryRejectedUnsafeOutcomeCount",
                failureInjectionSummary.rejectedUnsafeScenarioCount, 32);
    setProperty("FailureBoundaryClassificationUsesInterpreter", false);
    setProperty("FailureBoundaryClassificationHardwareExecution", false);
    setProperty("SymbolicInterpreterValid", symbolicInterpreterValid);
    setProperty("SymbolicInterpreterSchemaVersion",
                kExecutionSafetySchemaVersion, 8);
    setProperty("SymbolicInterpreterTraceCount",
                symbolicInterpreterSummary.totalTraceCount, 32);
    setProperty("SymbolicInterpreterLifecycleTraceCount",
                symbolicInterpreterSummary.lifecycleTraceCount, 32);
    setProperty("SymbolicInterpreterTransitionTraceCount",
                symbolicInterpreterSummary.transitionTraceCount, 32);
    setProperty("SymbolicInterpreterPublicationTraceCount",
                symbolicInterpreterSummary.publicationTraceCount, 32);
    setProperty("SymbolicInterpreterContainmentTraceCount",
                symbolicInterpreterSummary.containmentTraceCount, 32);
    setProperty("SymbolicInterpreterSuccessfulTraceCount",
                symbolicInterpreterSummary.successfulTraceCount, 32);
    setProperty("SymbolicInterpreterRejectedMutationCount",
                symbolicInterpreterSummary.rejectedMutationCount, 32);
    setProperty("SymbolicInterpreterExecutionAuthorized", false);
    setProperty("SymbolicInterpreterHardwareExecution", false);

    DMAProbeResult descriptorDMA = {};
    DMAProbeResult stagingDMA = {};
    const bool descriptorDMAValid = probeSingleSegmentDMA(
        kBeaconDescriptorRingSize, descriptorDMA);
    const bool stagingDMAValid = probeSingleSegmentDMA(
        kFirmwareStagingPacketSize, stagingDMA);
    const bool dmaFeasibilityValid = descriptorDMAValid && stagingDMAValid;

    setProperty("DMAFeasibilityProbeAttempted", true);
    setProperty("DMAFeasibilityProbeComplete", dmaFeasibilityValid);
    setProperty("DMAFeasibilityAddressBits", static_cast<UInt64>(32), 8);
    setProperty("DMAFeasibilityAlignment", static_cast<UInt64>(kDMAAlignment), 32);
    setProperty("DMAFeasibilitySingleSegmentRequired", true);
    setProperty("DMAFeasibilityBuffersZeroFilled",
                descriptorDMA.zeroFilled && stagingDMA.zeroFilled);
    setProperty("DMAFeasibilityResourcesRetained", false);
    setProperty("DMAFeasibilityAddressesTransient", true);
    setProperty("DMADescriptorRequestedLength", descriptorDMA.requestedLength, 32);
    setProperty("DMADescriptorProbeStage", dmaProbeStageName(descriptorDMA.stage));
    setProperty("DMADescriptorProbeStatus",
                static_cast<UInt64>(descriptorDMA.status), 32);
    setProperty("DMADescriptorSegmentCount", descriptorDMA.segmentCount, 32);
    setProperty("DMADescriptorTransientAddress", descriptorDMA.address, 32);
    setProperty("DMADescriptorSegmentLength", descriptorDMA.length, 32);
    setProperty("DMAStagingRequestedLength", stagingDMA.requestedLength, 32);
    setProperty("DMAStagingProbeStage", dmaProbeStageName(stagingDMA.stage));
    setProperty("DMAStagingProbeStatus",
                static_cast<UInt64>(stagingDMA.status), 32);
    setProperty("DMAStagingSegmentCount", stagingDMA.segmentCount, 32);
    setProperty("DMAStagingTransientAddress", stagingDMA.address, 32);
    setProperty("DMAStagingSegmentLength", stagingDMA.length, 32);
    setProperty("DMAAddressWrittenToDevice", false);
    setProperty("PCIBusMasteringEnabled", false);

    DMATemplateResult dmaTemplate = {};
    PreparedDMA persistentDescriptor = {};
    PreparedDMA persistentStaging = {};
    const bool dmaTemplateValid = firmwareManifestValid &&
        prepareDMATemplate(firmwareSectionStart + kFirmwareHeaderSize,
                           dmaTemplate, persistentDescriptor,
                           persistentStaging);
    if (dmaTemplateValid) {
        descriptorBuffer_ = persistentDescriptor.buffer;
        descriptorCommand_ = persistentDescriptor.command;
        descriptorSet_ = persistentDescriptor.descriptorSet;
        descriptorPrepared_ = persistentDescriptor.prepared;
        stagingBuffer_ = persistentStaging.buffer;
        stagingCommand_ = persistentStaging.command;
        stagingSet_ = persistentStaging.descriptorSet;
        stagingPrepared_ = persistentStaging.prepared;
        persistentDescriptor = {};
        persistentStaging = {};
    } else {
        releasePreparedDMA(persistentStaging);
        releasePreparedDMA(persistentDescriptor);
    }
    setProperty("DMATemplateValidationAttempted", firmwareManifestValid);
    setProperty("DMATemplateValidationComplete", dmaTemplateValid);
    setProperty("DMATemplateStatus",
                static_cast<UInt64>(dmaTemplate.status), 32);
    setProperty("DMATemplateDescriptorAddress", dmaTemplate.descriptorAddress, 32);
    setProperty("DMATemplateStagingAddress", dmaTemplate.stagingAddress, 32);
    setProperty("DMATemplatePacketBufferSize", dmaTemplate.packetBufferSize, 16);
    setProperty("DMATemplatePacketPSBLength", dmaTemplate.packetPSBLength, 16);
    setProperty("DMATemplatePacketAddress", dmaTemplate.packetAddress, 32);
    setProperty("DMATemplatePayloadBufferSize", dmaTemplate.payloadBufferSize, 16);
    setProperty("DMATemplatePayloadAddress", dmaTemplate.payloadAddress, 32);
    setProperty("DMATemplateTXWord0", dmaTemplate.txWord0, 32);
    setProperty("DMATemplateTXWord1", dmaTemplate.txWord1, 32);
    setProperty("DMATemplateTXWord3", dmaTemplate.txWord3, 32);
    setProperty("DMATemplateTXWord8", dmaTemplate.txWord8, 32);
    setProperty("DMATemplateTXChecksum", dmaTemplate.txChecksum, 16);
    setProperty("DMATemplateRoundTripValid", dmaTemplate.roundTripValid);
    setProperty("DMATemplatePayloadValid", dmaTemplate.payloadValid);
    setProperty("DMATemplateResourcesReleased", false);
    setProperty("DMATemplateDeviceVisible", false);
    setProperty("DMATemplateQueueDoorbellWritten", false);
    setProperty("PersistentDMAAllocated", dmaTemplateValid);
    setProperty("PersistentDMAPrepared", dmaTemplateValid &&
                descriptorPrepared_ && stagingPrepared_);
    setProperty("PersistentDMADeviceVisible", false);
    setProperty("PersistentDMAReleasePoint", "ServiceStopOrFailedStart");
    if (dmaTemplateValid)
        setProperty("FirmwareTransportBlocker", "PersistentRingNotDeviceConfigured");

    IOMemoryMap *map = device->mapDeviceMemoryWithRegister(
        kIOPCIConfigBaseAddress2,
        kIOMapInhibitCache |
            ((experimentArmed || powerExecutorArmed) ? 0 : kIOMapReadOnly));
    bool mmioValid = false;
    bool sequenceValidated = false;
    UInt32 sysCfg1 = 0xffffffffU;
    UInt8 cr = 0xffU;
    UInt8 plannedWriteCount = 0;
    RegisterSnapshot snapshots[kProjectedRegisterCount] = {};
    ProjectedRegister projectedRegisters[kProjectedRegisterCount] = {};
    DryRunCommand dryRunPlan[kPowerCommandCount] = {};
    PollBaselineSample pollBaseline[kBaselineSampleCount] = {};
    size_t dryRunPlanCount = 0;
    bool dryRunPlanValidated = false;
    UInt8 pollReadyHitCount = 0;
    UInt8 powerOnDoneHitCount = 0;
    UInt8 powerOffDoneHitCount = 0;
    UInt8 pollBaselineTransitionCount = 0;
    UInt8 pollBaselineSysPowerMinimum = 0xffU;
    UInt8 pollBaselineSysPowerMaximum = 0;
    UInt8 pollBaselineControlMinimum = 0xffU;
    UInt8 pollBaselineControlMaximum = 0;
    ExperimentResult experimentResult = ExperimentResult::Disarmed;
    UInt8 experimentCompletedWrites = 0;
    UInt8 experimentCompletedPolls = 0;
    UInt8 experimentFailedCommand = 0xffU;
    UInt8 experimentMacAfter = 0xffU;
    PowerExecutorResult powerExecutorResult = PowerExecutorResult::Disarmed;
    PowerExecutionJournalRecord
        powerExecutionJournal[kPowerExecutionJournalCapacity] = {};
    size_t powerExecutionJournalCount = 0;
    UInt8 powerExecutorCompletedWrites = 0;
    UInt8 powerExecutorCompletedPolls = 0;
    UInt8 powerExecutorFailedStep = 0xffU;
    UInt8 powerExecutorMacAfter = 0xffU;
    PreSystemSnapshot preSystem = {};
    bool preSystemStable = false;
    QueueRegisterSnapshot queueSnapshot = {};
    QueueProjectedCommand queuePlan[4] = {};
    bool queueSnapshotStable = false;
    bool queuePlanValid = false;
    TRXAllocationProbeResult trxAllocation = {};
    TRXDevicePlanRecord trxDevicePlan[kTRXDevicePlanCapacity] = {};
    size_t trxDevicePlanCount = 0;
    UInt8 trxDevicePlanBaseCount = 0;
    UInt8 trxDevicePlanEntryCount = 0;
    UInt32 retainedBeaconRingAddress = 0;
    bool retainedBeaconRingValid = false;
    bool trxDevicePlanValid = false;
    FirmwareRegisterBaseline
        firmwareRegisterBaseline[kFirmwareRegisterBaselineCount] = {};
    UInt8 firmwareRegisterBaselineStableCount = 0;
    UInt16 trxAllocationPCICommandBefore = 0xffffU;
    UInt16 trxAllocationPCICommandAfter = 0xffffU;

    if (!map) {
        IOLog("RTL8821CEProbe: failed to map BAR2\n");
    } else {
        const IOVirtualAddress address = map->getVirtualAddress();
        const IOByteCount length = map->getLength();
        setProperty("BAR2MappedLength", static_cast<UInt64>(length), 64);

        if (address == 0 || length != 0x10000 || 0x03d9 >= length ||
            kRegisterCr >= length ||
            sizeof(UInt32) > length - kRegisterSysCfg1) {
            IOLog("RTL8821CEProbe: BAR2 mapping is too short or invalid (length 0x%llx)\n",
                  static_cast<UInt64>(length));
        } else {
            const UInt16 enabledCommand = static_cast<UInt16>(command | kIOPCICommandMemorySpace);
            device->configWrite16(kIOPCIConfigCommand, enabledCommand);
            const UInt16 activeCommand = device->configRead16(kIOPCIConfigCommand);
            setProperty("PCICommandDuringMMIO", activeCommand, 16);

            if ((activeCommand & kCommandDecodeMask) != kIOPCICommandMemorySpace) {
                IOLog("RTL8821CEProbe: unsafe PCI command after memory enable: 0x%04x\n",
                      activeCommand);
            } else {
                const volatile UInt8 *base = reinterpret_cast<const volatile UInt8 *>(address);
                sysCfg1 = OSReadLittleInt32(base, kRegisterSysCfg1);
                const UInt32 sysCfg1Repeat = OSReadLittleInt32(base, kRegisterSysCfg1);
                cr = base[kRegisterCr];
                IODelay(50);
                const UInt8 crRepeat = base[kRegisterCr];

                sequenceValidated = validatePowerSequence(length, kPowerOnSequence,
                                                          plannedWriteCount) &&
                                    validatePowerSequence(length, kPowerOffSequence,
                                                          plannedWriteCount) &&
                                    plannedWriteCount == 27;

                for (size_t index = 0;
                     index < kProjectedRegisterCount;
                     index++) {
                    snapshots[index] = {
                        kSnapshotOffsets[index], base[kSnapshotOffsets[index]], 0
                    };
                    projectedRegisters[index] = {
                        kSnapshotOffsets[index],
                        snapshots[index].value,
                        snapshots[index].value,
                        snapshots[index].value,
                        0,
                        0,
                        0,
                    };
                }

                if (sequenceValidated) {
                    dryRunPlanValidated = planPowerSequence(
                        kPowerOnSequence, projectedRegisters, kProjectedRegisterCount,
                        dryRunPlan, kPowerCommandCount, dryRunPlanCount, true);

                    for (size_t index = 0; index < kProjectedRegisterCount; index++)
                        projectedRegisters[index].afterPowerOff =
                            projectedRegisters[index].afterPowerOn;

                    dryRunPlanValidated = dryRunPlanValidated && planPowerSequence(
                        kPowerOffSequence, projectedRegisters, kProjectedRegisterCount,
                        dryRunPlan, kPowerCommandCount, dryRunPlanCount, false) &&
                        dryRunPlanCount == kPowerCommandCount;
                }

                for (UInt8 index = 0; index < kBaselineSampleCount; index++) {
                    const UInt8 sysPowerState = base[0x0006];
                    const UInt8 systemPowerControl = base[0x0005];
                    pollBaseline[index] = {sysPowerState, systemPowerControl};

                    if ((sysPowerState & 0x02U) == 0x02U)
                        pollReadyHitCount++;
                    if ((systemPowerControl & 0x01U) == 0)
                        powerOnDoneHitCount++;
                    if ((systemPowerControl & 0x02U) == 0)
                        powerOffDoneHitCount++;
                    if (index != 0 &&
                        (pollBaseline[index - 1].sysPowerState != sysPowerState ||
                         pollBaseline[index - 1].systemPowerControl != systemPowerControl))
                        pollBaselineTransitionCount++;

                    if (sysPowerState < pollBaselineSysPowerMinimum)
                        pollBaselineSysPowerMinimum = sysPowerState;
                    if (sysPowerState > pollBaselineSysPowerMaximum)
                        pollBaselineSysPowerMaximum = sysPowerState;
                    if (systemPowerControl < pollBaselineControlMinimum)
                        pollBaselineControlMinimum = systemPowerControl;
                    if (systemPowerControl > pollBaselineControlMaximum)
                        pollBaselineControlMaximum = systemPowerControl;

                    if (index + 1 < kBaselineSampleCount)
                        IODelay(kBaselineSampleIntervalMicroseconds);
                }

                preSystem = {
                    base[0x001c],
                    OSReadLittleInt32(base, 0x0074),
                    OSReadLittleInt32(base, 0x0064),
                    OSReadLittleInt32(base, 0x004c),
                    OSReadLittleInt32(base, 0x0040),
                    base[0x0002],
                    base[0x001f],
                    OSReadLittleInt32(base, 0x00ec),
                    base[0x0004],
                    OSReadLittleInt16(base, 0x0080),
                    base[0x03d9],
                };

                IODelay(50);
                const PreSystemSnapshot repeated = {
                    base[0x001c],
                    OSReadLittleInt32(base, 0x0074),
                    OSReadLittleInt32(base, 0x0064),
                    OSReadLittleInt32(base, 0x004c),
                    OSReadLittleInt32(base, 0x0040),
                    base[0x0002],
                    base[0x001f],
                    OSReadLittleInt32(base, 0x00ec),
                    base[0x0004],
                    OSReadLittleInt16(base, 0x0080),
                    base[0x03d9],
                };
                preSystemStable = equalPreSystemSnapshot(preSystem, repeated);

                queueSnapshot = {
                    base[kRegisterPCICtrl3],
                    base[kRegisterBeaconWork],
                    0,
                    OSReadLittleInt32(base, kRegisterBeaconRingBase),
                    OSReadLittleInt32(base, kRegisterRWPTRClear),
                };
                IODelay(50);
                const QueueRegisterSnapshot queueRepeated = {
                    base[kRegisterPCICtrl3],
                    base[kRegisterBeaconWork],
                    0,
                    OSReadLittleInt32(base, kRegisterBeaconRingBase),
                    OSReadLittleInt32(base, kRegisterRWPTRClear),
                };
                queueSnapshotStable =
                    queueSnapshot.pciControl3 == queueRepeated.pciControl3 &&
                    queueSnapshot.beaconWork == queueRepeated.beaconWork &&
                    queueSnapshot.beaconRingBase == queueRepeated.beaconRingBase &&
                    queueSnapshot.rwPointerClear == queueRepeated.rwPointerClear;

                memcpy(firmwareRegisterBaseline,
                       kFirmwareRegisterBaselineTemplate,
                       sizeof(firmwareRegisterBaseline));
                for (size_t index = 0;
                     index < kFirmwareRegisterBaselineCount; index++)
                    firmwareRegisterBaseline[index].first = readRegisterWidth(
                        base, firmwareRegisterBaseline[index].offset,
                        firmwareRegisterBaseline[index].width);
                IODelay(50);
                for (size_t index = 0;
                     index < kFirmwareRegisterBaselineCount; index++) {
                    FirmwareRegisterBaseline &baseline =
                        firmwareRegisterBaseline[index];
                    baseline.second = readRegisterWidth(
                        base, baseline.offset, baseline.width);
                    baseline.stable = baseline.first == baseline.second;
                    if (baseline.stable)
                        firmwareRegisterBaselineStableCount++;
                }

                const UInt32 projectedPCIControl3 =
                    queueSnapshot.pciControl3 | 0xf7U;
                const UInt32 projectedBeaconWork =
                    queueSnapshot.beaconWork | 0x10U;
                queuePlan[0] = {
                    0, 1, static_cast<UInt16>(kRegisterPCICtrl3),
                    queueSnapshot.pciControl3, 0x000000f7U, 0x000000f7U,
                    projectedPCIControl3,
                };
                queuePlan[1] = {
                    1, 4, static_cast<UInt16>(kRegisterBeaconRingBase),
                    queueSnapshot.beaconRingBase, 0xffffffffU,
                    dmaTemplate.descriptorAddress, dmaTemplate.descriptorAddress,
                };
                queuePlan[2] = {
                    2, 4, static_cast<UInt16>(kRegisterRWPTRClear),
                    queueSnapshot.rwPointerClear, 0xffffffffU, 0xffffffffU,
                    0xffffffffU,
                };
                queuePlan[3] = {
                    3, 1, static_cast<UInt16>(kRegisterBeaconWork),
                    queueSnapshot.beaconWork, 0x00000010U, 0x00000010U,
                    projectedBeaconWork,
                };
                queuePlanValid = queueSnapshotStable && dmaTemplateValid &&
                    dmaTemplate.descriptorAddress != 0 &&
                    (dmaTemplate.descriptorAddress & (kDMAAlignment - 1)) == 0;

                mmioValid = sequenceValidated && dryRunPlanValidated &&
                            sysCfg1 == sysCfg1Repeat &&
                            sysCfg1 == kExpectedSysCfg1 && cr == 0xeaU &&
                            crRepeat == cr && revision == 0 &&
                             subsystemVendor == 0x10ec && subsystem == 0xc821 &&
                             preSystemStable && queuePlanValid && command == 0 &&
                             firmwareRegisterBaselineStableCount ==
                                 kFirmwareRegisterBaselineCount &&
                            bars[2] == 0xfce00004U && bars[3] == 0 &&
                            preSystem.reservedControl == 0x00 &&
                            preSystem.hciOptionControl == 0x00000435 &&
                            preSystem.padControl1 == 0x06243000 &&
                            preSystem.ledConfig == 0x00628282 &&
                            preSystem.gpioMuxConfig == 0x00000000 &&
                            preSystem.systemFunctionEnable == 0xdc &&
                            preSystem.rfControl == 0x00 &&
                            preSystem.wlrf1 == 0x00000000 &&
                            preSystem.systemPowerControl == 0x12 &&
                            preSystem.firmwareControl == 0x0001 &&
                            preSystem.rpwm == 0x00;

                if (mmioValid && experimentArmed) {
                    volatile UInt8 *writableBase =
                        reinterpret_cast<volatile UInt8 *>(address);
                    experimentResult = executePowerOnExperiment(
                        writableBase, experimentCompletedWrites,
                        experimentCompletedPolls, experimentFailedCommand,
                        experimentMacAfter);
                }
                if (mmioValid && powerExecutorArmed &&
                    powerOnlyContractValid && powerExecutorSimulationValid &&
                    !experimentArmed) {
                    volatile UInt8 *writableBase =
                        reinterpret_cast<volatile UInt8 *>(address);
                    powerExecutorResult = executePowerOnlyContract(
                        writableBase, powerOnlyContract,
                        powerOnlyContractCount, powerExecutionJournal,
                        kPowerExecutionJournalCapacity,
                        powerExecutionJournalCount,
                        powerExecutorCompletedWrites,
                        powerExecutorCompletedPolls,
                        powerExecutorFailedStep, powerExecutorMacAfter);
                    powerExecutorAttempted_ = powerExecutionJournalCount != 0;
                    powerExecutorQuarantined_ = powerExecutorAttempted_;
                }

            }

            UInt16 restoredCommand = 0xffffU;
            for (UInt8 attempt = 0; attempt < 3; attempt++) {
                device->configWrite16(kIOPCIConfigCommand, command);
                restoredCommand = device->configRead16(kIOPCIConfigCommand);
                if (restoredCommand == command)
                    break;
                IODelay(50);
            }
            setProperty("PCICommandRestored", restoredCommand, 16);
            if (restoredCommand != command) {
                IOLog("RTL8821CEProbe: failed to restore PCI command (wanted 0x%04x, got 0x%04x)\n",
                      command, restoredCommand);
                mmioValid = false;
            }
        }

        map->release();
    }

    if (mmioValid && !powerExecutorQuarantined_) {
        trxAllocationPCICommandBefore =
            device->configRead16(kIOPCIConfigCommand);
        if ((trxAllocationPCICommandBefore & kIOPCICommandBusMaster) == 0) {
            PreparedDMA *retainedRings = nullptr;
            PreparedDMA *retainedPayloads = nullptr;
            preparePersistentTRXAllocation(
                trxAllocation, retainedRings, retainedPayloads);
            if (trxAllocation.allocationComplete) {
                trxRings_ = retainedRings;
                trxPayloads_ = retainedPayloads;
                trxRingCount_ = trxAllocation.ringMappingCount;
                trxPayloadCount_ = trxAllocation.payloadMappingCount;
                retainedBeaconRingValid = materializeRetainedBeaconRing(
                    retainedRings, trxRingCount_, descriptorBuffer_,
                    retainedBeaconRingAddress);
                trxDevicePlanValid = retainedBeaconRingValid &&
                    buildTRXDevicePlan(
                        retainedRings, trxRingCount_, trxDevicePlan,
                        kTRXDevicePlanCapacity, trxDevicePlanCount,
                        trxDevicePlanBaseCount, trxDevicePlanEntryCount);
                if (retainedBeaconRingValid) {
                    queuePlan[1].value = retainedBeaconRingAddress;
                    queuePlan[1].projected = retainedBeaconRingAddress;
                    queuePlanValid = queuePlanValid &&
                        retainedBeaconRingAddress != 0 &&
                        (retainedBeaconRingAddress &
                         (kDMAAlignment - 1)) == 0;
                }
            }
        }
        trxAllocationPCICommandAfter =
            device->configRead16(kIOPCIConfigCommand);
        if (trxAllocationPCICommandAfter != trxAllocationPCICommandBefore ||
            (trxAllocationPCICommandAfter & kIOPCICommandBusMaster) != 0)
            mmioValid = false;
        if (trxAllocation.allocationComplete &&
            (trxRingCount_ != kTRXResourceCount ||
             trxPayloadCount_ != kRXRingEntries ||
             !retainedBeaconRingValid || !trxDevicePlanValid ||
             !firmwareIDDMAPlanValid))
            mmioValid = false;
    }

    device->close(this);

    if (!mmioValid) {
        IOLog("RTL8821CEProbe: MMIO identity probe failed safely\n");
        releasePersistentDMA();
        super::stop(provider);
        return false;
    }

    setProperty("SYS_CFG1", sysCfg1, 32);
    setProperty("ChipCut", (sysCfg1 >> 12) & 0xfU, 8);
    setProperty("ChipVendorField", (sysCfg1 >> 16) & 0xfU, 8);
    setProperty("ProductionChip", (sysCfg1 & (1U << 23)) == 0);
    setProperty("LDOStrap", (sysCfg1 & (1U << 24)) != 0);
    setProperty("RFTypeStrap", (sysCfg1 & (1U << 27)) != 0 ? "2T2R" : "1T1R");
    setProperty("MACControl", cr, 8);
    setProperty("MACPowerState", cr == 0xeaU ? "Off" : "PreviouslyActiveOrUnknown");
    setProperty("MMIOProbeComplete", kOSBooleanTrue);
    setProperty("PowerCycleAttempted", experimentArmed || powerExecutorArmed);
    setProperty("PowerSequenceValidated", sequenceValidated);
    setProperty("PlannedPowerWriteCount", plannedWriteCount, 8);
    setProperty("PowerFSMDryRunComplete", dryRunPlanValidated);
    setProperty("PowerFSMExecutionAuthorized", experimentArmed);
    setProperty("PowerFSMExecutionReady", false);
    setProperty("PowerFSMExecutionBlocker", experimentArmed ?
                "ExplicitColdRecoveryRiskAccepted" :
                (experimentRequested ? "ExecutorCompileTimeDisabled" :
                 "MissingDualBootArgumentAuthorization"));
    setProperty("PowerFSMPhaseCount", static_cast<UInt64>(5), 8);
    setProperty("PowerFSMCommandCount", static_cast<UInt64>(dryRunPlanCount), 8);
    setProperty("PowerFSMOnCommandCount", static_cast<UInt64>(kPowerOnCommandCount), 8);
    setProperty("PowerFSMOffCommandCount", static_cast<UInt64>(kPowerOffCommandCount), 8);
    setProperty("PowerFSMPollCount", static_cast<UInt64>(3), 8);
    setProperty("PowerFSMPollIntervalMicroseconds",
                static_cast<UInt64>(kPowerPollIntervalMicroseconds), 32);
    setProperty("PowerFSMPollIterations",
                static_cast<UInt64>(kPowerPollIterations), 32);
    setProperty("PowerFSMPollMaximumAttempts",
                static_cast<UInt64>(kPowerPollMaximumAttempts), 8);
    setProperty("PowerFSMPollWorstCaseMicroseconds",
                static_cast<UInt64>(kPowerPollIntervalMicroseconds) *
                    kPowerPollIterations * kPowerPollMaximumAttempts * 3,
                64);
    setProperty("PowerFSMPollTimeoutRecovery", "ForbiddenInDryRun");
    setProperty("PowerFSMPollBaselineReadOnly", true);
    setProperty("PowerFSMPollBaselineSampleCount", kBaselineSampleCount, 8);
    setProperty("PowerFSMPollBaselineIntervalMicroseconds",
                static_cast<UInt64>(kBaselineSampleIntervalMicroseconds), 32);
    setProperty("PowerFSMPollBaselineTransitionCount", pollBaselineTransitionCount, 8);
    setProperty("PowerFSMPollReadyBaselineHitCount", pollReadyHitCount, 8);
    setProperty("PowerFSMPowerOnDoneBaselineHitCount", powerOnDoneHitCount, 8);
    setProperty("PowerFSMPowerOffDoneBaselineHitCount", powerOffDoneHitCount, 8);
    setProperty("PowerFSMPollBaselineAllTargetsObserved",
                pollReadyHitCount == kBaselineSampleCount &&
                    powerOnDoneHitCount == kBaselineSampleCount &&
                    powerOffDoneHitCount == kBaselineSampleCount);
    setProperty("PowerFSMPollBaselineStable", pollBaselineTransitionCount == 0);
    setProperty("PowerFSMPollBaselinePredictsPostWriteState", false);
    setProperty("PowerFSMPollBaselineSYS_PWR_STATE_Min", pollBaselineSysPowerMinimum, 8);
    setProperty("PowerFSMPollBaselineSYS_PWR_STATE_Max", pollBaselineSysPowerMaximum, 8);
    setProperty("PowerFSMPollBaselineSYS_PWR_CTRL_Min", pollBaselineControlMinimum, 8);
    setProperty("PowerFSMPollBaselineSYS_PWR_CTRL_Max", pollBaselineControlMaximum, 8);
    setProperty("PowerExperimentResult", experimentResultName(experimentResult));
    setProperty("PowerExperimentCompletedWriteCount", experimentCompletedWrites, 8);
    setProperty("PowerExperimentCompletedPollCount", experimentCompletedPolls, 8);
    setProperty("PowerExperimentFailedCommand", experimentFailedCommand, 8);
    setProperty("PowerExperimentMACAfter", experimentMacAfter, 8);
    setProperty("PowerExperimentQuarantined",
                experimentArmed && experimentResult != ExperimentResult::Completed);
    setProperty("PowerExecutorV2SchemaVersion", kPowerExecutorSchemaVersion, 8);
    setProperty("PowerExecutorV2ContractValid", powerOnlyContractValid);
    setProperty("PowerExecutorV2ContractStepCount",
                static_cast<UInt64>(powerOnlyContractCount), 8);
    setProperty("PowerExecutorV2ContractWriteCount", powerOnlyWriteCount, 16);
    setProperty("PowerExecutorV2ContractPollCount", powerOnlyPollCount, 16);
    setProperty("PowerExecutorV2SimulationValid", powerExecutorSimulationValid);
    setProperty("PowerExecutorV2SimulationScenarioCount",
                powerExecutorSimulation.scenarioCount, 16);
    setProperty("PowerExecutorV2ExecutionAuthorized", powerExecutorArmed);
    setProperty("PowerExecutorV2ExecutionAttempted",
                powerExecutorAttempted_);
    setProperty("PowerExecutorV2Result",
                powerExecutorResultName(powerExecutorResult));
    setProperty("PowerExecutorV2JournalRecordCount",
                static_cast<UInt64>(powerExecutionJournalCount), 8);
    setProperty("PowerExecutorV2JournalPersistent", false);
    setProperty("PowerExecutorV2JournalIntentRequiredBeforeEffect", true);
    setProperty("PowerExecutorV2JournalTerminalRecordBestEffort", true);
    setProperty("PowerExecutorV2CompletedWriteCount",
                powerExecutorCompletedWrites, 8);
    setProperty("PowerExecutorV2CompletedPollCount",
                powerExecutorCompletedPolls, 8);
    setProperty("PowerExecutorV2FailedStep", powerExecutorFailedStep, 8);
    setProperty("PowerExecutorV2MACAfter", powerExecutorMacAfter, 8);
    setProperty("PowerExecutorV2Quarantined",
                powerExecutorQuarantined_);
    setProperty("PowerExecutorV2PostIntentHardwareWorkAuthorized", false);
    setProperty("PowerExecutorV2MappingsProgrammedToDevice", false);
    setProperty("PowerExecutorV2HostOnlyMappingReleaseAuthorized", true);
    setProperty("PowerExecutorV2InvocationContext",
                "SynchronousStartBeforeRegisterService");
    setProperty("PowerExecutorV2CommandGateInstantiated", false);
    setProperty("PowerExecutorV2BusMasteringRequired", false);
    setProperty("PowerExecutorV2DMAAddressProgrammingAuthorized", false);
    setProperty("PowerExecutorV2FirmwareUploadAuthorized", false);
    setProperty("FirmwareUploadAttempted", false);
    setProperty("DMAConfigurationAttempted", false);
    setProperty("InterruptConfigurationAttempted", false);
    setProperty("AutomaticPowerOffAttempted", false);
    setProperty("PreSystemSnapshotStable", preSystemStable);
    setProperty("RSV_CTRLOriginal", preSystem.reservedControl, 8);
    setProperty("RSV_CTRLProjected", static_cast<UInt64>(0), 8);
    setProperty("HCI_OPT_CTRLOriginal", preSystem.hciOptionControl, 32);
    setProperty("HCI_OPT_CTRLProjected", preSystem.hciOptionControl | 0x00000100U, 32);
    setProperty("PAD_CTRL1Original", preSystem.padControl1, 32);
    setProperty("PAD_CTRL1Projected", preSystem.padControl1 | 0x30000000U, 32);
    setProperty("LED_CFGOriginal", preSystem.ledConfig, 32);
    setProperty("LED_CFGProjected", preSystem.ledConfig & ~0x06000000U, 32);
    setProperty("GPIO_MUXCFGOriginal", preSystem.gpioMuxConfig, 32);
    setProperty("GPIO_MUXCFGProjected", preSystem.gpioMuxConfig | 0x00000004U, 32);
    setProperty("SYS_FUNC_ENOriginal", preSystem.systemFunctionEnable, 8);
    setProperty("SYS_FUNC_ENProjected", preSystem.systemFunctionEnable & ~0x03U, 8);
    setProperty("RF_CTRLOriginal", preSystem.rfControl, 8);
    setProperty("RF_CTRLProjected", preSystem.rfControl & ~0x07U, 8);
    setProperty("WLRF1Original", preSystem.wlrf1, 32);
    setProperty("WLRF1Projected", preSystem.wlrf1 & ~0x07000000U, 32);
    setProperty("SYS_PWR_CTRLOriginal", preSystem.systemPowerControl, 8);
    setProperty("MCUFW_CTRLOriginal", preSystem.firmwareControl, 16);
    setProperty("RPWMOriginal", preSystem.rpwm, 8);
    setProperty("QueueConfigurationBaselineReadOnly", true);
    setProperty("QueueConfigurationSnapshotStable", queueSnapshotStable);
    setProperty("QueueConfigurationPlanValid",
                queuePlanValid && trxDevicePlanValid);
    setProperty("QueueConfigurationCommandCount", static_cast<UInt64>(4), 8);
    setProperty("QueueConfigurationExecutionAuthorized", false);
    setProperty("QueueConfigurationExecutionAttempted", false);
    setProperty("QueueConfigurationBlocker",
                "GlobalRWPTRClearRequiresCompleteTRXRings");
    setProperty("QueueConfigurationRequiresBusMastering", true);
    setProperty("QueueConfigurationRequiresPoweredMAC", true);
    setProperty("QueueConfigurationGlobalResetAffectsAllQueues", true);
    setProperty("QueuePCICtrl3Original", queueSnapshot.pciControl3, 8);
    setProperty("QueueBeaconRingBaseOriginal", queueSnapshot.beaconRingBase, 32);
    setProperty("QueueBeaconRingBaseProjected", retainedBeaconRingAddress, 32);
    setProperty("QueueRWPTRClearOriginal", queueSnapshot.rwPointerClear, 32);
    setProperty("QueueRWPTRClearProjected", static_cast<UInt64>(0xffffffffU), 32);
    setProperty("QueueBeaconWorkOriginal", queueSnapshot.beaconWork, 8);
    setProperty("QueueBeaconWorkProjected",
                static_cast<UInt64>(queueSnapshot.beaconWork | 0x10U), 8);

    UInt32 trxTXRingBytes = 0;
    UInt32 trxRXRingBytes = 0;
    UInt64 trxRXPayloadBytes = 0;
    UInt8 trxDeviceConfiguredCount = 0;
    const bool trxResourcePlanValid = validateTRXResourcePlan(
        trxTXRingBytes, trxRXRingBytes, trxRXPayloadBytes,
        trxDeviceConfiguredCount);
    setProperty("TRXResourcePlanValid", trxResourcePlanValid);
    setProperty("TRXResourceCount", static_cast<UInt64>(kTRXResourceCount), 8);
    setProperty("TRXTXQueueCount", static_cast<UInt64>(8), 8);
    setProperty("TRXRXQueueCount", static_cast<UInt64>(1), 8);
    setProperty("TRXDeviceConfiguredRingCount", trxDeviceConfiguredCount, 8);
    setProperty("TRXUnconfiguredRingCount", static_cast<UInt64>(0), 8);
    setProperty("TRXHardwareRequiredRingCount", static_cast<UInt64>(9), 8);
    setProperty("TRXUpstreamAllocatedRXQueueCount", static_cast<UInt64>(2), 8);
    setProperty("TRXRXC2HDeclaredByUpstream", true);
    setProperty("TRXRXC2HAllocatedByGenericLoop", true);
    setProperty("TRXRXC2HDeviceConfigured", false);
    setProperty("TRXRXC2HRuntimeConsumed", false);
    setProperty("TRXRXC2HTransportViaMPDU", true);
    setProperty("TRXRXC2HClassification", "DeadGenericAllocationArtifact");
    setProperty("TRXTXRingBytes", trxTXRingBytes, 32);
    setProperty("TRXRXRingBytes", trxRXRingBytes, 32);
    setProperty("TRXRXPayloadBytes", trxRXPayloadBytes, 64);
    setProperty("TRXCoherentRingBytes",
                static_cast<UInt64>(trxTXRingBytes) + trxRXRingBytes, 64);
    setProperty("TRXTotalMappedBytes",
                static_cast<UInt64>(trxTXRingBytes) + trxRXRingBytes +
                    trxRXPayloadBytes, 64);
    setProperty("TRXGlobalPointerResetSafe", false);
    setProperty("TRXDevicePlanValid", trxDevicePlanValid);
    setProperty("TRXDevicePlanRecordCount",
                static_cast<UInt64>(trxDevicePlanCount), 8);
    setProperty("TRXDevicePlanBaseWriteCount", trxDevicePlanBaseCount, 8);
    setProperty("TRXDevicePlanEntryCountWriteCount",
                trxDevicePlanEntryCount, 8);
    setProperty("TRXDevicePlanIncludesGlobalPointerReset",
                trxDevicePlanValid);
    setProperty("TRXDevicePlanIncludesH2CIndexReset", trxDevicePlanValid);
    setProperty("TRXDevicePlanIncludesDMAControl", trxDevicePlanValid);
    setProperty("TRXDevicePlanRequiresBusMasterDisabledDuringProgramming",
                true);
    setProperty("TRXDevicePlanExecutionAuthorized", false);
    setProperty("TRXDevicePlanExecutionAttempted", false);
    setProperty("RetainedBeaconDescriptorHostMaterialized",
                retainedBeaconRingValid);
    setProperty("RetainedBeaconDescriptorAddress",
                retainedBeaconRingAddress, 32);
    setProperty("RetainedBeaconDescriptorSynchronizedForDevice", false);
    setProperty("RetainedBeaconDescriptorDeviceVisible", false);
    setProperty("TRXAllocationAttempted", trxAllocation.attempted);
    setProperty("TRXAllocationComplete", trxAllocation.allocationComplete);
    setProperty("TRXAllocationStage", dmaProbeStageName(trxAllocation.stage));
    setProperty("TRXAllocationStatus",
                static_cast<UInt64>(trxAllocation.status), 32);
    setProperty("TRXAllocationFailedResourceIndex",
                trxAllocation.failedResourceIndex, 32);
    setProperty("TRXAllocationFailedPayloadIndex",
                trxAllocation.failedPayloadIndex, 32);
    setProperty("TRXAllocationRingMappingCount",
                trxAllocation.ringMappingCount, 32);
    setProperty("TRXAllocationPayloadMappingCount",
                trxAllocation.payloadMappingCount, 32);
    setProperty("TRXAllocationPeakMappingCount",
                trxAllocation.peakMappingCount, 32);
    setProperty("TRXAllocationRingBytes", trxAllocation.ringBytes, 64);
    setProperty("TRXAllocationPayloadBytes", trxAllocation.payloadBytes, 64);
    setProperty("TRXAllocationTotalMappedBytes",
                trxAllocation.ringBytes + trxAllocation.payloadBytes, 64);
    setProperty("TRXAllocationResourcesReleased",
                trxAllocation.resourcesReleased);
    setProperty("TRXAllocationRetainedMappingCount",
                static_cast<UInt64>(trxRingCount_) + trxPayloadCount_, 32);
    setProperty("TRXAllocationPeakTotalActiveMappingCount",
                static_cast<UInt64>(trxAllocation.peakMappingCount) +
                    (dmaTemplateValid ? 2 : 0), 32);
    setProperty("TRXAllocationModel",
                "NineSeparateRings+512SeparateRXPayloads");
    setProperty("TRXAllocationPhysicalContiguityRequired", false);
    setProperty("TRXAllocationSingleIOVMSegmentRequired", true);
    setProperty("TRXAllocationFailureNonFatal", true);
    setProperty("TRXAllocationDeviceVisible", false);
    setProperty("PersistentTRXAllocated", trxAllocation.allocationComplete);
    setProperty("PersistentTRXPrepared", trxAllocation.allocationComplete &&
                trxRingCount_ == kTRXResourceCount &&
                trxPayloadCount_ == kRXRingEntries);
    setProperty("PersistentTRXDeviceVisible", false);
    setProperty("PersistentTRXReleasePoint", "ServiceStopOrFailedStart");
    setProperty("RXMPDUDescriptorHostMaterialized",
                trxAllocation.rxDescriptorMaterialized);
    setProperty("RXMPDUDescriptorFormatValid",
                trxAllocation.rxDescriptorFormatValid);
    setProperty("RXMPDUDescriptorSize",
                static_cast<UInt64>(sizeof(PCIRXBufferDescriptor)), 8);
    setProperty("RXMPDUDescriptorCount",
                static_cast<UInt64>(kRXRingEntries), 32);
    setProperty("RXMPDUDescriptorRingBytes",
                static_cast<UInt64>(kRXRingEntries) *
                    sizeof(PCIRXBufferDescriptor), 32);
    setProperty("RXMPDUBufferSize", static_cast<UInt64>(kRXBufferSize), 16);
    setProperty("RXMPDUInitialTag", static_cast<UInt64>(0), 16);
    setProperty("RXMPDUDescriptorValidCount",
                trxAllocation.rxDescriptorValidCount, 32);
    setProperty("RXMPDUInitialTagZeroCount",
                trxAllocation.rxDescriptorZeroTagCount, 32);
    setProperty("RXMPDUDescriptorAddressMatchCount",
                trxAllocation.rxDescriptorAddressMatchCount, 32);
    setProperty("RXMPDUDescriptorAddressRangeValidCount",
                trxAllocation.rxDescriptorAddressRangeValidCount, 32);
    setProperty("RXMPDUDescriptorOwnBitPresent", false);
    setProperty("RXMPDUDescriptorEORBitPresent", false);
    setProperty("RXMPDUDescriptorDirection", "InOut");
    setProperty("RXMPDUPayloadDirection", "In");
    setProperty("RXMPDUDescriptorSynchronizedForDevice", false);
    setProperty("RXMPDUDescriptorDeviceVisible", false);
    setProperty("RXMPDUQueueBaseWritten", false);
    setProperty("RXMPDUQueueCountWritten", false);
    setProperty("RXMPDUQueueIndexWritten", false);
    setProperty("RXMPDUGlobalPointerResetWritten", false);
    setProperty("RXMPDUDMAControlWritten", false);
    setProperty("TXDescriptorHostMaterialized",
                trxAllocation.txDescriptorFormatValid &&
                    retainedBeaconRingValid);
    setProperty("TXDescriptorFormatValid",
                trxAllocation.txDescriptorFormatValid &&
                    retainedBeaconRingValid);
    setProperty("TXDescriptorSize",
                static_cast<UInt64>(kTXRingDescriptorSize), 8);
    setProperty("TXDescriptorCount", static_cast<UInt64>(1025), 32);
    setProperty("TXDescriptorPreMaterializationZeroCount",
                trxAllocation.txDescriptorZeroCount, 32);
    setProperty("TXDescriptorCurrentZeroCount",
                trxAllocation.txDescriptorZeroCount -
                    (retainedBeaconRingValid ? 1 : 0), 32);
    setProperty("TXRingValidCount", trxAllocation.txRingValidCount, 8);
    setProperty("TXRingAddressRangeValidCount",
                trxAllocation.txRingAddressRangeValidCount, 8);
    setProperty("TXDescriptorInitialState", "ZeroIdleBeforeBeaconTemplate");
    setProperty("TXDescriptorCurrentState",
                retainedBeaconRingValid ?
                    "1024ZeroIdle+1HostBeaconTemplate" :
                    "Unvalidated");
    setProperty("TXPayloadMappingCount", static_cast<UInt64>(0), 32);
    setProperty("TXDescriptorSynchronizedForDevice", false);
    setProperty("TXDescriptorDeviceVisible", false);
    setProperty("TXQueueBaseWritten", false);
    setProperty("TXQueueCountWritten", false);
    setProperty("TXQueueIndexWritten", false);
    setProperty("TXGlobalPointerResetWritten", false);
    setProperty("TXDMAControlWritten", false);
    setProperty("FirmwareIDDMAPlanValid", firmwareIDDMAPlanValid);
    setProperty("FirmwareIDDMAPlanRecordCount",
                static_cast<UInt64>(firmwareIDDMAPlanCount), 8);
    setProperty("FirmwareIDDMASourceAddress", kIDDMAFirmwareSource, 32);
    setProperty("FirmwareIDDMAPollIterations", kIDDMAPollIterations, 32);
    setProperty("FirmwareIDDMAPollIntervalMicroseconds",
                kIDDMAPollIntervalMicroseconds, 32);
    setProperty("FirmwareIDDMAPollWorstCaseMicroseconds",
                static_cast<UInt64>(firmwareIDDMAPlanCount) *
                    2 * kIDDMAPollIterations *
                    kIDDMAPollIntervalMicroseconds, 64);
    setProperty("FirmwareIDDMAPollCount",
                static_cast<UInt64>(firmwareIDDMAPlanCount) * 2, 16);
    setProperty("FirmwareIDDMAChecksumResetCount",
                static_cast<UInt64>(3), 8);
    setProperty("FirmwareIDDMAChecksumValidationCount",
                static_cast<UInt64>(3), 8);
    setProperty("FirmwareIDDMARequiresStagingRebuildPerRecord", true);
    setProperty("FirmwareIDDMARequiresDescriptorChecksumPerRecord", true);
    setProperty("FirmwareIDDMARequiresBeaconTransferPerRecord", true);
    setProperty("FirmwareRegisterBaselineReadOnly", true);
    setProperty("FirmwareRegisterBaselineCount",
                static_cast<UInt64>(kFirmwareRegisterBaselineCount), 8);
    setProperty("FirmwareRegisterBaselineStableCount",
                firmwareRegisterBaselineStableCount, 8);
    setProperty("FirmwareRegisterBaselineAllStable",
                firmwareRegisterBaselineStableCount ==
                    kFirmwareRegisterBaselineCount);
    setProperty("FirmwareIDDMAExecutionAuthorized", false);
    setProperty("FirmwareIDDMAExecutionAttempted", false);
    setProperty("FirmwareIDDMASourceWritten", false);
    setProperty("FirmwareIDDMADestinationWritten", false);
    setProperty("FirmwareIDDMAControlWritten", false);
    setProperty("TRXAllocationPCICommandBefore",
                trxAllocationPCICommandBefore, 16);
    setProperty("TRXAllocationPCICommandAfter",
                trxAllocationPCICommandAfter, 16);
    setProperty("TRXResourceBlocker",
                trxAllocation.allocationComplete ?
                    "PersistentTRXNotDeviceConfigured" :
                    "HardwareRequiredTRXAllocationFeasibilityFailed");

    OSData *snapshotData = OSData::withBytes(snapshots, sizeof(snapshots));
    if (snapshotData) {
        setProperty("PowerRegisterSnapshot", snapshotData);
        snapshotData->release();
    }

    OSData *planData = OSData::withBytes(dryRunPlan, sizeof(dryRunPlan));
    if (planData) {
        setProperty("PowerFSMDryRunPlan", planData);
        planData->release();
    }

    OSData *projectionData = OSData::withBytes(projectedRegisters,
                                               sizeof(projectedRegisters));
    if (projectionData) {
        setProperty("PowerFSMProjectedRegisters", projectionData);
        projectionData->release();
    }

    OSData *pollBaselineData = OSData::withBytes(pollBaseline, sizeof(pollBaseline));
    if (pollBaselineData) {
        setProperty("PowerFSMPollBaselineSamples", pollBaselineData);
        pollBaselineData->release();
    }

    OSData *firmwarePlanData = OSData::withBytes(
        firmwareTransferPlan,
        static_cast<unsigned int>(firmwareTransferPlanCount *
                                  sizeof(firmwareTransferPlan[0])));
    if (firmwarePlanData) {
        setProperty("FirmwareTransferPlan", firmwarePlanData);
        firmwarePlanData->release();
    }

    OSData *firmwareIDDMAPlanData = OSData::withBytes(
        firmwareIDDMAPlan,
        static_cast<unsigned int>(firmwareIDDMAPlanCount *
                                  sizeof(firmwareIDDMAPlan[0])));
    if (firmwareIDDMAPlanData) {
        setProperty("FirmwareIDDMAPlan", firmwareIDDMAPlanData);
        firmwareIDDMAPlanData->release();
    }

    OSData *firmwareRegisterBaselineData = OSData::withBytes(
        firmwareRegisterBaseline, sizeof(firmwareRegisterBaseline));
    if (firmwareRegisterBaselineData) {
        setProperty("FirmwareRegisterBaseline", firmwareRegisterBaselineData);
        firmwareRegisterBaselineData->release();
    }

    OSData *lifecyclePlanData = OSData::withBytes(
        kLifecyclePlan, sizeof(kLifecyclePlan));
    if (lifecyclePlanData) {
        setProperty("CanonicalLifecyclePlan", lifecyclePlanData);
        lifecyclePlanData->release();
    }

    OSData *postPowerPlanData = OSData::withBytes(
        kPostPowerPlan, sizeof(kPostPowerPlan));
    if (postPowerPlanData) {
        setProperty("PostPowerSystemPlan", postPowerPlanData);
        postPowerPlanData->release();
    }

    OSData *firmwareSetupPlanData = OSData::withBytes(
        kFirmwareSetupPlan, sizeof(kFirmwareSetupPlan));
    if (firmwareSetupPlanData) {
        setProperty("FirmwareTemporarySetupPlan", firmwareSetupPlanData);
        firmwareSetupPlanData->release();
    }

    OSData *reservedPagePlanData = OSData::withBytes(
        kReservedPagePlan, sizeof(kReservedPagePlan));
    if (reservedPagePlanData) {
        setProperty("ReservedPageTransactionPlan", reservedPagePlanData);
        reservedPagePlanData->release();
    }

    OSData *finalFIFOPlanData = OSData::withBytes(
        kFinalFIFOPlan, sizeof(kFinalFIFOPlan));
    if (finalFIFOPlanData) {
        setProperty("FinalFIFOPlan", finalFIFOPlanData);
        finalFIFOPlanData->release();
    }

    OSData *fifoSummaryData = OSData::withBytes(
        &kFIFOPlanSummary, sizeof(kFIFOPlanSummary));
    if (fifoSummaryData) {
        setProperty("FinalFIFOPlanSummary", fifoSummaryData);
        fifoSummaryData->release();
    }

    OSData *efuseDependencyData = OSData::withBytes(
        &kEfuseDependencyPlan, sizeof(kEfuseDependencyPlan));
    if (efuseDependencyData) {
        setProperty("EfuseDependencyPlan", efuseDependencyData);
        efuseDependencyData->release();
    }

    OSData *interruptPlanData = OSData::withBytes(
        kInterruptPlan, sizeof(kInterruptPlan));
    if (interruptPlanData) {
        setProperty("InterruptLifecyclePlan", interruptPlanData);
        interruptPlanData->release();
    }

    OSData *dmaStateData = OSData::withBytes(
        kDMAStates, sizeof(kDMAStates));
    if (dmaStateData) {
        setProperty("DMAStatePlan", dmaStateData);
        dmaStateData->release();
    }

    OSData *dmaTransitionData = OSData::withBytes(
        kDMATransitions, sizeof(kDMATransitions));
    if (dmaTransitionData) {
        setProperty("DMATransitionPlan", dmaTransitionData);
        dmaTransitionData->release();
    }

    OSData *dmaPublicationData = OSData::withBytes(
        kDMAPublicationPlan, sizeof(kDMAPublicationPlan));
    if (dmaPublicationData) {
        setProperty("DMAPublicationPlan", dmaPublicationData);
        dmaPublicationData->release();
    }

    OSData *rollbackPolicyData = OSData::withBytes(
        kRollbackPolicies, sizeof(kRollbackPolicies));
    if (rollbackPolicyData) {
        setProperty("RollbackPolicy", rollbackPolicyData);
        rollbackPolicyData->release();
    }

    OSData *mmioPublicationData = OSData::withBytes(
        kMMIOPublicationContracts, sizeof(kMMIOPublicationContracts));
    if (mmioPublicationData) {
        setProperty("MMIOPublicationContract", mmioPublicationData);
        mmioPublicationData->release();
    }

    OSData *serializedCommandData = OSData::withBytes(
        kSerializedCommandContracts, sizeof(kSerializedCommandContracts));
    if (serializedCommandData) {
        setProperty("SerializedCommandContract", serializedCommandData);
        serializedCommandData->release();
    }

    OSData *executionJournalData = OSData::withBytes(
        kExecutionJournalContracts, sizeof(kExecutionJournalContracts));
    if (executionJournalData) {
        setProperty("ExecutionJournalContract", executionJournalData);
        executionJournalData->release();
    }

    OSData *mappingExposureData = OSData::withBytes(
        kMappingExposureClasses, sizeof(kMappingExposureClasses));
    if (mappingExposureData) {
        setProperty("MappingExposureLedger", mappingExposureData);
        mappingExposureData->release();
    }

    OSData *mappingExposureTransitionData = OSData::withBytes(
        kMappingExposureTransitions, sizeof(kMappingExposureTransitions));
    if (mappingExposureTransitionData) {
        setProperty("MappingExposureTransitionPlan",
                    mappingExposureTransitionData);
        mappingExposureTransitionData->release();
    }

    OSData *synchronizationGenerationData = OSData::withBytes(
        kSynchronizationGenerationContracts,
        sizeof(kSynchronizationGenerationContracts));
    if (synchronizationGenerationData) {
        setProperty("SynchronizationGenerationContract",
                    synchronizationGenerationData);
        synchronizationGenerationData->release();
    }

    OSData *failureInjectionData = OSData::withBytes(
        &failureInjectionSummary, sizeof(failureInjectionSummary));
    if (failureInjectionData) {
        setProperty("FailureBoundaryClassificationSummary", failureInjectionData);
        failureInjectionData->release();
    }

    OSData *symbolicInterpreterData = OSData::withBytes(
        &symbolicInterpreterSummary, sizeof(symbolicInterpreterSummary));
    if (symbolicInterpreterData) {
        setProperty("SymbolicInterpreterSummary", symbolicInterpreterData);
        symbolicInterpreterData->release();
    }

    OSData *powerOnlyContractData = OSData::withBytes(
        powerOnlyContract,
        static_cast<unsigned int>(powerOnlyContractCount *
                                  sizeof(powerOnlyContract[0])));
    if (powerOnlyContractData) {
        setProperty("PowerExecutorV2Contract", powerOnlyContractData);
        powerOnlyContractData->release();
    }

    OSData *powerExecutorSimulationData = OSData::withBytes(
        &powerExecutorSimulation, sizeof(powerExecutorSimulation));
    if (powerExecutorSimulationData) {
        setProperty("PowerExecutorV2SimulationSummary",
                    powerExecutorSimulationData);
        powerExecutorSimulationData->release();
    }

    OSData *powerExecutionJournalData = OSData::withBytes(
        powerExecutionJournal,
        static_cast<unsigned int>(powerExecutionJournalCount *
                                  sizeof(powerExecutionJournal[0])));
    if (powerExecutionJournalData) {
        setProperty("PowerExecutorV2Journal", powerExecutionJournalData);
        powerExecutionJournalData->release();
    }

    OSData *queueSnapshotData = OSData::withBytes(
        &queueSnapshot, sizeof(queueSnapshot));
    if (queueSnapshotData) {
        setProperty("QueueConfigurationSnapshot", queueSnapshotData);
        queueSnapshotData->release();
    }

    OSData *queuePlanData = OSData::withBytes(queuePlan, sizeof(queuePlan));
    if (queuePlanData) {
        setProperty("QueueConfigurationPlan", queuePlanData);
        queuePlanData->release();
    }

    OSData *trxResourceData = OSData::withBytes(
        kTRXResources, sizeof(kTRXResources));
    if (trxResourceData) {
        setProperty("TRXResourcePlan", trxResourceData);
        trxResourceData->release();
    }

    OSData *trxDevicePlanData = OSData::withBytes(
        trxDevicePlan,
        static_cast<unsigned int>(trxDevicePlanCount *
                                  sizeof(trxDevicePlan[0])));
    if (trxDevicePlanData) {
        setProperty("TRXDevicePlan", trxDevicePlanData);
        trxDevicePlanData->release();
    }

    IOLog("RTL8821CEProbe: power FSM dry-run planned %u commands in 5 phases; %u poll-baseline samples with %u transitions; experiment %s, result %s; FLR %s; SYS_CFG1 0x%08x cut %u vendor %u RF %s; initial CR 0x%02x; PCI command restored to 0x%04x\n",
          static_cast<unsigned>(dryRunPlanCount),
          kBaselineSampleCount, pollBaselineTransitionCount,
          experimentArmed ? "armed" : "disarmed",
          experimentResultName(experimentResult),
          functionLevelResetSupported ? "supported" : "unsupported",
          sysCfg1, (sysCfg1 >> 12) & 0xfU, (sysCfg1 >> 16) & 0xfU,
          (sysCfg1 & (1U << 27)) != 0 ? "2T2R" : "1T1R", cr, command);

    registerService();
    return true;
}

void RTL8821CEProbe::stop(IOService *provider)
{
    releasePersistentDMA();
    IOLog("RTL8821CEProbe: stopped\n");
    super::stop(provider);
}

void RTL8821CEProbe::releasePersistentDMA()
{
    PreparedDMA *payloads = static_cast<PreparedDMA *>(trxPayloads_);
    for (UInt32 index = trxPayloadCount_; index > 0; index--)
        releasePreparedDMA(payloads[index - 1]);
    if (payloads)
        IOFree(payloads, sizeof(PreparedDMA) * kRXRingEntries);
    trxPayloads_ = nullptr;
    trxPayloadCount_ = 0;

    PreparedDMA *rings = static_cast<PreparedDMA *>(trxRings_);
    for (UInt32 index = trxRingCount_; index > 0; index--)
        releasePreparedDMA(rings[index - 1]);
    if (rings)
        IOFree(rings, sizeof(PreparedDMA) * kTRXResourceCount);
    trxRings_ = nullptr;
    trxRingCount_ = 0;

    if (stagingPrepared_ && stagingCommand_) {
        stagingCommand_->complete();
        stagingPrepared_ = false;
    }
    if (stagingSet_ && stagingCommand_) {
        stagingCommand_->clearMemoryDescriptor(false);
        stagingSet_ = false;
    }
    if (stagingCommand_) {
        stagingCommand_->release();
        stagingCommand_ = nullptr;
    }
    if (stagingBuffer_) {
        stagingBuffer_->release();
        stagingBuffer_ = nullptr;
    }

    if (descriptorPrepared_ && descriptorCommand_) {
        descriptorCommand_->complete();
        descriptorPrepared_ = false;
    }
    if (descriptorSet_ && descriptorCommand_) {
        descriptorCommand_->clearMemoryDescriptor(false);
        descriptorSet_ = false;
    }
    if (descriptorCommand_) {
        descriptorCommand_->release();
        descriptorCommand_ = nullptr;
    }
    if (descriptorBuffer_) {
        descriptorBuffer_->release();
        descriptorBuffer_ = nullptr;
    }
}
