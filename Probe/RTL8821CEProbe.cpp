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

static_assert(sizeof(PCITXBufferElement) == 8,
              "unexpected PCI TX buffer element layout");
static_assert(sizeof(QueueRegisterSnapshot) == 12,
              "unexpected queue snapshot layout");
static_assert(sizeof(QueueProjectedCommand) == 20,
              "unexpected queue command layout");
static_assert(sizeof(TRXResourcePlan) == 36,
              "unexpected TRX resource layout");

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

bool prepareSingleSegmentDMA(UInt32 length, PreparedDMA &dma)
{
    dma = {};
    const mach_vm_address_t physicalMask =
        0x00000000ffffffffULL &
        ~(static_cast<mach_vm_address_t>(kDMAAlignment) - 1);
    dma.buffer = IOBufferMemoryDescriptor::inTaskWithPhysicalMask(
        kernel_task, kIODirectionOut | kIOMemoryPhysicallyContiguous,
        length, physicalMask);
    if (!dma.buffer)
        return false;

    void *bytes = dma.buffer->getBytesNoCopy();
    if (!bytes)
        return false;
    bzero(bytes, length);

    IOReturn status = IODMACommand::weakWithSpecification(
        &dma.command, kIODMACommandOutputHost32, 32, length,
        IODMACommand::kMapped, length, kDMAAlignment);
    if (status != kIOReturnSuccess || !dma.command)
        return false;

    status = dma.command->setMemoryDescriptor(dma.buffer, false);
    if (status != kIOReturnSuccess)
        return false;
    dma.descriptorSet = true;

    status = dma.command->prepare(0, length);
    if (status != kIOReturnSuccess)
        return false;
    dma.prepared = true;

    IODMACommand::Segment32 segments[2] = {};
    UInt64 offset = 0;
    UInt32 segmentCount = 2;
    status = dma.command->gen32IOVMSegments(&offset, segments, &segmentCount);
    if (status != kIOReturnSuccess || segmentCount != 1 || offset != length ||
        segments[0].fLength != length ||
        static_cast<UInt64>(segments[0].fIOVMAddr) + length > 0x100000000ULL)
        return false;

    dma.address = segments[0].fIOVMAddr;
    dma.length = segments[0].fLength;
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
    bool valid = prepareSingleSegmentDMA(kBeaconDescriptorRingSize, descriptor) &&
                 prepareSingleSegmentDMA(kFirmwareStagingPacketSize, staging);

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
    const bool experimentArmed = kPowerExperimentBuildEnabled && experimentRequested;

    setProperty("PowerExperimentEnableArgumentPresent", experimentEnablePresent);
    setProperty("PowerExperimentConfirmationArgumentPresent",
                experimentConfirmationPresent);
    setProperty("PowerExperimentRequested", experimentRequested);
    setProperty("PowerExperimentBuildEnabled", kPowerExperimentBuildEnabled);
    setProperty("PowerExperimentArmed", experimentArmed);
    setProperty("PowerExperimentOneShot", true);
    setProperty("PowerExperimentAutomaticRecovery", false);
    setProperty("PowerExperimentFailureRecovery", "ColdPowerRemoval");

    const UInt64 firmwareSectionSize = static_cast<UInt64>(
        firmwareSectionEnd - firmwareSectionStart);
    FirmwareHeader firmwareHeader = {};
    FirmwareTransferPlan firmwareTransferPlan[kFirmwareTransferPlanCapacity] = {};
    size_t firmwareTransferPlanCount = 0;
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
    setProperty("FirmwareTransportPlanned", firmwareManifestValid);
    setProperty("FirmwareTransportReady", false);
    setProperty("FirmwareTransportBlocker", "NoIOKitCoherentBeaconTXRing");
    setProperty("FirmwareRequiresBusMastering", true);
    setProperty("FirmwareRequiresBeaconTXQueue", true);
    setProperty("FirmwareRequiresIDDMA", true);

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
        kIOMapInhibitCache | (experimentArmed ? 0 : kIOMapReadOnly));
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
    PreSystemSnapshot preSystem = {};
    bool preSystemStable = false;
    QueueRegisterSnapshot queueSnapshot = {};
    QueueProjectedCommand queuePlan[4] = {};
    bool queueSnapshotStable = false;
    bool queuePlanValid = false;

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
    setProperty("PowerCycleAttempted", experimentArmed);
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
    setProperty("QueueConfigurationPlanValid", queuePlanValid);
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
    setProperty("QueueBeaconRingBaseProjected", dmaTemplate.descriptorAddress, 32);
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
    setProperty("TRXAllocationAttempted", false);
    setProperty("TRXResourceBlocker", "HardwareRequiredTRXResourcesNotAllocated");

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
