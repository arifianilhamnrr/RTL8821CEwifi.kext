#pragma once

#include <IOKit/IOService.h>
#include <IOKit/pci/IOPCIDevice.h>

class IOBufferMemoryDescriptor;
class IODMACommand;

class RTL8821CEProbe : public IOService {
    OSDeclareDefaultStructors(RTL8821CEProbe)

public:
    bool start(IOService *provider) override;
    void stop(IOService *provider) override;

private:
    void releasePersistentDMA();

    IOBufferMemoryDescriptor *descriptorBuffer_ {nullptr};
    IODMACommand *descriptorCommand_ {nullptr};
    IOBufferMemoryDescriptor *stagingBuffer_ {nullptr};
    IODMACommand *stagingCommand_ {nullptr};
    bool descriptorSet_ {false};
    bool descriptorPrepared_ {false};
    bool stagingSet_ {false};
    bool stagingPrepared_ {false};
    void *trxRings_ {nullptr};
    void *trxPayloads_ {nullptr};
    UInt32 trxRingCount_ {0};
    UInt32 trxPayloadCount_ {0};
    bool powerExecutorAttempted_ {false};
    bool powerExecutorQuarantined_ {false};
};
