
#include "model/model.hh"

#include "debug/Model.hh"
#include "model/enums.hh"
#include "sim/system.hh"

const size_t IndexTypeBytes = 4;
const size_t ValueTypeBytes = 8;

namespace gem5
{

Model::Model(const Params& params):
    ClockedObject(params),
    requestorId(params.system->getRequestorId(this)), port(this, "port"),
    baseIndexerAddr(params.base_indexer_addr),
    baseValuesAddr(params.base_values_addr),
    intRegFileSize(params.int_regfile_size), intRegUsed(0),
    fpRegFileSize(params.fp_regfile_size), fpRegUsed(0),
    requestGenLatency(params.request_gen_latency),
    requestGenBandwidth(params.request_gen_bandwidth),
    firstGeneratorAvailableTime(0),
    nextGenEvent([this](){ processNextGenEvent(); }, name() + ".GenEvent"),
    requestBufferSize(params.request_buffer_size),
    sendBandwidth(params.send_bandwidth),
    firstPortAvailableTime(0),
    nextSendEvent([this](){ processNextSendEvent(); }, name() + ".SendEvent")
{
    generatorBusyUntil.resize(requestGenBandwidth);
    for (int i; i < requestGenBandwidth; i++) {
        generatorBusyUntil[i] = 0;
    }
    portBusyUntil.resize(sendBandwidth);
    for (int i; i < sendBandwidth; i++) {
        portBusyUntil[i] = 0;
    }
}

Port&
Model::getPort(const std::string& if_name, PortID idx)
{
    if (if_name == "port") {
        return port;
    } else {
        return ClockedObject::getPort(if_name, idx);
    }
}

void
Model::addKernel(int id, int delta, int count, enums::SpatterKernelType type, std::vector<int> indices)
{
    DPRINTF(Model, "%s: Adding kernel with id: %d, delta: %d, count: %d, type: %s.\n",
        __func__, id, delta, count, enums::SpatterKernelTypeStrings[type]);
    Kernel new_kernel(id, delta, count, type);
    new_kernel.setIndices(indices);
    kernels.push(new_kernel);
}

void
Model::startup()
{
    scheduleNextGenEvent(curTick());
}

void
Model::ModelPort::sendPacket(PacketPtr pkt) {
    panic_if(blocked(), "Should never try to send if port is blocked.");
    if (!sendTimingReq(pkt)) {
        blockedPacket = pkt;
        DPRINTF(Model, "%s: Port blocked when sending %s.\n", __func__, pkt->print());
    }
}

void
Model::ModelPort::recvReqRetry()
{
    DPRINTF(Model, "%s: Port received a ReqRetry.\n", __func__);
    if (!sendTimingReq(blockedPacket)) {
        DPRINTF(Model, "%s: Port blocked when sending %s.\n", __func__, blockedPacket->print());
    } else {
        blockedPacket = nullptr;
        owner->recvReqRetry();
    }
}

void
Model::recvReqRetry()
{
    if (nextSendEvent.pending()) {
        nextSendEvent.wake();
        scheduleNextSendEvent(nextCycle());
    }
}

bool
Model::ModelPort::recvTimingResp(PacketPtr pkt) {
    return owner->recvTimingResp(pkt);
}

bool
Model::recvTimingResp(PacketPtr pkt)
{
    DPRINTF(Model, "%s: Received pkt: %s.\n", __func__, pkt->print());
    assert(pkt->isResponse());
    if (pkt->isWrite()) {
        std::cout << "Received Write Response." << std::endl;
    }
    IndexInfo* index_info = pkt->findNextSenderState<IndexInfo>();
    if (index_info != nullptr) {
        // TODO: record travel time for indexer accesses.
        receiveBuffer.push(pkt);
        receiveBufferInsertionTime.push(curTick());
        // REMEMBER: might need to wake nextGenEvent up?
        // IDEA: Separate pending to pendingInput and pendingOutput?
        // I think for now we should not wake up nextGenEvent,
        // because nextGenEvent does not sleep if there are not
        // enough registers available, which is something that
        // this scope of the if does not influence.
    } else {
        // if it doesn't have IndexInfo it's the response for the
        // last level of indirect accesses.
        // TODO: record travel time with and without reading indexer.
        delete pkt;
        // CAUTION: We're going to decrement fpRegUsed here,
        // it could cause inaccuracies if processNextGenEvent
        // is called after recvTimingResp.
        // it may be fine though since we don't simulate the rest,
        // it's probably not going to ever be an issue since
        // fpRegFileSize is probably >> requestBufferSize
        fpRegUsed--;
    }
    if (!nextGenEvent.pending()) {
        scheduleNextGenEvent(nextCycle());
    }
    return true;
}

void
Model::scheduleNextGenEvent(Tick when)
{
    bool have_index = !receiveBuffer.empty();
    bool have_fp_reg = fpRegUsed < fpRegFileSize;
    bool have_kernel = (!kernels.empty()) && (!kernels.front().done());
    bool have_int_reg = intRegUsed < intRegFileSize;
    bool have_int_work_space = have_kernel && have_int_reg;
    bool have_fp_work_space = have_index && have_fp_reg;
    bool have_work = have_int_work_space || have_fp_work_space;
    Tick schedule_tick = std::max(when, firstGeneratorAvailableTime);
    if (have_work && (!nextGenEvent.scheduled())) {
        schedule(nextGenEvent, schedule_tick);
        firstGeneratorAvailableTime = MaxTick;
    }
}

void
Model::processNextGenEvent()
{
    assert(!nextGenEvent.pending());
    int req_buf_before = requestBuffer.size();
    // track changes to intRegUsed in this variable and apply it
    // at the end of the for loop. This way if we free a register
    // in the for loop, other iterations of the for loop won't
    // observe this change. This matches what happens in real h/w.
    int int_reg_used_delta = 0;
    // track this independetly to prevent different iterations inside
    // for loop observing change to h/w resources, i.e we can't rely
    // intRegFileSize - intRegUsed to see if we have registers to allocate
    // since they don't change until after the for loop
    int available_int_regs_this_cycle = intRegFileSize - intRegUsed;
    int fp_reg_used_delta = 0;
    // same explanation as available_int_regs_this_cycle
    int available_fp_regs_this_cycle = fpRegFileSize - fpRegUsed;
    for (int i = 0; i < requestGenBandwidth; i++) {
        if (generatorBusyUntil[i] > curTick()) {
            // if ith "AGU" is busy then skip it.
            // however, we still need to update firstGeneratorAvailableTime
            firstGeneratorAvailableTime =
                std::min(firstGeneratorAvailableTime, generatorBusyUntil[i]);
            DPRINTF(Model, "%s: AGU[%d] is busy this cycle.\n", __func__, i);
            continue;
        }
        if (!(requestBuffer.size() < requestBufferSize)) {
            // if no space left in the requestBuffer sleep
            // whoever pops from requestBuffer wakes us up.
            nextGenEvent.sleep();
            break;
        }
        // Now we know that AGU[i] is available and there is room
        // in the requestBuffer to put the packet.
        bool have_index = !receiveBuffer.empty();
        bool have_fp_reg = available_fp_regs_this_cycle > 0;
        bool have_kernel = (!kernels.empty()) && (!kernels.front().done());
        bool have_int_reg = available_int_regs_this_cycle > 0;
        if (have_index && have_fp_reg &&
            (ticksToCycles(curTick() - receiveBufferInsertionTime.front()) >= 1)) {
            PacketPtr index_pkt = receiveBuffer.front();
            IndexInfo* index_info = index_pkt->findNextSenderState<IndexInfo>();
            // occupy on fp register
            available_fp_regs_this_cycle--;
            fp_reg_used_delta++;
            // make AGU busy for the next requestGenLatency cycles.
            generatorBusyUntil[i] = clockEdge(Cycles(requestGenLatency));
            // not going to happen in this code, but figure out if
            // the current AGU is going to be available before what
            // was previosuly thought. this is possible if we had
            // non-uniform latencies for operations.
            firstGeneratorAvailableTime =
                std::min(firstGeneratorAvailableTime, generatorBusyUntil[i]);
            // if from a scatter kernel do write, otherwise do read.
            MemCmd cmd = index_info->type() == enums::SpatterKernelType::scatter ? MemCmd::WriteReq : MemCmd::ReadReq;
            // use the value from index which is index into Values Array.
            Addr values_addr = baseValuesAddr - (index_info->value() * ValueTypeBytes);
            // create the request for the address and size
            RequestPtr req = std::make_shared<Request>(values_addr, IndexTypeBytes, 0, requestorId);
            // Dummy PC to have PC-based prefetchers latch on;
            // get entropy into higher bits
            req->setPC(((Addr)requestorId) << 2);

            // finally create the packet and allocate host memory for data.
            PacketPtr pkt = new Packet(req, cmd);
            uint8_t* pkt_data = new uint8_t[req->getSize()];
            pkt->dataDynamic(pkt_data);

            // push to requestBuffer
            requestBuffer.push(pkt);
            requestBufferInsertionTime.push(curTick());
            DPRINTF(Model, "%s: Pushed pkt: %s to requestBuffer. This packet "
                        "is for accessing VALUES.\n", __func__, pkt->print());
            // now deallocate resources for reading the index
            delete index_pkt;
            int_reg_used_delta--;
            receiveBuffer.pop();
            receiveBufferInsertionTime.pop();
        } else if (have_kernel && have_int_reg){
            Kernel& front = kernels.front();
            int indexer_index, values_index;
            std::tie(indexer_index, values_index) = front.next();
            // occupy an int register
            available_int_regs_this_cycle--;
            int_reg_used_delta++;
            generatorBusyUntil[i] = clockEdge(Cycles(requestGenLatency));
            firstGeneratorAvailableTime =
                std::min(firstGeneratorAvailableTime, generatorBusyUntil[i]);

            IndexInfo* index_info = new IndexInfo(values_index, front.type());
            Addr indexer_addr = baseIndexerAddr + (indexer_index * IndexTypeBytes);
            RequestPtr req = std::make_shared<Request>(indexer_addr, IndexTypeBytes, 0, requestorId);
            // Dummy PC to have PC-based prefetchers latch on;
            // get entropy into higher bits
            req->setPC(((Addr)requestorId) << 2);
            PacketPtr pkt = new Packet(req, MemCmd::ReadReq);
            uint8_t* pkt_data = new uint8_t[req->getSize()];
            pkt->dataDynamic(pkt_data);
            pkt->pushSenderState(index_info);

            requestBuffer.push(pkt);
            requestBufferInsertionTime.push(curTick());
            DPRINTF(Model, "%s: Pushed pkt: %s to requestBuffer. This packet "
                        "is for accessing INDEX.\n", __func__, pkt->print());

            if (front.done()) {
                DPRINTF(Model, "%s: Done with kernel %d type: %s.\n",
                    __func__, front.id(), enums::SpatterKernelTypeStrings[front.type()]);
                kernels.pop();
            }
        } else {
            //
            DPRINTF(Model, "%s: Nothing more could be done this cycle.\n", __func__);
            DPRINTF(Model, "%s: Here is h/w status report: "
                "{KERNELS_REMAIN: %d, INDEXES_REMAIN: %d, INT_REG_USED: %d, "
                "FP_REG_USED: %d, REQ_BUFF_SIZE: %d}.\n",
                __func__, kernels.size(), receiveBuffer.size(),
                fpRegUsed, requestBuffer.size());
            break;
        }
    }
    intRegUsed += int_reg_used_delta;
    fpRegUsed += fp_reg_used_delta;

    bool did_work = (requestBuffer.size() - req_buf_before) > 0;
    if (did_work && (!nextSendEvent.pending())) {
        scheduleNextSendEvent(nextCycle());
    }

    if (!nextGenEvent.pending()) {
        scheduleNextGenEvent(firstGeneratorAvailableTime);
    }
}

void
Model::scheduleNextSendEvent(Tick when)
{
    bool have_work = !requestBuffer.empty();
    Tick schedule_tick = std::max(when, firstPortAvailableTime);
    if (have_work && (!nextSendEvent.scheduled())) {
        schedule(nextSendEvent, schedule_tick);
        firstPortAvailableTime = MaxTick;
    }
}

void
Model::processNextSendEvent()
{
    int req_buf_before = requestBuffer.size();
    for (int i = 0; i < sendBandwidth; i++) {
        if (portBusyUntil[i] > curTick()) {
            DPRINTF(Model, "%s: Port[%d] is busy this cycle.\n", __func__, i);
            firstPortAvailableTime = std::min(firstGeneratorAvailableTime, portBusyUntil[i]);
            continue;
        }
        if (requestBuffer.empty()) {
            DPRINTF(Model, "%s: No packets to send this cycle.\n", __func__);
            break;
        }
        Tick insertion_time = requestBufferInsertionTime.front();
        PacketPtr pkt = requestBuffer.front();
        if (ticksToCycles(curTick() - insertion_time) < 1) {
            DPRINTF(Model, "%s: Packet at front of requestBuffer is not ready to be sent this cycle.\n", __func__);
            break;
        }
        DPRINTF(Model, "%s: Sending pkt: %s to port[%d].\n", __func__, pkt->print(), i);
        port.sendPacket(pkt);
        requestBuffer.pop();
        requestBufferInsertionTime.pop();
        // NOTE: We assume the port will be busy for 1 cycle.
        portBusyUntil[i] = clockEdge(Cycles(1));
        firstPortAvailableTime = std::min(firstPortAvailableTime, portBusyUntil[i]);
        // Now if we put the port in blocked state no point in continuing
        // the loop. also no point in scheduling nextSendEvent.
        if (port.blocked()) {
            nextSendEvent.sleep();
            break;
        }
    }
    bool did_work = (req_buf_before - requestBuffer.size()) > 0;
    if (did_work && (nextGenEvent.pending())) {
        // since this event might open up space for output of nextGenEvent,
        // it should wake it up if nextGenEvent is asleep.
        nextGenEvent.wake();
        scheduleNextGenEvent(nextCycle());
    }
    if (!nextSendEvent.pending()) {
        scheduleNextSendEvent(nextCycle());
    }
}

} // namespace gem5
