
#ifndef __MODEL_MODEL_HH__
#define __MODEL_MODEL_HH__

#include <deque>
#include <queue>
#include <vector>

#include "enums/SpatterKernelType.hh"
#include "mem/packet.hh"
#include "mem/port.hh"
#include "model/enums.hh"
#include "params/Model.hh"
#include "sim/clocked_object.hh"
#include "sim/eventq.hh"

namespace gem5
{

using enums::SpatterKernelType;

class Model: public ClockedObject
{
  private:
    class Kernel {
      private:
        uint32_t _id;
        uint32_t _delta;
        uint32_t _count;

        SpatterKernelType _type;

        // needed to iterate over _indices multiple times.
        uint32_t _index;
        uint32_t _tail;
        std::deque<uint32_t> _indices;

        // current iteration over _indices
        uint32_t _iteration;
      public:
        Kernel(uint32_t id, uint32_t delta, uint32_t count, SpatterKernelType type):
            _id(id), _delta(delta), _count(count), _type(type),
            _index(0), _tail(0), _iteration(0) {}

        uint32_t id() { return _id; }
        SpatterKernelType type() { return _type; }
        void setIndices(std::vector<uint32_t> indices)
        {
            _indices.assign(indices.begin(), indices.end());
            _tail = indices.size();
        }
        bool done() { return _iteration == _count; }
        std::tuple<uint32_t, uint32_t> next()
        {
            int front = _indices.front();
            int ret = (_delta * _iteration) + front;
            _indices.pop_front();
            _indices.push_back(front);
            _index++;
            _tail--;
            if (_tail == 0) {
                _tail = _indices.size();
                _iteration++;
            }
            return std::make_tuple(_index - 1, ret);
        }
    };

    class SleepyEvent : public EventFunctionWrapper
    {
      // TODO: split pending into pendingInput and pendingOutput
      private:
        SleepyState _state;

      public:
        SleepyEvent(const std::function<void(void)> &callback,
                    const std::string &name):
            EventFunctionWrapper(callback, name), _state(SleepyState::AWAKE)
        {}
        // a SleepyEvent will only be asleep if it is pending output bandwidth
        bool pending() { return _state == SleepyState::ASLEEP; }
        void sleep() { _state = SleepyState::ASLEEP; }
        void wake() { _state = SleepyState::AWAKE; }
    };

    class ModelPort: public RequestPort
    {
      private:
        Model* owner;
        PacketPtr blockedPacket;

      public:
        ModelPort(Model* owner, const std::string& name):
            RequestPort(name), owner(owner), blockedPacket(nullptr) {}

        void sendPacket(PacketPtr pkt);
        bool blocked() const { return blockedPacket != nullptr; }

      protected:
        virtual bool recvTimingResp(PacketPtr pkt) override;
        virtual void recvReqRetry() override;
    };

    struct IndexInfo : public Packet::SenderState
    {
        uint32_t _value;
        SpatterKernelType _type;
        IndexInfo(uint32_t value, SpatterKernelType type):
            _value(value), _type(type) {}
        uint32_t value() { return _value; }
        SpatterKernelType type() { return _type; }
    };
    std::queue<Kernel> kernels;

    RequestorID requestorId;
    ModelPort port;

    // base physical address for the indexer array
    Addr baseIndexerAddr;
    // base physical address for the values array
    Addr baseValuesAddr;

    // size of the register files,
    //for every memory instruction we need to allocate one register.
    int intRegFileSize;
    int intRegUsed;
    int fpRegFileSize;
    int fpRegUsed;
    // laterncy to generate A request
    int requestGenLatency;
    // number of requests generated per event
    int requestGenBandwidth;
    // tracking smallest tick when at least one "AGU" is available;
    Tick firstGeneratorAvailableTime;
    // tracking the busy state of our so called "AGU"s.
    std::vector<Tick> generatorBusyUntil;
    SleepyEvent nextGenEvent;
    void processNextGenEvent();
    // put requests to the cache in the request buffer.
    int requestBufferSize;
    std::queue<PacketPtr> requestBuffer;
    // same explanation as receiveBufferInsertionTime
    std::queue<Tick> requestBufferInsertionTime;
    // if nextGenEvent has to be schedule at tick when then schedule it.
    // this function should only be called when nextGenEvent is not pending.
    void scheduleNextGenEvent(Tick when);

    // bandwidth to issue memory requests to cache,
    // this is supposed to model the number of cache ports
    // we will assume it takes 1 cycle to issue memory requests
    int sendBandwidth;
    Tick firstPortAvailableTime;
    std::vector<Tick> portBusyUntil;
    SleepyEvent nextSendEvent;
    void processNextSendEvent();
    // if nextSendEvent has to be schedule at tick when then schedule it.
    // this function should only be called when nextSendEvent is not pending.
    void scheduleNextSendEvent(Tick when);

    // put the memory responses here.
    // no need to limit the size of this buffer.
    // it's a response buffer and it will automatically
    // be limited by requestBufferSize, intRegFileSize, fpRegFileSize
    std::queue<PacketPtr> receiveBuffer;
    // track insertion time for every packet in receiveBuffer
    // we use it to correctly impose a minimum cycle
    // latency before dequeueing any packet.
    std::queue<Tick> receiveBufferInsertionTime;

  public:
    PARAMS(Model);
    Model(const Params& params);

    Port&
    getPort(const std::string& if_name, PortID idx = InvalidPortID) override;

    virtual void startup() override;

    void recvReqRetry();
    bool recvTimingResp(PacketPtr pkt);

    // PyBindMethod to interface adding a kernel with python JSON frontend.
    void addKernel(uint32_t id, uint32_t delta, uint32_t count, SpatterKernelType type, std::vector<uint32_t> indices);
};

} // namespace gem5

#endif // __MODEL_MODEL_HH__
