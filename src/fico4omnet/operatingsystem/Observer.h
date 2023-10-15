#ifndef FICO4OMNET_OBSERVER_H_
#define FICO4OMNET_OBSERVER_H_

#include "omnetpp/clistener.h"
#include "omnetpp/csimplemodule.h"

#include <deque>
#include <unordered_map>
#include <utility>

#include "omnetpp/simtime_t.h"

namespace FiCo4OMNeT {
class Observer
    : public omnetpp::cListener
    , public omnetpp::cSimpleModule {
public:
	void receiveSignal(omnetpp::cComponent* src, omnetpp::simsignal_t id, omnetpp::cObject* value,
	                   omnetpp::cObject* details) override;

protected:
	void initialize() override;
	void handleMessage(omnetpp::cMessage* msg) override;

private:
	void readSignal(omnetpp::cObject* value);
	void writeSignal(omnetpp::cObject* value);

	using Record        = std::pair<unsigned long, omnetpp::simtime_t>;
	using BufferType    = std::deque<Record>;
	using SignalBufferT = std::pair<omnetpp::simsignal_t, BufferType>;

	static constexpr size_t BufferLimit = 100;

	std::unordered_map<std::string, SignalBufferT> database{};
};
}   // namespace FiCo4OMNeT
#endif