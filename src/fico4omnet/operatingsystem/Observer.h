#ifndef FICO4OMNET_OBSERVER_H_
#define FICO4OMNET_OBSERVER_H_

#include "omnetpp/clistener.h"
#include "omnetpp/csimplemodule.h"

#include <deque>
#include <tuple>
#include <unordered_map>
#include <utility>

#include "omnetpp/simtime_t.h"

#include "DataDictionaryValue_m.h"
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
	enum class Node { Unknown, CGW, SCU, VCU };
	void                 readSignal(omnetpp::cObject* value, Node n);
	void                 writeSignal(omnetpp::cObject* value);
	omnetpp::simsignal_t createSignal(std::string node, const DataDictionaryValue* ddValue);

	using Record     = std::pair<unsigned long, omnetpp::simtime_t>;   // (write count, timestamp)
	using BufferType = std::deque<Record>;
	using CGWSignal  = omnetpp::simsignal_t;
	using SCUSignal  = omnetpp::simsignal_t;
	using VCUSignal  = omnetpp::simsignal_t;
	using SignalBufferT = std::tuple<CGWSignal, SCUSignal, VCUSignal, BufferType>;

	static constexpr size_t BufferLimit = 100;

	std::unordered_map<std::string, SignalBufferT> database{};
};
}   // namespace FiCo4OMNeT
#endif