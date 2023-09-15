#ifndef FICO4OMNET_CANLYSOURCEAPP_H_
#define FICO4OMNET_CANLYSOURCEAPP_H_

#include "CanDataFrame_m.h"
#include "ScheduleMsg_m.h"
#include "fico4omnet/applications/can/source/CanTrafficSourceAppBase.h"
#include "omnetpp/clistener.h"
#include "omnetpp/cmessage.h"
#include "omnetpp/simtime_t.h"

#include <map>
#include <optional>
#include <string>
#include <unordered_map>

namespace FiCo4OMNeT {
class CanLySourceApp
    : public CanTrafficSourceAppBase
    , public omnetpp::cListener {
public:
	~CanLySourceApp() noexcept override;

	/*
	 * The given frame is registered as an initialization step before the simulation starts
	 */
	void registerFrame(unsigned int frameId, const char* busName);

	/**
	 * overload from cListener for receiving the buffer lengths of the "hardware" buffer
	 */
	void receiveSignal(omnetpp::cComponent* src, omnetpp::simsignal_t id, omnetpp::uintval_t value,
	                   omnetpp::cObject* details) override;

protected:
	void initialize(int stage) override;
	void handleMessage(omnetpp::cMessage* msg) override;

private:
	void handleIncomingFrame(CanDataFrame* frame);
	void handleResume();
	void handlePause();

	int frameToGateId(const CanDataFrame* frame);

	long                        bufferSize() const;
	std::optional<unsigned int> nextFrame() const;

	TaskState                             state{TaskState::Blocked};
	std::map<unsigned int, CanDataFrame*> softwareBuffer{};
	std::map<std::string, int>            busIndex{};
	omnetpp::uintval_t                    hardwareBufferLength{0};
	omnetpp::uintval_t                    maxHardwareBufferLength{};
	omnetpp::simtime_t                    period{};
	omnetpp::simtime_t                    executionTime{};
	omnetpp::simtime_t                    executionTimeLeft{};

	// txDF signal resides in CANTrafficSourceAppBase
	omnetpp::simsignal_t overrunSignal;
	omnetpp::simsignal_t bufferLengthSignal;

	omnetpp::cMessage* selfmsg{nullptr};
	omnetpp::cMessage* scheduleMsg{nullptr};

	std::optional<unsigned int> msgToSend{std::nullopt};
};
}   // namespace FiCo4OMNeT
#endif