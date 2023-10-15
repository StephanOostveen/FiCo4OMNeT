#ifndef FICO4OMNET_CANLYSOURCEAPP_H_
#define FICO4OMNET_CANLYSOURCEAPP_H_

#include "CanDataFrame_m.h"
#include "ScheduleMsg_m.h"
#include "fico4omnet/applications/can/source/CanTrafficSourceAppBase.h"
#include "omnetpp/clistener.h"
#include "omnetpp/cmessage.h"
#include "omnetpp/simtime_t.h"

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
	void registerFrame(unsigned int frameId, const std::string& busName);

	/**
	 * overload from cListener for receiving the buffer lengths of the "hardware" buffer
	 */
	void receiveSignal(omnetpp::cComponent* src, omnetpp::simsignal_t id, omnetpp::uintval_t value,
	                   omnetpp::cObject* details) override;

protected:
	void initialize(int stage) override;
	void handleMessage(omnetpp::cMessage* msg) override;

private:
	struct BusInfo {
		omnetpp::uintval_t   hardwareBufferSize;
		omnetpp::simsignal_t bufferLengthSignal;
		int                  gateId;
	};
	void handleIncomingFrame(CanDataFrame* frame);
	void handleResume();
	void handlePause();

	int frameToGateId(const CanDataFrame* frame);

	long bufferSize(const std::string& busName) const;
	void selectNextFrames();

	BusInfo createBusInfo(const std::string& busName, int gateID);

	TaskState state{TaskState::Blocked};
	std::unordered_map<std::string, std::map<unsigned int, CanDataFrame*>> softwareBuffer{};

	std::unordered_map<std::string, BusInfo> busInfo{};
	omnetpp::uintval_t                       maxHardwareBufferLength{};
	omnetpp::simtime_t                       period{};
	omnetpp::simtime_t                       executionTime{};
	omnetpp::simtime_t                       executionTimeLeft{};

	// txDF signal resides in CANTrafficSourceAppBase
	omnetpp::simsignal_t overrunSignal;

	omnetpp::cMessage* selfmsg{nullptr};
	omnetpp::cMessage* scheduleMsg{nullptr};

	std::unordered_map<std::string, std::optional<unsigned int>> msgToSend;
};
}   // namespace FiCo4OMNeT
#endif