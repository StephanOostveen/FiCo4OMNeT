#ifndef FICO4OMNET_CANLYSINKAPP_H_
#define FICO4OMNET_CANLYSINKAPP_H_

#include "CanDataFrame_m.h"
#include "omnetpp/clistener.h"
#include "omnetpp/cmessage.h"
#include "omnetpp/csimplemodule.h"
#include "omnetpp/simtime_t.h"

#include "FrameRequest_m.h"
#include "ScheduleMsg_m.h"

#include <unordered_map>
#include <vector>

namespace FiCo4OMNeT {
class CanLySinkApp : public omnetpp::cSimpleModule {
public:
	~CanLySinkApp() noexcept override;

	/* Register that the given frame should be received*/
	void registerFrame();

protected:
	void initialize() override;
	void handleMessage(omnetpp::cMessage* msg) override;

private:
	class BufferedFrame {
	public:
		explicit BufferedFrame(CanDataFrame* input) noexcept
		    : frame(input) {}

		~BufferedFrame() noexcept { delete frame; }
		BufferedFrame(const BufferedFrame& other)
		    : frame(other.frame->dup())
		    , readCount(other.readCount) {}

		BufferedFrame& operator=(const BufferedFrame& other) {
			if (this != &other) {
				delete frame;
				frame     = other.frame->dup();
				readCount = other.readCount;
			}
			return *this;
		}

		[[nodiscard]] CanDataFrame* read() noexcept {
			++readCount;
			return frame;
		}

		[[nodiscard]] unsigned int getReadCount() const noexcept { return readCount; }

	private:
		CanDataFrame* frame{nullptr};
		unsigned int  readCount{0};
	};

	void handleFrameRequest(FrameRequest* request);
	void handleIncomingFrame(CanDataFrame* frame);
	void handleResume();
	void handlePause();
	void handleSelfMsg();

	TaskState                             state{TaskState::Blocked};
	std::map<unsigned int, BufferedFrame> softwareBuffer{};
	omnetpp::simtime_t                    executionTime{};
	omnetpp::simtime_t                    executionTimeLeft{};

	omnetpp::simsignal_t rxDFSignal;
	omnetpp::simsignal_t overrunSignal;    // Multiple writes without read
	omnetpp::simsignal_t underrunSignal;   // Multiple reads without write
	omnetpp::simsignal_t softbufferLengthSignal;

	omnetpp::cMessage*         selfmsg{nullptr};
	std::vector<CanDataFrame*> receivedFrames{};
};
}   // namespace FiCo4OMNeT

#endif