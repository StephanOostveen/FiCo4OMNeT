#include "CanLySinkApp.h"
#include "CanDataFrame_m.h"
#include "ScheduleMsg_m.h"
#include "omnetpp/cexception.h"
#include "omnetpp/checkandcast.h"
#include "omnetpp/cmessage.h"
#include "omnetpp/regmacros.h"
#include <iterator>
#include <utility>

namespace FiCo4OMNeT {
Define_Module(CanLySinkApp);   // NOLINT

CanLySinkApp::~CanLySinkApp() noexcept {
	for (auto* frame : receivedFrames) {
		cancelAndDelete(frame);
	}
	cancelAndDelete(selfmsg);
}

void CanLySinkApp::initialize() {
	rxDFSignal     = registerSignal("rxDf");
	overrunSignal  = registerSignal("overrun");
	underrunSignal = registerSignal("underrun");

	softbufferLengthSignal = registerSignal("softbufferLength");

	executionTime = par("executionTime").doubleValue();
	// NOLINTNEXTLINE(cppcoreguidelines-owning-memory)
	selfmsg = new omnetpp::cMessage{"CANLySinkApp execution finished"};
}

void CanLySinkApp::registerFrame(unsigned int frameId, const std::string& busName) {
	Enter_Method_Silent();
	// NOLINTNEXTLINE(hicpp-no-array-decay,cppcoreguidelines-pro-bounds-array-to-pointer-decay)
	EV << "Registering to receive frame:" << frameId << " from bus: " << busName << "\n";
	// std::pair<std::string, unsigned int> key{busName, frameId};
	softwareBuffer.emplace(std::make_pair(busName, frameId), nullptr);
}

void CanLySinkApp::handleMessage(omnetpp::cMessage* msg) {
	if (msg->isSelfMessage()) {
		// Execution of receiving can frames from the bus has finished.
		delete msg;   // NOLINT(cppcoreguidelines-owning-memory)
		handleSelfMsg();
	} else if (msg->arrivedOn("getFrame$i")) {
		// Logical/Application requests a dataframe, the requested frame id is in the msg.
		auto* request = omnetpp::check_and_cast<FrameRequest*>(msg);
		handleFrameRequest(request);
		// Delete is handled by handleFrameRequest
	} else if (msg->arrivedOn("controllerIn")) {
		delete msg;   // NOLINT(cppcoreguidelines-owning-memory)
		throw omnetpp::cRuntimeError("CanLySinkApp shouldnt receive a controllerIn msg");
	} else if (msg->arrivedOn("dataIn")) {
		auto* frame = omnetpp::check_and_cast<CanDataFrame*>(msg);
		handleIncomingFrame(frame);
		// No delete needed as frame is stored in local buffer
	} else if (msg->arrivedOn("scheduler$i")) {
		auto* schedulerMsg = omnetpp::check_and_cast<SchedulerEvent*>(msg);
		if (schedulerMsg->getState() == TaskState::Running) {
			delete msg;   // NOLINT(cppcoreguidelines-owning-memory)
			handleResume();
		} else if (schedulerMsg->getState() == TaskState::Paused) {
			delete msg;   // NOLINT(cppcoreguidelines-owning-memory)
			handlePause();
		} else {
			delete msg;   // NOLINT(cppcoreguidelines-owning-memory)
			throw omnetpp::cRuntimeError("CANLySinkApp received illegal TaskState");
		}
	} else {
		delete msg;   // NOLINT(cppcoreguidelines-owning-memory)
		throw omnetpp::cRuntimeError("Received unhandled message");
	}
}

/*
 * The execution of the reception task finished, all received CAN messages should now be stored in
 * the software buffer such that logicals can request the updated CAN message. The task is then
 * blocked until new messages are received.
 */
void CanLySinkApp::handleSelfMsg() {
	if (receivedFrames.empty()) {
		throw omnetpp::cRuntimeError(
		    "CanLySinkApp executed but the receivedFrames vector was empty");
	}
	if (state != TaskState::Running) {
		throw omnetpp::cRuntimeError(
		    "CanLySinkApp received self message while not in running state");
	}
	for (auto* frame : receivedFrames) {
		std::pair<std::string, unsigned int> key{frame->getBusName(), frame->getCanID()};
		if (auto it = softwareBuffer.find(key); it != std::end(softwareBuffer)) {
			auto readCount = it->second.getReadCount();
			if (readCount == 0) {
				// Old copy of frame was not read, signal overrun.
				emit(overrunSignal, frame->getCanID());
			}
			it->second = BufferedFrame{frame};
		} else {
			throw omnetpp::cRuntimeError("CanLySinkApp buffered a non registered frame");
		}
	}
	receivedFrames.clear();
	emit(softbufferLengthSignal, receivedFrames.size());
	if (hasGUI()) {
		bubble("Updated buffered frames");
	}
	state              = TaskState::Blocked;
	auto* blockedFrame = new SchedulerEvent();   // NOLINT(cppcoreguidelines-owning-memory)
	blockedFrame->setState(TaskState::Blocked);
	send(blockedFrame, "scheduler$o");
}

/*
 * A logical requested the latest available copy of a frame, find and transmit the copy from the
 * software buffer. If no frame with the requested CAN ID was available, an error is thrown, all CAN
 * IDs should be registered before the simulation starts.
 */
void CanLySinkApp::handleFrameRequest(FrameRequest* request) {
	std::pair<std::string, unsigned int> key{request->getBusName(), request->getFrameID()};
	if (auto it = softwareBuffer.find(key); it != std::cend(softwareBuffer)) {
		auto* gate = request->getArrivalGate();
		if (auto* frame = it->second.read(); frame != nullptr) {
			// A valid frame was received previously
			send(frame->dup(), gate->getOtherHalf());

			if (it->second.getReadCount() > 1) {
				// Multiple reads occured on the same received frame.
				emit(underrunSignal, frame->getCanID());
			}
		} else {
			// No valid frame was received yet, create a dummy
			// TODO: add dummy flag or something in the CANDataFrame
			auto* dummy = new CanDataFrame();   // NOLINT(cppcoreguidelines-owning-memory)
			dummy->setBusName(request->getBusName());
			dummy->setCanID(request->getFrameID());
			dummy->setRtr(false);
			dummy->setPeriod(1.0);
			send(dummy, gate->getOtherHalf());
		}
		delete request;   // NOLINT(cppcoreguidelines-owning-memory)
	} else {
		auto        id = request->getFrameID();
		std::string busName{request->getBusName()};
		delete request;   // NOLINT(cppcoreguidelines-owning-memory)
		throw omnetpp::cRuntimeError(
		    "CanLySinkApp unregistered CAN ID request for id: %u on bus:\"%s\"", id,
		    busName.c_str());
	}
}

void CanLySinkApp::handleIncomingFrame(CanDataFrame* frame) {
	// Received frame from the CAN bus, store it temporarily until task completion
	if (hasGUI()) {
		bubble("Received frame");
	}
	receivedFrames.emplace_back(frame);
	if (state == TaskState::Blocked) {
		state          = TaskState::Ready;
		auto* readyMsg = new SchedulerEvent();   // NOLINT(cppcoreguidelines-owning-memory)
		readyMsg->setState(TaskState::Ready);
		send(readyMsg, "scheduler$o");
	}
	emit(rxDFSignal, frame);
	emit(softbufferLengthSignal, receivedFrames.size());
}

void CanLySinkApp::handleResume() {
	if (hasGUI()) {
		bubble("resumed");
	}
	if (state == TaskState::Running) {
		throw omnetpp::cRuntimeError("CANLySinkApp received resume while state was Running");
	}
	if (state == TaskState::Blocked) {
		throw omnetpp::cRuntimeError("CANLySinkApp received resume while state was Blocked");
	}
	if (state == TaskState::Ready) {
		executionTimeLeft = executionTime;
		state             = TaskState::Running;
	}
	scheduleAfter(executionTimeLeft, selfmsg);
}

void CanLySinkApp::handlePause() {
	if (hasGUI()) {
		bubble("paused");
	}
	if (state == TaskState::Ready) {
		throw omnetpp::cRuntimeError("CanLySinkApp received resume while state was Ready");
	}
	if (state == TaskState::Blocked) {
		throw omnetpp::cRuntimeError("CanLySinkApp received resume while state was Blocked");
	}
	if (state == TaskState::Paused) {
		throw omnetpp::cRuntimeError("CanLySinkApp received resume while state was Paused");
	}
	auto startTime    = selfmsg->getSendingTime();
	auto timePassed   = omnetpp::simTime() - startTime;
	executionTimeLeft = executionTimeLeft - timePassed;
	state             = TaskState::Paused;
}
}   // namespace FiCo4OMNeT