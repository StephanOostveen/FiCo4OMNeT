#include "CanLySinkApp.h"
#include "CanDataFrame_m.h"
#include "ScheduleMsg_m.h"
#include "omnetpp/cexception.h"
#include "omnetpp/checkandcast.h"
#include "omnetpp/cmessage.h"
#include "omnetpp/regmacros.h"
#include <iterator>

namespace FiCo4OMNeT {
Define_Module(CanLySinkApp);

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
	selfmsg       = new omnetpp::cMessage{"CANLySinkApp execution finished"};
}

void CanLySinkApp::handleMessage(omnetpp::cMessage* msg) {
	if (msg->isSelfMessage()) {
		// Execution of receiving can frames from the bus has finished.
		delete msg;
		handleSelfMsg();
	} else if (msg->arrivedOn("getFrame")) {
		// Logical/Application requests a dataframe, the requested frame id is in the msg.
		auto* request = omnetpp::check_and_cast<FrameRequest*>(msg);
		handleFrameRequest(request);
		// Delete is handled by handleFrameRequest
	} else if (msg->arrivedOn("controllerIn")) {
		delete msg;
		throw omnetpp::cRuntimeError("CanLySinkApp shouldnt receive a controllerIn msg");
	} else if (msg->arrivedOn("dataIn")) {
		auto* frame = omnetpp::check_and_cast<CanDataFrame*>(msg);
		handleIncomingFrame(frame);
		// No delete needed as frame is stored in local buffer
	} else if (msg->arrivedOn("scheduler")) {
		auto* schedulerMsg = omnetpp::check_and_cast<SchedulerEvent*>(msg);
		if (schedulerMsg->getState() == TaskState::Running) {
			delete msg;
			handleResume();
		} else if (schedulerMsg->getState() == TaskState::Paused) {
			delete msg;
			handlePause();
		} else {
			delete msg;
			throw omnetpp::cRuntimeError("CANLySinkApp received illegal TaskState");
		}
	} else {
		delete msg;
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
		if (auto it = softwareBuffer.find(frame->getCanID()); it != std::end(softwareBuffer)) {
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
	bubble("Updated buffered frames");
	state              = TaskState::Blocked;
	auto* blockedFrame = new SchedulerEvent();
	blockedFrame->setState(TaskState::Blocked);
	send(blockedFrame, "scheduler$o");
}

/*
 * A logical requested the latest available copy of a frame, find and transmit the copy from the
 * software buffer. If no frame with the requested CAN ID was available, an error is thrown, all CAN
 * IDs should be registered before the simulation starts.
 */
void CanLySinkApp::handleFrameRequest(FrameRequest* request) {
	if (auto it = softwareBuffer.find(request->getFrameID()); it != std::cend(softwareBuffer)) {
		auto* gate  = request->getArrivalGate();
		auto* frame = it->second.read();
		send(frame->dup(), gate->getOtherHalf());

		if (it->second.getReadCount() > 1) {
			// Multiple reads occured on the same received frame.
			emit(underrunSignal, frame->getCanID());
		}
		delete request;
	} else {
		delete request;
		throw omnetpp::cRuntimeError("CanLySinkApp unregistered CAN ID request");
	}
}

void CanLySinkApp::handleIncomingFrame(CanDataFrame* frame) {
	// Received frame from the CAN bus, store it temporarily until task completion
	bubble("Received frame");
	receivedFrames.emplace_back(frame);
	if (state == TaskState::Blocked) {
		state          = TaskState::Ready;
		auto* readyMsg = new SchedulerEvent();
		readyMsg->setState(TaskState::Ready);
		send(readyMsg, "scheduler$o");
	}
	emit(rxDFSignal, frame);
	emit(softbufferLengthSignal, receivedFrames.size());
}

void CanLySinkApp::handleResume() {
	bubble("resumed");
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
	bubble("paused");
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