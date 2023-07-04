#include "CanLySourceApp.h"

#include "CanDataFrame_m.h"
#include "CanPortInput.h"
#include "CanTrafficSourceAppBase.h"

#include "ScheduleMsg_m.h"
#include "omnetpp/ccontextswitcher.h"
#include "omnetpp/cexception.h"
#include "omnetpp/checkandcast.h"
#include "omnetpp/clistener.h"
#include "omnetpp/cmessage.h"
#include "omnetpp/cmodule.h"
#include "omnetpp/csimulation.h"
#include "omnetpp/regmacros.h"
#include "omnetpp/simkerneldefs.h"

#include <algorithm>
#include <cstddef>
#include <cstring>
#include <iterator>
#include <limits>
#include <optional>

namespace FiCo4OMNeT {
Define_Module(CanLySourceApp);   // NOLINT

CanLySourceApp::~CanLySourceApp() noexcept {
	for (auto frame : softwareBuffer) {
		cancelAndDelete(frame.second);
	}
	cancelAndDelete(selfmsg);
	cancelAndDelete(scheduleMsg);
}

void CanLySourceApp::initialize(int stage) {
	if (stage == 0) {
		auto* listener = static_cast<cIListener*>(this);
		getSimulation()->getSystemModule()->subscribe("length", listener);
		auto size = getParentModule()->par("hardwareBufferSize").intValue();
		if (0 <= size
		    && static_cast<omnetpp::uintval_t>(size)
		           <= std::numeric_limits<omnetpp::uintval_t>::max()) {
			maxHardwareBufferLength = static_cast<omnetpp::uintval_t>(size);
		} else {
			throw omnetpp::cRuntimeError(
			    "[CanTrafficSourceAppBase::initialize] hardwareBufferSize param "
			    "contained an illegal value");
		}
		executionTime = par("executionTime").doubleValue();
		period        = par("period").doubleValue();

		overrunSignal      = registerSignal("overrun");
		bufferLengthSignal = registerSignal("softbufferLength");

		// NOLINTBEGIN(cppcoreguidelines-owning-memory)
		selfmsg     = new omnetpp::cMessage{"CANLySourceApp execution finished"};
		scheduleMsg = new omnetpp::cMessage{"CANLySourceApp rescheduled itself"};
		// NOLINTEND(cppcoreguidelines-owning-memory)
	}
	CanTrafficSourceAppBase::initialize(stage);
}

// TODO: add signal name check
void CanLySourceApp::receiveSignal(omnetpp::cComponent* src, omnetpp::simsignal_t /*unused*/,
                                   omnetpp::uintval_t   value, omnetpp::cObject* /*unused*/) {
	if (auto* srcModule = dynamic_cast<omnetpp::cModule*>(src); srcModule != nullptr) {
		// src is a module and not a channel
		if (this->getParentModule()->containsModule(srcModule)
		    && std::strstr(srcModule->getNedTypeName(), "CanOutputBuffer") != nullptr) {
			// src is a CanOutputBuffer from this cannode
			// NOLINTNEXTLINE(hicpp-no-array-decay,cppcoreguidelines-pro-bounds-array-to-pointer-decay)
			EV << getFullPath() << " received a 'length=" << value << "' signal from cModule "
			   << src->getFullPath() << "\n";
			hardwareBufferLength = value;
			if (state == TaskState::Blocked && value < maxHardwareBufferLength
			    && 0 < bufferSize()) {
				// We were blocked but there is room available in the hardwarebuffer
				auto* msg = new SchedulerEvent();   // NOLINT(cppcoreguidelines-owning-memory)
				msg->setState(TaskState::Ready);
				send(msg, "scheduler$o");
				state = TaskState::Ready;
			}
		} else {
			// NOLINTNEXTLINE(hicpp-no-array-decay,cppcoreguidelines-pro-bounds-array-to-pointer-decay)
			EV << getFullPath() << " received a 'length' signal from cModule " << src->getFullPath()
			   << "\n";
		}
	} else {
		// NOLINTNEXTLINE(hicpp-no-array-decay,cppcoreguidelines-pro-bounds-array-to-pointer-decay)
		EV << getFullPath() << " received a 'length' signal from something that wasnt a cModule "
		   << src->getFullPath() << "\n";
	}
}

void CanLySourceApp::registerFrame(unsigned int frameId, const char* busName) {
	Enter_Method_Silent();

	auto nrOfCanBusses = getParentModule()->getSubmoduleVectorSize("canDevice");
	bool found         = false;
	for (int i = 0; i < nrOfCanBusses; ++i) {
		const auto*       device = getParentModule()->getSubmodule("canDevice", i);
		const auto* const canBus =
		    device->gate("gate$o")->getPathEndGate()->getOwnerModule()->getParentModule();

		if (std::strcmp(busName, canBus->getName()) == 0) {
			found      = true;
			auto* port = omnetpp::check_and_cast<CanPortInput*>(
			    device->getSubmodule("canNodePort")->getSubmodule("canPortInput"));
			port->registerOutgoingDataFrame(frameId, this->gate("remoteIn"));
			break;
		}
	}
	if (!found) {
		throw omnetpp::cRuntimeError("failed to find bus:\"%s\"", busName);
	}
	// TODO: merge identical loops
	found = false;
	for (int i = 0; i < gateSize("out"); ++i) {
		auto* outGate     = gate("out", i);
		auto* endGate     = outGate->getPathEndGate();   // CanOutputBuffer.in[0]
		auto* lyCanDevice = endGate->getOwnerModule()->getParentModule();
		auto* canBus =
		    lyCanDevice->gate("gate$o")->getPathEndGate()->getOwnerModule()->getParentModule();
		if (std::strcmp(busName, canBus->getName()) == 0) {
			found             = true;
			busIndex[busName] = outGate->getId();
			break;
		}
	}
	if (!found) {
		throw omnetpp::cRuntimeError("failed to find bus from the out[] gate:\"%s\"", busName);
	}
}

int CanLySourceApp::frameToGateId(const CanDataFrame* frame) {
	if (busIndex.find(frame->getBusName()) != std::cend(busIndex)) {
		return busIndex[frame->getBusName()];
	}
	throw omnetpp::cRuntimeError("Couldnt find gate for busname:\"%s\"", frame->getBusName());
}

void CanLySourceApp::handleMessage(omnetpp::cMessage* msg) {
	if (msg->arrivedOn("remoteIn")) {
		delete msg;   // NOLINT(cppcoreguidelines-owning-memory)
		throw omnetpp::cRuntimeError("CanLySourceApp received a remote frame");
	}

	if (msg->isSelfMessage() && msg == selfmsg) {
		// Execution time of task finished.
		if (!msgToSend) {
			delete msg;   // NOLINT(cppcoreguidelines-owning-memory)
			throw omnetpp::cRuntimeError("CanLySourceApp tried to transmit a non existing frame");
		}
		if (hardwareBufferLength >= maxHardwareBufferLength) {
			delete msg;   // NOLINT(cppcoreguidelines-owning-memory)
			throw omnetpp::cRuntimeError(
			    "CanLySourceApp tried to transmit a frame while the hardware buffer was full");
		}
		if (state != TaskState::Running) {
			delete msg;   // NOLINT(cppcoreguidelines-owning-memory)
			throw omnetpp::cRuntimeError(
			    "CanLySourceApp received self message while not in running state");
		}
		// Forward frame
		auto* frame                = softwareBuffer[*msgToSend];
		softwareBuffer[*msgToSend] = nullptr;
		msgToSend                  = std::nullopt;
		if (hasGUI()) {
			bubble("sent buffered frame to hardware");
		}
		send(frame, frameToGateId(frame));
		emit(sentDFSignal, frame);
		emit(bufferLengthSignal, bufferSize());

		auto* blockedMsg = new SchedulerEvent();   // NOLINT(cppcoreguidelines-owning-memory)
		blockedMsg->setState(TaskState::Blocked);
		send(blockedMsg, "scheduler$o");
		state = TaskState::Blocked;
		// Determine next state
		if (bufferSize() != 0) {
			// Buffer not empty reschedule yourself
			scheduleAfter(period, scheduleMsg);
		}
	} else if (msg->isSelfMessage() && msg == scheduleMsg) {
		if (state != TaskState::Blocked) {
			// TODO: Task overrun, rewrite similar to Logical
		}
		auto* readyMsg = new SchedulerEvent();   // NOLINT(cppcoreguidelines-owning-memory)
		readyMsg->setState(TaskState::Ready);
		send(readyMsg, "scheduler$o");
		state = TaskState::Ready;
	} else if (msg->arrivedOn("in")) {
		// Received CANDataFrame from a task, forward or store it, mark task ready when needed
		auto* df = omnetpp::check_and_cast<CanDataFrame*>(msg);
		handleIncomingFrame(df);
	} else if (msg->arrivedOn("scheduler$i")) {
		auto* event = omnetpp::check_and_cast<SchedulerEvent*>(msg);
		switch (event->getState()) {
		case TaskState::Running:
			// Received message from scheduler to resume
			delete msg;   // NOLINT(cppcoreguidelines-owning-memory)
			handleResume();
			break;
		case TaskState::Paused:
			// Received message from scheduler to pause
			delete msg;   // NOLINT(cppcoreguidelines-owning-memory)
			handlePause();
			break;
		case TaskState::Ready:
		case TaskState::Blocked:
		default:
			delete msg;   // NOLINT(cppcoreguidelines-owning-memory)
			throw omnetpp::cRuntimeError(
			    "CanLySourceApp received illegal taskstate from scheduler");
			break;
		}
	} else {
		delete msg;   // NOLINT(cppcoreguidelines-owning-memory)
		throw omnetpp::cRuntimeError("CanLySourceApp received a message on an unhandled port");
	}
}

void CanLySourceApp::handleIncomingFrame(CanDataFrame* frame) {
	if (frame->getRtr()) {
		throw omnetpp::cRuntimeError(
		    "[CanLySourceApp::handleIncomingFrame] incoming frame was a remote frame");
	}

	if (hardwareBufferLength < maxHardwareBufferLength && 0 == bufferSize()) {
		// hardware buffer has room and software buffer is empty.
		if (hasGUI()) {
			bubble("sent frame to hardware");
		}
		send(frame, frameToGateId(frame));
		emit(sentDFSignal, frame);
	} else {
		// We need to store the message in a software buffer
		auto& it = softwareBuffer[frame->getCanID()];
		if (it != nullptr) {
			// message overrun, emit signal and override stored value
			emit(overrunSignal, frame->getCanID());
			delete it;   // NOLINT(cppcoreguidelines-owning-memory)
		}
		it = frame;

		auto size = bufferSize();
		emit(bufferLengthSignal, size);

		if (state == TaskState::Blocked) {
			auto* msg = new SchedulerEvent();   // NOLINT(cppcoreguidelines-owning-memory)
			msg->setState(TaskState::Ready);
			send(msg, "scheduler$o");
			state = TaskState::Ready;
		}
	}
}
void CanLySourceApp::handleResume() {
	if (hasGUI()) {
		bubble("resumed");
	}
	if (state == TaskState::Running) {
		throw omnetpp::cRuntimeError("CANLySourceApp received resume while state was Running");
	}
	if (state == TaskState::Blocked) {
		throw omnetpp::cRuntimeError("CANLySourceApp received resume while state was Blocked");
	}
	if (state == TaskState::Ready) {
		executionTimeLeft = executionTime;
		state             = TaskState::Running;
		msgToSend         = nextFrame();
	}
	scheduleAfter(executionTimeLeft, selfmsg);
}

void CanLySourceApp::handlePause() {
	if (hasGUI()) {
		bubble("paused");
	}
	if (state == TaskState::Ready) {
		throw omnetpp::cRuntimeError("CANLySourceApp received resume while state was Ready");
	}
	if (state == TaskState::Blocked) {
		throw omnetpp::cRuntimeError("CANLySourceApp received resume while state was Blocked");
	}
	if (state == TaskState::Paused) {
		throw omnetpp::cRuntimeError("CANLySourceApp received resume while state was Paused");
	}
	auto startTime    = selfmsg->getSendingTime();
	auto timePassed   = omnetpp::simTime() - startTime;
	executionTimeLeft = executionTimeLeft - timePassed;
	state             = TaskState::Paused;
}

long CanLySourceApp::bufferSize() const {
	return std::count_if(std::cbegin(softwareBuffer), std::cend(softwareBuffer),
	                     [](const auto& tuple) noexcept { return tuple.second != nullptr; });
}

std::optional<unsigned int> CanLySourceApp::nextFrame() const {
	auto it = std::find_if(std::begin(softwareBuffer), std::end(softwareBuffer),
	                       [](const auto& tuple) noexcept { return tuple.second != nullptr; });

	if (it == std::cend(softwareBuffer)) {
		// Software buffer is empty
		return {std::nullopt};
	}

	return {it->first};
}
}   // namespace FiCo4OMNeT