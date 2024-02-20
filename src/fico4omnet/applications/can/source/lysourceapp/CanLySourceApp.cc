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
#include <string>

namespace FiCo4OMNeT {
Define_Module(CanLySourceApp);   // NOLINT

CanLySourceApp::~CanLySourceApp() noexcept {
	for (auto& [busName, buffer] : softwareBuffer) {
		for (auto& [canId, frame] : buffer) {
			cancelAndDelete(frame);
		}
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

		overrunSignal = registerSignal("overrun");

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
	Enter_Method_Silent();
	if (auto* srcModule = dynamic_cast<omnetpp::cModule*>(src); srcModule != nullptr) {
		// src is a module and not a channel
		if (this->getParentModule()->containsModule(srcModule)
		    && std::strstr(srcModule->getNedTypeName(), "CanOutputBuffer") != nullptr) {
			// src is a CanOutputBuffer from this cannode
			// NOLINTNEXTLINE(hicpp-no-array-decay,cppcoreguidelines-pro-bounds-array-to-pointer-decay)
			EV << omnetpp::simTime() << " received a 'length=" << value << "' signal from cModule "
			   << src->getFullPath() << "\n";
			const auto*       canDevice = srcModule->getParentModule();
			const auto* const canBus =
			    canDevice->gate("gate$o")->getPathEndGate()->getOwnerModule()->getParentModule();
			auto [it, success] = busInfo.insert({canBus->getName(), {0, 0, 0}});
			if (success) {
				// Inserted a bus from this lyphysical, get the appropriate gateid and insert it.
				auto gateID = canDevice->gate("tx")->getPathStartGate()->getId();
				it->second  = createBusInfo(canBus->getName(), gateID);
				softwareBuffer[canDevice->getName()] = {};
				EV << omnetpp::simTime() << " Inserted a new canbus:'" << canBus->getName()
				   << "' into the canbusinfo map\n";
			}
			it->second.hardwareBufferSize = value;
			if (state == TaskState::Blocked && value < maxHardwareBufferLength
			    && 0 < bufferSize(canBus->getName()) && !scheduleMsg->isScheduled()) {
				// We were blocked but there is room available in the hardwarebuffer
				auto* msg = new SchedulerEvent();   // NOLINT(cppcoreguidelines-owning-memory)
				msg->setState(TaskState::Ready);
				send(msg, "scheduler$o");
				state = TaskState::Ready;
				EV << omnetpp::simTime() << " Became ready due to received length signal\n";
			}
		}
	}
}

void CanLySourceApp::registerFrame(unsigned int frameId, const std::string& busName) {
	Enter_Method_Silent();
	// NOLINTNEXTLINE(hicpp-no-array-decay,cppcoreguidelines-pro-bounds-array-to-pointer-decay)
	EV << "Registering to transmit frame:" << frameId << " on bus: " << busName << "\n";

	auto nrOfCanBusses = getParentModule()->getSubmoduleVectorSize("canDevice");
	bool found         = false;
	for (int i = 0; i < nrOfCanBusses; ++i) {
		const auto*       device = getParentModule()->getSubmodule("canDevice", i);
		const auto* const canBus =
		    device->gate("gate$o")->getPathEndGate()->getOwnerModule()->getParentModule();

		if (std::strcmp(busName.c_str(), canBus->getName()) == 0) {
			found      = true;
			auto* port = omnetpp::check_and_cast<CanPortInput*>(
			    device->getSubmodule("canNodePort")->getSubmodule("canPortInput"));
			port->registerOutgoingDataFrame(frameId, this->gate("remoteIn"));
			break;
		}
	}
	if (!found) {
		throw omnetpp::cRuntimeError("failed to find bus:\"%s\"", busName.c_str());
	}
	// TODO: merge identical loops
	found = false;
	for (int i = 0; i < gateSize("out"); ++i) {
		auto* outGate     = gate("out", i);
		auto* endGate     = outGate->getPathEndGate();   // CanOutputBuffer.in[0]
		auto* lyCanDevice = endGate->getOwnerModule()->getParentModule();
		auto* canBus =
		    lyCanDevice->gate("gate$o")->getPathEndGate()->getOwnerModule()->getParentModule();
		if (std::strcmp(busName.c_str(), canBus->getName()) == 0) {
			found = true;
			if (auto it = busInfo.find(busName); it == std::cend(busInfo)) {
				busInfo[busName]        = createBusInfo(busName, outGate->getId());
				softwareBuffer[busName] = {};
			}
			break;
		}
	}
	if (!found) {
		throw omnetpp::cRuntimeError("failed to find bus from the out[] gate:\"%s\"",
		                             busName.c_str());
	}
}

CanLySourceApp::BusInfo CanLySourceApp::createBusInfo(const std::string& busName, int gateID) {
	std::string signalName(busName);
	signalName += "_softBufferLength";
	auto  bufferLengthSignal = registerSignal(signalName.c_str());
	auto* warmupFilterVector = omnetpp::cResultFilterType::get("warmup")->create();
	auto* vectorRecorder     = omnetpp::cResultRecorderType::get("vector")->create();
	auto* warmupFilterStats  = omnetpp::cResultFilterType::get("warmup")->create();
	auto* statsRecorder      = omnetpp::cResultRecorderType::get("stats")->create();

	std::string title{"Software buffer length of bus '"};
	title += busName;

	auto* attributesVector                   = new omnetpp::opp_string_map;
	(*attributesVector)["title"]             = title;
	(*attributesVector)["unit"]              = "packets";
	(*attributesVector)["interpolationmode"] = "sample-hold";

	auto* attributesStats = new omnetpp::opp_string_map(*attributesVector);

	omnetpp::cResultRecorder::Context vectorCtx{this, signalName.c_str(), "vector", nullptr,
	                                            attributesVector};
	omnetpp::cResultRecorder::Context statsCtx{this, signalName.c_str(), "stats", nullptr,
	                                           attributesStats};
	vectorRecorder->init(&vectorCtx);
	statsRecorder->init(&statsCtx);

	subscribe(bufferLengthSignal, warmupFilterVector);
	subscribe(bufferLengthSignal, warmupFilterStats);
	warmupFilterVector->addDelegate(vectorRecorder);
	warmupFilterStats->addDelegate(statsRecorder);
	emit(bufferLengthSignal, 0);
	return {0, bufferLengthSignal, gateID};
}

int CanLySourceApp::frameToGateId(const CanDataFrame* frame) {
	if (const auto it = busInfo.find(frame->getBusName()); it != std::cend(busInfo)) {
		return it->second.gateId;
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
		if (state != TaskState::Running) {
			delete msg;   // NOLINT(cppcoreguidelines-owning-memory)
			throw omnetpp::cRuntimeError(
			    "CanLySourceApp received self message while not in running state");
		}

		auto sendIt = std::begin(msgToSend);
		while (sendIt != std::cend(msgToSend)) {
			auto& [busName, msgOpt] = *sendIt;
			if (!msgOpt) {
				delete msg;   // NOLINT(cppcoreguidelines-owning-memory)
				throw omnetpp::cRuntimeError(
				    "CanLySourceApp tried to transmit a non existing frame");
			}
			auto it = busInfo.find(busName);
			if (it == std::cend(busInfo)
			    || it->second.hardwareBufferSize >= maxHardwareBufferLength) {
				delete msg;   // NOLINT(cppcoreguidelines-owning-memory)
				throw omnetpp::cRuntimeError("CanLySourceApp tried to transmit a frame while "
				                             "the hardware buffer of '%s' was full",
				                             busName.c_str());
			}
			// Forward frame
			auto* frame                      = softwareBuffer[busName][*msgOpt];
			softwareBuffer[busName][*msgOpt] = nullptr;

			send(frame, frameToGateId(frame));
			it->second.hardwareBufferSize = it->second.hardwareBufferSize + 1;

			emit(sentDFSignal, frame);
			emit(busInfo[busName].bufferLengthSignal, bufferSize(busName));
			sendIt = msgToSend.erase(sendIt);
		}
		
		for (auto& [busName, info] : busInfo) {
			auto& [name, buf] = *softwareBuffer.find(busName);
			int count = 0;
			while (info.hardwareBufferSize < maxHardwareBufferLength) {
				auto frameIt =
				    std::find_if(std::begin(buf), std::end(buf), [](const auto& tuple) noexcept {
					    return tuple.second != nullptr;
				    });
				if (frameIt == std::end(buf)) {
					break;
				}

				auto* frame     = frameIt->second;
				frameIt->second = nullptr;
				send(frame, frameToGateId(frame));
				info.hardwareBufferSize = info.hardwareBufferSize + 1;

				emit(sentDFSignal, frame);
				emit(info.bufferLengthSignal, bufferSize(busName));
				++count;
			}
			EV << omnetpp::simTime() << "Put " << count << " frames in the hardware buffer of bus " << busName << "\n";
		}

		auto* blockedMsg = new SchedulerEvent();   // NOLINT(cppcoreguidelines-owning-memory)
		blockedMsg->setState(TaskState::Blocked);
		send(blockedMsg, "scheduler$o");
		state = TaskState::Blocked;
		// Determine next state
		for (const auto& [name, buf] : softwareBuffer) {
			if (bufferSize(name) != 0) {
				// Buffer not empty reschedule yourself
				EV << omnetpp::simTime() << " rescheduled because buffer for bus '" << name
				   << "' wasn't empty\n";
				scheduleAfter(period, scheduleMsg);
				break;
			}
		}

		EV << omnetpp::simTime() << " Execution finished\n";
	} else if (msg->isSelfMessage() && msg == scheduleMsg) {
		if (state == TaskState::Blocked) {
			EV << omnetpp::simTime() << " Was Blocked, became ready due to schedulemsg\n";
			state          = TaskState::Ready;
			auto* readyMsg = new SchedulerEvent();   // NOLINT(cppcoreguidelines-owning-memory)
			readyMsg->setState(TaskState::Ready);
			send(readyMsg, "scheduler$o");
		} else {
			// TODO: Task overrun
			EV << omnetpp::simTime() << "Logical overrun, state was:" << state << "\n";
		}
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
		delete frame;
		throw omnetpp::cRuntimeError(
		    "[CanLySourceApp::handleIncomingFrame] incoming frame was a remote frame");
	}
	const std::string busName(frame->getBusName());
	if (auto it = busInfo.find(busName); it == std::cend(busInfo)) {
		std::string msg("tried to send frame for unknown bus: '");
		msg += frame->getBusName();
		msg += "'";
		delete frame;
		throw omnetpp::cRuntimeError("%s", msg.c_str());
	} else if (it->second.hardwareBufferSize < maxHardwareBufferLength
	           && 0 == bufferSize(busName)) {
		// hardware buffer has room and software buffer is empty.
		EV << omnetpp::simTime() << " Sent frame to hardware " << busName << "\n";
		send(frame, frameToGateId(frame));
		it->second.hardwareBufferSize = it->second.hardwareBufferSize + 1;
		emit(sentDFSignal, frame);
	} else {
		EV << omnetpp::simTime() << " Buffering frame for bus " << busName << "\n";
		// We need to store the message in a software buffer
		auto softBufIt = softwareBuffer.find(busName);
		if (softBufIt == std::cend(softwareBuffer)) {
			throw omnetpp::cRuntimeError("tried to find softbuffer of frame for unknown bus: '%s'",
			                             frame->getBusName());
		}

		auto& message = softBufIt->second[frame->getCanID()];
		if (message != nullptr) {
			// message overrun, emit signal and override stored value
			emit(overrunSignal, frame->getCanID());
			delete message;   // NOLINT(cppcoreguidelines-owning-memory)
		}
		message = frame;

		auto size = bufferSize(busName);
		emit(it->second.bufferLengthSignal, size);

		if (state == TaskState::Blocked) {
			EV << omnetpp::simTime()
			   << " Became ready because we needed to transmit a frame but the software buffer "
			      "was full\n";
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
	if (state == TaskState::Paused) {
		EV << omnetpp::simTime() << " Resumed from pause\n";   // NOLINT
		state = TaskState::Running;
	}
	if (state == TaskState::Ready) {
		EV << omnetpp::simTime() << " Task was ready and became running\n";
		executionTimeLeft = executionTime;
		state             = TaskState::Running;
		selectNextFrames();
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
	EV << omnetpp::simTime() << " Paused\n";
	auto startTime    = selfmsg->getSendingTime();
	auto timePassed   = omnetpp::simTime() - startTime;
	executionTimeLeft = executionTimeLeft - timePassed;
	state             = TaskState::Paused;
	cancelEvent(selfmsg);
}

long CanLySourceApp::bufferSize(const std::string& busName) const {
	const auto& it = softwareBuffer.find(busName);
	if (it == std::cend(softwareBuffer)) {
		return 0;
	}
	return std::count_if(std::cbegin(it->second), std::cend(it->second),
	                     [](const auto& tuple) noexcept { return tuple.second != nullptr; });
}

void CanLySourceApp::selectNextFrames() {
	for (const auto& [busName, buf] : softwareBuffer) {
		auto frameIt = std::find_if(std::begin(buf), std::end(buf), [](const auto& tuple) noexcept {
			return tuple.second != nullptr;
		});
		if (frameIt != std::cend(buf)
		    && busInfo[busName].hardwareBufferSize < maxHardwareBufferLength) {
			EV << omnetpp::simTime() << " Selected to transmit frame id " << frameIt->first
			   << " for bus " << busName << "\n";
			msgToSend[busName] = {frameIt->first};
		}
	}
}
}   // namespace FiCo4OMNeT