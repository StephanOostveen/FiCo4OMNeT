#include "Logical.h"

#include "CanDataFrame_m.h"
#include "CanList_m.h"
#include "DataDictionaryValue_m.h"
#include "FrameRequest_m.h"
#include "ScheduleMsg_m.h"
#include "omnetpp/cexception.h"
#include "omnetpp/checkandcast.h"
#include "omnetpp/cmessage.h"
#include "omnetpp/csimulation.h"
#include "omnetpp/regmacros.h"

#include <algorithm>
#include <cstddef>
#include <cstring>

namespace FiCo4OMNeT {
Define_Module(Logical);   // NOLINT

Logical::~Logical() noexcept {
	cancelAndDelete(executionMsg);
	cancelAndDelete(scheduleMsg);
	for (auto* dd : receivedFrameDicts) {
		delete dd;   // NOLINT(cppcoreguidelines-owning-memory)
	}
	for (auto* dd : localDicts) {
		delete dd;   // NOLINT(cppcoreguidelines-owning-memory)
	}
}

void Logical::initialize() {
	period        = par("period").doubleValue();
	executionTime = par("executionTime").doubleValue();

	parseCANInput();
	parseCANOutput();

	executionMsg = new omnetpp::cMessage{};   // NOLINT(cppcoreguidelines-owning-memory)
	scheduleMsg  = new omnetpp::cMessage{};   // NOLINT(cppcoreguidelines-owning-memory)
	scheduleAt(0, scheduleMsg);
}

void Logical::parseCANInput() {
	// get the object

	// For each CANDataFrame search the matching bus in the model, validate it exists and this
	// Logical is connected to it. Register the CAN frame reception in the Sink app, add the
	// relevant gate in the canInput vector.

	const auto* const ptr  = omnetpp::check_and_cast<const CanList*>(par("canInput").objectValue());
	const auto        size = ptr->getDefinitionArraySize();
	for (size_t i = 0; i < size; ++i) {
		const auto& frameDefinition = ptr->getDefinition(i);
	}
}

void Logical::handleMessage(omnetpp::cMessage* msg) {
	if (msg->isSelfMessage() && msg == scheduleMsg) {
		if (state == TaskState::Blocked) {
			state          = TaskState::Ready;
			auto* readyMsg = new SchedulerEvent();   // NOLINT(cppcoreguidelines-owning-memory)
			readyMsg->setState(TaskState::Ready);
			send(readyMsg, "scheduler$o");
		} else {
			// TODO: Task overrun
		}
		scheduleAfter(period, scheduleMsg);
	} else if (msg->isSelfMessage() && msg == executionMsg) {
		// Execution finished
		if (state != TaskState::Running) {
			delete msg;   // NOLINT(cppcoreguidelines-owning-memory)
			throw omnetpp::cRuntimeError(
			    "Logical received self message while not in running state");
		}
		writeDataDicts();
		sendCANFrames();

		auto* blockedMsg = new SchedulerEvent();   // NOLINT(cppcoreguidelines-owning-memory)
		blockedMsg->setState(TaskState::Blocked);
		send(blockedMsg, "scheduler$o");
		state = TaskState::Blocked;

	} else if (msg->arrivedOn("getFrame$i")) {
		auto* frame = omnetpp::check_and_cast<CanDataFrame*>(msg);
		localyStoreReceivedFrame(frame);
	} else if (msg->arrivedOn("getDataDict$i")) {
		auto* value = omnetpp::check_and_cast<DataDictionaryValue*>(msg);
		localyStoreReadDataDict(value);
	} else if (msg->arrivedOn("scheduler$i")) {
		auto* event = omnetpp::check_and_cast<SchedulerEvent*>(msg);
		switch (event->getState()) {
		case TaskState::Running:
			delete msg;   // NOLINT(cppcoreguidelines-owning-memory)
			handleResume();
			break;
		case TaskState::Paused:
			delete msg;   // NOLINT(cppcoreguidelines-owning-memory)
			handlePause();
			break;
		case TaskState::Ready:
		case TaskState::Blocked:
		default:
			delete msg;   // NOLINT(cppcoreguidelines-owning-memory)
			throw omnetpp::cRuntimeError("Logical received illegal taskstate from scheduler");
			break;
		}
	} else {
		delete msg;   // NOLINT(cppcoreguidelines-owning-memory)
		throw omnetpp::cRuntimeError{"Logical received message from unhandled gate"};
	}
}

void Logical::handleResume() {
	bubble("resumed");
	if (state == TaskState::Running) {
		throw omnetpp::cRuntimeError("Logical received resume while taskstate was running");
	}
	if (state == TaskState::Blocked) {
		throw omnetpp::cRuntimeError("Logical received resume while taskstate was blocked");
	}
	if (state == TaskState::Ready) {
		// start execution
		executionTimeLeft = executionTime;
		state             = TaskState::Running;
		requestDataDict();
		requestCANFrames();
	}
	scheduleAfter(executionTimeLeft, executionMsg);
}

void Logical::handlePause() {
	bubble("paused");
	bubble("paused");
	if (state == TaskState::Ready) {
		throw omnetpp::cRuntimeError("Logical received resume while state was Ready");
	}
	if (state == TaskState::Blocked) {
		throw omnetpp::cRuntimeError("Logical received resume while state was Blocked");
	}
	if (state == TaskState::Paused) {
		throw omnetpp::cRuntimeError("Logical received resume while state was Paused");
	}
	auto startTime    = executionMsg->getSendingTime();
	auto timePassed   = omnetpp::simTime() - startTime;
	executionTimeLeft = executionTimeLeft - timePassed;
	state             = TaskState::Paused;
}

void Logical::requestDataDict() {
	for (int i = 0; i < gateSize("getDataDict$o"); ++i) {
		auto* ddGate = gate("getDataDict$o", i);
		auto* msg    = new omnetpp::cMessage();   // NOLINT(cppcoreguidelines-owning-memory)
		send(msg, ddGate);
	}
}

void Logical::requestCANFrames() {
	// for (int i = 0; i < gateSize("getFrame$o"); ++i) {
	// 	auto* const       outGate        = gate("getFrame$o", 0);
	// 	auto* const       sinkAppEndGate = outGate->getPathEndGate();
	// 	const auto* const sinkAppName    = sinkAppEndGate->getOwnerModule()->getName();

	// }
	for (const auto& canMsg : canInput) {
		auto* frameRequest = new FrameRequest();   // NOLINT(cppcoreguidelines-owning-memory)
		frameRequest->setFrameID(canMsg.definition->getCanID());
		send(frameRequest, canMsg.gateId);
	}
}

void Logical::writeDataDicts() {
	const auto generationTime = omnetpp::simTime();
	// Write the datadicts that were received from CAN, they are simply forwarded and hence their
	// timestamp doesnt change
	for (auto* dd : receivedFrameDicts) {
		// Find the output gate to send the received datadict to as is.
		auto it = std::find_if(cbegin(canBusOutputDicts), cend(canBusOutputDicts),
		                       [&dd](const auto& ddDef) {
			                       const auto* const name = ddDef.definition->getDdName();
			                       return 0 == std::strcmp(dd->getDdName(), name);
		                       });
		if (it == cend(canBusOutputDicts)) {
			throw omnetpp::cRuntimeError(
			    "Logical failed to find output gate for a datadict received from CAN");
		}
		// Set the generation time to the current time.
		dd->setGenerationTime(generationTime);
		send(dd->dup(), it->gateId);
		delete dd;   // NOLINT(cppcoreguidelines-owning-memory)
	}
	receivedFrameDicts.clear();

	// Write the datadicts created by this logical
	for (const auto& dd : localOutputDicts) {
		auto* ddValue = new DataDictionaryValue();   // NOLINT(cppcoreguidelines-owning-memory)
		ddValue->setDdName(dd.definition->getDdName());
		ddValue->setGenerationTime(generationTime);
		ddValue->setMinimalDependencyTime(minimalDependencyTime);
		send(ddValue, dd.gateId);
	}
}

void Logical::sendCANFrames() {
	for (const auto& canDef : canOutput) {
		auto* canFrame = new CanDataFrame();   // NOLINT(cppcoreguidelines-owning-memory)
		// TODO: Populate CANDataFrame with the Datadicts and generation/minimaldependency time.
		canFrame->setDisplayString("");
		canFrame->setCanID(canDef.definition->getCanID());
		canFrame->setRtr(false);
		canFrame->setPeriod(1.0);
		send(canFrame, canDef.gateId);
	}
}

/**
 * Takes ownership of the pointer
 * @param frame
 */
void Logical::localyStoreReceivedFrame(CanDataFrame* frame) {
	bubble("read frame");

	// TODO: Unpack Datadicts and change minimalDependency time accordingly.

	delete frame;   // NOLINT(cppcoreguidelines-owning-memory)
}

/**
 * Takes ownership of the pointer @code{value}
 * @param value
 */
void Logical::localyStoreReadDataDict(DataDictionaryValue* value) {
	bubble("read datadict");

	auto it = std::find_if(cbegin(localDicts), cend(localDicts),
	                       [receivedVal = value](const auto& value) {
		                       const auto* const name = value->getDdName();
		                       return 0 == std::strcmp(receivedVal->getDdName(), name);
	                       });

	if (it != std::cend(localDicts)) {
		// We already buffered a dictionary with the same name, throw an error
		delete value;   // NOLINT(cppcoreguidelines-owning-memory)
		throw omnetpp::cRuntimeError("Logical received an already buffered datadict");
	}

	localDicts.emplace_back(value);
	minimalDependencyTime = std::min(minimalDependencyTime, value->getMinimalDependencyTime());
}
}   // namespace FiCo4OMNeT