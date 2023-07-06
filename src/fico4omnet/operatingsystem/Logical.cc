#include "Logical.h"

#include "CanDataFrame_m.h"
#include "CanList_m.h"
#include "CanLySinkApp.h"
#include "CanLySourceApp.h"
#include "DataDictionaryValue_m.h"
#include "FrameRequest_m.h"
#include "ScheduleMsg_m.h"

#include "omnetpp/cexception.h"
#include "omnetpp/checkandcast.h"
#include "omnetpp/cmessage.h"
#include "omnetpp/cpacket.h"
#include "omnetpp/csimulation.h"
#include "omnetpp/regmacros.h"

#include <algorithm>
#include <cstddef>
#include <cstring>
#include <iterator>
#include <string>

namespace FiCo4OMNeT {
Define_Module(Logical);   // NOLINT

Logical::~Logical() noexcept {
	cancelAndDelete(executionMsg);
	cancelAndDelete(scheduleMsg);

	for (auto* dd : localDicts) {
		delete dd;   // NOLINT(cppcoreguidelines-owning-memory)
	}
}

void Logical::initialize() {
	period        = par("period").doubleValue();
	executionTime = par("executionTime").doubleValue();

	parseCANInput();
	parseCANOutput();
	parseDDOutput();
	EV << getFullPath() << " receives: " << canBusOutputDicts.size()
	   << " datadicts, generates: " << localOutputDicts.size() << '\n';
	executionMsg = new omnetpp::cMessage{};   // NOLINT(cppcoreguidelines-owning-memory)
	scheduleMsg  = new omnetpp::cMessage{};   // NOLINT(cppcoreguidelines-owning-memory)
	scheduleAt(0, scheduleMsg);
}

void Logical::parseCANInput() {
	const auto* const ptr  = omnetpp::check_and_cast<const CanList*>(par("canInput").objectValue());
	const auto        size = ptr->getDefinitionArraySize();

	for (size_t i = 0; i < size; ++i) {
		const auto gateId = gate("getFrame$o")->getBaseId();
		auto*      sinkAppModule =
		    omnetpp::check_and_cast<CanLySinkApp*>(getParentModule()->getSubmodule("sinkApp"));

		const auto& frameDefinition = ptr->getDefinition(i);
		sinkAppModule->registerFrame(frameDefinition.getCanID(), frameDefinition.getBus());
		canInput.emplace_back(&frameDefinition, gateId);

		// create datastructure for efficiently unpacking a frame into the datadicts
		const auto ddSize = frameDefinition.getDdArraySize();

		unsigned bitSum = 0;
		for (size_t j = 0; j < ddSize; ++j) {
			const auto& ddDefinition = frameDefinition.getDd(j);

			bitSum += ddDefinition.getBitSize();

			const int ddVectorSize = gateSize("setDataDict");
			bool      found        = false;
			for (int k = 0; k < ddVectorSize; ++k) {
				const auto* const ddGate   = gate("setDataDict", k);
				const auto* const ddModule = ddGate->getPathEndGate()->getOwnerModule();
				const auto* const name     = ddModule->par("name").stringValue();
				if (0 == std::strcmp(name, ddDefinition.getDdName())) {
					// Match
					const int ddGateId = ddGate->getId();
					canBusOutputDicts.emplace_back(&ddDefinition, ddGateId);
					found = true;
					break;
				}
			}
			if (!found) {
				throw omnetpp::cRuntimeError("Failed to find datadict to write from CAN: \"%s\"",
				                             ddDefinition.getDdName());
			}
		}

		if (bitSum > frameDefinition.getSizeInBytes() * 8) {
			throw omnetpp::cRuntimeError(
			    "Sum of DD bits was higher than nr of bytes for canmessage id:%u on bus:%s",
			    frameDefinition.getCanID(), frameDefinition.getBus());
		}
	}
}

void Logical::parseCANOutput() {
	const auto* const ptr = omnetpp::check_and_cast<const CanList*>(par("canOutput").objectValue());
	const auto        size = ptr->getDefinitionArraySize();

	const int gateId = gate("frameOut")->getId();

	unsigned bitsum = 0;
	for (std::size_t i = 0; i < size; ++i) {
		const auto& frameDefinition = ptr->getDefinition(i);
		auto* const sourceAppModule =
		    omnetpp::check_and_cast<CanLySourceApp*>(getParentModule()->getSubmodule("sourceApp"));
		sourceAppModule->registerFrame(frameDefinition.getCanID(), frameDefinition.getBus());
		canOutput.emplace_back(&frameDefinition, gateId);

		auto ddSize = frameDefinition.getDdArraySize();
		for (std::size_t j = 0; j < ddSize; ++j) {
			const auto& ddDef = frameDefinition.getDd(j);
			bitsum += ddDef.getBitSize();
		}
		if (bitsum > frameDefinition.getSizeInBytes() * 8) {
			throw omnetpp::cRuntimeError(
			    "Sum of DD bits was higher than nr of bytes for canmessage id:%u on bus:%s",
			    frameDefinition.getCanID(), frameDefinition.getBus());
		}
	}
}

void Logical::parseDDOutput() {
	const auto&       ddOut = par("dataDictOut");
	const auto* const ptr   = omnetpp::check_and_cast<const DataDictList*>(ddOut.objectValue());
	const auto        size  = ptr->getDefinitionArraySize();
	EV << getFullPath() << " dataDictOut array size: " << size << "\n";
	for (std::size_t i = 0; i < size; ++i) {
		const auto& ddDefinition = ptr->getDefinition(i);

		const int ddVectorSize = gateSize("setDataDict");
		bool      found        = false;
		for (int k = 0; k < ddVectorSize; ++k) {
			const auto* const ddGate   = gate("setDataDict", k);
			const auto* const ddModule = ddGate->getPathEndGate()->getOwnerModule();
			const auto* const name     = ddModule->par("name").stringValue();
			if (0 == std::strcmp(name, ddDefinition.getDdName())) {
				// Match
				const int ddGateId = ddGate->getId();
				localOutputDicts.emplace_back(&ddDefinition, ddGateId);
				found = true;
				break;
			}
		}
		if (!found) {
			throw omnetpp::cRuntimeError("Failed to find datadict to write: \"%s\"",
			                             ddDefinition.getDdName());
		}
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
			if (hasGUI()) {
				bubble("Logical overrun");
				EV << "Logical overrun: " << getFullPath() << "\n";
			}
		}
		scheduleAfter(period, scheduleMsg);
	} else if (msg->isSelfMessage() && msg == executionMsg) {
		// Execution finished
		if (hasGUI()) {
			bubble("finished");
		}
		if (state != TaskState::Running) {
			delete msg;   // NOLINT(cppcoreguidelines-owning-memory)
			throw omnetpp::cRuntimeError(
			    "Logical received self message while not in running state");
		}
		writeDataDicts();
		sendCANFrames();
		clearReceivedDataDicts();   // Clears the datadicts that originate locally

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
	if (hasGUI()) {
		bubble("resumed");
	}
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
	if (hasGUI()) {
		bubble("paused");
	}
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
	for (const auto& canMsg : canInput) {
		auto* frameRequest = new FrameRequest();   // NOLINT(cppcoreguidelines-owning-memory)
		frameRequest->setFrameID(canMsg.definition->getCanID());
		frameRequest->setBusName(canMsg.definition->getBus());
		send(frameRequest, canMsg.gateId);
	}
}

void Logical::writeDataDicts() {
	const auto generationTime = omnetpp::simTime();
	// Write the datadicts that were received from CAN, they are simply forwarded and hence their
	// timestamp doesnt change
	for (const auto& dd : receivedFrameDicts) {
		// Find the output gate to send the received datadict to as is.
		auto it = std::find_if(cbegin(canBusOutputDicts), cend(canBusOutputDicts),
		                       [&dd](const auto& ddDef) {
			                       const auto* const name = ddDef.definition->getDdName();
			                       return 0 == std::strcmp(dd.getDdName(), name);
		                       });
		if (it == cend(canBusOutputDicts)) {
			throw omnetpp::cRuntimeError(
			    "Logical failed to find output gate for a datadict received from CAN");
		}
		// Set the generation time to the current time.
		auto* copy = dd.dup();
		copy->setGenerationTime(generationTime);
		send(copy, it->gateId);
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

		canFrame->setDisplayString("");
		canFrame->setBusName(canDef.definition->getBus());
		canFrame->setCanID(canDef.definition->getCanID());
		canFrame->setRtr(false);
		canFrame->setPeriod(0.0);
		canFrame->setGenerationTime(omnetpp::simTime());

		auto dataFieldByteLength = canDef.definition->getSizeInBytes();
		// Calculate the nr of header, stuffing and footer bits for a given fieldlength in bytes
		auto frameOverheadBits = calculateOverHead(dataFieldByteLength);
		canFrame->setBitLength(frameOverheadBits);
		auto* payload = createFramePayload(canDef.definition);
		payload->setTimestamp();
		payload->setByteLength(dataFieldByteLength);
		canFrame->encapsulate(payload);
		send(canFrame, canDef.gateId);
	}
}

DataDictionaryValueList* Logical::createFramePayload(const CanDataFrameDefinition* canDef) {
	auto* values = new DataDictionaryValueList;   // NOLINT(cppcoreguidelines-owning-memory)

	for (std::size_t i = 0; i < canDef->getDdArraySize(); ++i) {
		const auto& ddDef = canDef->getDd(i);

		auto it = std::find_if(std::cbegin(localDicts), std::cend(localDicts),
		                       [&ddDef](const auto* receivedDDValue) {
			                       const auto* const name = ddDef.getDdName();
			                       return 0 == std::strcmp(receivedDDValue->getDdName(), name);
		                       });
		if (it != std::cend(localDicts)) {
			// Data dict was received from a local Logical
			values->appendValue(**it);
		} else {
			// Other cases, dd is created by this logical, maybe others??
			DataDictionaryValue value{};
			value.setDdName(ddDef.getDdName());
			value.setGenerationTime(omnetpp::simTime());
			value.setMinimalDependencyTime(minimalDependencyTime);
			values->appendValue(value);
		}
	}
	return values;
}

unsigned Logical::calculateOverHead(unsigned dataLength) {
	// Assume we only use CAN 2.0B with 11 bit identifier messages
	return DataFrameControlBits + calculateStuffingBits(dataLength);
}

unsigned int Logical::calculateStuffingBits(unsigned int dataLength) {
	// Get the stuffing percentage for this message, draw from a uniform distribution
	auto bitStuffingPercentage = par("bitStuffingPercentage").doubleValue();
	return static_cast<unsigned int>(((ControlBitsEligibleForStuffing + (dataLength * 8) - 1) / 4)
	                                 * bitStuffingPercentage);
}

/**
 * Takes ownership of the pointer
 * @param frame
 */
void Logical::localyStoreReceivedFrame(CanDataFrame* frame) {
	if (hasGUI()) {
		bubble("read frame");
	}

	if (frame->hasEncapsulatedPacket()) {
		const auto* list =
		    omnetpp::check_and_cast<DataDictionaryValueList*>(frame->getEncapsulatedPacket());

		for (std::size_t i = 0; i < list->getValueArraySize(); ++i) {
			const auto& value = list->getValue(i);
			receivedFrameDicts.emplace_back(value);
			minimalDependencyTime =
			    std::min(minimalDependencyTime, value.getMinimalDependencyTime());
		}
	} else {
		// TODO: Startup effect, how to handle properly?
		EV << getFullPath() << " received a CAN frame without payload\n";
	}
	delete frame;   // NOLINT(cppcoreguidelines-owning-memory)
}

/**
 * Takes ownership of the pointer @code{value}
 * @param value
 */
void Logical::localyStoreReadDataDict(DataDictionaryValue* value) {
	if (hasGUI()) {
		bubble("read datadict");
	}

	auto it = std::find_if(cbegin(localDicts), cend(localDicts),
	                       [receivedVal = value](const auto& value) {
		                       const auto* const name = value->getDdName();
		                       return 0 == std::strcmp(receivedVal->getDdName(), name);
	                       });

	if (it != std::cend(localDicts)) {
		// We already buffered a dictionary with the same name, throw an error
		std::string ddName(value->getDdName());
		delete value;   // NOLINT(cppcoreguidelines-owning-memory)
		throw omnetpp::cRuntimeError("Logical received an already buffered datadict: \"%s\"",
		                             ddName.c_str());
	}

	localDicts.emplace_back(value);
	minimalDependencyTime = std::min(minimalDependencyTime, value->getMinimalDependencyTime());
}

void Logical::clearReceivedDataDicts() {
	for (auto* ddValue : localDicts) {
		delete ddValue;
	}
	localDicts.clear();
}
}   // namespace FiCo4OMNeT