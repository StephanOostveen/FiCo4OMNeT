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

	readSignal  = registerSignal("read");
	writeSignal = registerSignal("write");

	parseCANInput();
	parseCANOutput();
	parseDDOutput();

	EV << getFullPath() << " receives from CAN: " << canBusOutputDicts.size()
	   << " datadicts, generates locally : " << localOutputDicts.size() << '\n';
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

	for (std::size_t i = 0; i < size; ++i) {
		const auto& frameDefinition = ptr->getDefinition(i);
		auto* const sourceAppModule =
		    omnetpp::check_and_cast<CanLySourceApp*>(getParentModule()->getSubmodule("sourceApp"));
		sourceAppModule->registerFrame(frameDefinition.getCanID(), frameDefinition.getBus());
		canOutput.emplace_back(&frameDefinition, gateId);

		auto     ddSize = frameDefinition.getDdArraySize();
		unsigned bitsum = 0;
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
	EV << " dataDictOut array size: " << size << "\n";
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
			EV << omnetpp::simTime() << " Was Blocked, became ready\n";
			state          = TaskState::Ready;
			auto* readyMsg = new SchedulerEvent();   // NOLINT(cppcoreguidelines-owning-memory)
			readyMsg->setState(TaskState::Ready);
			send(readyMsg, "scheduler$o");
		} else {
			// TODO: Task overrun
			if (hasGUI()) {
				EV << omnetpp::simTime() << "Logical overrun: " << getFullPath() << "\n";
			}
		}
		scheduleAfter(period, scheduleMsg);
	} else if (msg->isSelfMessage() && msg == executionMsg) {
		// Execution finished
		if (hasGUI()) {
			EV << omnetpp::simTime() << " Finished execution\n";
		}
		if (state != TaskState::Running) {
			delete msg;   // NOLINT(cppcoreguidelines-owning-memory)
			executionMsg = nullptr;
			throw omnetpp::cRuntimeError(
			    "Logical received self message while not in running state %i",
			    static_cast<int>(state));
		}
		writeDataDicts();
		sendCANFrames();
		clearReceivedDataDicts();   // Clears the datadicts that originate locally
		++cycleCounter;             // All datadicts are written and transmitted, increment counter

		auto* blockedMsg = new SchedulerEvent();   // NOLINT(cppcoreguidelines-owning-memory)
		blockedMsg->setState(TaskState::Blocked);
		send(blockedMsg, "scheduler$o");
		EV << omnetpp::simTime() << " state was: " << state << " became Blocked\n";
		state = TaskState::Blocked;

	} else if (msg->arrivedOn("getFrame$i")) {
		auto* frame = omnetpp::check_and_cast<CanDataFrame*>(msg);
		localyStoreReceivedFrame(frame);
	} else if (msg->arrivedOn("getDataDict$i")) {
		auto* value = omnetpp::check_and_cast<DataDictionaryValue*>(msg);
		localyStoreReadDataDict(value);
		emit(readSignal, value);
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
	if (state == TaskState::Running) {
		throw omnetpp::cRuntimeError("Logical received resume while taskstate was running");
	}
	if (state == TaskState::Blocked) {
		throw omnetpp::cRuntimeError("Logical received resume while taskstate was blocked");
	}
	if (state == TaskState::Paused) {
		EV << omnetpp::simTime() << " Resumed from pause\n";   // NOLINT
		state = TaskState::Running;
	}
	if (state == TaskState::Ready) {
		// start execution
		EV << omnetpp::simTime() << " Task was ready and became running\n";
		executionTimeLeft = executionTime;
		state             = TaskState::Running;
		requestDataDict();
		requestCANFrames();
	}
	scheduleAfter(executionTimeLeft, executionMsg);
}

void Logical::handlePause() {
	if (state == TaskState::Ready) {
		throw omnetpp::cRuntimeError("Logical received pause while state was Ready");
	}
	if (state == TaskState::Blocked) {
		throw omnetpp::cRuntimeError("Logical received pause while state was Blocked");
	}
	if (state == TaskState::Paused) {
		throw omnetpp::cRuntimeError("Logical received pause while state was Paused");
	}
	EV << omnetpp::simTime() << " Paused\n";
	auto startTime    = executionMsg->getSendingTime();
	auto timePassed   = omnetpp::simTime() - startTime;
	executionTimeLeft = executionTimeLeft - timePassed;
	state             = TaskState::Paused;
	cancelEvent(executionMsg);
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
	// Write the datadicts that were received from CAN, they are simply forwarded and hence their
	// timestamp or writecount doesnt change
	EV << omnetpp::simTime() << "Writing " << receivedFrameDicts.size()
	   << "datadicts received from CAN\n";
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
		send(copy, it->gateId);
		emit(writeSignal, &dd);
	}
	receivedFrameDicts.clear();

	const auto generationTime = omnetpp::simTime();
	// Write the datadicts created by this logical
	for (const auto& dd : localOutputDicts) {
		auto* ddValue = new DataDictionaryValue();   // NOLINT(cppcoreguidelines-owning-memory)
		ddValue->setDdName(dd.definition->getDdName());
		ddValue->setGenerationTime(generationTime);
		ddValue->setWriteCount(cycleCounter);

		send(ddValue->dup(), dd.gateId);
		emit(writeSignal, ddValue);
		localyStoreReadDataDict(ddValue);
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
		canFrame->setTimestamp(omnetpp::simTime());

		auto dataFieldByteLength = canDef.definition->getSizeInBytes();
		// Calculate the nr of header, stuffing and footer bits for a given fieldlength in bytes
		auto frameOverheadBits =
		    calculateOverHead(dataFieldByteLength, canDef.definition->getCanID());
		canFrame->setBitLength(frameOverheadBits);
		auto* payload = createFramePayload(canDef.definition);
		payload->setTimestamp();
		payload->setByteLength(dataFieldByteLength);
		canFrame->encapsulate(payload);

		if (canFrame->getBitLength() > 160) {
			EV << omnetpp::simTime() << " CanFrame that was too large: " << canFrame->getCanID()
			   << " frame bits: " << canFrame->getBitLength()
			   << " frameOverhead: " << frameOverheadBits << " encapsulated packet bitsize: "
			   << canFrame->getEncapsulatedPacket()->getBitLength() << "\n";
		}
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
			// Data dict was received from a local Logical/created by this logical
			values->appendValue(**it);
		} else {
			// Other cases, dd is not written
			throw omnetpp::cRuntimeError("Missing Datadictionary %s when creating payload for "
			                             "frame with id: %u on bus: %s",
			                             ddDef.getDdName(), canDef->getCanID(), canDef->getBus());
		}
	}
	return values;
}

unsigned Logical::calculateOverHead(unsigned dataLength, unsigned frameID) {
	static constexpr unsigned smallestExtendedID = (1U << 11U);
	if (frameID < smallestExtendedID) {
		// CAN 2.0B with 11 bit identifier message
		return DataFrameControlBits + calculateStuffingBits(dataLength, false);
	}
	// CAN 2.0B with 29 bit identifier message
	static constexpr unsigned extendedIDBits = 20;   // 18 + 2 substitute bits
	return DataFrameControlBits + extendedIDBits + calculateStuffingBits(dataLength, true);
}

unsigned int Logical::calculateStuffingBits(unsigned int dataLength, bool isExtendedId) {
	// Get the stuffing percentage for this message, draw from a uniform distribution
	auto bitStuffingPercentage = par("bitStuffingPercentage").doubleValue();
	if (isExtendedId) {
		static constexpr unsigned extendedIDBits = 20;   // 18 + 2 substitute bits
		auto                      maxStuffBits =
		    ((ControlBitsEligibleForStuffing + extendedIDBits + (dataLength * 8) - 1) / 4);
		auto stuffbits = static_cast<unsigned int>(maxStuffBits * bitStuffingPercentage);
		if (stuffbits > 29) {
			EV << omnetpp::simTime() << " stuffbits:" << stuffbits << "\n";
		}
		return stuffbits;
	}
	return static_cast<unsigned int>(((ControlBitsEligibleForStuffing + (dataLength * 8) - 1) / 4)
	                                 * bitStuffingPercentage);
}

/**
 * Takes ownership of the pointer
 * @param frame
 */
void Logical::localyStoreReceivedFrame(CanDataFrame* frame) {
	if (frame->hasEncapsulatedPacket()) {
		if (const auto* list =
		        dynamic_cast<DataDictionaryValueList*>(frame->getEncapsulatedPacket());
		    list != nullptr) {
			for (std::size_t i = 0; i < list->getValueArraySize(); ++i) {
				const auto& value = list->getValue(i);
				receivedFrameDicts.emplace_back(value);
			}
			EV << "Saved " << list->getValueArraySize() << " datadicts from canframe "
			   << frame->getCanID() << "\n";
		}
	} else {
		// TODO: Startup effect
		auto definition = std::find_if(
		    std::cbegin(canInput), std::cend(canInput),
		    [frameID = frame->getCanID(), busName = frame->getBusName()](const auto& def) {
			    return def.definition->getCanID() == frameID
			           && std::strcmp(def.definition->getBus(), busName) == 0;
		    });
		if (definition == std::cend(canInput)) {
			EV << omnetpp::simTime()
			   << " received an unknown CAN frame without payload with id: " << frame->getCanID()
			   << " on bus: " << frame->getBusName() << "\n";
		} else {
			auto size = definition->definition->getDdArraySize();
			for (size_t i = 0; i < size; ++i) {
				const auto&         ddDef      = definition->definition->getDd(i);
				const auto*         ddName     = ddDef.getDdName();
				omnetpp::simtime_t  genTime    = 0;
				unsigned long       writeCount = 0;
				DataDictionaryValue dd{};
				dd.setDdName(ddName);
				dd.setGenerationTime(genTime);
				dd.setWriteCount(writeCount);
				receivedFrameDicts.emplace_back(dd);
			}
		}
	}
	delete frame;   // NOLINT(cppcoreguidelines-owning-memory)
}

/**
 * Takes ownership of the pointer @code{value}
 * @param value
 */
void Logical::localyStoreReadDataDict(DataDictionaryValue* value) {
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
}

void Logical::clearReceivedDataDicts() {
	for (auto* ddValue : localDicts) {
		delete ddValue;
	}
	localDicts.clear();
}
}   // namespace FiCo4OMNeT