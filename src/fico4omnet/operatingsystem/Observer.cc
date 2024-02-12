#include "Observer.h"
#include "DataDictionaryValue_m.h"
#include "omnetpp/cexception.h"
#include "omnetpp/checkandcast.h"
#include "omnetpp/clistener.h"
#include "omnetpp/cmodule.h"
#include "omnetpp/cobject.h"
#include "omnetpp/cresultfilter.h"
#include "omnetpp/cresultrecorder.h"
#include "omnetpp/csimulation.h"
#include "omnetpp/opp_string.h"
#include "omnetpp/regmacros.h"
#include "omnetpp/simtime_t.h"
#include <algorithm>
#include <cstring>
#include <iterator>
#include <string>
#include <tuple>
#include <utility>

namespace FiCo4OMNeT {
Define_Module(Observer);   // NOLINT

void Observer::initialize() {
	auto* listener = static_cast<omnetpp::cIListener*>(this);
	getSimulation()->getSystemModule()->subscribe("read", listener);
	getSimulation()->getSystemModule()->subscribe("write", listener);
}

void Observer::handleMessage(omnetpp::cMessage* /*unused*/) {
	// Does not receive any messages, it is simply a listener logging the read/writes of all the
	// datadictionaries
}

void Observer::receiveSignal(omnetpp::cComponent* src, omnetpp::simsignal_t id,
                             omnetpp::cObject* value, omnetpp::cObject* details) {
	Enter_Method_Silent();
	if (auto* srcModule = dynamic_cast<omnetpp::cModule*>(src); srcModule != nullptr) {
		// src is a module and not a channel
		if (std::strcmp(getSignalName(id), "read") == 0) {
			readSignal(value);
		} else if (std::strcmp(getSignalName(id), "write") == 0) {
			writeSignal(value);
		} else {
			throw omnetpp::cRuntimeError("Unknown signal '%s'", getSignalName(id));
		}
	} else {
		throw omnetpp::cRuntimeError("Received a signal from something that wasnt a cModule: %s",
		                             src->getFullPath().c_str());
	}
}

void Observer::readSignal(omnetpp::cObject* value) {
// TODO:
// 	split per Physical;
	auto*     ddValue = omnetpp::check_and_cast<DataDictionaryValue*>(value);
	if (const auto signalBufferIt = database.find(ddValue->getDdName());
	    signalBufferIt != std::cend(database)) {
		const auto& [signal, buffer] = signalBufferIt->second;

		bool found       = false;
		auto currentTime = omnetpp::simTime();
		for (const auto& [writeCount, generationTime] : buffer) {
			if (writeCount == ddValue->getWriteCount()) {
				emit(signal, currentTime - ddValue->getGenerationTime());
				found = true;
				break;
			}
		}
		if (!found) {
			EV << "Could not find matching write of read of datadictionary '"
			   << ddValue->getDdName() << "' with writecount: " << ddValue->getWriteCount()
			   << " and generation time: " << ddValue->getGenerationTime()
			   << " at time: " << omnetpp::simTime() << "\n";
			emit(signal, SIMTIME_MAX);
		}
	} else {
		EV << "Received read signal for unregistered datadictionary: '" << ddValue->getDdName()
		   << "'\n";
	}
}

void Observer::writeSignal(omnetpp::cObject* value) {
	const auto* ddValue = omnetpp::check_and_cast<DataDictionaryValue*>(value);

	auto signalBufferIt = database.find(ddValue->getDdName());
	if (signalBufferIt == std::cend(database)) {
		// First time receiving a signal for this DataDictionary, start by registering it.
		std::string signalName{ddValue->getDdName()};
		signalName += "_age";
		auto  signal             = registerSignal(signalName.c_str());
		auto* warmupFilterVector = omnetpp::cResultFilterType::get("warmup")->create();
		auto* vectorRecorder     = omnetpp::cResultRecorderType::get("vector")->create();
		auto* warmupFilterStats  = omnetpp::cResultFilterType::get("warmup")->create();
		auto* statsRecorder      = omnetpp::cResultRecorderType::get("stats")->create();

		std::string title{"Age of "};
		title += ddValue->getDdName();
		title += " when read";

		auto* attributesVector       = new omnetpp::opp_string_map;
		(*attributesVector)["title"] = title;
		(*attributesVector)["unit"]  = "seconds";

		auto* attributesStats = new omnetpp::opp_string_map(*attributesVector);

		omnetpp::cResultRecorder::Context vectorCtx{this, signalName.c_str(), "vector", nullptr,
		                                            attributesVector};
		omnetpp::cResultRecorder::Context statsCtx{this, signalName.c_str(), "stats", nullptr,
		                                           attributesStats};
		vectorRecorder->init(&vectorCtx);
		statsRecorder->init(&statsCtx);

		subscribe(signal, warmupFilterVector);
		subscribe(signal, warmupFilterStats);
		warmupFilterVector->addDelegate(vectorRecorder);
		warmupFilterStats->addDelegate(statsRecorder);

		database.emplace(std::piecewise_construct, std::forward_as_tuple(ddValue->getDdName()),
		                 std::forward_as_tuple(signal, BufferType{}));
		signalBufferIt = database.find(ddValue->getDdName());
		EV << "Registered write signal for " << ddValue->getDdName() << "\n";
	}

	auto& [signal, buffer] = signalBufferIt->second;
	if (buffer.size() > BufferLimit) {
		buffer.pop_front();
	}
	buffer.emplace_back(ddValue->getWriteCount(), ddValue->getGenerationTime());
}
}   // namespace FiCo4OMNeT