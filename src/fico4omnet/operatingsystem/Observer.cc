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
		const auto* name = src->getParentModule()->getName();
		Node        n    = Node::Unknown;
		if (std::strcmp(name, "CGW") == 0) {
			n = Node::CGW;
		} else if (std::strcmp(name, "SCU") == 0) {
			n = Node::SCU;
		} else if (std::strcmp(name, "VCU") == 0) {
			n = Node::VCU;
		}

		if (std::strcmp(getSignalName(id), "read") == 0) {
			EV << "Read from module with name " << name << "\n";
			readSignal(value, n);
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

void Observer::readSignal(omnetpp::cObject* value, Node n) {
	auto* ddValue = omnetpp::check_and_cast<DataDictionaryValue*>(value);
	if (const auto signalBufferIt = database.find(ddValue->getDdName());
	    signalBufferIt != std::cend(database)) {
		const auto& [cgwSignal, scuSignal, vcuSignal, buffer] = signalBufferIt->second;

		bool found       = false;
		auto currentTime = omnetpp::simTime();
		for (const auto& [writeCount, generationTime] : buffer) {
			if (writeCount == ddValue->getWriteCount()) {
				switch (n) {
				case Node::CGW:
					emit(cgwSignal, currentTime - ddValue->getGenerationTime());
					break;
				case Node::SCU:
					emit(scuSignal, currentTime - ddValue->getGenerationTime());
					break;
				case Node::VCU:
					emit(vcuSignal, currentTime - ddValue->getGenerationTime());
					break;
				case Node::Unknown:
					throw omnetpp::cRuntimeError("Unknown node '%i'", static_cast<int>(n));
				}

				found = true;
				break;
			}
		}
		if (!found) {
			EV << "Could not find matching write of read of datadictionary '"
			   << ddValue->getDdName() << "' with writecount: " << ddValue->getWriteCount()
			   << " and generation time: " << ddValue->getGenerationTime()
			   << " at time: " << omnetpp::simTime() << "\n";
			switch (n) {
			case Node::CGW:
				emit(cgwSignal, SIMTIME_MAX);
				break;
			case Node::SCU:
				emit(scuSignal, SIMTIME_MAX);
				break;
			case Node::VCU:
				emit(vcuSignal, SIMTIME_MAX);
				break;
			case Node::Unknown:
				throw omnetpp::cRuntimeError("Unknown node '%i'", static_cast<int>(n));
			}
		}
	} else {
		EV << "Received read signal for unregistered datadictionary: '" << ddValue->getDdName()
		   << "'\n";
	}
}

omnetpp::simsignal_t Observer::createSignal(std::string node, const DataDictionaryValue* ddValue) {
	node += ddValue->getDdName();
	node += "_age";
	auto  signal             = registerSignal(node.c_str());
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

	omnetpp::cResultRecorder::Context vectorCtx{this, node.c_str(), "vector", nullptr,
	                                            attributesVector};
	omnetpp::cResultRecorder::Context statsCtx{this, node.c_str(), "stats", nullptr,
	                                           attributesStats};
	vectorRecorder->init(&vectorCtx);
	statsRecorder->init(&statsCtx);

	subscribe(signal, warmupFilterVector);
	subscribe(signal, warmupFilterStats);
	warmupFilterVector->addDelegate(vectorRecorder);
	warmupFilterStats->addDelegate(statsRecorder);

	return signal;
}

void Observer::writeSignal(omnetpp::cObject* value) {
	const auto* ddValue = omnetpp::check_and_cast<DataDictionaryValue*>(value);

	auto signalBufferIt = database.find(ddValue->getDdName());
	if (signalBufferIt == std::cend(database)) {
		// First time receiving a signal for this DataDictionary, start by registering it.
		auto cgwSignal = createSignal("CGW", ddValue);
		auto scuSignal = createSignal("SCU", ddValue);
		auto vcuSignal = createSignal("VCU", ddValue);
		database.emplace(std::piecewise_construct, std::forward_as_tuple(ddValue->getDdName()),
		                 std::forward_as_tuple(cgwSignal, scuSignal, vcuSignal, BufferType{}));
		signalBufferIt = database.find(ddValue->getDdName());
		EV << "Registered write signal for " << ddValue->getDdName() << "\n";
	}

	auto& [cgwSignal, scuSignal, vcuSignal, buffer] = signalBufferIt->second;
	if (buffer.size() > BufferLimit) {
		buffer.pop_front();
	}
	buffer.emplace_back(ddValue->getWriteCount(), ddValue->getGenerationTime());
}
}   // namespace FiCo4OMNeT