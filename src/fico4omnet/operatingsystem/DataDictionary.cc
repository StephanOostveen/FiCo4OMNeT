#include "DataDictionary.h"
#include "DataDictionaryValue_m.h"
#include "omnetpp/cexception.h"
#include "omnetpp/checkandcast.h"
#include "omnetpp/cmessage.h"
#include "omnetpp/regmacros.h"

namespace FiCo4OMNeT {
Define_Module(DataDictionary);   // NOLINT

void DataDictionary::initialize() {}

void DataDictionary::handleMessage(omnetpp::cMessage* msg) {
	if (msg->arrivedOn("set")) {
		auto* ddValue         = omnetpp::check_and_cast<DataDictionaryValue*>(msg);
		generationTime        = ddValue->getGenerationTime();
		minimalDependencyTime = ddValue->getMinimalDependencyTime();
	} else if (msg->arrivedOn("get$i")) {
		auto* ddValue = new DataDictionaryValue();   // NOLINT(cppcoreguidelines-owning-memory)
		ddValue->setDdName(par("name").stringValue());
		ddValue->setGenerationTime(generationTime);
		ddValue->setMinimalDependencyTime(minimalDependencyTime);

		auto* arrivalGate = msg->getArrivalGate();
		send(ddValue, arrivalGate->getOtherHalf());
	} else {
		delete msg;   // NOLINT(cppcoreguidelines-owning-memory)
		throw omnetpp::cRuntimeError("DataDictionary doesnt know how to handle this message");
	}
	delete msg;   // NOLINT(cppcoreguidelines-owning-memory)
}
}   // namespace FiCo4OMNeT