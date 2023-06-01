#include "Logical.h"

#include "CanList_m.h"
#include "DataDictionary_m.h"

#include "omnetpp/cexception.h"
#include "omnetpp/checkandcast.h"
#include "omnetpp/cmessage.h"
#include "omnetpp/regmacros.h"
namespace FiCo4OMNeT {
Define_Module(Logical);

Logical::~Logical() noexcept {}

void Logical::initialize() {
	const auto* const ptr  = omnetpp::check_and_cast<const CanList*>(par("canInput").objectValue());
	const auto        size = ptr->getDefinitionArraySize();

	if (size != 0) {
		throw omnetpp::cRuntimeError("Logical had non zero size definition array");
	}
}

void Logical::handleMessage(omnetpp::cMessage* msg) {}
}   // namespace FiCo4OMNeT