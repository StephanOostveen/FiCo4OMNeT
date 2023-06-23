#include "DataDictionary.h"
#include "omnetpp/cmessage.h"
#include "omnetpp/regmacros.h"

namespace FiCo4OMNeT {
Define_Module(DataDictionary);   // NOLINT
DataDictionary::~DataDictionary() noexcept {}

void DataDictionary::initialize() {}

void DataDictionary::handleMessage(omnetpp::cMessage* msg) {}
}   // namespace FiCo4OMNeT