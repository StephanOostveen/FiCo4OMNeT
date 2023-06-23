#ifndef FiCo4OMNeT_LOGICAL_H
#define FiCo4OMNeT_LOGICAL_H

#include "omnetpp/cmessage.h"
#include "omnetpp/csimplemodule.h"
namespace FiCo4OMNeT {
class DataDictionary : public omnetpp::cSimpleModule {
public:
	~DataDictionary() noexcept override;

protected:
	void initialize() override;
	void handleMessage(omnetpp::cMessage* msg) override;
};

}   // namespace FiCo4OMNeT
#endif