#ifndef FiCo4OMNeT_LOGICAL_H
#define FiCo4OMNeT_LOGICAL_H

#include "omnetpp/cmessage.h"
#include "omnetpp/csimplemodule.h"
namespace FiCo4OMNeT {
class DataDictionary : public omnetpp::cSimpleModule {
public:
	~DataDictionary() noexcept override = default;

protected:
	void initialize() override;
	void handleMessage(omnetpp::cMessage* msg) override;

private:
	omnetpp::simtime_t generationTime{0};
	unsigned long      writeCount{0};
};

}   // namespace FiCo4OMNeT
#endif