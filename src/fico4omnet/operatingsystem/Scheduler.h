#ifndef FiCo4OMNeT_SCHEDULER_H
#define FiCo4OMNeT_SCHEDULER_H

#include "omnetpp/cgate.h"
#include "omnetpp/cmessage.h"
#include "omnetpp/csimplemodule.h"
#include "omnetpp/simkerneldefs.h"
#include "omnetpp/simtime_t.h"

#include <optional>
#include <random>
#include <vector>

namespace FiCo4OMNeT {
class Scheduler : public omnetpp::cSimpleModule {
public:
	~Scheduler() noexcept override;

protected:
	void initialize(int stage) override;
	int  numInitStages() const override;
	void handleMessage(omnetpp::cMessage* msg) override;

private:
	struct Task {
		Task(double period_, omnetpp::intval_t priority_, omnetpp::cGate* inGate_,
		     omnetpp::cGate* outGate_, bool periodic_)
		    : period(period_)
		    , priority(priority_)
		    , inGate(inGate_)
		    , outGate(outGate_)
		    , periodic(periodic_) {}

		// NOLINTBEGIN(misc-non-private-member-variables-in-classes)
		omnetpp::simtime_t period;
		omnetpp::intval_t  priority;
		omnetpp::cGate*    inGate;
		omnetpp::cGate*    outGate;
		bool               periodic;
		// NOLINTEND(misc-non-private-member-variables-in-classes)
	};

	bool isExecuting{false};   // Whether the scheduler is the running task or not.
	int  nrOfTasks{0};

	std::vector<Task>   blocked{};
	std::vector<Task>   ready{};
	std::vector<Task>   paused{};
	std::optional<Task> running{std::nullopt};

	omnetpp::simtime_t executionTime{};
	omnetpp::simtime_t interarrivalTime{};

	omnetpp::cMessage* selfmsg{nullptr};
};
}   // namespace FiCo4OMNeT
#endif