#include "Scheduler.h"

#include "ScheduleMsg_m.h"

#include "omnetpp/cexception.h"
#include "omnetpp/checkandcast.h"
#include "omnetpp/cmessage.h"
#include "omnetpp/regmacros.h"

#include <algorithm>
#include <iterator>
#include <optional>
#include <vector>
namespace FiCo4OMNeT {
Define_Module(Scheduler);

Scheduler::~Scheduler() noexcept {
	cancelAndDelete(selfmsg);
}

void Scheduler::initialize(int stage) {
	if (stage == 0) {
		executionTime    = par("executionTime").doubleValue();
		interarrivalTime = par("interarrivalTime").doubleValue();
		selfmsg          = new omnetpp::cMessage{};   // NOLINT(cppcoreguidelines-owning-memory)
	} else if (stage == 1) {
		const auto taskGateInSize  = gateSize("tasks$i");
		const auto taskGateOutSize = gateSize("tasks$o");
		if (taskGateInSize != taskGateOutSize) {
			throw omnetpp::cRuntimeError(
			    "Scheduler tasks in and out gate were of different length");
		}
		nrOfTasks = taskGateInSize;
		// Create queues for blocked, ready, paused and running tasks
		// Provide easy access to their priority and in/out gate
		blocked.reserve(static_cast<size_t>(nrOfTasks));
		ready.reserve(static_cast<size_t>(nrOfTasks));
		paused.reserve(static_cast<size_t>(nrOfTasks));
		for (int i = 0; i < nrOfTasks; ++i) {
			auto* const       taskGate = gate("tasks$o", i);
			auto* const       endGate  = taskGate->getPathEndGate();
			const auto* const task     = endGate->getOwnerModule();

			const auto period   = task->par("period").doubleValue();
			const auto priority = task->par("priority").intValue();
			// Might need to change gate ptrs to gate ID for uniqueness and stability
			blocked.emplace_back(period, priority, taskGate->getOtherHalf()->getId(),
			                     taskGate->getId());
		}
		// Start the Physical with the execution of the scheduler
		isExecuting = true;
		scheduleAt(executionTime, selfmsg);
	}
}

int Scheduler::numInitStages() const {
	return 2;
}

void Scheduler::handleMessage(omnetpp::cMessage* msg) {
	if (msg->isSelfMessage() && !isExecuting) {
		// Scheduler needs to execute, pause the running task, tasks "schedule" themselves
		if (running.has_value()) {
			auto* pauseMsg = new SchedulerEvent();   // NOLINT(cppcoreguidelines-owning-memory)
			pauseMsg->setState(TaskState::Paused);
			send(pauseMsg, running->outGate);

			paused.emplace_back(*running);
			running = std::nullopt;
		}
		isExecuting = true;
		scheduleAfter(executionTime, selfmsg);
	} else if (msg->isSelfMessage() && isExecuting) {
		// Scheduler finished execution, resume the highest priority paused or ready task.
		std::vector<Task> eligibleTasks{};
		eligibleTasks.reserve(static_cast<size_t>(nrOfTasks));

		auto priorityComp = [](const auto& lhs, const auto& rhs) {
			return lhs.priority < rhs.priority;
		};
		const auto highestReady  = std::max_element(cbegin(ready), std::cend(ready), priorityComp);
		const auto highestPaused = std::max_element(cbegin(paused), cend(paused), priorityComp);

		// copy the elements for which the predicate returns true
		auto copyComp = [](auto priority) {
			return [priorityLevel = priority](const auto& element) {
				return priorityLevel == element.priority;
			};
		};
		if (highestReady != std::cend(ready) && highestPaused != std::cend(paused)) {
			EV << "Highest ready: " << highestReady->priority
			   << " Highest paused: " << highestPaused->priority << "\n";
			if (highestReady->priority < highestPaused->priority) {
				// We need to schedule a paused task
				std::copy_if(cbegin(paused), cend(paused), std::back_inserter(eligibleTasks),
				             copyComp(highestPaused->priority));
			} else if (highestPaused->priority < highestReady->priority) {
				std::copy_if(cbegin(ready), cend(ready), std::back_inserter(eligibleTasks),
				             copyComp(highestReady->priority));
			} else if (highestPaused->priority == highestReady->priority) {
				std::copy_if(cbegin(paused), cend(paused), std::back_inserter(eligibleTasks),
				             copyComp(highestPaused->priority));
				std::copy_if(cbegin(ready), cend(ready), std::back_inserter(eligibleTasks),
				             copyComp(highestReady->priority));
			}
		} else if (highestReady == std::cend(ready) && highestPaused != std::cend(paused)) {
			EV << " Highest paused: " << highestPaused->priority << "\n";
			std::copy_if(cbegin(paused), cend(paused), std::back_inserter(eligibleTasks),
			             copyComp(highestPaused->priority));
		} else if (highestReady != std::cend(ready) && highestPaused == std::cend(paused)) {
			EV << "Highest ready: " << highestReady->priority << "\n";
			std::copy_if(cbegin(ready), cend(ready), std::back_inserter(eligibleTasks),
			             copyComp(highestReady->priority));
		}
		// select a random task from the eligible tasks
		if (eligibleTasks.size() > 0) {
			const auto  index = intuniform(0, eligibleTasks.size() - 1);
			const auto& task  = eligibleTasks.at(index);

			// find the task in the ready or paused queues, remove it and put it in the running
			// slot
			auto taskEquality = [&task](const auto& e) { return task.inGate == e.inGate; };

			if (auto itReady = std::find_if(cbegin(ready), cend(ready), taskEquality);
			    itReady != cend(ready)) {
				ready.erase(itReady);
			} else if (auto itPaused = std::find_if(cbegin(paused), cend(paused), taskEquality);
			           itPaused != cend(paused)) {
				paused.erase(itPaused);
			} else {
				throw omnetpp::cRuntimeError("Finding a suitable Task to run failed");
			}

			running          = task;
			auto* runningMsg = new SchedulerEvent();   // NOLINT(cppcoreguidelines-owning-memory)
			runningMsg->setState(TaskState::Running);
			send(runningMsg, running->outGate);
		}
		isExecuting = false;
		auto delta  = interarrivalTime - executionTime;
		scheduleAfter(delta, selfmsg);
	} else {
		// Msg came from a task, handle the blocked/ready message accordingly
		auto* event = omnetpp::check_and_cast<SchedulerEvent*>(msg);
		switch (event->getState()) {
		case TaskState::Blocked:
			if (!running.has_value()) {
				throw omnetpp::cRuntimeError(
				    "Scheduler received \"TaskState::Blocked\" while no task was running");
			}
			if (running->inGate != event->getArrivalGate()->getId()) {
				throw omnetpp::cRuntimeError(
				    "Scheduler received \"TaskState::Blocked\" from a task that was not running");
			}
			if (isExecuting) {
				throw omnetpp::cRuntimeError(
				    "Scheduler received \"TaskState::Blocked\" during scheduler execution");
			}
			// Put the just finished running task in the blocked queue and trigger the execution of
			// the scheduler immediately.
			blocked.emplace_back(*running);
			running = std::nullopt;
			cancelEvent(selfmsg);
			scheduleAfter(0, selfmsg);
			break;
		case TaskState::Ready:
			if (running.has_value() && running->inGate == event->getArrivalGate()->getId()) {
				// Running task transmitted a Ready signal, assume that it finished.
				// Put running task in ready queue, execute the scheduler.
				ready.emplace_back(*running);
				running = std::nullopt;
				cancelEvent(selfmsg);
				scheduleAfter(0, selfmsg);
				break;
			}
			if (auto it = std::find_if(cbegin(blocked), cend(blocked),
			                           [inGate = event->getArrivalGate()->getId()](const auto& e) {
				                           return inGate == e.inGate;
			                           });
			    it != cend(blocked)) {
				// Blocked task became Ready, put it in ready queue
				ready.emplace_back(*it);
				blocked.erase(it);

				if (!running.has_value() && !isExecuting) {
					// processor is idle and scheduler is not executing, trigger the scheduler
					cancelEvent(selfmsg);
					scheduleAfter(0, selfmsg);
				} else if (running.has_value() && !isExecuting
				           && running->priority < it->priority) {
					// Higher priority message became ready while scheduler was not executing.
					// Force the execution of the scheduler to determine the new task to run
					cancelEvent(selfmsg);
					scheduleAfter(0, selfmsg);
				}
				// Other cases: scheduler is running or priority is lower than running task, do not
				// notify the scheduler
			} else {
				throw omnetpp::cRuntimeError(
				    "Scheduler received \"TaskState::Ready\" from a non blocked task");
			}

			break;
		case TaskState::Paused:
			throw omnetpp::cRuntimeError("Scheduler received \"TaskState::Paused\"");
			break;
		case TaskState::Running:
			throw omnetpp::cRuntimeError("Scheduler received \"TaskState::Running\"");
			break;
		default:
			throw omnetpp::cRuntimeError("Scheduler received invalid Taskstate");
		}
		delete msg;   // NOLINT(cppcoreguidelines-owning-memory)
	}
}
}   // namespace FiCo4OMNeT