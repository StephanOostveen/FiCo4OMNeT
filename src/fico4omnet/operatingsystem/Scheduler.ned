package fico4omnet.operatingsystem;

simple Scheduler {
    parameters:
        @class(Scheduler);
        @display("i=block/timer");
        double executionTime @unit(s) = default(0.001ms);
        double interarrivalTime @unit(s) = default(1ms);

    gates:
        inout tasks[] @labels(SchedulerEvent);
}