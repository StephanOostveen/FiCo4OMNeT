package fico4omnet.operatingsystem;

import fico4omnet.operatingsystem.ITask;
import fico4omnet.operatingsystem.CanList;

simple Logical like ITask {
    parameters:
        @class(Logical);
        @display("i=block/app");

        double period @unit(s)= default(50ms);
        double executionTime @unit(s) = default(0.1ms);
        bool periodic = default(true);
        int priority = default(6); 

        object canInput = default(CanList{messages: []});
        object canOutput = default(CanList{messages: []});
    gates:
        output frameOut @labels(DataFrame); // Transmit a frame on the CAN bus
        inout getFrame; // Read a received CAN frame
        inout scheduler @labels(SchedulerEvent);
}