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

        object canInput = default(FiCo4OMNeT::CanList{definition: []});
        object canOutput = default(FiCo4OMNeT::CanList{definition: []});
    gates:
        output frameOut[] @labels(DataFrame); // Transmit a frame on the CAN bus
        inout getFrame[]; // Read a received CAN frame
        inout getDataDict[]; // Read the value of a stored datadict
        output setDatadict[]; // Write to a datadict
        inout scheduler @labels(SchedulerEvent);
}