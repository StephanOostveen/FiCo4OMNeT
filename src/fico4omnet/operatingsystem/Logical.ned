package fico4omnet.operatingsystem;

import fico4omnet.operatingsystem.ITask;
import fico4omnet.operatingsystem.CanList;
import fico4omnet.operatingsystem.DataDictList;

simple Logical like ITask {
    parameters:
        @class(Logical);
        @display("i=block/app;t=$name");

        double period @unit(s)= default(50ms);
        double executionTime @unit(s) = default(0.1ms);
        bool periodic = default(true);
        int priority;
        string name;

        volatile double bitStuffingPercentage = uniform(0,1);
        
        object canInput = default(FiCo4OMNeT::CanList{definition: []}); //CANFrames read by this logical
        object canOutput = default(FiCo4OMNeT::CanList{definition: []}); //CANFrames transmitted
        object dataDictOut = default(FiCo4OMNeT::DataDictList{definition: []}); //Datadicts written by this logical that are not received from CAN
        
    gates:
        output frameOut @labels(DataFrame); // Transmit a frame on the CAN bus
        inout getFrame; // Read a received CAN frame
        inout getDataDict[]; // Read the value of a stored datadict
        output setDataDict[]; // Write to a datadict
        inout scheduler @labels(SchedulerEvent);
}