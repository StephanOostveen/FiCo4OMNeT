package fico4omnet.applications.can.sink.lysinkapp;

import fico4omnet.applications.ISinkApplication;
import fico4omnet.operatingsystem.ITask;

// This module acts as the OpenECU CAN receiving task, it takes CAN messages from the
// bus and buffers them for the applications

simple CanLySinkApp like ISinkApplication, ITask {
    parameters:
        @class(CanLySinkApp);
        @display("i=block/sink");

        double period @unit(s)= 2ms;
        double executionTime @unit(s) = 0.1ms;
        bool periodic = false;
        int priority; //14 + 3 RX has higher priority

        //Signals
        @signal[rxDF](type=CanDataFrame);
        // CAN message with id x was received before the last queued instance was read
        @signal[overrun](type=long);
        @signal[underrun](type=long);
        @signal[softbufferLength](type=long);
        
        //Statistics
        @statistic[rxDf](title="source appp received data frame"; source=rxDF; record=count);
        @statistic[overrun](title="CAN message overrun"; record=vector);
        @statistic[underrun](title="CAN message underrun"; record=vector);
        @statistic[softbufferLength](title="Software buffer length"; unit=packets; record=vector,stats; interpolationmode=sample-hold);

    gates:
        // Output towards the applications
        inout getFrame[] @labels(FrameRequest);

        input controllerIn @directIn;
        input dataIn @directIn;
        // Gates for the scheduler that are inherited from ITask
        inout scheduler @labels(SchedulerEvent);
}