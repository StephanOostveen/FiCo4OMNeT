package fico4omnet.applications.can.source.lysourceapp;

import fico4omnet.applications.ISourceApplication;
import fico4omnet.operatingsystem.ITask;

// This module acts as the OpenECU CAN transmitting task, it takes CAN messages from the
// applications and takes care of transmission.

simple CanLySourceApp like ISourceApplication, ITask {
    parameters:
        @class(CanLySourceApp);
        @display("i=block/source");

        double period @unit(s)= 2ms;
        double executionTime @unit(s) = 0.1ms;
        bool periodic = false;
        int priority = 4; // TX has lower priority

        //Signals
        @signal[txDF](type=CanDataFrame);
        // CAN message with id x was queued for transmission before the last queued instance was transmitted
        @signal[overrun](type=long);
        @signal[softbufferLength](type=long);

        //Statistics
        @statistic[txDf](title="source appp transmitted data frame"; source=txDF; record=count);
        @statistic[overrun](title="CAN message overrun"; record=vector);
        @statistic[softbufferLength](title="Software buffer length"; unit=packets; record=vector,stats; interpolationmode=sample-hold);
    
        //Needed for inheriting from the CanTrafficSourceAppBase
        //The Data Frames ID(s) - String parameter (int) separated with commas
        string idDataFrames = default("0");					
        //data Frame period(s) for sending of messages - String parameter (int) separated with commas - unit: s 
        string periodicityDataFrames = default("0");		
        //Datafield-length - String parameter (int) separated with commas - unit: byte
        string dataLengthDataFrames = default("0");			
        //Offset for the first transmission of the data frames - String parameter (double) separated with commas - unit: s
        string initialDataFrameOffset = default("0");
        //The Remote Frame ID(s) - String parameter (int) separated with commas
        string idRemoteFrames = default("0");	
        //Remote Frame period(s) for sending of messages - String parameter (int) separated with commas - unit: s
        string periodicityRemoteFrames = default("0");
        //Datafield-length - String parameter (int) separated with commas - unit: byte
        string dataLengthRemoteFrames = default("0");		
        //Offset for the first transmission of the remote frames - String parameter (double) separated with commas - unit: s
        string initialRemoteFrameOffset = default("0");		
        //Inaccuracy for the node which is added to the schedule time for each frame. 
        double periodInaccurracy @unit(s) = default(0s);
    gates:
        // Output towards the Hardware buffer, overrides ISourceApplication::out
        output out[] @labels(DataFrame);

        //Received remote frames from other nodes arive at this gate, should never happen in our case
        // But keep it for compatibility with FiCo4OMNeT
        input remoteIn @directIn;
        // Frames from tasks to be transmitted
        input in[] @labels(DataFrame);
        // Gates for the scheduler that are inherited from ITask
        inout scheduler @labels(SchedulerEvent);
}