package fico4omnet.examplesLy.LYES;

import fico4omnet.nodes.can.LyPhysicalNode;
import fico4omnet.operatingsystem.DataDictionary;

module TestNode extends LyPhysicalNode {
    parameters:
        numDataDicts = 1;
        numLogicals = 1;
        sinkApp.priority = 14 + 4;
		//Source app has default prio of 4

        dd[0].name = "brake_applied";

        logical[0].name = "braking_system_manager";
        logical[0].priority = 9;
        logical[0].period = 10ms;
        logical[0].canInput = FiCo4OMNeT::CanList{definition: []};
        logical[0].canOutput = FiCo4OMNeT::CanList{definition: []};
        logical[0].dataDictOut = FiCo4OMNeT::DataDictList{definition: [] };
    
    connections allowunconnected:
        logical[0].setDataDict++ --> dd[0].set;
}