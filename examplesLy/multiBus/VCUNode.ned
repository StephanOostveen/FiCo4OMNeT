package fico4omnet.examplesLy.multiBus;

import fico4omnet.nodes.can.LyPhysicalNode;
import fico4omnet.operatingsystem.DataDictionary;

module VCUNode extends LyPhysicalNode {
    parameters:
        numDataDicts = 2;
        numLogicals = 2;

        dd[0].name = "SolarPower";
        dd[1].name = "LockState";

        // Solar Manager broadcasts the state of the PV panels.
        logical[0].name = "Solar Manager";
        logical[0].priority = 7;
        // logical[0].canInput = ;
        logical[0].canOutput = FiCo4OMNeT::CanList{definition: [
            {canID: 1, sizeInBytes: 4, bus: "vehicle_can", dd:[
                    {ddName: "SolarPower", bitSize: 32}
                ]
            }
        ]};
         // DataDictionaries created by this logical which are not received from the can bus. BMSState only in this case
         logical[0].dataDictOut = FiCo4OMNeT::DataDictList{definition: [
            {ddName: "SolarPower", bitSize: 32}
        ]};

        // Lock Manager receives commands from the CGW and locks/unlocks the vehicle.
        logical[1].name = "LockManager";
        logical[1].priority = 6;
        logical[1].canInput = FiCo4OMNeT::CanList{definition: [
            {canID: 2, sizeInBytes: 1, bus: "vehicle_can", dd:[
                    {ddName: "LockState", bitSize: 5}
                ]
            }
        ]};
        // logical[1].canOutput = ;
        // logical[1].dataDictOut = ;
    connections:
        logical[0].setDataDict++ --> dd[0].set;
        logical[1].setDataDict++ --> dd[1].set;
}