package fico4omnet.examplesLy.multiBus;

import fico4omnet.nodes.can.LyPhysicalNode;
import fico4omnet.operatingsystem.DataDictionary;

module CGWNode extends LyPhysicalNode {
    parameters:
        numDataDicts = 4;
        numLogicals = 2;

        dd[0].name = "BMSState";
        dd[1].name = "newInverterFirmwareVersion";
        dd[2].name = "SolarPower";
        dd[3].name = "LockState";

        // Logical communicating with the SCU
        logical[0].name = "SCU Manager";
        logical[0].priority = 6;
        logical[0].canInput = FiCo4OMNeT::CanList{definition: [
            {canID: 1, sizeInBytes: 1, bus: "powertrain_can", dd:[
                    {ddName: "BMSState", bitSize: 8}
                ]
            }
        ]};
        logical[0].canOutput = FiCo4OMNeT::CanList{definition: [
            {canID: 2, sizeInBytes: 4, bus: "powertrain_can", dd:[
                    {ddName: "newInverterFirmwareVersion", bitSize: 32}
                ]
            }
        ]};
        logical[0].dataDictOut = FiCo4OMNeT::DataDictList{definition: [
            {ddName: "newInverterFirmwareVersion", bitSize: 32}
        ]};

        // Logical communicating with VCU
        logical[1].name = "VCU Manager";
        logical[1].priority = 6;
        logical[1].canInput = FiCo4OMNeT::CanList{definition: [
            {canID: 1, sizeInBytes: 4, bus: "vehicle_can", dd:[
                    {ddName: "SolarPower", bitSize: 32}
                ]
            }
        ]};
        logical[1].canOutput = FiCo4OMNeT::CanList{definition: [
            {canID: 2, sizeInBytes: 1, bus: "vehicle_can", dd:[
                    {ddName: "LockState", bitSize: 5}
                ]
            }
        ]};
        logical[1].dataDictOut = FiCo4OMNeT::DataDictList{definition: [
            {ddName: "LockState", bitSize: 5}
        ]};
    connections:
        logical[0].setDataDict++ --> dd[0].set;
        logical[0].setDataDict++ --> dd[1].set;
        logical[1].setDataDict++ --> dd[2].set;
        logical[1].setDataDict++ --> dd[3].set;
}