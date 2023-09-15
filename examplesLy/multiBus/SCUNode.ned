package fico4omnet.examplesLy.multiBus;

import fico4omnet.nodes.can.LyPhysicalNode;
import fico4omnet.operatingsystem.DataDictionary;

module SCUNode extends LyPhysicalNode {
    parameters:
        numDataDicts = 2;
        numLogicals = 2;

        dd[0].name = "BMSState";
        dd[1].name = "newInverterFirmwareVersion";

        // BMS Manager broadcasts the state of the BMS, this is logged in the cloud by the CGW
        logical[0].name = "BMSManager";
        logical[0].priority = 7;
        // CAN messages read by this logical, none in this case
        //logical[0].canInput = default;
        // CAN message generate by this logical, one containing the BMSState in this case
        logical[0].canOutput = FiCo4OMNeT::CanList{definition: [
            {canID: 1, sizeInBytes: 1, bus: "powertrain_can", dd:[
                    {ddName: "BMSState", bitSize: 8}
                ]
            }
        ]};
        // DataDictionaries created by this logical which are not received from the can bus. BMSState only in this case
        logical[0].dataDictOut = FiCo4OMNeT::DataDictList{definition: [
            {ddName: "BMSState", bitSize: 8},
        ]};

        // Inverter manager receives the inverter firmware version that is available in the cloud 
        // from the CGW
        logical[1].name = "InverterManager";
        logical[1].priority = 6;
        logical[1].canInput = FiCo4OMNeT::CanList{definition: [
            {canID: 2, sizeInBytes: 4, bus: "powertrain_can", dd:[
                    {ddName: "newInverterFirmwareVersion", bitSize: 32}
                ]
            }
        ]};
        // logical[1].canOutput = default;
        // logical[1].dataDictOut = default;
    connections:
        logical[0].setDataDict++ --> dd[0].set;
        logical[1].setDataDict++ --> dd[1].set;
}