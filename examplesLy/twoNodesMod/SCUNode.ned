package fico4omnet.examplesLy.twoNodesMod;

import fico4omnet.nodes.can.LyPhysicalNode;
import fico4omnet.operatingsystem.DataDictionary;

module SCUNode extends LyPhysicalNode {
    parameters:
        numDataDicts = 3;
        numLogicals = 2;
        dd[0].name = "voltage";
        dd[1].name = "current";
        dd[2].name = "power";

        logical[0].name = "SSC";
        logical[0].priority = 7; // 6 is lowest task prio
        // {canID: x, sizeInBytes: y, bus: "name", dd: [{ddName: "name", bitSize: z}]}
        logical[0].canInput = FiCo4OMNeT::CanList{definition: [
            {canID: 1, sizeInBytes: 1, bus: "powertrain_can", dd: [
                    {ddName: "power", bitSize: 8}
                ]
            }
        ]};
        logical[0].canOutput = FiCo4OMNeT::CanList{definition: [
            {canID: 2, sizeInBytes: 2, bus: "powertrain_can", dd: [
                    {ddName: "voltage", bitSize: 8}, 
                    {ddName: "current", bitSize: 8}
                ]
            }
        ]};
        //logical[0].dataDictOut = default, does not generate other datadicts

        logical[1].name = "PowerController";
        logical[1].priority = 6; // 6 is lowest task prio
        //logical[1].canInput = default, does not read CAN messages
        //logical[1].canOutput = default, does not write CAN messages
        logical[1].dataDictOut = FiCo4OMNeT::DataDictList{definition: [
            {ddName: "voltage", bitSize: 8},
            {ddName: "current", bitSize: 8}
        ]};
    connections:
        // Logical[0] (ssc) gets voltage and current but sets power
        logical[0].getDataDict++ <--> dd[0].get++;
        logical[0].getDataDict++ <--> dd[1].get++;
        logical[0].setDatadict++ --> dd[2].set;

        // Logical[1] (powercontroller) gets power but sets voltage and current
        logical[1].setDatadict++ --> dd[0].set;
        logical[1].setDatadict++ --> dd[1].set;
        logical[1].getDataDict++ <--> dd[2].get++;
}