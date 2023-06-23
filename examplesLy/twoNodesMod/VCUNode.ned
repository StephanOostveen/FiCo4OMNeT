package fico4omnet.examplesLy.twoNodesMod;

import fico4omnet.nodes.can.LyPhysicalNode;

module VCUNode extends LyPhysicalNode {
    parameters:
        numDataDicts = 2;
        numLogicals = 1;
        
        dd[0].name = "voltage";
        dd[1].name = "current";
        // dd[2].name = "power";

        logical[0].name = "Inverter";
        logical[0].priority = 6; // 6 is lowest task prio

        logical[0].canInput = FiCo4OMNeT::CanList{definition: [
            {canID: 2, sizeInBytes: 2, bus: "powertrain_can", dd: [
                    {ddName: "voltage", bitSize: 8}, 
                    {ddName: "current", bitSize: 8}
                ]
            }
        ]};
        logical[0].canOutput = FiCo4OMNeT::CanList{definition: [
            {canID: 1, sizeInBytes: 1, bus: "powertrain_can", dd: [
                    {ddName: "power", bitSize: 8}
                ]
            }
        ]};
        
    connections:
        logical[0].setDatadict++ --> dd[0].set;
        logical[0].setDatadict++ --> dd[1].set;
}