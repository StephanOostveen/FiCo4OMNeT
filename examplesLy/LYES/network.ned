package fico4omnet.examplesLy.LYES;

import fico4omnet.bus.can.CanBus;
import fico4omnet.nodes.can.LyPhysicalNode;
import fico4omnet.examplesLy.LYES.SCUNode;
import fico4omnet.examplesLy.LYES.VCUNode;
import fico4omnet.examplesLy.LYES.CGWNode;

network LyCanNetwork {
    parameters:
    submodules:
        //CAN busses:
        powertrain: CanBus;
        vehicle: CanBus;
        solar: CanBus;
        telematics: CanBus;

        // Nodes:
        SCU: SCUNode;
        CGW: CGWNode;
        VCU: VCUNode;
        
    connections:
        powertrain.gate++ <--> SCU.gate++;
        powertrain.gate++ <--> CGW.gate++;

        vehicle.gate++ <--> VCU.gate++;
        vehicle.gate++ <--> CGW.gate++;

        solar.gate++ <--> VCU.gate++;
        telematics.gate++ <--> CGW.gate++;
}