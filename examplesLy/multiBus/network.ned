package fico4omnet.examplesLy.multiBus;

import fico4omnet.bus.can.CanBus;
import fico4omnet.nodes.can.LyPhysicalNode;
import fico4omnet.examplesLy.VCUNode;
import fico4omnet.examplesLy.SCUNode;
import fico4omnet.examplesLy.CGWNode;

network LyCanNetwork {
    parameters:
    submodules:
        //CAN busses:
        powertrain_can: CanBus;
        vehicle_can:    CanBus;
        // Nodes:
        CGW: CGWNode;
        VCU: VCUNode;
        SCU: SCUNode;

    connections:
        vehicle_can.gate++ <--> VCU.gate++;
        vehicle_can.gate++ <--> CGW.gate++;
        powertrain_can.gate++ <--> CGW.gate++;
        powertrain_can.gate++ <--> SCU.gate++;
}