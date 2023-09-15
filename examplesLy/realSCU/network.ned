package fico4omnet.examplesLy.realSCU;

import fico4omnet.bus.can.CanBus;
import fico4omnet.nodes.can.LyPhysicalNode;
import fico4omnet.examplesLy.realSCU.SCUNode;

network LyCanNetwork {
    parameters:
    submodules:
        //CAN busses:
        powertrain_can: CanBus;
        // Nodes:
        SCU: SCUNode;

    connections:
        powertrain_can.gate++ <--> SCU.gate++;
}