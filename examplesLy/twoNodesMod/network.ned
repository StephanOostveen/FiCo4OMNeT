package fico4omnet.examplesLy.twoNodesMod;

import fico4omnet.bus.can.CanBus;
import fico4omnet.nodes.can.LyPhysicalNode;
import fico4omnet.examplesLy.VCUNode;
import fico4omnet.examplesLy.SCUNode;

network LyCanNetwork {
    parameters:
    submodules:
        powertrain_can: CanBus;
        VCU: VCUNode;
        SCU: SCUNode;

    connections:
        powertrain_can.gate++ <--> VCU.gate++;
        powertrain_can.gate++ <--> SCU.gate++;
}