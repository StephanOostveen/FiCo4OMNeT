package fico4omnet.examplesLy.twoNodes;

import fico4omnet.bus.can.CanBus;
import fico4omnet.nodes.can.LyCanNode;

network LyCanNetwork {
    submodules:
        bus: CanBus {
            gates:
                gate[2];
        }
        node[2]: LyCanNode;
    connections:
        bus.gate[0] <--> node[0].gate;
        bus.gate[1] <--> node[1].gate;
}