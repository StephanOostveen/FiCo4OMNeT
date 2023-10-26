package fico4omnet.nodes.can;

import fico4omnet.linklayer.can.CanNodePort;
import fico4omnet.buffer.can.CanInputBuffer;
import fico4omnet.buffer.can.CanOutputBuffer;
import fico4omnet.scheduler.can.CanClock;

module ParametrizableCanDevice {
    parameters:
        @display("i=device/modem");
        //percentage of errors
        int errorperc;
    gates:
        inout gate;
        input tx;
    submodules:
        canNodePort: CanNodePort {
            gates:
                phygate;
        };

        bufferIn: CanInputBuffer {
            gates:
                in[1];
        };
        
        bufferOut: CanOutputBuffer {
            gates:
                in[1];
        };

        canClock: CanClock {
        };

    connections:
        canNodePort.phygate <--> gate;
        canNodePort.upperLayerIn <-- bufferOut.out;
        tx --> bufferOut.in[0];
}