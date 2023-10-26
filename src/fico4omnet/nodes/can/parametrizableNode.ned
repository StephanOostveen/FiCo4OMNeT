package fico4omnet.nodes.can;

import fico4omnet.applications.ISourceApplication;
import fico4omnet.applications.ISinkApplication;
import fico4omnet.applications.can.source.CanTrafficSourceAppBase;
import fico4omnet.applications.can.sink.CanTrafficSinkAppBase;
import fico4omnet.nodes.can.ParametrizableCanDevice;

module ParametrizableNode
{
    parameters:
        @display("i=device/modem");
    gates:
        inout gate[];
    submodules:
        canDevice[sizeof(gate)]: ParametrizableCanDevice {
            @display("i=device/modem");
        };

        sourceApp: <default("CanTrafficSourceAppBase")> like ISourceApplication;
        sinkApp: <default("CanTrafficSinkAppBase")> like ISinkApplication;

    connections:
        for i=0..sizeof(gate)-1 {
            gate[i] <--> canDevice[i].gate;
            sourceApp.out++ --> canDevice[i].tx;
        }
}