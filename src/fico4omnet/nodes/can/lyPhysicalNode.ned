package fico4omnet.nodes.can;

import fico4omnet.applications.ISourceApplication;
import fico4omnet.applications.ISinkApplication;
import fico4omnet.applications.can.source.lysourceapp.CanLySourceApp;
import fico4omnet.applications.can.sink.lysinkapp.CanLySinkApp;
import fico4omnet.nodes.can.LyCanDevice;
import fico4omnet.operatingsystem.Logical;
import fico4omnet.operatingsystem.Scheduler;


module LyPhysicalNode
{
    parameters:
        @display("i=device/modem");
        int hardwareBufferSize = default(2);
    gates:
        inout gate[];

    submodules:
        canDevice[sizeof(gate)]: LyCanDevice {
            @display("i=device/modem");
        };
        
        sourceApp: <default("CanLySourceApp")> like ISourceApplication {
            //@display("p=98,32,row,70");
        };
        
        sinkApp: <default("CanLySinkApp")> like ISinkApplication {
           // @display("p=203,32");
        };
                
        scheduler : Scheduler {
            //@display("p=149,32");
        };

        logical : Logical;
    connections:
        for i=0..sizeof(gate)-1 {
            gate[i] <--> canDevice[i].gate;
            sourceApp.out++ --> canDevice[i].tx;
        }
        sourceApp.scheduler <-->scheduler.tasks++;
        
        sinkApp.scheduler <--> scheduler.tasks++;
        
        logical.frameOut++ --> sourceApp.in++;
        logical.getFrame++ <--> sinkApp.getFrame++;
        logical.scheduler <--> scheduler.tasks++;
}
