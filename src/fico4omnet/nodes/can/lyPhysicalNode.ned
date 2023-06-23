package fico4omnet.nodes.can;

import fico4omnet.applications.ISourceApplication;
import fico4omnet.applications.ISinkApplication;
import fico4omnet.applications.can.source.lysourceapp.CanLySourceApp;
import fico4omnet.applications.can.sink.lysinkapp.CanLySinkApp;
import fico4omnet.nodes.can.LyCanDevice;
import fico4omnet.operatingsystem.Logical;
import fico4omnet.operatingsystem.Scheduler;
import fico4omnet.operatingsystem.DataDictionary;


module LyPhysicalNode
{
    parameters:
        @display("i=device/modem");
        int hardwareBufferSize = default(2);

        int numDataDicts;
        int numLogicals;
        sinkApp.priority = 14 + sizeof(gate);
        
    gates:
        inout gate[];

    submodules:
        canDevice[sizeof(gate)]: LyCanDevice {
            @display("i=device/modem");
        };
        
        sourceApp: <default("CanLySourceApp")> like ISourceApplication;
        sinkApp: <default("CanLySinkApp")> like ISinkApplication;
        scheduler : Scheduler;

        logical[numLogicals] : Logical;
        dd[numDataDicts] : DataDictionary;

    connections:
        for i=0..sizeof(gate)-1 {
            gate[i] <--> canDevice[i].gate;
            sourceApp.out++ --> canDevice[i].tx;
        }
        sourceApp.scheduler <-->scheduler.tasks++;
        sinkApp.scheduler <--> scheduler.tasks++;
        
        for i=0..sizeof(logical)-1 {
            logical[i].frameOut --> sourceApp.in++;
            logical[i].getFrame <--> sinkApp.getFrame++;
            logical[i].scheduler <--> scheduler.tasks++;
        }

}
