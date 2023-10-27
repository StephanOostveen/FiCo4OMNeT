package fico4omnet.examplesLy.LYES;

import fico4omnet.bus.can.CanBus;
import fico4omnet.nodes.can.ParametrizableNode;
import fico4omnet.nodes.can.LyPhysicalNode;
import fico4omnet.examplesLy.LYES.SCUNode;
import fico4omnet.examplesLy.LYES.VCUNode;
import fico4omnet.examplesLy.LYES.CGWNode;
import fico4omnet.operatingsystem.Observer;

network LyCanNetwork {
    parameters:
    submodules:
        //CAN busses:
        powertrain: CanBus;
        vehicle: CanBus;
        vehicle2: CanBus;
        solar: CanBus;
        telematics: CanBus;
        driver_support: CanBus;

        // Nodes:
        SCU: SCUNode;
        CGW: CGWNode;
        VCU: VCUNode;
        
        /////////////////////////////
        //Parametrizable end nodes:
        /////////////////////////////
        //CGW authentication manager:
        user_authentication_system: ParametrizableNode; //Vouchio hardware device
        user_authentication_app: ParametrizableNode; //Vouchio app, which runs/access through telematics
        alarm_system: ParametrizableNode; // FleetAccess
        mediaECU: ParametrizableNode;
        //CGW closure manager:
        tailgateLatch: ParametrizableNode;
        doorFrontLeft: ParametrizableNode;
        doorFrontRight: ParametrizableNode;
        doorRearLeft: ParametrizableNode;
        doorRearRight: ParametrizableNode;

        //VCU TMS:
        fillingtool: ParametrizableNode;
        compressor: ParametrizableNode;

        //Observer:
        Observer: Observer;
        
    connections:
        powertrain.gate++ <--> SCU.gate++;
        powertrain.gate++ <--> CGW.gate++;
        powertrain.gate++ <--> doorFrontLeft.gate++;
        powertrain.gate++ <--> doorRearRight.gate++;
        powertrain.gate++ <--> mediaECU.gate++;
        powertrain.gate++ <--> alarm_system.gate++;

        vehicle.gate++ <--> VCU.gate++;
        vehicle.gate++ <--> CGW.gate++;
        vehicle.gate++ <--> fillingtool.gate++;
        vehicle.gate++ <--> compressor.gate++;
        
        vehicle2.gate++ <--> CGW.gate++;
        vehicle2.gate++ <--> tailgateLatch.gate++;
        vehicle2.gate++ <--> doorRearLeft.gate++;
        vehicle2.gate++ <--> doorFrontRight.gate++;
        vehicle2.gate++ <--> user_authentication_system.gate++;
        
        solar.gate++ <--> VCU.gate++;
        
        telematics.gate++ <--> CGW.gate++;
        telematics.gate++ <--> user_authentication_app.gate++;

        driver_support.gate++ <--> VCU.gate++;
}