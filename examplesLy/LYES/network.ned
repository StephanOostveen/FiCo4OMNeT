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
        surrounding_sense: CanBus;
        drivetrain: CanBus;
        energy_management: CanBus;
        HVBS: CanBus;

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

        //VCU Driver Controls Manager
        stalks: ParametrizableNode;
        suroundsens1: ParametrizableNode;
        suroundsens2: ParametrizableNode;
        suroundsens3: ParametrizableNode;
        suroundsens4: ParametrizableNode;
        suroundsens5: ParametrizableNode;
        suroundsens6: ParametrizableNode;
        suroundsens7: ParametrizableNode;
        suroundsens8: ParametrizableNode;
        //VCU Solar
        dual_string_converter1: ParametrizableNode;
        dual_string_converter2: ParametrizableNode;
        dual_string_converter3: ParametrizableNode;
        dual_string_converter4: ParametrizableNode;
        dual_string_converter5: ParametrizableNode;
        dual_string_converter6: ParametrizableNode;
        dual_string_converter7: ParametrizableNode;
        dual_string_converter8: ParametrizableNode;
        dual_string_converter9: ParametrizableNode;
        dual_string_converter10: ParametrizableNode;
        dual_string_converter11: ParametrizableNode;
        dual_string_converter12: ParametrizableNode;
        dual_string_converter13: ParametrizableNode;
        dual_string_converter14: ParametrizableNode;

        //SCU prc:
        inverter_fl : ParametrizableNode;
        inverter_fr : ParametrizableNode;
        inverter_rl : ParametrizableNode;
        inverter_rr : ParametrizableNode;
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
        solar.gate++ <--> dual_string_converter1.gate++;
        solar.gate++ <--> dual_string_converter2.gate++;
        solar.gate++ <--> dual_string_converter3.gate++;
        solar.gate++ <--> dual_string_converter4.gate++;
        solar.gate++ <--> dual_string_converter5.gate++;
        solar.gate++ <--> dual_string_converter6.gate++;
        solar.gate++ <--> dual_string_converter7.gate++;
        solar.gate++ <--> dual_string_converter8.gate++;
        solar.gate++ <--> dual_string_converter9.gate++;
        solar.gate++ <--> dual_string_converter10.gate++;
        solar.gate++ <--> dual_string_converter11.gate++;
        solar.gate++ <--> dual_string_converter12.gate++;
        solar.gate++ <--> dual_string_converter13.gate++;
        solar.gate++ <--> dual_string_converter14.gate++;
        
        telematics.gate++ <--> CGW.gate++;
        telematics.gate++ <--> user_authentication_app.gate++;

        driver_support.gate++ <--> VCU.gate++;
        driver_support.gate++ <--> stalks.gate++;

        surrounding_sense.gate++ <--> VCU.gate++;
        surrounding_sense.gate++ <--> suroundsens1.gate++;
        surrounding_sense.gate++ <--> suroundsens2.gate++;
        surrounding_sense.gate++ <--> suroundsens3.gate++;
        surrounding_sense.gate++ <--> suroundsens4.gate++;
        surrounding_sense.gate++ <--> suroundsens5.gate++;
        surrounding_sense.gate++ <--> suroundsens6.gate++;
        surrounding_sense.gate++ <--> suroundsens7.gate++;
        surrounding_sense.gate++ <--> suroundsens8.gate++;

        drivetrain.gate++ <--> SCU.gate++;
        drivetrain.gate++ <--> inverter_fl.gate++;
        drivetrain.gate++ <--> inverter_fr.gate++;
        drivetrain.gate++ <--> inverter_rl.gate++;
        drivetrain.gate++ <--> inverter_rr.gate++;

        energy_management.gate++ <--> SCU.gate++;
        HVBS.gate++ <--> SCU.gate++;
}