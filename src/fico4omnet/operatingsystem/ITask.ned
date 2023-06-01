package fico4omnet.operatingsystem;

// Interface for OpenECU tasks that will be scheduled by the OS
moduleinterface ITask {
    parameters:
        double period @unit(s);
        double executionTime @unit(s);
        bool periodic;
        
        // Priority, lower numerical number is lower priority, folllows OpenECU table
        // http://support.openecu.com/doc_user/openecu_user_guide_c_api/openecu_user_guide_c_api.html#library_tasks
        int priority;
    gates:
        //Interfaces with the scheduler
        inout scheduler @labels(SchedulerEvent);
}