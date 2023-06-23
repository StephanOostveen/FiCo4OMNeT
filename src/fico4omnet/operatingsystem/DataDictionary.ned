package fico4omnet.operatingsystem;

simple DataDictionary {
    parameters:
        @display("i=block/blackboard;t=$name");
        string name;

    gates:
        inout get[];
        input set;
}
