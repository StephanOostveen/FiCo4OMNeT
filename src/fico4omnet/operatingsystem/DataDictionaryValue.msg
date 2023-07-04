namespace FiCo4OMNeT;

packet DataDictionaryValue {
    string ddName;
    simtime_t generationTime;
    simtime_t minimalDependencyTime;
}

packet DataDictionaryValueList {
    DataDictionaryValue value[];
}