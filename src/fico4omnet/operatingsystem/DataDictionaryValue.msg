namespace FiCo4OMNeT;

packet DataDictionaryValue {
    string ddName;
    simtime_t generationTime;
    unsigned long writeCount;
}

packet DataDictionaryValueList {
    DataDictionaryValue value[];
}