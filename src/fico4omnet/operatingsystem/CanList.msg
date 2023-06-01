import DataDictionary;

namespace FiCo4OMNeT;

class CanDataFrameDefinition extends cObject {
    unsigned int canID; // ID of the message
    unsigned int sizeInBytes;
    string bus;
    DataDictionary dd[];
}

class CanList extends cObject {
    CanDataFrameDefinition definition[];
}