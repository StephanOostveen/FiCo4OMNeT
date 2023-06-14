import DataDictionaryDefinition;

namespace FiCo4OMNeT;

class CanDataFrameDefinition extends cOwnedObject {
    unsigned int canID; // ID of the message
    unsigned int sizeInBytes;
    string bus;
    DataDictionaryDefinition dd[];
}

class CanList extends cOwnedObject {
    CanDataFrameDefinition definition[];
}