#include <iostream>
#include <stdint.h>
#include <string.h>
#include <vector>
#include <tr1/memory>

#include <Message.h>
#include <Transfer.h>
#include <Callbacks.h>
#include <SDO.h>
#include <PDO.h>

namespace CANOpen {

void PDO::writeMappingCount(std::vector<uint8_t> &d)
{
    d.push_back(objectCount);
}
void PDO::readMappingCount(const std::vector<uint8_t> &d)
{
    objectCount = d[0];
    std::cerr << "objectCount " << (int)objectCount << std::endl;
    mapped = objectCount>0;
}

void PDO::writeMappingData(int i, std::vector<uint8_t> &d)
{
    uint8_t *pd;
    d.push_back(mappedObjects[i].bits);
    d.push_back(mappedObjects[i].subindex);
    uint16_t index_le = htole16(mappedObjects[i].index);
    pd = (uint8_t *)&index_le;
    std::copy(pd, pd+sizeof(uint16_t), std::back_inserter(d));
}

void PDO::readMappingData(int i, const std::vector<uint8_t> &d)
{
    std::vector<uint8_t>::const_iterator b=d.begin();
    mappedObjects[i].bits = *b;
    b++;
    mappedObjects[i].subindex = *b;
    b++;
    mappedObjects[i].index = *b;
    b++;
    mappedObjects[i].index |= (*b)<<8;
    std::cerr << "mapping data " << i << " 0x" <<
        std::hex << mappedObjects[i].index << " 0x" <<
        (int)mappedObjects[i].subindex << " " << std::dec <<
        (int)mappedObjects[i].bits << std::endl;
}

void PDO::writeCommunicationData(std::vector<uint8_t> &d)
{
    uint32_t comm_params;
    comm_params = communication_parameters.identifier |
                  communication_parameters.extended<<29 |
                  communication_parameters.rtr<<30 |
                  communication_parameters.invalid<<31;
    comm_params = htole32(comm_params);
    uint8_t *pc = (uint8_t *)&comm_params;
    std::copy(pc, pc+sizeof(uint32_t), std::back_inserter(d));
}

void PDO::readCommunicationData(const std::vector<uint8_t> &d)
{
    uint32_t comm_params;
    std::vector<uint8_t>::const_iterator b=d.begin();
    comm_params = *b;
    b++;
    comm_params |= (*b<<8);
    b++;
    comm_params |= (*b<<16);
    b++;
    comm_params |= (*b<<24);
    comm_params = le32toh(comm_params);
    communication_parameters.identifier = comm_params & 0x1FFFFFFF;
    communication_parameters.extended = comm_params & 0x20000000;
    communication_parameters.rtr = comm_params & 0x40000000;
    communication_parameters.invalid = comm_params & 0x80000000;

    std::cerr << "com params: 0x" << std::hex << (int)comm_params <<
        std::dec << std::endl;
}

void PDO::writeCommunicationTypeData(std::vector<uint8_t> &d)
{
    d.push_back(communication_parameters.type);
}

void PDO::readCommunicationTypeData(const std::vector<uint8_t> &d)
{
    communication_parameters.type = d[0];
    std::cerr << "com type: 0x" << std::hex << (int)d[0] <<
        std::dec << std::endl;
}

void PDO::mappingComplete(SDO &sdo)
{
    mapped=true;
}

uint16_t RPDO::COBID(void)
{
    return 0x200+(((pdo_number-1)<<8)|node_id);
}

bool TPDO::handleMessage(const Message m)
{
    if(m.id == COBID())
    {
        data.clear();
        std::copy(&m.data[0], &m.data[8], std::back_inserter(data));
        callback(*this);
        return true;
    }
    return false;
}

uint16_t RPDO::communicationIndex(void)
{
    // Receive PDO communication parameters
    return 0x1400+pdo_number-1;
}

uint16_t RPDO::mappingIndex(void)
{
    // Receive PDO mapping parameters
    return 0x1600+pdo_number-1;
}

uint16_t TPDO::COBID(void)
{
    return 0x080+((pdo_number<<8)|node_id);
}

uint16_t TPDO::communicationIndex(void)
{
    // Transmit PDO communication parameters
    return 0x1800 + pdo_number-1;
}

uint16_t TPDO::mappingIndex(void)
{
    // Transmit PDO mapping parameters
    return 0x1A00+pdo_number-1;
}

void RPDO::send(std::vector<uint8_t> &data, PDOCallbackObject callback)
{
    Message m;

    m.id = COBID();
    m.RTR = false;
    m.EXTENDED = false;
    m.dlc=8;
    int tocopy=std::min((int)data.size(), 8);
    std::copy(data.begin(), data.begin()+tocopy, &m.data[0]);

    to_send.push_back( m );

    this->callback = callback;
}

}
