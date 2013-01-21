#include <stdint.h>
#include <iostream>
#include <vector>

#include <Message.h>
#include <Transfer.h>

namespace CANOpen {

void Transfer::pack(uint8_t value, std::vector<uint8_t> &data)
{
    data.push_back(0);
    data.back() = value;
}

void Transfer::pack(uint16_t value, std::vector<uint8_t> &data)
{
    // append two elements
    data.resize(data.size()+2);
    // get a pointer to the first of them
    std::vector<uint8_t>::reverse_iterator element=data.rbegin();
    element++;
    *((uint16_t *)&(*element)) = htole16(value);
}

void Transfer::pack(uint32_t value, std::vector<uint8_t> &data)
{
    data.resize(data.size() + 4);
    // point to the end
    std::vector<uint8_t>::reverse_iterator element=data.rbegin();
    // move back 3
    element+=3;
    *((uint32_t *)&(*element)) = htole32(value);
}

void Transfer::pack(int32_t value, std::vector<uint8_t> &data)
{
    data.resize(data.size() + 4);
    std::vector<uint8_t>::reverse_iterator element=data.rbegin();
    element+=3;
    *((int32_t *)&(*element)) = htole32(value);
}

void Transfer::unpack(std::vector<uint8_t> data, int index, int16_t &value)
{
    value = le16toh(*((uint16_t *)(data[index])));
}

void Transfer::unpack(std::vector<uint8_t> data, int index, uint16_t &value)
{
    value = le16toh(*((uint16_t *)(data[index])));
}

void Transfer::unpack(std::vector<uint8_t> data, int index, int32_t &value)
{
    value = le32toh(*((uint32_t *)(data[index])));
}

void Transfer::unpack(std::vector<uint8_t> data, int index, uint32_t &value)
{
    value = le32toh(*((uint32_t *)(data[index])));
}

}
