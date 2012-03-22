#include <stdint.h>
#include <iostream>

#include <Message.h>

namespace CANOpen {

std::ostream & operator<< (std::ostream &out, const Message &m)
{
    if(m.EXTENDED) {
        out << "ID(0x" << std::hex << (m.id&0x1fffffff) << "): ";
    } else {
        out << "ID(0x" << std::hex << (m.id&0x7ff) << "): ";
    }
    int len = ((m.dlc<=8)?m.dlc:8);
    for(int i=0;i<len;i++) {
        out << std::hex << (int)m.data[i] << " ";
    }
    out << ":";
    bool anyflags=false;
    if(m.EXTENDED) {
        anyflags = true;
        out << "EXTENDED";
    }
    if(m.RTR) {
        if(anyflags) out<<"|";
        out << "RTR";
        anyflags=true;
    }
     if(m.OVERRUN) {
        if(anyflags) out<<"|";
        out << "OVERRUN";
        anyflags=true;
    }
    if(m.ERROR) {
        if(anyflags) out<<"|";
        out << "ERROR";
        anyflags=true;
    }
    if(m.TXACK) {
        if(anyflags) out<<"|";
        out << "TXACK";
        anyflags=true;
    }
    if(m.TXRQ) {
        if(anyflags) out<<"|";
        out << "TXRQ";
        anyflags=true;
    }
    return out;
}

int Message::pack(uint8_t index, uint8_t value)
{
    data[index] = value;
    return index+1;
}

int Message::pack(uint8_t index, uint16_t value)
{
    *((uint16_t *)(&data[index])) = htole16(value);
    return index+2;
}

int Message::pack(uint8_t index, uint32_t value)
{
    *((uint32_t *)(&data[index])) = htole32(value);
    return index+4;
}

}
