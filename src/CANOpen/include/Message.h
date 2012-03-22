#ifndef __MESSAGE_H__
#define __MESSAGE_H__

namespace CANOpen {

struct Message {
    Message():
        id(0),
        dlc(8),
        EXTENDED(false),
        RTR(false),
        OVERRUN(false),
        ERROR(false),
        TXACK(false),
        TXRQ(false)
        {};
    int pack(uint8_t index, uint8_t value);
    int pack(uint8_t index, uint16_t value);
    int pack(uint8_t index, uint32_t value);

    long int id;
    unsigned int  dlc;
    bool EXTENDED : 1;
    bool RTR : 1;
    bool OVERRUN : 1;
    bool ERROR : 1;
    bool TXACK : 1;
    bool TXRQ : 1;
    uint8_t data[8];

    friend std::ostream & operator<< (std::ostream &out, const Message &m);
};


}

#endif // __MESSAGE_H__

