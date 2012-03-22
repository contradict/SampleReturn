#include <iostream>
#include <stdint.h>
#include <string.h>
#include <vector>

#include <Message.h>
#include <Transfer.h>
#include <Callbacks.h>
#include <EMCY.h>

namespace CANOpen {
typedef struct {
    uint16_t    value;
    uint16_t    mask;
    const char* name;
} errdescr;

static const errdescr EmergencyErrors[] = {
    {0x0000, 0x0000, "Unknown Error" },
    {0x0000, 0xFFFF, "Error Reset or No Error" },
    {0x1000, 0xFF00, "Generic Error" },
    {0x2000, 0xFF00, "Current" },
    {0x2100, 0xFF00, "current, device input side" },
    {0x2200, 0xFF00, "current, inside the device" },
    {0x2300, 0xFF00, "current, device output side" },
    {0x3000, 0xFF00, "Voltage" },
    {0x3100, 0xFF00, "mains voltage" },
    {0x3200, 0xFF00, "voltage inside the device" },
    {0x3300, 0xFF00, "output voltage" },
    {0x4000, 0xFF00, "Temperature" },
    {0x4100, 0xFF00, "ambient temperature" },
    {0x4200, 0xFF00, "device temperature" },
    {0x5000, 0xFF00, "Device hardware" },
    {0x6000, 0xFF00, "Device software" },
    {0x6100, 0xFF00, "internal software" },
    {0x6200, 0xFF00, "user software" },
    {0x6300, 0xFF00, "data set" },
    {0x7000, 0xFF00, "Additional modules" },
    {0x8000, 0xFF00, "Monitoring" },
    {0x8100, 0xFF00, "Communication" },
    {0x8110, 0xFFFF, "CAN overrun" },
    {0x8120, 0xFFFF, "Error Passive" },
    {0x8130, 0xFFFF, "Life Guard Error or Heartbeat Error" },
    {0x8140, 0xFFFF, "Recovered from Bus-Off" },
    {0x8200, 0xFF00, "Protocol Error" },
    {0x8210, 0xFFFF, "PDO not processed due to length error" },
    {0x8220, 0xFFFF, "Length exceeded" },
    {0x9000, 0xFF00, "External error" },
    {0xF000, 0xFF00, "Additional functions" },
    {0xFF00, 0xFF00, "Device specific" },
    {0x0000, 0x0000, NULL }
};


static const errdescr* find_most_specific_error(uint16_t errorcode)
{
    const errdescr* most_specific_error = EmergencyErrors;
    for(const errdescr *err = EmergencyErrors + 1; // skip unknown error
        err->name;
        err++)
    {
        if((err->mask&errorcode == err->value) &&
           (__builtin_popcount(err->mask)>__builtin_popcount(most_specific_error->mask)))
            {
                most_specific_error = err;
            }
    }
    return most_specific_error;
}

EMCY::EMCY( unsigned long node_id,
            EMCYCallbackObject callback) :
    Transfer("Default NMT", node_id),
    callback(callback),
    error_code(0),
    error_register(0)
{
}

uint16_t EMCY::COBID(void)
{
    return 0x80|node_id;
}

bool EMCY::handleMessage(Message m)
{
    if(m.id != COBID()) return false;
    error_code = m.data[0] + (m.data[1]<<8);
    error_register = m.data[2];
    memcpy(error_data, m.data+3, 5);
    callback(*this);
    return true;
}

const char *EMCY::error_string(void)
{
    return find_most_specific_error(error_code)->name;
}

const char *EMCY::error_string(const uint16_t static_error_code)
{
    return find_most_specific_error(static_error_code)->name;
}
}
