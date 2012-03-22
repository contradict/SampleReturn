#include <stdint.h>

#include <Interface.h>

namespace CANOpen {

static const struct errstrings {
    int errcode;
    const char *errstr;
} canopenerrors[] = {
    {ERR_INVALID_BAUD, "Invalid baud rate"},
    {ERR_SET_BAUD, "Unable to set baud"},
    {ERR_INVALID_OUTPUT_MODE, "Invalid output mode"},
    {ERR_SET_OUTPUT_MODE, "Unable to set output mode"},
    {ERR_BUS_STATE, "Unable to change bus state"},
    {ERR_SEND_MSG, "Unable to send message"},
    {ERR_OPEN_CHANNEL, "Unable open channel"},
    {0, "Unknown error"}
};

const char *Interface::strerror(int err)
{
    int i;
    for(i=0;canopenerrors[i].errcode<0;i++){
        if(err == canopenerrors[i].errcode) break;
    }
    return canopenerrors[i].errstr;
};

}
