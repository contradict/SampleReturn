#include <stdint.h>
#include <string.h>
#include <errno.h>
#include <iostream>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>

#include <canlib.h>

#include <Interface.h>
#include <Message.h>
#include <KvaserInterface.h>

namespace CANOpen {

#define DEFAULT_BAUD 250000

struct KvaserStandardBaud {
    unsigned long baud;
    int code;
} standard_baud[] = {
    { 1000000, canBITRATE_1M },
    { 500000, canBITRATE_500K },
    { 250000, canBITRATE_250K },
    { 125000, canBITRATE_125K },
    { 100000, canBITRATE_100K },
    { 62500, canBITRATE_62K },
    { 50000, canBITRATE_50K },
    { 83333, canBITRATE_83K },
    { 10000, canBITRATE_10K }
    };

KvaserInterface::KvaserInterface(int channel, uint32_t baud) :
    channel(channel)
{
    handle = canOpenChannel(channel, canWANT_EXCLUSIVE );
    if(handle<0) {
        std::cerr << "Unable to open channel " << channel << std::endl;
    } else {

        setBusState(0);

        if(setBaud(baud)<0) goto err_exit;

        if(setOutputMode(KVASER_OUTPUT_MODE_NORMAL)<0) goto err_exit;

        if(setBusState(1)<0) goto err_exit;
    }
    return;
err_exit:
    canClose(handle);
    handle=-1;
    return;
}

KvaserInterface::~KvaserInterface()
{
    if(handle>0) {
        setBusState(0);
        canClose(handle);
    }
}

int KvaserInterface::setBaud(uint32_t baud)
{
    int i=0;
    if(baud<=0) {
        return ERR_INVALID_BAUD;
    }
    for(i=0;standard_baud[i].baud>0;i++) {
        if(standard_baud[i].baud == baud) break;
    }
    if(standard_baud[i].baud == 0){
        return ERR_INVALID_BAUD;
    }

    std::cout << "setting baud " << baud << ", code: " << standard_baud[i].code << std::endl;
    canStatus ecode;
    ecode = canSetBusParams(handle,
            standard_baud[i].code,
            0,0,0,0,0);
    return 0;
    if(ecode<0) {
        char msgbuf[128];
        canGetErrorText(ecode, msgbuf, 128);
        std::cerr << "Unable to set bus params (" << ecode << "): " << msgbuf << std::endl;
        long freq;
        unsigned int tseg1, tseg2, sjw, noSamp, syncmode;
        canGetBusParams(handle, &freq, &tseg1, &tseg2, &sjw, &noSamp, &syncmode);
        std::cerr << freq << " " << tseg1 << " " << tseg2 << " " << sjw << " "
            << noSamp << " " << syncmode << std::endl;
        return ERR_SET_BAUD;
    }

    return 0;
}

int KvaserInterface::setOutputMode(int mode)
{
    canStatus ecode;
    ecode = canSetBusOutputControl(handle, mode);
    if(ecode<0){
        char msgbuf[128];
        canGetErrorText(ecode, msgbuf, 128);
        std::cerr << "Unable to set output mode (" << ecode << "): " << msgbuf << std::endl;
        return ERR_SET_OUTPUT_MODE;
    }
    return 0;
}

int KvaserInterface::setBusState(int state)
{
    canStatus ecode;
    if(state) {
        ecode = canBusOn(handle);
    }else {
        ecode = canBusOff(handle);
    }
    if(ecode<0){
        char msgbuf[128];
        canGetErrorText(ecode, msgbuf, 128);
        std::cerr << "Unable to set bus state to " << state << " (" << ecode << "): " << msgbuf << std::endl;
        return ERR_BUS_STATE;
    }
    return 0;
}

int KvaserInterface::sendMessage(const struct Message &m)
{
    canStatus ecode;
    unsigned int flag=0;
    uint8_t message[8];

    memcpy(message, m.data, 8);

    if(m.EXTENDED) flag |= canMSG_EXT;
    if(m.RTR) flag |= canMSG_RTR;
    ecode = canWrite(handle, m.id, message, m.dlc, flag);
    if(ecode<0){
        char msgbuf[128];
        canGetErrorText(ecode, msgbuf, 128);
        std::cerr << "Unable to send message '" << m << "' " << " (" << ecode << "): " << msgbuf << std::endl;
        return ERR_SEND_MSG;
    }
    return 0;
}

int KvaserInterface::readMessage(struct Message &m)
{
    canStatus ecode;
    unsigned int flag;
    unsigned long timestamp;

    ecode = canRead(handle, &m.id, &m.data, &m.dlc, &flag, &timestamp);
    if(ecode<0){
        char msgbuf[128];
        canGetErrorText(ecode, msgbuf, 128);
        std::cerr << "Unable to read message" << " (" << ecode << "): " << msgbuf << std::endl;
        return ERR_READ_MSG;
    }
 
    m.RTR = flag&canMSG_RTR;
    m.EXTENDED = flag&canMSG_EXT;
    m.OVERRUN = flag&canMSGERR_HW_OVERRUN || flag&canMSGERR_SW_OVERRUN;
    m.ERROR = flag&canMSG_ERROR_FRAME;
    m.TXACK = flag&canMSG_TXACK;
    m.TXRQ = flag&canMSG_TXRQ;

    return 0;
}

int KvaserInterface::getFD()
{
    int fd=-1;
    if(handle>=0)
        canGetRawHandle(handle, &fd);
    return fd;
}


}
