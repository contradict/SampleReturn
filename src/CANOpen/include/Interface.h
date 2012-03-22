
#ifndef __CAN_INTERFACE_H__
#define __CAN_INTERFACE_H__

namespace CANOpen {

#define ERR_INVALID_BAUD        -100
#define ERR_SET_BAUD            -101
#define ERR_INVALID_OUTPUT_MODE -102
#define ERR_SET_OUTPUT_MODE     -103
#define ERR_BUS_STATE           -104
#define ERR_SEND_MSG            -105
#define ERR_OPEN_CHANNEL        -106
#define ERR_READ_MSG            -107

class Interface {
    public:
        // Configureation
        virtual int setBaud(uint32_t baud) = 0;
        // ????

        // Communication
        // send immediately
        virtual int sendMessage(const struct Message &m) = 0;
        // block until available, then read
        virtual int readMessage(struct Message &m) = 0;
        // return file descriptor for select() or poll()
        virtual int getFD(void) = 0;

        virtual const char *strerror(int err);
};

}
#endif // __CAN_INTERFACE_H__
