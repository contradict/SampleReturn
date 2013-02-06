#ifndef __KVASER_INTERFACE_H__
#define __KVASER_INTERFACE_H__

namespace CANOpen {

#define KVASER_OUTPUT_MODE_NORMAL 4
#define KVASER_OUTPUT_MODE_SILENT 1
#define KVASER_OUTPUT_MODE_SELFRECEPTION 8
#define KVASER_OUTPUT_MODE_OFF 0

class KvaserInterface : public Interface {
    public:
        KvaserInterface(int channel, uint32_t baud);
        ~KvaserInterface();
        int setOutputMode(int mode);
        int setBusState(int state);
        // Configureation
        int setBaud(uint32_t baud);
        // ????

        // Communication
        // send immediately
        int sendMessage(const struct Message &m);
        // block until available, then read
        int readMessage(struct Message &m);
        // return file descriptor for select() or poll()
        int getFD(void);
    private:
        canHandle   handle;
        int         errcode;
        int         channel;

};

}
#endif // __KVASER_INTERFACE_H__
