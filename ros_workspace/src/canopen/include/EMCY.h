#ifndef __EMCY__H__
#define __EMCY__H__

namespace CANOpen {

class EMCY;
typedef CallbackObject<EMCY> EMCYCallbackObject;


class EMCY : public Transfer {
    public:
        EMCY(unsigned long node_id,
             EMCYCallbackObject callback=EMCYCallbackObject());
        uint16_t COBID(void);
        bool handleMessage(const Message m);
        const char *error_string(void);
        static const char *error_string(const uint16_t error_code);

        uint16_t error_code;
        uint16_t error_register;
        uint8_t error_data[5];

    private:
        EMCYCallbackObject callback;
};
}

#endif // __EMCY__H__


