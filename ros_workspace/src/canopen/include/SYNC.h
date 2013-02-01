#ifndef __SYNC_H__
#define __SYNC_H__
#include <stdint.h>

#include <Message.h>
#include <Transfer.h>
#include <Callbacks.h>

namespace CANOpen {

class SYNC;
typedef CallbackObject<SYNC> SYNCCallbackObject;

class SYNC : public Transfer {
    public:
        SYNC(uint16_t node_id,
             SYNCCallbackObject callback=SYNCCallbackObject()):
             Transfer("SYNC handler", node_id),
             callback(callback) { };

        uint16_t COBID(void) { return 0x80; };

        bool handleMessage(const Message m)
        {
            if(m.id == COBID())
                callback(*this);
            return false;
        };

    private:
        SYNCCallbackObject callback;

};

}
#endif // __SYNC_H__
