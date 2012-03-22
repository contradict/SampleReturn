#ifndef __TRANSFER_H__
#define __TRANSFER_H__
namespace CANOpen
{

class TransferCallbackReceiver{};

class Transfer : public TransferCallbackReceiver {
    public:
        Transfer(std::string name,
                long int node_id):
            name(name),
            node_id(node_id)
            {};

        virtual uint16_t COBID(void)=0;
        // true if message is for me
        virtual bool handleMessage(const Message m)=0;
        virtual void send(Message m){to_send.push_back(m);};

        // true if there is a message to send
        virtual bool canSend(void){ return !to_send.empty(); };
        // true if m has been modified to send
        // false if no message to send
        virtual bool nextMessage(Message &m)
        {
            if(to_send.empty()){
                return false;
            } else {
                m = to_send.front();
                to_send.erase(to_send.begin());
                return true;
            }
        };

        virtual std::string getName(void){return name;};

    protected:

        const std::string name;
        const uint32_t node_id;

        std::vector<Message> to_send;
};

}
#endif // __TRANSFER_H__
