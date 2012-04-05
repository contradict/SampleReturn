#ifndef __BUS__H__
#define __BUS__H__

namespace CANOpen {

typedef std::list<std::tr1::shared_ptr<Transfer> >::iterator transfer_iterator;

class Bus {
    public:
        Bus(std::tr1::shared_ptr<Interface> interface, bool log=false);
        int runonce(bool &canRead, bool &canWrite);
        int add(std::tr1::shared_ptr<Transfer> xfer);
        bool canSend(void);

    private:

        std::tr1::shared_ptr<Interface> interface;
        bool log;
        std::list<std::tr1::shared_ptr<Transfer> > active_transfers;
        transfer_iterator current_transfer;
};

}
#endif // __BUS__H__
