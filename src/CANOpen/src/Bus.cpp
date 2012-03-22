#include <iostream>
#include <stdint.h>
#include <vector>
#include <string>
#include <list>
#include <tr1/memory>

#include <Interface.h>
#include <Message.h>
#include <Transfer.h>
#include <Bus.h>

namespace CANOpen {
Bus::Bus(std::tr1::shared_ptr<Interface> interface, bool log):
    interface(interface),
    log(log)
{
    current_transfer=active_transfers.begin();
};

int Bus::runonce(bool &canRead, bool &canWrite)
{
    if(active_transfers.empty())
            return 0;
    if (canRead){
        Message m;
        int err = interface->readMessage(m);
        if(err<0)
            return err;
        if(log) std::cout << "recv: " << m << std::endl;
        for(transfer_iterator it=active_transfers.begin(); it!=active_transfers.end(); it++)
            if((*it)->handleMessage(m))
                break;
    }
    if( canWrite ) {
        transfer_iterator start=current_transfer;
        do {
            current_transfer++;
            if(current_transfer == active_transfers.end()) {
                current_transfer=active_transfers.begin();
            }
            if((*current_transfer)->canSend()) {
                Message m;
                (*current_transfer)->nextMessage(m);
                if(log) std::cout << "send: " << m << std::endl;
                interface->sendMessage(m);
                break;
            }
        } while(current_transfer != start);
    }

    if(canSend() == active_transfers.end()) {
        canWrite = false;
    } else {
        canWrite = true;
    }
    canRead = true;
    return 0;
}

transfer_iterator Bus::canSend(void)
{
    transfer_iterator it;
    for( it=active_transfers.begin(); it!=active_transfers.end(); it++)
        if((*it)->canSend())
            break;
    return it;
}

int Bus::add(std::tr1::shared_ptr<Transfer> xfer)
{
    active_transfers.push_back(xfer);
    return 0;
}

}
