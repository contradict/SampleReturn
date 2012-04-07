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
    Message m;
    if (canRead){
        int err = interface->readMessage(m);
        if(err<0)
            return err;
        if(log) std::cout << "recv: " << m << std::endl;
    }
    if(active_transfers.empty()) {
            canWrite = false;
            canRead = true;
            return 0;
    } else if(canRead) {
        for(transfer_iterator it=active_transfers.begin(); it!=active_transfers.end(); it++)
            if((*it)->handleMessage(m))
                break;
    }
    if( canWrite ) {
        for(int i=0;i<active_transfers.size();i++) {
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
        }
    }

    canWrite = canSend();
    canRead = true;
    return 0;
}

bool Bus::canSend(void)
{
    transfer_iterator it;
    for( it=active_transfers.begin(); it!=active_transfers.end(); it++) {
        if((*it)->canSend()) {
            return true;
        }
    }
    return false;
}

int Bus::add(std::tr1::shared_ptr<Transfer> xfer)
{
    active_transfers.push_back(xfer);
    return 0;
}

}
