#include <iostream>
#include <stdint.h>
#include <string.h>
#include <vector>
#include <map>

#include <Message.h>
#include <Transfer.h>
#include <Callbacks.h>
#include <invertmap.h>
#include <NMT.h>

namespace CANOpen {

static std::map<std::string, enum NodeState> create_NodeStates(void)
{
    std::map<std::string, enum NodeState> m;
    m["Initialising"]  = Initialising;
    m["Disconnected"]  = Disconnected;
    m["Connecting"]    = Connecting;
    m["Preparing"]     = Preparing;
    m["Stopped"]       = Stopped;
    m["Operational"]   = Operational;
    m["PreOperational"]= PreOperational;
    m["InvalidState"]  = InvalidState;
    return m;
};
std::map<std::string, enum NodeState> NodeStates = create_NodeStates();
std::map<enum NodeState, std::string> NodeStateNames = InvertMap(NodeStates);

static std::map<std::string, enum NodeControlCommand> create_NodeControlCommands(void)
{
    std::map<std::string, enum NodeControlCommand> m;
    m["StartRemoteNode"]     = StartRemoteNode;
    m["StopRemoteNode"]      = StopRemoteNode;
    m["EnterPreOperational"] = EnterPreOperational;
    m["ResetNode"]           = ResetNode;
    m["ResetCommunication"]  = ResetCommunication;
    return m;
};
std::map<std::string, enum NodeControlCommand> NodeControlCommands = create_NodeControlCommands();
std::map<enum NodeControlCommand, std::string> NodeControlCommandNames = InvertMap(NodeControlCommands);

NMT::NMT(uint16_t node_id, NMTCallbackObject callback) :
Transfer("Default NMT", node_id),
callback(callback),
node_state(NodeStates["Initialising"])
{
}

uint16_t NMT::COBID(void)
{
    return 0x700|node_id;
}

void NMT::printCommands(void)
{
    std::map<std::string, enum NodeControlCommand>::iterator it;

    for(it=NodeControlCommands.begin();it != NodeControlCommands.end(); it++) {
        std::cout << "'" << (*it).first << "' => '" << (*it).second <<"'"<< std::endl;
    }
}

bool NMT::validCommandName(std::string cmdname)
{
    return NodeControlCommands.count(cmdname)>0; 
}
enum NodeControlCommand NMT::nodeControlCommand(std::string st)
{
    return NodeControlCommands[st];
}
bool NMT::nodeControl(std::string cmdname)
{
    if(validCommandName(cmdname)){
        nodeControl(NodeControlCommands[cmdname]);
        return true;
    } else return false;
}
void NMT::nodeControl(enum NodeControlCommand cmd)
{
    Message m;

    m.id = 0x0;
    m.data[0] = (uint8_t)cmd;
    m.data[1] = node_id;
    m.dlc = 2;
    send( m );
}

Message NMT::nodeGuard()
{
    Message m;

    m.id = COBID();
    m.RTR = true;
    m.dlc = 0;
    return m;
}

bool NMT::handleMessage(Message m)
{
    if(m.id != COBID()) return false;
    enum NodeState msg_state = static_cast<enum NodeState>(m.data[0]);
    if(NodeStateNames.count(msg_state) == 1) {
        node_state = msg_state;
    }
    callback(*this);
    return true;
}

std::string NMT::nodeStateName(enum NodeState st)
{
    return NodeStateNames[st];
}
bool NMT::validStateName(std::string st)
{
    return NodeStates.count(st)>0; 
}
enum NodeState NMT::nodeState(std::string st)
{
    if(validStateName(st)){
        return NodeStates[st];
    } else {
        return InvalidState;
    }
}

}
