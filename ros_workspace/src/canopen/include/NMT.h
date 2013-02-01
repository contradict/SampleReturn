#ifndef __NMT_H__
#define __NMT_H__
namespace CANOpen {

enum NodeState {
    Initialising   = 0,
    Disconnected   = 1,
    Connecting     = 2,
    Preparing      = 3,
    Stopped        = 4,
    Operational    = 5,
    PreOperational = 127,
    InvalidState   = 255
};

enum NodeControlCommand {
    StartRemoteNode     = 1,
    StopRemoteNode      = 2,
    EnterPreOperational = 128,
    ResetNode           = 129,
    ResetCommunication  = 130,
};

class NMT;
typedef CallbackObject<NMT> NMTCallbackObject;

class NMT : public Transfer {
    public:
        NMT(uint16_t node_id,
            NMTCallbackObject callback=NMTCallbackObject());
        uint16_t COBID(void);
        static void printCommands(void);
        static bool validCommandName(std::string cmdname);
        static enum NodeControlCommand nodeControlCommand(std::string st);
        bool nodeControl(std::string cmd);
        void nodeControl(enum NodeControlCommand cmd);
        Message nodeGuard(void);
        bool handleMessage(const Message m);
        static std::string validState(enum NodeState st);
        static std::string nodeStateName(enum NodeState st);
        static bool validStateName(std::string st);
        static enum NodeState nodeState(std::string st);

        enum NodeState node_state;

    private:
        NMTCallbackObject callback;

};

}
#endif // __NMT_H__


