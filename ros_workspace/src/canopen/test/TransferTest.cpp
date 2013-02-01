#include <stdio.h>
#include <poll.h>
#include <iostream>
#include <boost/program_options.hpp>
#include <tr1/memory>
#include <list>
#include <vector>
#include <map>

#include <canlib.h>

#include <Interface.h>
#include <KvaserInterface.h>
#include <Message.h>
#include <Transfer.h>
#include <Bus.h>
#include <Callbacks.h>
#include <SDO.h>
#include <NMT.h>
#include <PDO.h>
#include <EMCY.h>
#include <DS301.h>

#define POLL_TIMEOUT_MS 1000

namespace po = boost::program_options;
using namespace CANOpen;

enum ToolOperation {
    ReadObjectDictionary,
    WriteObjectDictionary,
    SendNMT,
    CreateTPDO,
};

class Stopper : public TransferCallbackReceiver {
    public:
    Stopper():run(true){};
    void stop(SDO &sdo)
    {
        run = false;
        if(sdo.isError()) {
            if(sdo.abortCode() == 0) {
                std::cout << "Error during SDO transfer" << std::endl;
            } else {
                std::cout << "SDO Abort: " << sdo.abortString() << std::endl;
            }
        } else {
            std::vector<uint8_t> data = sdo.data;
            std::cout << "Received (" << std::dec << (int)data.size() << "): ";
            for(std::vector<uint8_t>::iterator byte = data.begin(); byte != data.end(); byte++){
                std::cout << std::hex << "0x" << (int)*byte << " ";
            }
            std::cout << std::dec << std::endl;
        }
        
    };
    void stop(DS301 &node){run = false;};
    void display(PDO &pdo)
    {
        std::cout << "PDO " << pdo.getName() << " received ";
        for(std::vector<uint8_t>::iterator it=pdo.data.begin();
                it != pdo.data.end();
                it++) {
            std::cout << std::hex << (int)*it << std::dec << " ";
        }
        std::cout << std::endl;
        run = false;
    };

    bool run;
};

Stopper stopit;

int main(int argc, char * argv[])
{
    po::options_description interface("Interface Options");

    interface.add_options()
        ("help", "This message")
        ("baud,b", po::value<int>()->default_value(1000000), "CAN baud rate")
        ("channel,c", po::value<int>()->default_value(0), "CAN device channel")
        ;

    po::options_description operation("Operation Options");
    operation.add_options()
        ("operation,o", po::value<std::string>()->default_value("read"),
         "read/write/NMT/CTPDO")
        ("keep-going,k", po::bool_switch()->default_value(false))
        ;

    po::options_description hidden("Hidden options");
    hidden.add_options()
        ("device_id", po::value<std::string>(), "node ID")
        ("index", po::value<std::string>(), "SDO index")
        ("subindex", po::value<std::string>(), "SDO subindex")
        ("data", po::value<std::vector<std::string> >(), "Data to write")
        ;

    po::positional_options_description pd;
    pd.add("device_id", 1);
    pd.add("index", 1);
    pd.add("subindex", 1);
    pd.add("data", -1);

    po::options_description cmdline_options;
    cmdline_options.add(interface).add(operation).add(hidden);

    po::options_description visible("Allowed options");
    visible.add(interface).add(operation);
        
    po::variables_map vm;
    po::store(po::command_line_parser(argc, argv).
            options(cmdline_options).positional(pd).run(), vm);
    po::notify(vm);

    if (vm.count("help")) {
        std::cout << visible << "\n";
        return 0;
    }

    enum ToolOperation op;
    std::string opstr = vm["operation"].as<std::string>();
    if(opstr.compare("write") == 0){
        op = WriteObjectDictionary;
    } else if(opstr.compare("read") == 0){
        op = ReadObjectDictionary;
    } else if(opstr.compare("NMT") == 0){
        op = SendNMT;
    } else if(opstr.compare("CTPDO") == 0){
        op = CreateTPDO;
    } else {
        std::cout << "Unknown operation: " << opstr << std::endl;
        return 1;
    }


    std::stringstream fromdec;
    std::stringstream fromhex;
    fromhex << std::hex;
    int device_id;
    fromhex << vm["device_id"].as<std::string>();
    fromhex >> device_id;
    fromhex.flush();
    fromhex.clear();
    std::cout << "node:" << std::hex << device_id << std::endl;
    std::cout << std::dec;

    int Index;
    int Subindex;
    std::vector<uint8_t> sdo_data;
    enum NodeControlCommand cmd;
    int pdo_number;
    const char *cmdname;
    switch(op){
        case ReadObjectDictionary:
        case WriteObjectDictionary:
            fromhex << vm["index"].as<std::string>();
            fromhex >> Index;
            fromhex.flush();
            fromhex.clear();
            std::cout << " index:" << std::hex << Index;
            std::cout << std::dec;

            fromhex << vm["subindex"].as<std::string>();
            fromhex >> Subindex;
            fromhex.flush();
            fromhex.clear();
            std::cout << " subindex:" << std::hex << Subindex << std::endl;
            std::cout << std::dec;
            break;
        case SendNMT:
            cmdname = vm["index"].as<std::string>().c_str();
            if(!NMT::validCommandName(cmdname)) {
                std::cout << "Invalid command: '" << cmdname << "'" << std::endl;
                NMT::printCommands();
                return 2;
            } else {
                cmd = NMT::nodeControlCommand(cmdname);
                std::cout << "Sending NMT " << cmdname << "(" << cmd << ")" <<
                    std::endl;
            }
            break;
        case CreateTPDO:
            fromdec << vm["index"].as<std::string>();
            fromdec >> pdo_number;
            std::cout << "Create TPDO " << pdo_number << std::endl;
            break;
    }

    std::tr1::shared_ptr<KvaserInterface> pintf(new KvaserInterface(
                                                vm["channel"].as< int >(),
                                                vm["baud"].as< int >()
                                               ));

    int canfd = pintf->getFD();
    std::cout << "can fd: " << canfd << std::endl;

    std::tr1::shared_ptr<Bus> pbus(new Bus(pintf, true));

    DS301 node(device_id, pbus);

    std::tr1::shared_ptr<SDOCallbackObject> pstopc(new
                    SDOCallbackObject(static_cast<TransferCallbackReceiver *>(&stopit),
                                      static_cast<SDOCallbackObject::CallbackFunction>(&Stopper::stop),
                                      static_cast<SDOCallbackObject::CallbackFunction>(&Stopper::stop)));

    std::tr1::shared_ptr<PDO> ptpdo;

    std::vector<std::string> option;
    switch(op) {
        case WriteObjectDictionary:
            option = vm["data"].as<std::vector<std::string> >();
            int value;
            std::cout << "writing: ";
            for(std::vector<std::string>::iterator it=option.begin(); it<option.end(); it++) {
                fromhex << std::hex << *it;
                fromhex >> value;
                fromhex.flush();
                fromhex.clear();
                std::cout << std::hex << value << " ";
                sdo_data.push_back(static_cast<uint8_t>(value));
            }
            std::cout << std::endl;
            std::cout << std::dec;
            node.writeObjectDictionary(Index, Subindex, sdo_data, pstopc);
            break;
        case ReadObjectDictionary:
            std::cout << "reading..." << std::endl;
            node.readObjectDictionary(Index, Subindex, pstopc);
            break; 
        case SendNMT:
            node.sendNMT(cmd,
                    DS301CallbackObject(static_cast<TransferCallbackReceiver *>(&stopit),
                                        static_cast<DS301CallbackObject::CallbackFunction>(&Stopper::stop)));
            break;
        case CreateTPDO:
            ptpdo = node.createTPDO("Test PDO", pdo_number,
                    PDOCallbackObject(static_cast<TransferCallbackReceiver *>(&stopit),
                        static_cast<PDOCallbackObject::CallbackFunction>(&Stopper::display))
                        );
            std::cout << "PDO COBID: 0x" << std::hex << ptpdo->COBID() << std::dec << std::endl;
            break;
    }

    struct pollfd pfd[2];
    pfd[0].fd = canfd;
    pfd[0].events = POLLIN | POLLOUT;
    pfd[1].fd = STDIN_FILENO;
    pfd[1].events = POLLIN;
    int ret;
    bool keepgoing=vm["keep-going"].as<bool>();
    while(ret >= 0 && (stopit.run || keepgoing)){
        ret = poll(pfd, 2, POLL_TIMEOUT_MS);
        if(ret>0){
            for(int i=0;i<2;i++) {
                if(pfd[i].revents) {
                    if(pfd[i].fd == canfd) {
                        bool canRead = pfd[i].revents&POLLIN;
                        bool canWrite = pfd[i].revents&POLLOUT;
                        pbus->runonce(canRead, canWrite);
                        pfd[i].events |= POLLIN;
                        if(canWrite) pfd[i].events |= POLLOUT;
                        else pfd[i].events &= ~POLLOUT;
                    } else if((pfd[i].fd == STDIN_FILENO) &&
                            (pfd[i].revents==POLLIN)) {
                        char c;
                        read(STDIN_FILENO, &c, 1);
                        if(c == 'q' || 'c' == 'Q') {
                            keepgoing=false;
                            stopit.run=false;
                        }
                    }
                }
            }
        }
    }

    if(ret<0) {
        perror("Poll error");
    }

    return 0;
}
