#include <poll.h>
#include <iostream>
#include <vector>
#include <list>
#include <stdint.h>
#include <map>
#include <tr1/memory>

#include <boost/program_options.hpp>

#include <canlib.h>

#include <Message.h>
#include <Transfer.h>
#include <Interface.h>
#include <Bus.h>
#include <Callbacks.h>
#include <SDO.h>
#include <PDO.h>
#include <EMCY.h>
#include <NMT.h>
#include <SYNC.h>
#include <DS301.h>
#include <CopleyServo.h>

#include <KvaserInterface.h>

namespace po = boost::program_options;
using namespace CANOpen;

#define POLL_TIMEOUT_MS 1000

int main(int argc, char * argv[])
{
    po::options_description interface("Interface Options");

    interface.add_options()
        ("help", "This message")
        ("baud,b", po::value<int>()->default_value(1000000), "CAN baud rate")
        ("channel,c", po::value<int>()->default_value(0), "CAN device channel")
        ;

    po::options_description hidden("Hidden options");
    hidden.add_options()
        ("device_id", po::value<std::string>(), "node ID")
        ;

    po::positional_options_description pd;
    pd.add("device_id", 1);

    po::options_description cmdline_options;
    cmdline_options.add(interface).add(hidden);

    po::options_description visible("Allowed options");
    visible.add(interface);
        
    po::variables_map vm;
    po::store(po::command_line_parser(argc, argv).
            options(cmdline_options).positional(pd).run(), vm);
    po::notify(vm);

    if (vm.count("help")) {
        std::cout << visible << "\n";
        return 0;
    }

    std::stringstream fromdec;
    int device_id;
    fromdec << vm["device_id"].as<std::string>();
    fromdec >> device_id;
    fromdec.flush();
    fromdec.clear();
    std::cout << "node:" << device_id << std::endl;

    std::tr1::shared_ptr<KvaserInterface> pintf(new KvaserInterface(
                                                vm["channel"].as< int >(),
                                                vm["baud"].as< int >()
                                               ));

    int canfd = pintf->getFD();
    std::cout << "can fd: " << canfd << std::endl;

    std::tr1::shared_ptr<Bus> pbus(new Bus(pintf, true));

    CopleyServo servo(device_id, pbus);

    struct pollfd pfd[2];
    pfd[0].fd = canfd;
    pfd[0].events = POLLIN | POLLOUT;
    pfd[1].fd = STDIN_FILENO;
    pfd[1].events = POLLIN;
    int ret;
    bool keepgoing=true;
    while(ret >= 0 && keepgoing){
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
