#include <poll.h>
#include <iostream>
#include <boost/program_options.hpp>

#include <canlib.h>

#include <Interface.h>
#include <KvaserInterface.h>
#include <Message.h>

namespace po = boost::program_options;
using namespace CANOpen;
int main(int argc, char * argv[])
{
    po::options_description desc("Options");

    desc.add_options()
        ("help", "This message")
        ("device,d", po::value<std::string>()->default_value("/dev/leaf0"), "CAN interface device")
        ("baud,b", po::value<int>()->default_value(1000000), "CAN baud rate")
        ("channel,c", po::value<int>()->default_value(0), "CAN device channel")
        ;

    po::variables_map vm;
    po::store(po::command_line_parser(argc, argv).
            options(desc).run(), vm);
    po::notify(vm);

    KvaserInterface intf = KvaserInterface(
                                           vm["channel"].as< int >(),
                                           vm["baud"].as< int >()
                                           );

    int fd = intf.getFD();
    std::cout << "can fd: " << fd << std::endl;
    Message m;
    struct pollfd pfd;
    int ret, rret;
    while(ret >= 0){
        pfd.fd = fd;
        pfd.events = POLLIN;
        ret = poll(&pfd, 1, 1000);
        if(ret>0){
            intf.readMessage(m);
            std::cout << m << std::endl;
        } else if(ret<0) {
            std::cout << ret << " exiting" << std::endl;
            break;
        }
    }

    return 0;
}
