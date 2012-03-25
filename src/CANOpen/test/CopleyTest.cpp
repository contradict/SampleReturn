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

enum ToolOperation {
    Monitor,
    Home,
    EnableSync,
    DisableSync,
    Enable,
    SetVelocity,
    SetPosition
};


int main(int argc, char * argv[])
{
    std::string visible_descr = std::string(argv[0]);
    visible_descr += " [options] device_id ";
    po::options_description visible(visible_descr);
    po::variables_map vm;
    try {
        po::options_description interface("Interface Options");
        interface.add_options()
            ("help,h", "This message")
            ("baud,b", po::value<int>()->default_value(1000000), "CAN baud rate")
            ("channel,c", po::value<int>()->default_value(0), "CAN device channel")
            ;

        po::options_description operation("Operation Options");
        operation.add_options()
            ("operation,o", po::value<std::string>()->default_value("monitor"),
             "monitor|home|sync|nosync|enable|velocity|postion")
            ("sync-interval,s", po::value<float>()->default_value(0.0),
             "floating-point seconds")
            ("velocity,v", po::value<int32_t>()->default_value(0),
             "0.1 counts/sec")
            ("position,p", po::value<int32_t>()->default_value(0),
             "counts")
            ("keep-going,k", po::bool_switch()->default_value(false))
            ;

        po::options_description hidden("Hidden options");
        hidden.add_options()
            ("device_id", po::value<std::string>()->required(), "node ID")
            ;

        po::positional_options_description pd;
        pd.add("device_id", -1);

        po::options_description cmdline_options;
        cmdline_options.add(interface).add(operation).add(hidden);

        visible.add(interface).add(operation);
     
        po::store(po::command_line_parser(argc, argv).
                options(cmdline_options).positional(pd).run(), vm);

        if (vm.count("help")) {
            std::cout << visible << "\n";
            return 0;
        }

        po::notify(vm);
    } catch(po::required_option) {
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

    enum ToolOperation op;
    std::string opstr = vm["operation"].as<std::string>();
    if(opstr.compare("monitor") == 0){
        op = Monitor;
    } else if(opstr.compare("home") == 0){
        op = Home;
    } else if(opstr.compare("sync") == 0){
        op = EnableSync;
    } else if(opstr.compare("nosync") == 0){
        op = DisableSync;
    } else if(opstr.compare("enable") == 0){
        op = Enable;
    } else if(opstr.compare("velocity") == 0){
        op = SetVelocity;
    } else if(opstr.compare("position") == 0){
        op = SetPosition;
    } else {
        std::cout << "Unknown operation: " << opstr << std::endl;
        return 1;
    }

    std::tr1::shared_ptr<KvaserInterface> pintf(new KvaserInterface(
                                                vm["channel"].as< int >(),
                                                vm["baud"].as< int >()
                                               ));

    int canfd = pintf->getFD();
    std::cout << "can fd: " << canfd << std::endl;

    std::tr1::shared_ptr<Bus> pbus(new Bus(pintf, true));

    CopleyServo servo(device_id, pbus);

    float sync_interval_sec;
    uint32_t sync_interval_usec;
    switch(op){
        case EnableSync:
            sync_interval_sec = vm["sync-interval"].as<float>();
            sync_interval_usec = (uint32_t)round(sync_interval_sec*1e6);
            if(sync_interval_usec>0) {
                std::cout << "set sync interval: " << sync_interval_usec << std::endl;
                servo.setSyncInterval(sync_interval_usec);
            }
            std::cout << "Enable Sync" << std::endl;
            servo.enableSync(true);
            break;
        case DisableSync:
            servo.enableSync(false);
            break;
        default:
            break;
    }
    struct pollfd pfd[2];
    pfd[0].fd = canfd;
    pfd[0].events = POLLIN | POLLOUT;
    pfd[1].fd = STDIN_FILENO;
    pfd[1].events = POLLIN;
    int ret;
    bool keepgoing=true;
    bool opdone=false;
    int32_t velocity, position;
    while(ret >= 0 && keepgoing ){
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
                        if( c == 'q' ) {
                            keepgoing=false;
                        }
                    }
                }
            }
        }
        if(servo.ready() && !opdone ) {
            switch(op){
                case Monitor:
                    break;
                case Home:
                    servo.home();
                    break;
                case EnableSync:
                case DisableSync:
                    break;
                case Enable:
                    servo.enable();
                    break;
                 case SetVelocity:
                    velocity = vm["velocity"].as<int32_t>();
                    servo.setVelocity(velocity);
                    break;
                 case SetPosition:
                    position = vm["position"].as<int32_t>();
                    servo.setPosition(position);
                    break;
             }
            opdone=true;
        }
    }
    fflush(stdin);
    if(ret<0) {
        perror("Poll error");
    }

    return 0;
}
