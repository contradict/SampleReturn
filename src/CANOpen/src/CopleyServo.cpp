#include <iostream>
#include <vector>
#include <list>
#include <stdint.h>
#include <map>
#include <tr1/memory>

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

namespace CANOpen {
CopleyServo::CopleyServo(long int node_id, std::tr1::shared_ptr<Bus> bus) :
    DS301(node_id,
          bus,
          false),
    sync(node_id,
         SYNCCallbackObject(static_cast<TransferCallbackReceiver *>(this),
                            static_cast<SYNCCallbackObject::CallbackFunction>(&CopleyServo::syncCallback)))
{
   status_pdo = createTPDO("Status/Position", 3,
           PDOCallbackObject(static_cast<TransferCallbackReceiver *>(this),
                             static_cast<PDOCallbackObject::CallbackFunction>(&CopleyServo::statusPDOCallback)));
   setTPDOType(status_pdo, 2);
   control_pdo = createRPDO("Control/Mode of Operation", 2);

   // Read mode of operation
   readObjectDictionary(0x6061, 0, std::tr1::shared_ptr<SDOCallbackObject>(
           new SDOCallbackObject(static_cast<TransferCallbackReceiver *>(this),
                                 static_cast<SDOCallbackObject::CallbackFunction>(&CopleyServo::_initialModeOfOperation))));
   // Read control word
   readObjectDictionary(0x6040, 0, std::tr1::shared_ptr<SDOCallbackObject>(
           new SDOCallbackObject(static_cast<TransferCallbackReceiver *>(this),
                                 static_cast<SDOCallbackObject::CallbackFunction>(&CopleyServo::_initialControlWord))));
}

void CopleyServo::_initialModeOfOperation(SDO &sdo)
{
    mode_of_operation = sdo.data[0];
    std::cout << "Mode of Operation: " << (int)mode_of_operation << std::endl;
}

void CopleyServo::_initialControlWord(SDO &sdo)
{
    control_word =  sdo.data[0] | (sdo.data[1]<<8);
    std::cout << "Control: " << std::hex << control_word << std::dec << std::endl;
}

void CopleyServo::statusPDOCallback(PDO &pdo)
{
    status_word = pdo.data[0] | (pdo.data[1]<<8);
    std::cout << "Status: " <<
        ((status_word&0x0001)?"Ready to Switch On ":"") <<
        ((status_word&0x0002)?"Switched On ":"") <<
        ((status_word&0x0004)?"Operation Enabled ":"") <<
        ((status_word&0x0008)?"Fault ":"") <<
        ((status_word&0x0010)?"Voltage Enabled ":"") <<
        ((status_word&0x0020)?"":"Quick Stop ") <<
        ((status_word&0x0040)?"Switch On Disabled ":"") <<
        ((status_word&0x0080)?"Warning ":"") <<
        ((status_word&0x0100)?"Trajectory Abort ":"") <<
        ((status_word&0x0200)?"Remote ":"") <<
        ((status_word&0x0400)?"Target Reached ":"") <<
        ((status_word&0x0800)?"Internal Limit ":"");
    switch(mode_of_operation) {
        case 1:
            // Profile Position
            std::cout << ((status_word&0x1000)?"Setpoint Ack ":"") <<
                         ((status_word&0x2000)?"Following Error ":"");
            break;
        case 3:
            // Profile Velocity
            std::cout << ((status_word&0x1000)?"Speed==0 ":"") <<
                         ((status_word&0x2000)?"Maximum Slippage Error ":"");
            break;
        case 4:
            // Profile Torque
            std::cout << ((status_word&0x1000)?"Reserved (PT12) ":"") <<
                         ((status_word&0x2000)?"Reserved (PT13) ":"");
            break;
        case 6:
            // Homing
            std::cout << ((status_word&0x1000)?"Homing Attained ":"") <<
                         ((status_word&0x2000)?"Homing Error ":"");
            break;
        case 7:
            // interpolated position
            std::cout << ((status_word&0x1000)?"Interpolated Position Active ":"") <<
                         ((status_word&0x2000)?"Reserved (IP13) ":"");
            break;
        case 8:
            // Cyclic Synchronous Position
        case 9:
            // Cyclic Synchronous Velocity
        case 10:
            // Cyclic Synchronous Torque
            break;
        default:
            break;
    }
    std::cout << ((status_word&0x4000)?"Performing Move ":"") <<
                 ((status_word&0x8000)?"Reserved (15)":"") <<
                 std::endl;

    position = (int32_t)(((uint32_t)pdo.data[2]) |
                         ((uint32_t)pdo.data[3]<<8) |
                         ((uint32_t)pdo.data[4]<<16) |
                         ((uint32_t)pdo.data[5]<<24));

    std::cout << "Position: " << position << std::endl;
}

void CopleyServo::syncCallback(SYNC &sync)
{
}


}
