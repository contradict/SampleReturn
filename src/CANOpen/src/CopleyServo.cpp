#include <iostream>
#include <vector>
#include <forward_list>
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
#include <invertmap.h>

#include <CopleyServo.h>

namespace CANOpen {

static std::map<std::string, enum OperationMode> create_OperationNameMode(void)
{
    std::map<std::string, enum OperationMode> m;
    m["ProfilePosition"]          = ProfilePosition;
    m["ProfileVelocity"]          = ProfileVelocity;
    m["ProfileTorque"]            = ProfileTorque;
    m["Homing"]                   = Homing;
    m["InterpolatedPosition"]     = InterpolatedPosition;
    m["CyclicSynchonousPosition"] = CyclicSynchonousPosition;
    m["CyclicSynchronousVelocity"]= CyclicSynchronousVelocity;
    m["CyclicSynchronousTorque"]  = CyclicSynchronousTorque;
    return m;
};
std::map<std::string, enum OperationMode> OperationNameMode =
create_OperationNameMode();
std::map<enum OperationMode, std::string> OperationModeName =
InvertMap(OperationNameMode);


CopleyServo::CopleyServo(long int node_id, std::tr1::shared_ptr<Bus> bus) :
    DS301(node_id,
          bus,
          false),
    allReady(false)
{
    sync = std::tr1::shared_ptr<SYNC>(new SYNC(node_id,
         SYNCCallbackObject(static_cast<TransferCallbackReceiver *>(this),
                            static_cast<SYNCCallbackObject::CallbackFunction>(&CopleyServo::syncCallback))));
    bus->add(sync);

    std::vector<struct PDOMap> maps;
    struct PDOMap map;
    // Status Word
    map.index = 0x6041;
    map.subindex = 0;
    map.bits = 16;
    maps.push_back(map);
    // Mode of Operation
    map.index = 0x6060;
    map.subindex = 0;
    map.bits = 8;
    maps.push_back(map);
    // Immediate event
    status_mode_pdo = mapTPDO("Status/Mode", 1, maps, 0xff,
            PDOCallbackObject(static_cast<TransferCallbackReceiver *>(this),
                static_cast<PDOCallbackObject::CallbackFunction>(&CopleyServo::statusModePDOCallback)));
    bus->add(status_mode_pdo);

    maps.clear();
    // Position Actual Value
    map.index = 0x6064;
    map.subindex = 0;
    map.bits = 32;
    maps.push_back(map);
    // Actual Velocity
    map.index = 0x6069;
    map.subindex = 0;
    map.bits = 32;
    maps.push_back(map);
    // Every 1 sync messages
    position_velocity_pdo = mapTPDO("Position/Velocity", 2, maps, 0x01,
            PDOCallbackObject(static_cast<TransferCallbackReceiver *>(this),
                static_cast<PDOCallbackObject::CallbackFunction>(&CopleyServo::positionVelocityPDOCallback)));
    bus->add(position_velocity_pdo);

    // Disable sending of TPDO3-4
    std::vector<uint8_t> data;
    data.push_back(0);
    writeObjectDictionary(0x1802, 2, data);
    writeObjectDictionary(0x1803, 2, data);

    // Control Word
    maps.clear();
    map.index = 0x6040;
    map.subindex = 0;
    map.bits = 16;
    maps.push_back(map);
    // Mode of Operation
    map.index = 0x6060;
    map.subindex = 0;
    map.bits = 8;
    maps.push_back(map);
    // Immediate action
    control_mode_pdo = mapRPDO("Control Word/Mode of Operation", 1, maps, 0xFF);
    bus->add(control_mode_pdo);

    // Position Commanded Value
    maps.clear();
    map.index = 0x607A;
    map.subindex = 0;
    map.bits = 32;
    maps.push_back(map);
    // Commit on sync
    position_pdo = mapRPDO("Position Commanded Value", 2, maps, 0xFF);
    bus->add(position_pdo);

    // Target Velocity
    maps.clear();
    map.index = 0x60FF;
    map.subindex = 0x0;
    map.bits = 32;
    maps.push_back(map);
    // Commit on sync
    velocity_pdo = mapRPDO("Target Velocity", 3, maps, 0);
    bus->add(velocity_pdo);

    // Read initial control word
    readObjectDictionary(0x6040, 0, std::tr1::shared_ptr<SDOCallbackObject>(
                new SDOCallbackObject(static_cast<TransferCallbackReceiver *>(this),
                    static_cast<SDOCallbackObject::CallbackFunction>(&CopleyServo::_initialControlWord))));
}


void CopleyServo::_initialControlWord(SDO &sdo)
{
    control_word =  sdo.data[0] | (sdo.data[1]<<8);
    std::cout << "Control: " << std::hex << control_word << std::dec << std::endl;
    sendNMT(StartRemoteNode);
}

void CopleyServo::statusModePDOCallback(PDO &pdo)
{
    status_word = pdo.data[0] | (pdo.data[1]<<8);
    mode_of_operation = CopleyServo::operationMode(pdo.data[2]);
    std::cout << "Status: " <<
        ((status_word&0x0001)?"Ready to Switch On, ":"") <<
        ((status_word&0x0002)?"Switched On, ":"") <<
        ((status_word&0x0004)?"Operation Enabled, ":"") <<
        ((status_word&0x0008)?"Fault, ":"") <<
        ((status_word&0x0010)?"Voltage Enabled, ":"") <<
        ((status_word&0x0020)?"":"Quick Stop, ") <<
        ((status_word&0x0040)?"Switch On Disabled, ":"") <<
        ((status_word&0x0080)?"Warning, ":"") <<
        ((status_word&0x0100)?"Trajectory Abort, ":"") <<
        ((status_word&0x0200)?"Remote, ":"") <<
        ((status_word&0x0400)?"Target Reached, ":"") <<
        ((status_word&0x0800)?"Internal Limit, ":"");
    switch(mode_of_operation) {
        case ProfilePosition:
            std::cout << ((status_word&0x1000)?"Setpoint Ack, ":"") <<
                         ((status_word&0x2000)?"Following Error, ":"");
            break;
        case ProfileVelocity:
            std::cout << ((status_word&0x1000)?"Speed==0, ":"") <<
                         ((status_word&0x2000)?"Maximum Slippage Error, ":"");
            break;
        case ProfileTorque:
            std::cout << ((status_word&0x1000)?"Reserved (PT12), ":"") <<
                         ((status_word&0x2000)?"Reserved (PT13), ":"");
            break;
        case Homing:
            std::cout << ((status_word&0x1000)?"Homing Attained, ":"") <<
                         ((status_word&0x2000)?"Homing Error, ":"");
            break;
        case InterpolatedPosition:
            std::cout << ((status_word&0x1000)?"Interpolated Position Active, ":"") <<
                         ((status_word&0x2000)?"Reserved (IP13), ":"");
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
    std::cout << ((status_word&0x4000)?"Performing Move, ":"") <<
                 ((status_word&0x8000)?"Reserved (15)":"") <<
                 std::endl;

    std::cout << "Mode of Operation: " << operationModeName(mode_of_operation) << std::endl;
}

void CopleyServo::positionVelocityPDOCallback(PDO &pdo)
{
    position = (int32_t)(((uint32_t)pdo.data[0]) |
                         ((uint32_t)pdo.data[1]<<8) |
                         ((uint32_t)pdo.data[2]<<16) |
                         ((uint32_t)pdo.data[3]<<24));

    velocity = (int32_t)(((uint32_t)pdo.data[4]) |
                         ((uint32_t)pdo.data[5]<<8) |
                         ((uint32_t)pdo.data[6]<<16) |
                         ((uint32_t)pdo.data[7]<<24));

    std::cout << "Position: " << position << " Velocity: " << velocity << std::endl;
}

void CopleyServo::syncCallback(SYNC &sync)
{
    if(!allReady) {
        std::cout <<
            "status_mode: " << status_mode_pdo->mapped <<
            " position_velocity: " << position_velocity_pdo->mapped <<
            " control_mode: " << control_mode_pdo->mapped <<
            " position: " << position_pdo->mapped <<
            " velocity: " << velocity_pdo->mapped << std::endl;
    }
    if( status_mode_pdo->mapped &&
            position_velocity_pdo->mapped &&
            control_mode_pdo->mapped &&
            position_pdo->mapped &&
            velocity_pdo->mapped &&
            !allReady) {
        allReady=true;
    }
}

bool CopleyServo::ready(void)
{
    return allReady;
}

enum OperationMode CopleyServo::operationMode(uint8_t m)
{
    switch(m) {
        case 1:
            return ProfilePosition;
        case 3:
            return ProfileVelocity;
        case 4:
            return ProfileTorque;
        case 6:
            return Homing;
        case 7:
            return InterpolatedPosition;
        case 8:
            return CyclicSynchonousPosition;
        case 9:
            return CyclicSynchronousVelocity;
        case 10:
            return CyclicSynchronousTorque;
        default:
            return UnknownMode;
    }
}

enum OperationMode CopleyServo::operationMode(std::string m)
{
    if(OperationNameMode.count(m)>0) {
        return OperationNameMode[m];
    } else {
        return UnknownMode;
    }
}

std::string CopleyServo::operationModeName(enum OperationMode m)
{
    if(OperationModeName.count(m)>0) {
        return OperationModeName[m];
    } else {
        return "UnknownMode";
    }
}

void CopleyServo::modeControl(uint16_t set, uint16_t clear,enum OperationMode
        mode, PDOCallbackObject callback)
{
    control_word |= set;
    control_word &= ~clear;
    mode_of_operation = mode;
    std::vector<uint8_t> data;
    data.resize(3);
    *((uint16_t *)(&data[0])) = htole16(control_word);
    data[2] = mode_of_operation;
    control_mode_pdo->send(data, callback);
}

void CopleyServo::control(uint16_t set, uint16_t clear)
{
    modeControl(set, clear, mode_of_operation);
}

void CopleyServo::mode(enum OperationMode mode)
{
    modeControl(0, 0, mode);
}


void CopleyServo::enable(void)
{
    control(
            CONTROL_SWITCH_ON | CONTROL_ENABLE_VOLTAGE | CONTROL_ENABLE_OPERATION,
            CONTROL_NEW_SETPOINT
           );
}

void CopleyServo::home(void)
{
    modeControl(0, CONTROL_NEW_SETPOINT, Homing);
    control( CONTROL_NEW_SETPOINT| CONTROL_CHANGE_SET_IMMEDIATE, 0);
}

void CopleyServo::setVelocity(int32_t v)
{
    modeControl(0, 0, ProfileVelocity);
    velocity_pdo->send(v);
}

void CopleyServo::setPosition(int32_t p)
{
    modeControl(0, CONTROL_NEW_SETPOINT | CONTROL_CHANGE_SET_IMMEDIATE, ProfilePosition);
    position_pdo->send(p,
            PDOCallbackObject(static_cast<TransferCallbackReceiver *>(this),
                              static_cast<PDOCallbackObject::CallbackFunction>(&CopleyServo::_positionGo)));
}

void CopleyServo::_positionGo(PDO &pdo)
{
    modeControl(CONTROL_NEW_SETPOINT | CONTROL_CHANGE_SET_IMMEDIATE, 0, ProfilePosition);
}

}
