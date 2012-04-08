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
    sync(std::tr1::shared_ptr<SYNC>(new SYNC(node_id,
         SYNCCallbackObject(static_cast<TransferCallbackReceiver *>(this),
                            static_cast<SYNCCallbackObject::CallbackFunction>(&CopleyServo::syncCallback))))),
    syncProducer(false),
    allReady(false),
    gotPV(false),
    mapsCreated(false)
{
    bus->add(sync);
}

CopleyServo::CopleyServo(long int node_id, int sync_interval, std::tr1::shared_ptr<Bus> bus) :
    DS301(node_id,
          bus,
          false),
    sync(std::tr1::shared_ptr<SYNC>(new SYNC(node_id,
         SYNCCallbackObject(static_cast<TransferCallbackReceiver *>(this),
                            static_cast<SYNCCallbackObject::CallbackFunction>(&CopleyServo::syncCallback))))),
    syncProducer(true),
    syncInterval(sync_interval),
    allReady(false),
    gotPV(false),
    mapsCreated(false)
{
    bus->add(sync);
}


void CopleyServo::initialize(void)
{
    sendNMT(ResetNode,
            DS301CallbackObject(static_cast<TransferCallbackReceiver *>(this),
                static_cast<DS301CallbackObject::CallbackFunction>(&CopleyServo::_initialize))
           );

}

void CopleyServo::_initialize(DS301 &node)
{
    std::cout << "Initialize" << std::endl;
    allReady = false;
    gotPV = false;

    _mapPDOs();

    std::cout << "Read control word" << std::endl;
    // Read initial control word
    readObjectDictionary(0x6040, 0, std::tr1::shared_ptr<SDOCallbackObject>(
                new SDOCallbackObject(static_cast<TransferCallbackReceiver *>(this),
                    static_cast<SDOCallbackObject::CallbackFunction>(&CopleyServo::_initialControlWord))));

    if(syncProducer) {
        setSyncInterval(syncInterval);
        enableSync(true);
    }
}

void CopleyServo::_mapPDOs(void)
{
    std::vector<struct PDOMap> maps;
    struct PDOMap map;

    std::cout << "Map PDOs" << std::endl;
    mapsCreated = false;

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

    mapsCreated = true;
}


void CopleyServo::_initialControlWord(SDO &sdo)
{
    control_word =  sdo.data[0] | (sdo.data[1]<<8);
    std::cout << "Control: " << std::hex << control_word << std::dec << std::endl;
    sendNMT(StartRemoteNode,
            DS301CallbackObject(static_cast<TransferCallbackReceiver *>(this),
            static_cast<DS301CallbackObject::CallbackFunction>(&CopleyServo::_initialize))
           );
}

void CopleyServo::statusModePDOCallback(PDO &pdo)
{
    uint16_t new_status_word = pdo.data[0] | (pdo.data[1]<<8);
    enum OperationMode new_mode_of_operation = CopleyServo::operationMode(pdo.data[2]);

    uint16_t rising_status = new_status_word & (~status_word);

    switch(new_mode_of_operation) {
        case ProfilePosition:
            if(rising_status&STATUS_TARGET_REACHED) {
                position_callback(*this);
            }
            break;
        case ProfileVelocity:
            break;
        case ProfileTorque:
            break;
        case Homing:
            if(rising_status&STATUS_HOMING_ATTAINED) {
                home_callback(*this);
            }
            break;
        case InterpolatedPosition:
            break;
        case CyclicSynchonousPosition:
            break;
        case CyclicSynchronousVelocity:
            break;
        case CyclicSynchronousTorque:
            break;
        default:
            break;
 
    }

    status_word = new_status_word;
    mode_of_operation = new_mode_of_operation;
    //_printStatusAndMode();
}

void CopleyServo::_printStatusAndMode(void)
{
    std::cout << "Status: " <<
        ((status_word&STATUS_READY)?"Ready to Switch On, ":"") <<
        ((status_word&STATUS_SWITCHED_ON)?"Switched On, ":"") <<
        ((status_word&STATUS_OPERATION_ENABLED)?"Operation Enabled, ":"") <<
        ((status_word&STATUS_FAULT)?"Fault, ":"") <<
        ((status_word&STATUS_VOLTAGE_ENABLED)?"Voltage Enabled, ":"") <<
        ((status_word&STATUS_QUICK_STOP)?"":"Quick Stop, ") <<
        ((status_word&STATUS_SWITCH_ON_DISABLED)?"Switch On Disabled, ":"") <<
        ((status_word&STATUS_WARNING)?"Warning, ":"") <<
        ((status_word&STATUS_TRAJECTORY_ABORT)?"Trajectory Abort, ":"") <<
        ((status_word&STATUS_REMOTE)?"Remote, ":"") <<
        ((status_word&STATUS_TARGET_REACHED)?"Target Reached, ":"") <<
        ((status_word&STATUS_INTERNAL_LIMIT)?"Internal Limit, ":"");
     switch(mode_of_operation) {
        case ProfilePosition:
            std::cout << ((status_word&STATUS_SETPOINT_ACK)?"Setpoint Ack, ":"") <<
                         ((status_word&STATUS_FOLLOWING_ERROR)?"Following Error, ":"");
            break;
        case ProfileVelocity:
            std::cout << ((status_word&STATUS_SPEED_ZERO)?"Speed==0, ":"") <<
                         ((status_word&STATUS_MAXIMUM_SLIPPAGE)?"Maximum Slippage Error, ":"");
            break;
        case ProfileTorque:
            std::cout << ((status_word&STATUS_RESERVED_PT12)?"Reserved (PT12), ":"") <<
                         ((status_word&STATUS_RESERVED_PT13)?"Reserved (PT13), ":"");
            break;
        case Homing:
            std::cout << ((status_word&STATUS_HOMING_ATTAINED)?"Homing Attained, ":"") <<
                         ((status_word&STATUS_HOMING_ERROR)?"Homing Error, ":"");
            break;
        case InterpolatedPosition:
            std::cout << ((status_word&STATUS_INTERPOLATED_POSITION)?"Interpolated Position Active, ":"") <<
                         ((status_word&STATUS_RESERVED_IP13)?"Reserved (IP13), ":"");
            break;
        case CyclicSynchonousPosition:
            // Cyclic Synchronous Position
            break;
        case CyclicSynchronousVelocity:
            // Cyclic Synchronous Velocity
            break;
        case CyclicSynchronousTorque:
            // Cyclic Synchronous Torque
            break;
        default:
            break;
    }
    std::cout << ((status_word&STATUS_PERFORMING_MOVE)?"Performing Move, ":"") <<
                 ((status_word&STATUS_RESERVED_15)?"Reserved (15)":"") <<
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

    gotPV = true;
    pv_callback(*this);
    //std::cout << "Position: " << position << " Velocity: " << velocity << std::endl;
}

/*
   bool CopleyServo::getPV(uint32_t &P, uint32_t &V, bool clear)
{
    bool gotPV_ = gotPV;
    P = position;
    V = velocity;
    if(clear) gotPV=false;
    return gotPV_;
}
*/

void CopleyServo::syncCallback(SYNC &sync)
{
    if(!mapsCreated) {
        return;
    }
    if(!allReady) {
        std::cout <<
            "status_mode: " << status_mode_pdo->mapped <<
            " position_velocity: " << position_velocity_pdo->mapped <<
            " control_mode: " << control_mode_pdo->mapped <<
            " position: " << position_pdo->mapped <<
            " velocity: " << velocity_pdo->mapped <<
            " PV: " << gotPV << std::endl;
    }
    if( status_mode_pdo->mapped &&
            position_velocity_pdo->mapped &&
            control_mode_pdo->mapped &&
            position_pdo->mapped &&
            velocity_pdo->mapped &&
            gotPV &&
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

void CopleyServo::home(DS301CallbackObject callback)
{
    home_callback = callback;
    modeControl(0, CONTROL_NEW_SETPOINT, Homing); 
    control( CONTROL_NEW_SETPOINT| CONTROL_CHANGE_SET_IMMEDIATE, 0);
}

void CopleyServo::setVelocity(int32_t v)
{
    modeControl(0, 0, ProfileVelocity);
    velocity_pdo->send(v);
}

void CopleyServo::setPosition(int32_t p, DS301CallbackObject callback)
{
    position_callback = callback;
    modeControl(0, CONTROL_NEW_SETPOINT | CONTROL_CHANGE_SET_IMMEDIATE, ProfilePosition);
    position_pdo->send(p,
            PDOCallbackObject(static_cast<TransferCallbackReceiver *>(this),
                              static_cast<PDOCallbackObject::CallbackFunction>(&CopleyServo::_positionGo)));
}

void CopleyServo::_positionGo(PDO &pdo)
{
    modeControl(CONTROL_NEW_SETPOINT | CONTROL_CHANGE_SET_IMMEDIATE, 0, ProfilePosition);
}

void CopleyServo::pvCallback(DS301CallbackObject cb)
{
    pv_callback = cb;
}

}
