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
         SYNCCallbackObject(
            static_cast<TransferCallbackReceiver *>(this),
            static_cast<SYNCCallbackObject::CallbackFunction>(
                 &CopleyServo::syncCallback
            )
         )))),
    emcy(std::tr1::shared_ptr<EMCY>(new EMCY(node_id,
         EMCYCallbackObject(static_cast<TransferCallbackReceiver *>(this),
                            static_cast<EMCYCallbackObject::CallbackFunction>(&CopleyServo::emcyCallback))))),
    syncProducer(false),
    allReady(false),
    gotPV(false),
    mapsCreated(false)
{
    bus->add(sync);
    bus->add(emcy);
}

CopleyServo::CopleyServo(long int node_id, int sync_interval, std::tr1::shared_ptr<Bus> bus) :
    DS301(node_id,
          bus,
          false),
    sync(std::tr1::shared_ptr<SYNC>(new SYNC(node_id,
         SYNCCallbackObject(static_cast<TransferCallbackReceiver *>(this),
                            static_cast<SYNCCallbackObject::CallbackFunction>(&CopleyServo::syncCallback))))),
    emcy(std::tr1::shared_ptr<EMCY>(new EMCY(node_id,
         EMCYCallbackObject(static_cast<TransferCallbackReceiver *>(this),
                            static_cast<EMCYCallbackObject::CallbackFunction>(&CopleyServo::emcyCallback))))),
    syncProducer(true),
    syncInterval(sync_interval),
    allReady(false),
    gotPV(false),
    mapsCreated(false)
{
    bus->add(sync);
    bus->add(emcy);
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
    // trajectory buffer status (for pvt mode)
    map.index = 0x2012;
    map.subindex = 0;
    map.bits = 32;
    maps.push_back(map);
    // Immediate event
    status_mode_pdo = mapTPDO("Status/Mode/PVT Buffer Status", 1, maps, 0xff,
            PDOCallbackObject(static_cast<TransferCallbackReceiver *>(this),
                static_cast<PDOCallbackObject::CallbackFunction>(&CopleyServo::statusModePDOCallback)));
    bus->add(status_mode_pdo);

    maps.clear();
    // Position Actual Value
    map.index = 0x6064;
    map.subindex = 0;
    map.bits = 32;
    maps.push_back(map);
    // Load Encoder Velocity
    map.index = 0x2231;
    map.subindex = 0;
    map.bits = 32;
    maps.push_back(map);
    // Every 1 sync messages
    position_velocity_pdo = mapTPDO("Position/Velocity", 2, maps, 0x01,
            PDOCallbackObject(static_cast<TransferCallbackReceiver *>(this),
                static_cast<PDOCallbackObject::CallbackFunction>(&CopleyServo::positionVelocityPDOCallback)));
    bus->add(position_velocity_pdo);

    maps.clear();
    // Bus Voltage
    map.index = 0x2201;
    map.subindex = 0;
    map.bits = 16;
    maps.push_back(map);
    // Input pins
    map.index = 0x2190;
    map.subindex = 0;
    map.bits = 16;
    maps.push_back(map);
    // Every 1 sync messages
    input_pdo = mapTPDO("Inputs", 3, maps, 0x01,
            PDOCallbackObject(static_cast<TransferCallbackReceiver *>(this),
                static_cast<PDOCallbackObject::CallbackFunction>(&CopleyServo::inputPDOCallback)));
    bus->add(input_pdo);

    // Disable sending of TPDO4
    std::vector<uint8_t> data;
    data.push_back(0);
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
    // outputs
    map.index = 0x2194;
    map.subindex = 0;
    map.bits = 16;
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

    // Target position, velocity and time
    maps.clear();
    map.index = 0x2010;
    map.subindex = 0x0;
    map.bits = 64;
    maps.push_back(map);
    // Commit on sync
    pvt_pdo = mapRPDO("Target Position, Velocity, & Time", 4, maps, 0);
    bus->add(pvt_pdo);

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
    readObjectDictionary(0x2194, 0, std::tr1::shared_ptr<SDOCallbackObject>(
                new SDOCallbackObject(static_cast<TransferCallbackReceiver *>(this),
                    static_cast<SDOCallbackObject::CallbackFunction>(&CopleyServo::_initialOutputPins))));

}

void CopleyServo::_initialOutputPins(SDO &sdo)
{
    output_pins = sdo.data[0] | (sdo.data[1]<<8);
    std::cout << "Initial output pins: " << std::hex << output_pins << std::dec
        << std::endl;
}

void CopleyServo::statusModePDOCallback(PDO &pdo)
{
    uint16_t new_status_word = pdo.data[0] | (pdo.data[1]<<8);
    enum OperationMode new_mode_of_operation = CopleyServo::operationMode(pdo.data[2]);

    uint16_t rising_status  = (~status_word) &   new_status_word;
    uint16_t falling_status =   status_word  & (~new_status_word);
    status_word = new_status_word;

    // Trajectory buffer status part of the pdo
    uint16_t nextSegmentExpected = pdo.data[3] | (pdo.data[4]<<8);
    uint8_t freeBufferSlots = pdo.data[5];
    uint8_t statusByte = pdo.data[6];

    // if there are now more free slots than we had before, we've consumed one
    bool segmentConsumed = (m_freeBufferSlots < freeBufferSlots);
    
    m_expectedSegmentId = nextSegmentExpected;
    m_freeBufferSlots = freeBufferSlots;

    uint8_t risingPvtStatus = (~m_pvtStatusByte) & statusByte;
    m_pvtStatusByte = statusByte;

    mode_of_operation = new_mode_of_operation;

    switch(mode_of_operation) {
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
            if(risingPvtStatus&PVT_GOT_ERROR)
            {
                handlePvtError(risingPvtStatus);
            }
            if(segmentConsumed)
            {
                // if we've consumed a segment, check to see if there
                // are any more segments waiting
                handlePvtSegmentConsumed();
            }
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

    if( (rising_status & STATUS_SWITCHED_ON) ||
            (falling_status & STATUS_SWITCHED_ON) ) {
        enable_callback(*this);
    }

    if( rising_status | falling_status)
    {
        status_callback(*this);
    }

    //_printStatusAndMode();
}

void CopleyServo::_printStatusAndMode()
{
    std::cout << "Status: ";
    writeStatus(std::cout);
    std::cout << std::endl;
    std::cout << "Mode of Operation: ";
    writeMode(std::cout);
    std::cout << std::endl;
}

void CopleyServo::writeMode(std::ostream &out)
{
    out << operationModeName(mode_of_operation);
}

void CopleyServo::writeStatus(std::ostream &out)
{
    out << "Status: " <<
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
            out << ((status_word&STATUS_SETPOINT_ACK)?"Setpoint Ack, ":"") <<
                         ((status_word&STATUS_FOLLOWING_ERROR)?"Following Error, ":"");
            break;
        case ProfileVelocity:
            out << ((status_word&STATUS_SPEED_ZERO)?"Speed==0, ":"") <<
                         ((status_word&STATUS_MAXIMUM_SLIPPAGE)?"Maximum Slippage Error, ":"");
            break;
        case ProfileTorque:
            out << ((status_word&STATUS_RESERVED_PT12)?"Reserved (PT12), ":"") <<
                         ((status_word&STATUS_RESERVED_PT13)?"Reserved (PT13), ":"");
            break;
        case Homing:
            out << ((status_word&STATUS_HOMING_ATTAINED)?"Homing Attained, ":"") <<
                         ((status_word&STATUS_HOMING_ERROR)?"Homing Error, ":"");
            break;
        case InterpolatedPosition:
            out << ((status_word&STATUS_INTERPOLATED_POSITION)?"Interpolated Position Active, ":"") <<
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
    out << ((status_word&STATUS_PERFORMING_MOVE)?"Performing Move, ":"") <<
                 ((status_word&STATUS_RESERVED_15)?"Reserved (15)":"");

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

void CopleyServo::inputPDOCallback(PDO &pdo)
{
    uint16_t new_bus_voltage = pdo.data[0] | (pdo.data[1]<<8);
    uint16_t new_input_pins = pdo.data[2] | (pdo.data[3]<<8);
    // bus volage register is in 0.1V steps
    bus_voltage = new_bus_voltage/10.0;

    if( (input_pins ^ new_input_pins) != 0) {
        input_callback(*this, input_pins, new_input_pins);
    }
    input_pins = new_input_pins;

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
        modeControl(
                CONTROL_SWITCH_ON|CONTROL_ENABLE_VOLTAGE|CONTROL_RESET_FAULT|CONTROL_QUICK_STOP,
                    CONTROL_ENABLE_OPERATION|CONTROL_HALT,
                    ProfilePosition
        );
    }
}

void CopleyServo::emcyCallback(EMCY &emcy)
{
    emcy_callback(*this);
    std::cout << "EMERGENCY: " << emcy.error_string() << std::endl;
}

void CopleyServo::setEMCYCallback(DS301CallbackObject cb)
{
    emcy_callback = cb;
}

void CopleyServo::setStatusCallback(DS301CallbackObject cb)
{
    status_callback = cb;
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

void CopleyServo::modeControl(
        uint16_t set,
        uint16_t clear,
        enum OperationMode mode,
        PDOCallbackObject callback
    )
{
    control_word |= set;
    control_word &= ~clear;
    mode_of_operation = mode;
    std::vector<uint8_t> data;
    Transfer::pack(control_word, data);
    Transfer::pack((uint8_t)mode_of_operation, data);
    Transfer::pack(output_pins, data);
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

void CopleyServo::output(uint16_t set, uint16_t clear)
{
    output_pins |= set;
    output_pins &= ~clear;
    modeControl(0, 0, mode_of_operation);
}

void CopleyServo::enable(bool state,
        DS301CallbackObject callback)
{
    if(state) {
        control(
                CONTROL_SWITCH_ON | CONTROL_ENABLE_VOLTAGE | CONTROL_ENABLE_OPERATION,
                CONTROL_RESET_FAULT | CONTROL_NEW_SETPOINT
               );
    } else {
        control(CONTROL_RESET_FAULT,
                CONTROL_SWITCH_ON | CONTROL_ENABLE_VOLTAGE |
                CONTROL_ENABLE_OPERATION | CONTROL_NEW_SETPOINT
               );
    }
    enable_callback = callback;
}

bool CopleyServo::enabled(void)
{
    return status_word & STATUS_SWITCHED_ON != 0;
}

void CopleyServo::home(DS301CallbackObject callback)
{
    home_callback = callback;
    modeControl(0, CONTROL_NEW_SETPOINT, Homing);
    control( CONTROL_NEW_SETPOINT, 0);
}

void CopleyServo::setVelocity(int32_t v)
{
    modeControl(0, 0, ProfileVelocity);
    velocity_pdo->send(v);
}

void CopleyServo::setPosition(int32_t p, DS301CallbackObject callback)
{
    position_callback = callback;
    modeControl(
            0,
            CONTROL_NEW_SETPOINT | CONTROL_CHANGE_SET_IMMEDIATE,
            ProfilePosition
    );
    position_pdo->send(
            p,
            PDOCallbackObject(static_cast<TransferCallbackReceiver *>(this),
                              static_cast<PDOCallbackObject::CallbackFunction>(&CopleyServo::_positionGo)));
}

void CopleyServo::_positionGo(PDO &pdo)
{
    modeControl(CONTROL_NEW_SETPOINT | CONTROL_CHANGE_SET_IMMEDIATE, 0, ProfilePosition);
}

void CopleyServo::setPVCallback(DS301CallbackObject cb)
{
    pv_callback = cb;
}

void CopleyServo::inputPinFunction(int pin_index, enum InputPinFunction function)
{
    std::vector<uint8_t> data;
    Transfer::pack(function, data);
    writeObjectDictionary(0x2192, pin_index, data);
}

void CopleyServo::inputPinPullups(uint16_t pullups)
{
    std::vector<uint8_t> data;
    Transfer::pack(pullups, data);
    writeObjectDictionary(0x2191, 0, data);
}


void CopleyServo::outputPinFunction(int pin_index, enum OutputPinFunction function,
        std::vector<uint8_t> parameters,
        bool activeLow)
{
    std::vector<uint8_t> data;
    uint16_t funcword = function;
    if(activeLow) {
        funcword |= 1<<8;
    } else {
        funcword &= ~(1<<8);
    }
    Transfer::pack(funcword, data);
    if(parameters.size()<4) {
        parameters.resize(4, 0);
    }
    std::copy(parameters.begin(), parameters.end(), std::back_inserter(data));
    writeObjectDictionary(0x2193, pin_index, data);
}

void CopleyServo::setInputCallback(InputChangeCallback cb)
{
    input_callback = cb;
}

void CopleyServo::clearPvtError(uint8_t errorFlags)
{
    std::vector<uint8_t> data;
    // ensure that data is 8 bytes long
    data.assign(8, 0);
    data[0] = PVT_HEADER_CLEAR_ERRORS;
    data[1] = errorFlags;
    pvt_pdo->send(data);
}

void CopleyServo::popPvtBuffer(uint8_t numToPop)
{
    std::vector<uint8_t> data;
    // ensure that data is 8 bytes long
    data.assign(8, 0);
    data[0] = PVT_HEADER_POP_BUFFER;
    data[1] = numToPop;
    pvt_pdo->send(data);
}

void CopleyServo::clearPvtBuffer()
{
    std::vector<uint8_t> data;
    // ensure that data is 8 bytes long
    data.assign(8, 0);
    data[0] = PVT_HEADER_CLEAR_BUFFER;
    pvt_pdo->send(data);
    // clear the active segments. they're no longer valid
    m_activeSegments.clear();
}

void CopleyServo::resetSegmentId()
{
    std::vector<uint8_t> data;
    // ensure that data is 8 bytes long
    data.assign(8, 0);
    data[0] = PVT_HEADER_RESET_SEGMENT_ID;
    pvt_pdo->send(data);

    m_currentSegmentId = 0;
    m_expectedSegmentId = 0;
    // clear the active segments. they're no longer valid
    m_activeSegments.clear();
}

void CopleyServo::handlePvtError(uint8_t statusByte)
{
    // reset the errors (probably not the best plan, maybe change this?)
    // note that we have to clear the error before we can fix a sequence
    // error
    clearPvtError(statusByte);

    // reset the error message. there may be more than one error here.
    m_lastErrorMessage = "";
    // figure out what kind of error we have
    if(statusByte&PVT_SEQUENCE_ERROR)
    {
        // this is not good. it means we missed some segment
        m_lastErrorMessage += "sequence ID error!\n";

        // check to see if we have the sequence number the servo wants.
        bool found = false;
        for(auto segment : m_activeSegments)
        {
            if(segment.id == m_expectedSegmentId)
            {
                // we have the segment the controller is looking for!
                bool found = true;
                sendPvtSegment(segment);
                break;
            }
        }

        // if we didn't find the segment the conroller wanted, we're
        // totally hosed.
        if(!found)
        {
            // reset everything back to a known state
            clearPvtBuffer();
            resetSegmentId();
        }
    }

    if(statusByte&PVT_BUFFER_UNDERFLOW)
    {
        // we ran out of things to do!
        m_lastErrorMessage += "buffer underflow!\n";

        // try to recover from this by adding more segments from
        // active segments, if we have them. it is very likely we don't
        uint8_t slots = m_freeBufferSlots;
        for(auto segment : m_activeSegments)
        {
            if(slots == 0)
            {
                // if we're out of slots, break
                break;
            }

            if(segment.id >= m_expectedSegmentId)
            {
                // if this segment is the expected one or higher, add it
                sendPvtSegment(segment);
                slots--;
            }
        }
    }

    if(statusByte&PVT_BUFFER_OVERFLOW)
    {
        // too many things to do!
        m_lastErrorMessage += "buffer overflow!\n";
    }

    if(statusByte&PVT_BUFFER_EMPTY)
    {
        // we ran out of things to do!
        m_lastErrorMessage += "buffer empty!\n";
    }

    // call the error callback
    error_callback(*this);
}

void CopleyServo::handlePvtSegmentConsumed()
{
    // if we just consumed a segment, check to see if we've got more
    // segments to send.
    // first, remove all the segments with lower sequence numbers
    while((m_activeSegments.size() > 0) &&
          (m_activeSegments.front().id < m_expectedSegmentId))
    {
        m_activeSegments.pop_front();
    }

    // send one new segment if we happen to have the next one the
    // servo expects to get
    if((m_activeSegments.size() > 0) &&
       (m_freeBufferSlots > 0) &&
       (m_activeSegments.front().id == m_expectedSegmentId))
    {
        // send this segment.
        sendPvtSegment(m_activeSegments.front());
    }
}

void CopleyServo::addPvtSegment(int32_t position, int32_t velocity, uint8_t duration, bool lastInSequence)
{
    // ensure that we're in InterpolatedPosition mode.
    // note that this doesn't cause the control word bit flip. you'll
    // need to do that yourself as needed.
    modeControl(0, 0, InterpolatedPosition);

    // make a PvtSegment struct instance for this new segment
    PvtSegment segment;
    segment.id = m_currentSegmentId;
    m_currentSegmentId++;
    segment.position = position;
    segment.velocity = velocity;
    segment.time = duration;

    // add the new segment to the back of the active list.
    m_activeSegments.push_back(segment);

    // send this if we think we can.
    if(m_freeBufferSlots > 0)
    {
        sendPvtSegment(segment);

        // decrement the free buffer slots between status updates
        // this keeps us from overflowing when we have a ton of adds between
        // status pdos. this seems unlikely, but possible.
        m_freeBufferSlots--;
    }

    // if this is the last segment in a sequence, add a special extra
    // sequence to make sure we actually get to the right place.
    if(lastInSequence)
    {
        addPvtSegment(position, velocity, duration, false);
    }
}

void CopleyServo::startPvtMove()
{
    // flip bit 4 in the control word to 0 so we can cause a 0 to 1
    // transition. this causes the servo to start a pvt segment
    modeControl(0, CONTROL_NEW_SETPOINT, InterpolatedPosition);
    control( CONTROL_NEW_SETPOINT, 0);
}

void CopleyServo::stopPvtMove(uint8_t duration, bool emergency)
{
    // clear the pvt buffer.
    clearPvtBuffer();

    // if this is an emergency, just disable pvt mode by flipping
    // the control bit.
    if(emergency)
    {
        modeControl(0, CONTROL_NEW_SETPOINT, InterpolatedPosition);
    }
    else
    {
        // add a pvt segment that takes duration time and is at 0
        // velocity. leave position in place if we're absolute
        if(m_isAbsolute)
        {
            // in absolute mode, set position to whatever it is now
            // and set velocity to zero
            addPvtSegment(this->position, 0, true);
        }
        else
        {
            // set position and velocity to 0 in relative mode
            addPvtSegment(0, 0, duration, true);
        }
    }
}

bool CopleyServo::sendPvtSegment(PvtSegment &segment)
{
    // send this segment to the servo, return true if it actually happened
    if(m_freeBufferSlots <= 0)
    {
        // we don't have any buffer slots here, don't send it
        return false;
    }
    std::vector<uint8_t> data;
    // ensure that data is 8 bytes long
    data.assign(8, 0);

    // get the sequence number in the right place
    uint8_t headerByte = (uint8_t)(segment.id&PVT_HEADER_SEQUENCE_MASK);

    // figure out the segment mode. these are on page 215 of the copley manual
    if(m_isAbsolute)
    {
        // XXX TODO: support the faster mode (mode 1)
        // for absolute 0.1 counts per sec we're done because it is mode 0
    }
    else
    {
        // XXX TODO: support the faster mode (mode 3)
        // for relative 0.1 counts per sec, we need to specify mode 2
        headerByte |= 2 << 3;
    }

    data[0] = headerByte;

    // byte 1 is time
    data[1] = segment.time;

    // bytes 2-4 are 24 bit position (relative or absolute)
    // please excuse my epic cast
    data[2] = ((uint8_t *)&segment.position)[0];
    data[3] = ((uint8_t *)&segment.position)[1];
    data[4] = ((uint8_t *)&segment.position)[2];

    // bytes 5-7 are 24 bit velocity
    // please excuse my epic cast
    data[5] = ((uint8_t *)&segment.velocity)[0];
    data[6] = ((uint8_t *)&segment.velocity)[1];
    data[7] = ((uint8_t *)&segment.velocity)[2];

    pvt_pdo->send(data);
    return true;
}

}
