#ifndef __COPLEY_SERVO_H__
#define __COPLEY_SERVO_H__
namespace CANOpen {

enum OperationMode {
    UnknownMode     = 0,
    ProfilePosition = 1,
    ProfileVelocity = 3,
    ProfileTorque   = 4,
    Homing          = 6,
    InterpolatedPosition = 7,
    CyclicSynchonousPosition = 8,
    CyclicSynchronousVelocity = 9,
    CyclicSynchronousTorque = 10,
};

#define CONTROL_SWITCH_ON        0x0001
#define CONTROL_ENABLE_VOLTAGE   0x0002
#define CONTROL_QUICK_STOP       0x0004
#define CONTROL_ENABLE_OPERATION 0x0008
#define CONTROL_NEW_SETPOINT     0x0010
#define CONTROL_CHANGE_SET_IMMEDIATE 0x0020
#define CONTROL_MOVE_RELATIVE    0x0040
#define CONTROL_RESET_FAULT      0x0080
#define CONTROL_HALT             0x0100

#define STATUS_READY             0x0001
#define STATUS_SWITCHED_ON       0x0002
#define STATUS_OPERATION_ENABLED 0x0004
#define STATUS_FAULT             0x0008
#define STATUS_VOLTAGE_ENABLED   0x0010
#define STATUS_QUICK_STOP        0x0020
#define STATUS_SWITCH_ON_DISABLED 0x0040
#define STATUS_WARNING           0x0080
#define STATUS_TRAJECTORY_ABORT  0x0100
#define STATUS_REMOTE            0x0200
#define STATUS_TARGET_REACHED    0x0400
#define STATUS_INTERNAL_LIMIT    0x0800
#define STATUS_SETPOINT_ACK      0x1000
#define STATUS_FOLLOWING_ERROR   0x2000
#define STATUS_SPEED_ZERO        0x1000
#define STATUS_MAXIMUM_SLIPPAGE  0x2000
#define STATUS_RESERVED_PT12     0x1000
#define STATUS_RESERVED_PT13     0x2000
#define STATUS_HOMING_ATTAINED   0x1000
#define STATUS_HOMING_ERROR      0x2000
#define STATUS_INTERPOLATED_POSITION 0x1000
#define STATUS_RESERVED_IP13     0x2000
#define STATUS_PERFORMING_MOVE   0x4000
#define STATUS_RESERVED_15       0x8000
 
class CopleyServo : public DS301 {
    public:
        CopleyServo(long int node_id, std::tr1::shared_ptr<Bus> bus);
        CopleyServo(long int node_id, int sync_interval, std::tr1::shared_ptr<Bus> bus);

        void initialize(void);

        void control(uint16_t set, uint16_t clear);
        void mode(enum OperationMode mode);
        void modeControl(uint16_t set, uint16_t clear,enum OperationMode mode,
                PDOCallbackObject callback=PDOCallbackObject());

        void enable(bool state=true,
                DS301CallbackObject callback=DS301CallbackObject());
        bool enabled(void);

        void home(DS301CallbackObject callback=DS301CallbackObject());

        void setVelocity(int32_t v);
        void setPosition(int32_t p, DS301CallbackObject
                callback=DS301CallbackObject() );

        enum OperationMode operationMode(uint8_t m);
        static enum OperationMode operationMode(std::string m);
        static std::string operationModeName(enum OperationMode m);

        bool ready(void);

        uint16_t status_word;
        uint16_t control_word;
        enum OperationMode mode_of_operation;
        void setPVCallback(DS301CallbackObject cb);
        void setEMCYCallback(DS301CallbackObject cb);

        int32_t position;
        int32_t velocity;
        bool gotPV;

    private:
        void syncCallback(SYNC &sync);
        void _initialize(DS301 &node);
        void _mapPDOs(void);
        void emcyCallback(EMCY &emcy);
        void statusModePDOCallback(PDO &pdo);
        void positionVelocityPDOCallback(PDO &pdo);
        void _printStatusAndMode(void);

        void _initialModeOfOperation(SDO &sdo);
        void _initialControlWord(SDO &sdo);

        void _setPositionValue(PDO &pdo);
        void _positionGo(PDO &pdo);

        std::tr1::shared_ptr<SYNC> sync;
        std::tr1::shared_ptr<EMCY> emcy;

        std::tr1::shared_ptr<TPDO> status_mode_pdo;
        std::tr1::shared_ptr<TPDO> position_velocity_pdo;

        std::tr1::shared_ptr<RPDO> control_mode_pdo;
        std::tr1::shared_ptr<RPDO> position_pdo;
        std::tr1::shared_ptr<RPDO> velocity_pdo;

        DS301CallbackObject home_callback;
        DS301CallbackObject position_callback;
        DS301CallbackObject pv_callback;
        DS301CallbackObject enable_callback;
        DS301CallbackObject emcy_callback;

        bool syncProducer;
        int32_t syncInterval;
        bool allReady;
        bool mapsCreated;
};

}

#endif // __COPLEY_SERVO_H__
