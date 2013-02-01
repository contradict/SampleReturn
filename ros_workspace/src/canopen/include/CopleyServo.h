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

enum InputPinFunction {
    NoFunction,
    Reserved,
    RisingEdgeReset,
    FallingEdgeReset,
    PositiveLimitActiveHigh,
    PositiveLimitActiveLow,
    NegativeLimitActiveHigh,
    NegativeLimitActiveLow,
    MotorTemperatureSensorActiveHigh,
    MotorTemperatureSensorActiveLow,
    DisableHighClearFaluts,
    DisableLowClearFaults,
    RisingResetDisableHigh,
    FallingResetDisableLow,
    HomeActiveHigh,
    HomeActiveLow,
    DisableHigh,
    DisableLow,
    PWMSynchronization,
    HaltHigh,
    HaltLow,
    HighResolutionDivideHigh,
    HighResolutionDivideLow,
    HighSpeedPositionCaptureRising,
    HighSpeedPositionCaptureFalling,
    CounterRising,
    CounterFalling
};

enum OutputPinFunction {
    ManufacturerStatus,
    LatchedEventStatus,
    Manual,
    TrajectoryStatus,
    PositionBetween,
    PositionCrossesRising,
    PositionCrossesFalling,
    PositionCrossesAny
};

struct PVTData {
    bool relative;
    uint8_t duration_ms;
    uint32_t position;
    uint32_t velocity;
};

struct PVTMove {
    std::vector<struct PVTData> points;
    int segment_id;
    int executing_segment;
    DS301CallbackObject callback;
};

#define PVT_COMMAND_CLEAR_BUFFER           (0x80|0)
#define PVT_COMMAND_POP_SEGMENTS           (0X80|1)
#define PVT_COMMAND_CLEAR_ERRORS           (0X80|2)
#define PVT_COMMAND_ZERO_SEGMENT_ID        (0X80|3)
#define PVT_BUFFER_FORMAT_PVT              (0<<3)
#define PVT_BUFFER_FORMAT_PVT_FAST         (1<<3)
#define PVT_BUFFER_FORMAT_RELATIVE         (2<<3)
#define PVT_BUFFER_FORMAT_RELATIVE_FAST    (3<<3)
#define PVT_BUFFER_FORMAT_INITIAL_POSITION (4<<3)
#define PVT_BUFFER_FORMAT_PT               (5<<3)
#define PVT_BUFFER_FORMAT_PT_RELATIVE      (6<<3)
#define PVT_BUFFER_LENGTH                  32

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

        void inputPinFunction(int pin_index, enum InputPinFunction function);
        void inputPinPullups(uint16_t pullups);
        void outputPinFunction(int pin_index, enum OutputPinFunction function,
                std::vector<uint8_t> parameters, bool activeLow);
        void output(uint16_t set, uint16_t clear);

        class InputChangeCallback {
            public:
                typedef void
                    (TransferCallbackReceiver::*CallbackFunction)(CopleyServo &svo,
                        uint16_t old_pins, uint16_t new_pins);
                InputChangeCallback(TransferCallbackReceiver *r,
                               CallbackFunction fn
                               ) :
                    r(r), fn(fn) {};
                InputChangeCallback(std::tr1::shared_ptr<TransferCallbackReceiver> sr,
                               CallbackFunction fn
                              ) :
                    r(NULL), sr(sr), fn(fn) {};
                 InputChangeCallback() :
                    r(NULL),
                    fn(NULL) {};

                virtual void operator()(CopleyServo &cls, uint16_t old_pins, uint16_t
                        new_pins)
                {
                    if(r != NULL && fn != NULL) ((*r).*fn)(cls, old_pins, new_pins);
                    else if(sr != NULL && fn != NULL) ((*sr).*fn)(cls, old_pins,
                            new_pins);
                };

            private:
                TransferCallbackReceiver *r;
                std::tr1::shared_ptr<TransferCallbackReceiver> sr;
                CallbackFunction fn;
        };
        void setInputCallback(InputChangeCallback cb);
        void setPVCallback(DS301CallbackObject cb);
        void setEMCYCallback(DS301CallbackObject cb);

        uint16_t getInputPins(void){return input_pins;};
        uint16_t getOutputPins(void){return output_pins;};

        void pvtMove(std::vector<struct PVTData> points, bool preempt=false,
                DS301CallbackObject callback=DS301CallbackObject());

        int32_t position;
        int32_t velocity;
        bool gotPV;
    private:
        void syncCallback(SYNC &sync);
        void emcyCallback(EMCY &emcy);
        void statusModePDOCallback(PDO &pdo);
        void positionVelocityPDOCallback(PDO &pdo);
        void trajectoryStatusPDOCallback(PDO &pdo);

        void _initialize(DS301 &node);
        void _mapPDOs(void);
        void _printStatusAndMode(void);

        void _initialModeOfOperation(SDO &sdo);
        void _initialControlWord(SDO &sdo);
        void _initialOutputPins(SDO &sdo);

        void _setPositionValue(PDO &pdo);
        void _positionGo(PDO &pdo);

        void startPVTMove(void);

        std::tr1::shared_ptr<SYNC> sync;
        std::tr1::shared_ptr<EMCY> emcy;

        std::tr1::shared_ptr<TPDO> status_mode_pdo;
        std::tr1::shared_ptr<TPDO> position_velocity_pdo;
        std::tr1::shared_ptr<TPDO> trajectory_buffer_status_pdo;

        std::tr1::shared_ptr<RPDO> control_mode_pdo;
        std::tr1::shared_ptr<RPDO> position_pdo;
        std::tr1::shared_ptr<RPDO> velocity_pdo;
        std::tr1::shared_ptr<RPDO> ip_move_segment_pdo;

        DS301CallbackObject home_callback;
        DS301CallbackObject position_callback;
        DS301CallbackObject pv_callback;
        DS301CallbackObject enable_callback;
        DS301CallbackObject emcy_callback;
        InputChangeCallback input_callback;

        bool syncProducer;
        int32_t syncInterval;
        bool allReady;
        bool mapsCreated;

        uint16_t status_word;
        uint16_t input_pins;
        uint16_t output_pins;
        uint16_t control_word;
        double bus_voltage;
        enum OperationMode mode_of_operation;

        std::vector<std::tr1::shared_ptr<struct PVTMove> > pvt_moves;

        void sendNextPoint(uint16_t segment_id, std::vector<struct PVTData>::iterator point);
        void sendPoints(std::vector<struct PVTData> &points, int &start, int count);
};

}

#endif // __COPLEY_SERVO_H__
