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

#define PVT_SEQUENCE_ERROR       0x01
#define PVT_BUFFER_UNDERFLOW     0x02
#define PVT_BUFFER_OVERFLOW      0x04
#define PVT_BUFFER_EMPTY         0x80
#define PVT_GOT_ERROR            0x8f // was there any kind of error?

// pvt control headers. the segment header needs to be computed
#define PVT_HEADER_CLEAR_BUFFER  0x80
#define PVT_HEADER_POP_BUFFER    0x81
#define PVT_HEADER_CLEAR_ERRORS  0x82
#define PVT_HEADER_RESET_SEGMENT_ID 0x83
#define PVT_HEADER_SEQUENCE_MASK 0x0007

// the more data needed callback will fire if the number of buffer segments
// is less than or equal to this
#define PVT_MINIMUM_SEGMENTS 3

// from the documentation:
#define PVT_NUM_BUFFER_SLOTS 32

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

        // pvt mode methods
        void clearPvtBuffer(); // when starting a new maneuver.
        // absolute or relative positioning for pvt mode?
        void setPvtAbsolute() {m_isAbsolute = true;}
        void setPvtRelative() {m_isAbsolute = false;}
        // addPvtSegment will add the segment to the buffer
        // set lastInSequence to true if this is the last segment in a
        // sequence. this will cause an extra zero duration sequence to be
        // created to ensure you actually get to the last position
        void addPvtSegment(int32_t position, int32_t velocity, uint8_t duration, bool lastInSequence=false);
        void startPvtMove(); // flips the control bit to start a pvt move
        // stops a pvt move in duration milliseconds.
        // if emergency is true, it flips the control word bit that disables
        // pvt mode until you call startPvtMove again
        void stopPvtMove(uint8_t duration, bool emergency=false);
        int getPvtBufferDepth(); // how many things are in the pvt buffer?

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
        void setStatusCallback(DS301CallbackObject cb);
        void setErrorCallback(DS301CallbackObject cb);
        void setMoreDataNeededCallback(DS301CallbackObject cb);

        uint16_t getInputPins(void){return input_pins;};
        uint16_t getOutputPins(void){return output_pins;};
        uint16_t getStatusWord(void){return status_word;};

        void writeMode(std::ostream &out);
        void writeStatus(std::ostream &out);

        std::string getLastError() {return m_lastErrorMessage;}
        void resetLastError() {m_lastErrorMessage = "";}

        bool getNeedsToStart() {return ((PVT_NUM_BUFFER_SLOTS - m_expectedFreeBufferSlots) >= PVT_MINIMUM_SEGMENTS) && !m_pvtModeActive;}

        int32_t position;
        int32_t velocity;
        double bus_voltage;
        bool gotPV;
    private:
        void syncCallback(SYNC &sync);
        void emcyCallback(EMCY &emcy);
        void statusModePDOCallback(PDO &pdo);
        void positionVelocityPDOCallback(PDO &pdo);
        void inputPDOCallback(PDO &pdo);

        void _initialize(DS301 &node);
        void _mapPDOs(void);
        void _printStatusAndMode(void);

        void _initialModeOfOperation(SDO &sdo);
        void _initialControlWord(SDO &sdo);
        void _initialOutputPins(SDO &sdo);

        void _setPositionValue(PDO &pdo);
        void _positionGo(PDO &pdo);

        // a struct for keeping track of pvt segments.
        struct PvtSegment
        {
            uint16_t id;
            int32_t position;
            int32_t velocity;
            uint8_t time;
        };

        // internal pvt methods
        void handlePvtError(uint8_t statusByte);
        void handlePvtSegmentConsumed(); // if we have waiting segments, add them
        void clearPvtError(uint8_t errorFlags);
        void popPvtBuffer(uint8_t numToPop);
        void resetSegmentId();
        bool sendPvtSegment(PvtSegment &segment); // send the segment

        std::tr1::shared_ptr<SYNC> sync;
        std::tr1::shared_ptr<EMCY> emcy;

        std::tr1::shared_ptr<TPDO> status_mode_pdo;
        std::tr1::shared_ptr<TPDO> position_velocity_pdo;
        std::tr1::shared_ptr<TPDO> input_pdo;

        std::tr1::shared_ptr<RPDO> control_mode_pdo;
        std::tr1::shared_ptr<RPDO> position_pdo;
        std::tr1::shared_ptr<RPDO> velocity_pdo;
        std::tr1::shared_ptr<RPDO> pvt_pdo;

        DS301CallbackObject home_callback;
        DS301CallbackObject position_callback;
        DS301CallbackObject pv_callback;
        DS301CallbackObject enable_callback;
        DS301CallbackObject status_callback;
        DS301CallbackObject emcy_callback;
        DS301CallbackObject error_callback;
        DS301CallbackObject more_data_needed_callback;;
        InputChangeCallback input_callback;

        bool syncProducer;
        int32_t syncInterval;
        bool allReady;
        bool mapsCreated;

        uint16_t status_word;
        uint16_t input_pins;
        uint16_t output_pins;
        uint16_t control_word;
        enum OperationMode mode_of_operation;

        // members for PVT mode
        bool m_isAbsolute; // are positions absolute or relative?
        uint16_t m_currentSegmentId; // a counter
        uint16_t m_expectedSegmentId; // given by buffer status pdo
        uint8_t m_freeBufferSlots; // as reported by status pdo
        int  m_expectedFreeBufferSlots; // predict how many will be free next pdo
        uint8_t m_pvtStatusByte;    // given by buffer statud pdo
        std::string m_lastErrorMessage; // for logging the last error
        std::list<PvtSegment> m_activeSegments; // segments recently sent/to be sent
        bool m_pvtModeActive; // are we in pvt mode?
};

}

#endif // __COPLEY_SERVO_H__
