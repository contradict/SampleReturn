#ifndef __COPLEY_SERVO_H__
#define __COPLEY_SERVO_H__
namespace CANOpen {
class CopleyServo : public DS301 {
    public:
        CopleyServo(long int node_id, std::tr1::shared_ptr<Bus> bus);

        void setSyncInterval(uint32_t interval_us);
        void enableSync(bool enable);

    private:
        void syncCallback(SYNC &sync);
        void statusPDOCallback(PDO &pdo);

        void _initialModeOfOperation(SDO &sdo);
        void _initialControlWord(SDO &sdo);

        SYNC sync;

        std::tr1::shared_ptr<TPDO> status_pdo;

        std::tr1::shared_ptr<RPDO> control_pdo;

        uint16_t status_word;
        uint16_t control_word;
        uint8_t mode_of_operation;
        int32_t position;

};

}

#endif // __COPLEY_SERVO_H__
