#ifndef __PDO_H__
#define __PDO_H__
namespace CANOpen {

#define TXPDO_TYPE_SYNC(n)       (n)
#define TXPDO_TYPE_RTR_SYNC      252
#define TXPDO_TYPE_RTR_IMMEDIATE 253
#define TXPDO_TYPE_IMMEDIATE(n)  (254|(n&1))

class PDO;
typedef CallbackObject<PDO> PDOCallbackObject;

struct PDOMap {
    uint16_t index;
    uint8_t subindex;
    uint8_t bits;
};

class PDO : public Transfer {
    public:
        PDO(std::string name,
            unsigned long node_id,
            int pdo_number,
            PDOCallbackObject callback=PDOCallbackObject()
            ) : Transfer(name, node_id), pdo_number(pdo_number), mapped(false),
        callback(callback){};

        virtual uint16_t mappingIndex(void)=0;
        void writeMappingCount(std::vector<uint8_t> &d);
        void readMappingCount(const std::vector<uint8_t> &d);
        void writeMappingData(int i, std::vector<uint8_t> &d);
        void readMappingData(int i, const std::vector<uint8_t> &d) ;

        virtual uint16_t communicationIndex(void)=0;
        void writeCommunicationData(std::vector<uint8_t> &d);
        void readCommunicationData(const std::vector<uint8_t> &d);
        void writeCommunicationTypeData(std::vector<uint8_t> &d);
        void readCommunicationTypeData(const std::vector<uint8_t> &d);
        void mappingComplete(SDO &sdo);

        int pdo_number;
        bool mapped;
        uint8_t objectCount;
        struct PDOMap mappedObjects[4];
        struct {
            unsigned identifier:28;
            unsigned extended:1;
            unsigned rtr:1;
            unsigned invalid:1;
            uint8_t type;
        } communication_parameters;

        std::vector<uint8_t> data;

    protected:
        PDOCallbackObject callback;
};

class RPDO : public PDO {
    public:
        RPDO(std::string name,
             unsigned long node_id,
             int pdo_number,
             PDOCallbackObject callback=PDOCallbackObject()
            ) : PDO(name, node_id, pdo_number, callback){};

        uint16_t COBID(void);
        bool handleMessage(const Message m){return false;};
        bool nextMessage(Message &m)
        {
            if(to_send.empty()){
                return false;
            } else {
                m = to_send.front();
                to_send.erase(to_send.begin());
                callback(*this);
                return true;
            }
        };


        uint16_t communicationIndex(void);

        uint16_t mappingIndex(void);

        void send(std::vector<uint8_t> &data, PDOCallbackObject
                callback=PDOCallbackObject());
        void send(uint8_t value, PDOCallbackObject callback=PDOCallbackObject());
        void send(uint16_t value, PDOCallbackObject callback=PDOCallbackObject());
        void send(uint32_t value, PDOCallbackObject callback=PDOCallbackObject());
        void send(int32_t value, PDOCallbackObject callback=PDOCallbackObject());

};

class TPDO : public PDO {
    public:
        TPDO(std::string name,
             unsigned long node_id,
             int pdo_number,
             PDOCallbackObject callback) : PDO(name, node_id, pdo_number, callback) {};

        uint16_t COBID(void);
        bool handleMessage(const Message m);

        uint16_t communicationIndex(void);

        uint16_t mappingIndex(void);

};

}
#endif // __PDO_H__
