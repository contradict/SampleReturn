#ifndef __DS301_H__
#define __DS301_H__
namespace CANOpen {

class DS301;
typedef void (*DS301NotifyCallback)(DS301 &node);

struct SDOTransaction {
    bool write;
    uint16_t index;
    uint8_t subindex;
    std::vector<uint8_t> data;
    std::tr1::shared_ptr<SDOCallbackObject> callback;
};


class DS301 : public TransferCallbackReceiver {
    public:
        DS301(unsigned long node_id,
              std::tr1::shared_ptr<Bus> bus,
              bool extended=false);

        void readObjectDictionary(uint16_t index, uint8_t subindex,
                     std::tr1::shared_ptr<SDOCallbackObject> callback);
        void writeObjectDictionary(uint16_t index, uint8_t subindex, std::vector<uint8_t> &bytes,
                     std::tr1::shared_ptr<SDOCallbackObject>
                     callback=std::tr1::shared_ptr<SDOCallbackObject>(new
                             SDOCallbackObject())
                     );

        void sendNMT(enum NodeControlCommand cmd,
                     DS301NotifyCallback cb=NULL);

        std::tr1::shared_ptr<TPDO> createTPDO(std::string name,
                                              int pdo_number,
                                              PDOCallbackObject callback);
        std::tr1::shared_ptr<RPDO> createRPDO(std::string name,
                                              int pdo_number
                                             );
        std::tr1::shared_ptr<RPDO> mapRPDO(std::string name,
                                           int pdo_number,
                                           std::vector<struct PDOMap> mapped,
                                           uint8_t type
                                          );

        std::tr1::shared_ptr<TPDO> mapTPDO(std::string name,
                                           int pdo_number,
                                           std::vector<struct PDOMap> mapped,
                                           uint8_t type,
                                           PDOCallbackObject callback=PDOCallbackObject()
                                          );

        void setPDOType(std::tr1::shared_ptr<PDO> ppdo, uint8_t type);

        void enableSync(bool enable);
        void setSyncInterval(uint32_t interval_us);
    private:
        void mapPDO(std::tr1::shared_ptr<PDO> ppdo,
                    std::vector<struct PDOMap> mapped,
                    uint8_t type
                   );
        void nmt_callback(NMT &nmt);
        void sdo_callback(SDO &sdo);

        void insertSDOTransaction(std::tr1::shared_ptr<struct SDOTransaction> &t);
        void startNextTransaction(void);
        void checkSDOTransactionQueue(SDO &sdo);
        void checkSDOTransactionQueueError(SDO &sdo);

        void readPDO(std::tr1::shared_ptr<PDO> ppdo);

        const unsigned long node_id;
        std::tr1::shared_ptr<Bus> bus;
        bool EXTENDED;

        bool inCheck;

        std::tr1::shared_ptr<NMT> pnmt;
        DS301NotifyCallback nmt_notify;
        std::tr1::shared_ptr<SDO> psdo;

        std::vector<std::tr1::shared_ptr<struct SDOTransaction> > sdoTransactionQueue;

};
}
#endif // __DS301_H__

