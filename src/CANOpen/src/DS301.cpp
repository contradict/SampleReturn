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
#include <NMT.h>
#include <DS301.h>

namespace CANOpen {

DS301::DS301(unsigned long node_id, std::tr1::shared_ptr<Bus> b, bool extended) :
    node_id(node_id),
    bus(b),
    EXTENDED(extended),
    pnmt(new NMT(node_id,
                 NMTCallbackObject(
                     static_cast<TransferCallbackReceiver *>(this),
                     static_cast<NMTCallbackObject::CallbackFunction>(&DS301::nmt_callback)))),
    nmt_notify(NULL),
    psdo(new SDO("Default SDO", node_id,
                SDOCallbackObject(
                    static_cast<TransferCallbackReceiver *>(this),
                    static_cast<SDOCallbackObject::CallbackFunction>(&DS301::sdo_callback))))
{
    inCheck = false;
    bus->add(pnmt);
    bus->add(psdo);
}

void DS301::readObjectDictionary(uint16_t index, uint8_t subindex,
        std::tr1::shared_ptr<SDOCallbackObject> callback)
{
    std::tr1::shared_ptr<struct SDOTransaction> pt(new struct SDOTransaction);
    pt->write=false;
    pt->index=index;
    pt->subindex=subindex;
    pt->callback=callback;
    insertSDOTransaction(pt);
}

void DS301::writeObjectDictionary(uint16_t index, uint8_t subindex,
        std::vector<uint8_t> &bytes, std::tr1::shared_ptr<SDOCallbackObject> callback)
{
    std::tr1::shared_ptr<struct SDOTransaction> pt(new struct SDOTransaction);
    pt->write=true;
    pt->index=index;
    pt->subindex=subindex;
    pt->data=bytes;
    pt->callback=callback;
    insertSDOTransaction(pt);
}

void DS301::insertSDOTransaction(std::tr1::shared_ptr<struct SDOTransaction> &t)
{
    sdoTransactionQueue.push_back(t);
    if( sdoTransactionQueue.size() == 1) {
        startNextTransaction();
    }
}

void DS301::startNextTransaction(void)
{
    std::tr1::shared_ptr<struct SDOTransaction> ptr(sdoTransactionQueue.front());
    SDOCallbackObject
        intermediate_callback(static_cast<TransferCallbackReceiver *>(this),
                static_cast<SDOCallbackObject::CallbackFunction>(&DS301::checkSDOTransactionQueue),
                static_cast<SDOCallbackObject::CallbackFunction>(&DS301::checkSDOTransactionQueueError));
    if(ptr->write) {
        psdo->write(ptr->index, ptr->subindex, ptr->data, intermediate_callback);
    } else {
        psdo->read(ptr->index, ptr->subindex, intermediate_callback);
    }
}

void DS301::checkSDOTransactionQueue(SDO &sdo)
{
   std::tr1::shared_ptr<struct SDOTransaction> ptr(sdoTransactionQueue.front());
   sdoTransactionQueue.erase(sdoTransactionQueue.begin());
   inCheck = true;
   ptr->callback->operator()(sdo);
   if(!sdoTransactionQueue.empty()) {
       startNextTransaction();
   }
   inCheck = false;
}

void DS301::checkSDOTransactionQueueError(SDO &sdo)
{
   std::tr1::shared_ptr<struct SDOTransaction> ptr(sdoTransactionQueue.front());
   sdoTransactionQueue.erase(sdoTransactionQueue.begin());
   inCheck = true;
   ptr->callback->error(sdo);
   if(!sdoTransactionQueue.empty()) {
       startNextTransaction();
   }
   inCheck = false;
}


void DS301::nmt_callback(NMT &nmt)
{
    std::cerr << "NMT state transition: " << pnmt->nodeStateName(pnmt->node_state) << std::endl;
    if(nmt_notify != NULL) {
        nmt_notify(*this);
        nmt_notify=NULL;
    }
}

void DS301::sdo_callback(SDO &sdo)
{
}

void DS301::sendNMT(enum NodeControlCommand cmd, DS301NotifyCallback cb)
{
    nmt_notify = cb;
    pnmt->nodeControl(cmd);
}

class PDODataReader : public SDOCallbackObject {
    public:
        typedef void (PDO::*DataReader)(const std::vector<uint8_t> &);
        typedef void (PDO::*IndexedDataReader)(int i, const std::vector<uint8_t> &);
        PDODataReader(std::tr1::shared_ptr<PDO> ppdo,
                      DataReader rdr):
            SDOCallbackObject(),
            ppdo(ppdo),reader(rdr),readMappings(-1){};
        PDODataReader(std::tr1::shared_ptr<PDO> ppdo,
                      DataReader rdr,
                      DS301 *node):
            ppdo(ppdo),reader(rdr),node(node),readMappings(0){};
        PDODataReader(std::tr1::shared_ptr<PDO> ppdo,
                      IndexedDataReader rdr,
                      int index):
            ppdo(ppdo),ireader(rdr),readMappings(index){};
        void operator()(SDO &sdo){
            if(readMappings<0) {
                ((*ppdo).*reader)(sdo.data);
            } else if(readMappings==0) {
                ((*ppdo).*reader)(sdo.data);
                for(int i=0;i<ppdo->objectCount;i++) {
                    std::tr1::shared_ptr<SDOCallbackObject> pcb(new PDODataReader(ppdo,
                                static_cast<PDODataReader::IndexedDataReader>(&PDO::readMappingData),
                                i+1));
                    node->readObjectDictionary(ppdo->mappingIndex(), i+1, pcb);
                }
            } else if(readMappings>0) {
                ((*ppdo).*ireader)(readMappings, sdo.data);
            }
        };

    private:
        std::tr1::shared_ptr<PDO> ppdo;
        DataReader reader;
        IndexedDataReader ireader;
        int readMappings;
        DS301 *node;
};

void DS301::readPDO(std::tr1::shared_ptr<PDO> ppdo)
{
    std::tr1::shared_ptr<PDODataReader> prdr_c(new PDODataReader(ppdo,
                static_cast<PDODataReader::DataReader>(&PDO::readCommunicationData)));
    readObjectDictionary(ppdo->communicationIndex(), 1, prdr_c);
    std::tr1::shared_ptr<PDODataReader> prdr_ct(new PDODataReader(ppdo,
                static_cast<PDODataReader::DataReader>(&PDO::readCommunicationTypeData)));
    readObjectDictionary(ppdo->communicationIndex(), 2, prdr_ct);
    std::tr1::shared_ptr<PDODataReader> prdr_m(new PDODataReader(ppdo,
                static_cast<PDODataReader::DataReader>(&PDO::readMappingCount),
                this));
    readObjectDictionary(ppdo->mappingIndex(), 0, prdr_m);
}

std::tr1::shared_ptr<TPDO> DS301::createTPDO(std::string name,
                                            int pdo_number,
                                            PDOCallbackObject callback)
{
    std::tr1::shared_ptr<TPDO> ppdo(new TPDO(name,
                                            node_id,
                                            pdo_number,
                                            callback));
    readPDO(ppdo);
    bus->add(ppdo);
    return ppdo;
}

std::tr1::shared_ptr<RPDO> DS301::createRPDO(std::string name,
                                            int pdo_number
                                           )
{
    std::tr1::shared_ptr<RPDO> ppdo(new RPDO(name,
                                            node_id,
                                            pdo_number));
    readPDO(ppdo);
    bus->add(ppdo);
    return ppdo;
}

void DS301::mapPDO(std::tr1::shared_ptr<PDO> ppdo,
                   std::vector<struct PDOMap> mapped,
                   uint8_t type
                  )
{
    ppdo->mapped = false;
    bus->add(ppdo);
    ppdo->objectCount = mapped.size();
    for(int i=0; i<mapped.size(); i++){
        ppdo->mappedObjects[i] = mapped[i];
    }
    ppdo->communication_parameters.identifier = ppdo->COBID();
    ppdo->communication_parameters.extended = false;
    ppdo->communication_parameters.rtr = false;
    ppdo->communication_parameters.invalid = false;
    ppdo->communication_parameters.type = type;

    std::vector<uint8_t> data;
    // Disable PDO
    data.push_back(0);
    writeObjectDictionary(ppdo->mappingIndex(), 0, data);
    // write map
    for(int i=0;i<ppdo->objectCount;i++) {
        data.clear();
        ppdo->writeMappingData(i, data);
        writeObjectDictionary(ppdo->mappingIndex(), i+1, data);
    }
    // write comm params
    data.clear();
    ppdo->writeCommunicationData(data);
    writeObjectDictionary(ppdo->communicationIndex(), 1, data);
    data.clear();
    ppdo->writeCommunicationTypeData(data);
    writeObjectDictionary(ppdo->communicationIndex(), 2, data);
    // enable PDO
    data.clear();
    data.push_back(ppdo->objectCount);
    writeObjectDictionary(ppdo->mappingIndex(), 0, data,
            std::tr1::shared_ptr<SDOCallbackObject> (new 
            SDOCallbackObject(static_cast<std::tr1::shared_ptr<TransferCallbackReceiver> >(ppdo),
                static_cast<SDOCallbackObject::CallbackFunction>(&PDO::mappingComplete))));

}

std::tr1::shared_ptr<RPDO> DS301::mapRPDO(std::string name,
                                          int pdo_number,
                                          std::vector<struct PDOMap> mapped,
                                          uint8_t type
                                         ) 
{
    std::tr1::shared_ptr<RPDO> ppdo(new RPDO(name,
                                            node_id,
                                            pdo_number));
    mapPDO(ppdo, mapped, type);
    return ppdo;
}

std::tr1::shared_ptr<TPDO> DS301::mapTPDO(std::string name,
                                   int pdo_number,
                                   std::vector<struct PDOMap> mapped,
                                   uint8_t type,
                                   PDOCallbackObject callback
                                  )
{
    std::tr1::shared_ptr<TPDO> ppdo(new TPDO(name,
                                             node_id,
                                             pdo_number,
                                             callback));
    mapPDO(ppdo, mapped, type);
    return ppdo;
}

void DS301::setPDOType(std::tr1::shared_ptr<PDO> ppdo, uint8_t type)
{
    std::vector<uint8_t> data;
    data.push_back(type);
    writeObjectDictionary(ppdo->communicationIndex(), 2, data);
}

void DS301::setSyncInterval(uint32_t interval_us)
{
    std::vector<uint8_t> data;
    data.push_back((interval_us&0x000000FF)>> 0);
    data.push_back((interval_us&0x0000FF00)>> 8);
    data.push_back((interval_us&0x00FF0000)>>16);
    data.push_back((interval_us&0xFF000000)>>24);
    writeObjectDictionary(0x1006, 0x0, data);
}

void DS301::enableSync(bool enable)
{
    std::vector<uint8_t> data;
    data.push_back(0x80);
    data.push_back(0x00);
    data.push_back(0x00);
    data.push_back(enable?0x40:0x0);
    writeObjectDictionary(0x1005, 0x0, data);
}
}
