#ifndef __SDO_H__
#define __SDO_H__

namespace CANOpen
{

class SDO;
typedef CallbackObject<SDO> SDOCallbackObject;

class SDO : public Transfer {
    public:
        SDO(std::string name,
            unsigned long node_id,
            SDOCallbackObject callback=SDOCallbackObject(),
            int clientCOB=0x580,
            int serverCOB=0x600
           );
        uint16_t COBID();
        void read(uint16_t index, uint8_t subindex);
        void read(uint16_t index, uint8_t subindex,
                SDOCallbackObject callback);
        void write(uint16_t index, uint8_t subindex, std::vector<uint8_t> &bytes);
        void write(uint16_t index, uint8_t subindex, std::vector<uint8_t> &bytes,
                SDOCallbackObject callback);
        void abort(uint32_t code=0xFFFFFFFF);
        const char *abortString(void);
        uint32_t abortCode(void);
        bool isError(void);
        bool handleMessage(const Message m);
        bool inProgress(void){return _handleMessage != &SDO::_processMessageUnitializedError;};

        // CANOpen Object address
        uint16_t index;
        uint8_t subindex;

        // local data storage
        // array is allocated here for received data
        // and data to be sent is copied here
        // this array is always allocated and deleted by this
        // object
        std::vector <uint8_t> data;
    private:
        void doCallback();
        void doError(void);

        std::vector<SDOCallbackObject> callbacks;

        uint16_t clientCOB_ID, serverCOB_ID;
        // number of bytes expected for the transfer
        // also, always the size of the array allocated
        // for data
        uint32_t byte_count;
        // number of bytes received/transmitted
        uint32_t transfered_count;
        // Set by abort function to abort a domain transfer
        // or records the abort code when the remote end aborts
        uint32_t abort_code;
        bool error;
        // Should a CRC be computed fro block transfers?
        //bool CRCSupported;
        // Expected number of blocks in a block transfer
        //uint32_t block_count;
        // current segments/block;
        //uint32_t block_size;
        // last successful sequence number
        // If a sequence number is skipped, this flag stays
        // at the last value
        //uint32_t last_seqno;
        // function pointer for current processing function
        bool (SDO::*_handleMessage)(Message m);

        // message processing functions
        // dummy function which always returns error
        // pointer is resset to this when a transaction is finished
        bool _processMessageUnitializedError(Message m);
        // upload
        bool _processInitiateDomainUpload(Message m);
        bool _processUploadDomainSegment(Message m);
        // download
        bool _processInitiateDomainDownload(Message message);
        bool _processInitiateExpeditedDomainDownload(Message message);
        bool _processDownloadDomainSegment(Message message);
        bool _processCompleteDownloadDomainSegment(Message message);
        /*
        enum TransferHandleAction _processInitiateBlockUpload(Message m);
        enum TransferHandleAction _processUploadBlockSegment(Message message);
        */
        bool _createAbortMessage(Message m);

};

}
#endif // __SDO_H__
