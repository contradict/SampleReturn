#include <algorithm>
#include <vector>
#include <stdint.h>
#include <tr1/memory>

#include <Message.h>
#include <Transfer.h>
#include <Callbacks.h>
#include <SDO.h>

// Max number of bytes that can be tranfered in a single
// message, used to calcualte the expected number of messages
// in a block transfer
#define SDO_MAX_BYTES                 7
#define SDO_COMMAND_SPECIFIER_SHIFT   5
#define SDO_COMMAND_SPECIFIER_MASK    0xE0

// domain download
#define SDO_INITIATE_DOMAIN_DOWNLOAD  (1<<SDO_COMMAND_SPECIFIER_SHIFT)
#define SDO_ACK_DOMAIN_DOWNLOAD       (3<<SDO_COMMAND_SPECIFIER_SHIFT)
#define SDO_SEGMENT_DOMAIN_DOWNLOAD   (0<<SDO_COMMAND_SPECIFIER_SHIFT)
#define SDO_ACK_SEGMENT_DOMAIN_DOWNLOAD (1<<SDO_COMMAND_SPECIFIER_SHIFT)

// domain upload
#define SDO_INITIATE_DOMAIN_UPLOAD    (2<<SDO_COMMAND_SPECIFIER_SHIFT)
#define SDO_ACK_DOMAIN_UPLOAD         (2<<SDO_COMMAND_SPECIFIER_SHIFT)
#define SDO_REQ_SEGMENT_DOMAIN_UPLOAD (3<<SDO_COMMAND_SPECIFIER_SHIFT)
#define SDO_SEGMENT_DOMAIN_UPLOAD     (0<<SDO_COMMAND_SPECIFIER_SHIFT)

// abort either direction
#define SDO_ABORT_DOMAIN_TRANSFER     (4<<SDO_COMMAND_SPECIFIER_SHIFT)

// block download
#define SDO_INITIATE_BLOCK_DOWNLOAD   (6<<SDO_COMMAND_SPECIFIER_SHIFT)
#define SDO_ACK_BLOCK_DOWNLOAD        (5<<SDO_COMMAND_SPECIFIER_SHIFT)
#define SDO_SEGMENT_BLOCK_DOWNLOAD    (5<<SDO_COMMAND_SPECIFIER_SHIFT)
#define SDO_END_BLOCK_DOWNLOAD        (6<<SDO_COMMAND_SPECIFIER_SHIFT)
#define SDO_ACK_END_BLOCK_DOWNLOAD    (5<<SDO_COMMAND_SPECIFIER_SHIFT)

// block upload
#define SDO_INITIATE_BLOCK_UPLOAD     (5<<SDO_COMMAND_SPECIFIER_SHIFT)
#define SDO_BLOCK_UPLOAD_START        (3)
#define SDO_BLOCK_UPLOAD_NEXT         (2)
#define SDO_ACK_BLOCK_UPLOAD          (6<<SDO_COMMAND_SPECIFIER_SHIFT)
#define SDO_SEGMENT_BLOCK_UPLOAD      (6<<SDO_COMMAND_SPECIFIER_SHIFT)
#define SDO_END_BLOCK_UPLOAD          (6<<SDO_COMMAND_SPECIFIER_SHIFT)
#define SDO_ACK_END_BLOCK_UPLOAD      (5<<SDO_COMMAND_SPECIFIER_SHIFT)

#define SDO_SET_EXPEDITED_COUNT(n)   (((n)&0x03)<<2)
#define SDO_GET_EXPEDITED_COUNT(SCS) (4-((SCS&0x0C)>>2))
#define SDO_SET_SEGMENT_COUNT(n)     (((7-n)&0x07)<<1)
#define SDO_GET_SEGMENT_COUNT(SCS)   (7-((SCS&0x0E)>>1))
#define SDO_EXPEDITED_FLAG 0x02
#define SDO_DOMAIN_SIZE_FLAG 0x01
#define SDO_BLOCK_SIZE_FLAG 0x02
#define SDO_CRC_SUPPORT 0x04
#define SDO_DOMAIN_LAST_FLAG 0x01
#define SDO_SEGMENT_TOGGLE_FLAG 0x10
#define SDO_BLOCK_LAST_FLAG 0x80
#define SDO_BLOCK_SEQNO_MASK 0x7f


#define SDO_GET_32(msg) (( (uint32_t)msg.data[4]) |\
                         (((uint32_t)msg.data[5])<<8) |\
                         (((uint32_t)msg.data[6])<<16) |\
                         (((uint32_t)msg.data[7])<<24))

namespace CANOpen {
struct abortdescr {
    uint32_t value;
    const char *name;
};
static const struct abortdescr AbortCodes[] = {
    {0x00000000, "Unknown Error" },
    {0x05030000, "Toggle bit not alternated" },
    {0x05040000, "SDO protocol timed out" },
    {0x05040001, "Client/Server command specifier not valid or unknown" },
    {0x05040002, "Invalid block size (Block Transfer mode only)" },
    {0x05040003, "Invalid sequence number (Block Transfer mode only)" },
    {0x05030004, "CRC error (Block Transfer mode only)" },
    {0x05030005, "Out of memory" },
    {0x06010000, "Unsupported access to an object" },
    {0x06010001, "Attempt to read a write-only object" },
    {0x06010002, "Attempt to write a read-only object" },
    {0x06020000, "Object does not exist in the Object Dictionary" },
    {0x06040041, "Object can not be mapped to the PDO" },
    {0x06040042, "The number and length of the objects to be mapped would exceed PDO length" },
    {0x06040043, "General parameter incompatibility reason" },
    {0x06040047, "General internal incompatibility in the device" },
    {0x06060000, "Object access failed due to a hardware error" },
    {0x06060010, "Data type does not match, lengh of service parameter does not match" },
    {0x06060012, "Data type does not match, lengh of service parameter is too high" },
    {0x06060013, "Data type does not match, lengh of service parameter is too low" },
    {0x06090011, "Sub-index does not exist" },
    {0x06090030, "Value range of parameter exceeded (only for write access)" },
    {0x06090031, "Value of parameter written too high" },
    {0x06090032, "Value of parameter written too low" },
    {0x06090036, "Maximum value is less than minimum value" },
    {0x08000000, "General error" },
    {0x08000020, "Data can not be transferred or stored to the application" },
    {0x08000021, "Data can not be transferred or stored to the application because of local control" },
    {0x08000022, "Data can not be transferred or stored to the application because of the present device state" },
    {0x08000023, "Object Dictionary dynamic generation fails or no Object Dictionary is present (e.g. OD is generated from file and generation fails because of a file error)" },
    {0x00000000, NULL}
};

static const char *find_error_string(uint32_t error_code)
{
    const char *errstr=AbortCodes[0].name;
    for(const struct abortdescr *abt=AbortCodes+1;
            abt->name;
            abt++)
    {
        if(abt->value == error_code) {
            errstr = abt->name;
            break;
        }
    }
    return errstr;
}

SDO::SDO(
         std::string name,
         unsigned long node_id,
         SDOCallbackObject callback,
         int clientCOB,
         int serverCOB
         ):
Transfer(name, node_id),
serverCOB_ID(serverCOB),
clientCOB_ID(clientCOB),
byte_count(0),
transfered_count(0),
abort_code(0),
error(false),
//block_count(0),
_handleMessage(&SDO::_processMessageUnitializedError)
{
    callbacks.push_back(callback);
}

uint16_t SDO::COBID()
{
    return clientCOB_ID|node_id;
}

void SDO::doCallback(void)
{
    SDOCallbackObject cb = callbacks.back();
    if(callbacks.size()>1) callbacks.pop_back();
    cb(*this);
}

void SDO::doError(void)
{
    error=true;
    SDOCallbackObject cb = callbacks.back();
    if(callbacks.size()>1) callbacks.pop_back();
    cb.error(*this);
}

void SDO::read(uint16_t index, uint8_t subindex,
                SDOCallbackObject callback)
{
    callbacks.push_back(callback);
    read(index, subindex);
}


void SDO::read(uint16_t index, uint8_t subindex)
{
    Message message;

    this->index = index;
    this->subindex = subindex;
    error=false;

    data.clear();
    byte_count = 0;
    transfered_count = 0;

    abort_code = 0;

    message.id = node_id|serverCOB_ID;
    int pos = 0;
    /*
    if(block) {
        m.data[0]  = SDO_INITIATE_BLOCK_UPLOAD |
                     SDO_CRC_SUPPORT;
        block_size = 127;
        m.data[4]  = block_size;
        m.data[5]  = 4;
        _processMessage = &_processInitiateBlockUpload;
    } else {
    */
        pos = message.pack(pos, ((uint8_t)SDO_INITIATE_DOMAIN_UPLOAD));
        _handleMessage = &SDO::_processInitiateDomainUpload;
    //}
    pos = message.pack(pos, index);
    pos = message.pack(pos, subindex);

    //message.dlc = 4;

    send( message );
}

void SDO::write(uint16_t index, uint8_t subindex, std::vector<uint8_t> &bytes,
                SDOCallbackObject callback)
{
    callbacks.push_back(callback);
    write(index, subindex, bytes);
}

void SDO::write(uint16_t index, uint8_t subindex, std::vector<uint8_t> &bytes)
{
    Message message;

    this->index = index;
    this->subindex = subindex;
    error=false;

    byte_count = bytes.size();
    transfered_count = 0;
    data.reserve(byte_count);
    std::copy(bytes.begin(), bytes.end(), data.begin());

    abort_code = 0;

    message.id = node_id|serverCOB_ID;
    int pos = 0;
    pos = message.pack(pos,((uint8_t)(SDO_INITIATE_DOMAIN_DOWNLOAD |
                                      SDO_DOMAIN_SIZE_FLAG)));
    pos = message.pack(pos, index);
    pos = message.pack(pos, subindex);
    if(byte_count<=4) {
        message.data[0] |= SDO_EXPEDITED_FLAG;
        message.data[0] |= (4-byte_count)<<2;
        //message.dlc = 4+byte_count;
        std::copy(bytes.begin(), bytes.end(), &message.data[4]);
        transfered_count += byte_count;
        _handleMessage = &SDO::_processInitiateExpeditedDomainDownload;
    } else {
        /*
        if(block){
        } else {
        */
           pos =  message.pack(pos, byte_count);
        //}
        _handleMessage = &SDO::_processInitiateDomainDownload;
    }
    send( message );
}

void SDO::abort(uint32_t code)
{
    abort_code = code;
}

const char *SDO::abortString(void)
{
    return find_error_string(abort_code);
}

uint32_t SDO::abortCode(void)
{
    return abort_code;
}

bool SDO::isError(void)
{
    return error;
}

bool SDO::handleMessage(const Message m)
{
    if( m.id != COBID() ) return false;
    return (*this.*_handleMessage)(m);
}

bool SDO::_processMessageUnitializedError(Message m)
{
    doError();
    return false;
}

bool SDO::_processInitiateDomainUpload(Message message)
{
    uint8_t command_specifier = message.data[0];

    if((command_specifier&SDO_COMMAND_SPECIFIER_MASK) == SDO_ABORT_DOMAIN_TRANSFER) {
        abort_code = SDO_GET_32(message);
        doError();
        _handleMessage = &SDO::_processMessageUnitializedError;
        return true;
    }
    if((command_specifier&SDO_COMMAND_SPECIFIER_MASK) != SDO_INITIATE_DOMAIN_UPLOAD) {
        //Error
        doError();
        _handleMessage = &SDO::_processMessageUnitializedError;
        return false;
    }
    if(command_specifier & SDO_EXPEDITED_FLAG) {
        byte_count = SDO_GET_EXPEDITED_COUNT(command_specifier);
        data.reserve(byte_count);
        std::copy(&message.data[4], &message.data[4+byte_count], std::back_inserter(data));
        _handleMessage = &SDO::_processMessageUnitializedError;
        doCallback();
        return true;
    } else {
        if(command_specifier & SDO_DOMAIN_SIZE_FLAG) {
            byte_count = SDO_GET_32(message);
            data.reserve(byte_count);
        }
        _handleMessage = &SDO::_processUploadDomainSegment;
        return (*this.*_handleMessage)(message);
    }
}

bool SDO::_createAbortMessage(Message message)
{
    Message response;

    response.id = node_id|serverCOB_ID;
    int pos = 0;
    pos = response.pack(pos, ((uint8_t)SDO_ABORT_DOMAIN_TRANSFER));
    pos = response.pack(pos, abort_code);
    send(response);
    abort_code = 0;
    _handleMessage = &SDO::_processMessageUnitializedError;
    return true;
}

bool SDO::_processUploadDomainSegment(Message message)
{
    uint8_t command_specifier = message.data[0];

    if(abort_code != 0) {
        return _createAbortMessage(message);
    }
    if((command_specifier&SDO_COMMAND_SPECIFIER_MASK) == SDO_INITIATE_DOMAIN_UPLOAD) {
        Message response;
        response.id = node_id|serverCOB_ID;
        response.data[0] = SDO_REQ_SEGMENT_DOMAIN_UPLOAD;
        send(response);
        return true;
    } else if((command_specifier&SDO_COMMAND_SPECIFIER_MASK) == SDO_SEGMENT_DOMAIN_UPLOAD) {
        int msg_bytes = SDO_GET_SEGMENT_COUNT(command_specifier);
        if(byte_count<transfered_count+msg_bytes) {
            data.reserve(transfered_count + msg_bytes);
            byte_count = transfered_count+msg_bytes;
        }
        std::copy(&message.data[1], &message.data[1+msg_bytes], std::back_inserter(data));
        transfered_count += msg_bytes;
        if(message.data[0] & SDO_DOMAIN_LAST_FLAG) {
            _handleMessage = &SDO::_processMessageUnitializedError;
            doCallback();
            return true;
        } else {
            Message response;
            response.id = node_id|serverCOB_ID;
            response.data[0] = (message.data[0]&SDO_SEGMENT_TOGGLE_FLAG)^SDO_SEGMENT_TOGGLE_FLAG;
            // Not necessary since SDO_SEGMENT_DOMAIN_UPLOAD == 0x00
            //message.data[0] &= ~SDO_COMMAND_SPECIFIER_MASK;
            response.data[0] |= SDO_REQ_SEGMENT_DOMAIN_UPLOAD;
            send(response);
            return true;
        }
    } else if((command_specifier&SDO_COMMAND_SPECIFIER_MASK) == SDO_ABORT_DOMAIN_TRANSFER) {
        abort_code = SDO_GET_32(message);
        doError();
        _handleMessage = &SDO::_processMessageUnitializedError;
        return true;
    } else {
        //Error
        doError();
        _handleMessage = &SDO::_processMessageUnitializedError;
        return false;
    }
}

bool SDO::_processInitiateExpeditedDomainDownload(Message message)
{
    uint8_t command_specifier = message.data[0];

    if((command_specifier&SDO_COMMAND_SPECIFIER_MASK) != SDO_ACK_DOMAIN_DOWNLOAD) {
        //Error
        doError();
        _handleMessage = &SDO::_processMessageUnitializedError;
        return false;
    } else {
        _handleMessage = &SDO::_processMessageUnitializedError;
        doCallback();
        return true;
    }
}

bool SDO::_processInitiateDomainDownload(Message message)
{
    uint8_t command_specifier = message.data[0];

    if((command_specifier&SDO_COMMAND_SPECIFIER_MASK) != SDO_ACK_DOMAIN_DOWNLOAD) {
        //Error
        doError();
        _handleMessage = &SDO::_processMessageUnitializedError;
        return false;
    } else {
        _handleMessage = &SDO::_processDownloadDomainSegment;
        return (*this.*_handleMessage)(message);
    }
}

bool SDO::_processDownloadDomainSegment(Message message)
{
    uint8_t command_specifier = message.data[0];

    if(abort_code != 0) {
        return _createAbortMessage(message);
    }
    if( ((command_specifier&SDO_COMMAND_SPECIFIER_MASK) == SDO_ACK_DOMAIN_DOWNLOAD) ||
        ((command_specifier&SDO_COMMAND_SPECIFIER_MASK) == SDO_ACK_SEGMENT_DOMAIN_DOWNLOAD)) {
        Message response;
        response.id = node_id|serverCOB_ID;
        // Use bit fiddling here to preserve t flag
        if((command_specifier&SDO_COMMAND_SPECIFIER_MASK) == SDO_ACK_SEGMENT_DOMAIN_DOWNLOAD) {
            response.data[0] ^= SDO_SEGMENT_TOGGLE_FLAG;
        } else {
            response.data[0] &= ~SDO_SEGMENT_TOGGLE_FLAG;
        }
        response.data[0] &= ~SDO_COMMAND_SPECIFIER_MASK;
        // this value is 0
        //message.data[0] |= SDO_SEGMENT_DOMAIN_DOWNLOAD;
        uint8_t send_count = byte_count - transfered_count;
        if(send_count < SDO_MAX_BYTES){
            response.data[0] |= SDO_SET_SEGMENT_COUNT(send_count);
        } else {
            send_count = SDO_MAX_BYTES;
        }
        std::copy(data.begin()+transfered_count,
                  data.begin()+transfered_count+send_count,
                  &response.data[1]);
        transfered_count += send_count;
        if(transfered_count == byte_count) {
            response.data[0] |= SDO_DOMAIN_LAST_FLAG;
            _handleMessage = &SDO::_processCompleteDownloadDomainSegment;
        } else {
            response.data[0] &= ~SDO_DOMAIN_LAST_FLAG;
        }
        send(response);
        return true;
    } else if((command_specifier&SDO_COMMAND_SPECIFIER_MASK) == SDO_ABORT_DOMAIN_TRANSFER) {
        abort_code = SDO_GET_32(message);
        doError();
        _handleMessage = &SDO::_processMessageUnitializedError;
        return true;
    } else {
        //Error
        doError();
        _handleMessage = &SDO::_processMessageUnitializedError;
        return false;
    }
}

bool SDO::_processCompleteDownloadDomainSegment(Message message)
{
    uint8_t command_specifier = message.data[0];

    if((command_specifier&SDO_COMMAND_SPECIFIER_MASK) == SDO_ACK_SEGMENT_DOMAIN_DOWNLOAD) {
        _handleMessage = &SDO::_processMessageUnitializedError;
        doCallback();
        return true;
    } else {
        //Error
        doError();
        _handleMessage = &SDO::_processMessageUnitializedError;
        return false;
    }
}

/*
bool SDO::_processInitiateBlockUpload(Message &message)
{
    uint8_t command_specifier = message.data[0];

    if((command_specifier&SDO_COMMAND_SPECIFIER_MASK) == SDO_INITIATE_DOMAIN_UPLOAD) {
        return _processInitiateDomainUpload(message);
    } else if((command_specifier&SDO_COMMAND_SPECIFIER_MASK) == SDO_ACK_BLOCK_UPLOAD) {
        CRCSupported = command_specifier & SDO_CRC_SUPPORT;
        if(command_specifier & SDO_BLOCK_SIZE_FLAG) {
            byte_count = SDO_GET_32(message);
            data.reserve(byte_count);
            block_count = byte_count/SDO_MAX_BYTES;
            if(byte_count%SDO_MAX_BYTES>0) block_count++;
            TODO This is certainly wrong
            block_list.resize(block_count);
        }
        message.id = self.serverCOB();
        message.data[0] = SDO_INITIATE_BLOCK_UPLOAD | 
                          SDO_BLOCK_UPLOAD_START;
        _handleMessage = &_processUploadBlockSegment;
        return Send;
    } else {
        //Error
        doError();
        _handleMessage = &SDO::_processMessageUnitializedError;
        return false;
    }
}
*/
/*
bool SDO::_processUploadBlockSegment(Message &message)
{
    uint8_t command_specifier = message.data[0];

    if((command_specifier&SDO_COMMAND_SPECIFIER_MASK) == SDO_SEGMENT_BLOCK_UPLOAD) {
        uint8_t seqno = command_specifier&SDO_BLOCK_SEQNO_MASK;
        bool last = command_specifier&SDO_BLOCK_LAST_FLAG;
        uint32_t msg_bytes=7;
        uint16_t CRC;
        if(last) {
            transfered_count -= ((command_specifier&0x1c)>>2);
            data.reserve(transfered_count);
            if(CRCSupported) {
                CRC = message.data[1] | (message.data[2]<<8);

            }

        }
        if(byte_count<transfered_count+msg_bytes) {
            data.reserve(transfered_count + msg_bytes);
            byte_count = transfered_count+msg_bytes;
        }
        std::copy(&message.data[1], &message.data[1+msg_bytes], std::back_inserter(data));
        if((seqno==0) || ((last_seqno+1) == seqno))
            last_seqno = seqno;
            if( !last ) {
                msg_bytes = 7;
        transfered_count += msg_bytes;
 
        }
        if( (!last) && (seqno == block_size) ) {
            message.id = self.serverCOB();
            messate.data[0] = SDO_SEGMENT_BLOCK_UPLOAD |
                              SDO_BLOCK_UPLOAD_NEXT;
            message.data[1] = last_seqno;
            message.data[2] = block_size;
            return Send;
        }

    } else {
        //Error
        doError();
        _handleMessage = &SDO::_processMessageUnitializedError;
        return false;
    }
}
*/
}
