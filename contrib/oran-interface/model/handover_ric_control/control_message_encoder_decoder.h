#ifndef CONTROL_MESSAGE_ENCODER_DECODER_H
#define CONTROL_MESSAGE_ENCODER_DECODER_H


// #include <mdclog/mdclog.h>
#include <vector>

// extern "C" {
#include "handover_item.h"
#include "all_handovers.h"
#include "cell_handovers_list.h" 
#include "handover_list.h"
// }

#define MAX_SCTP_BUFFER     10000

namespace ns3{
namespace ric_control {

// #ifdef __cplusplus
// extern "C" {
// #endif


  typedef struct sctp_buffer{
    int length;
    // uint8_t buffer[MAX_SCTP_BUFFER];
    uint8_t* buffer;
  } sctp_buffer_t;

  int e2ap_asn1c_encode_handover_item(CellHandoverItem_t* pdu, unsigned char **buffer);

  int e2ap_asn1c_encode_all_handovers_item_list(CellHandoverItemList_t* pdu, unsigned char **buffer);

  int e2ap_asn1c_encode_all_handovers(AllHandoversList_t* pdu, unsigned char **buffer);

  std::map<long, std::map<long, long>> decode_handover_control_message(uint8_t* buffer, size_t buffSize);

  std::map<long, std::map<long, long>> extract_handover_map(AllHandoversList_t* pdu);

  extern struct asn_dec_rval_s e2ap_asn1c_decode_handover_item(CellHandoverItem_t *pdu, enum asn_transfer_syntax syntax, unsigned char *buffer, int len);

  extern sctp_buffer_t* gnerate_e2ap_encode_handover_control_message(uint16_t* ue_id, uint16_t* start_position, uint16_t* optimized, size_t size);


// #ifdef __cplusplus
// }
// #endif

}
}

#endif