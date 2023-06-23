// #include <mdclog/mdclog.h>
#include <vector>
#include <iostream>
#include <list>
#include <set>
#include <algorithm>
#include <memory>
#include <map>

#include "control_message_encoder_decoder.h"

namespace ns3{

namespace ric_control {

template<typename T>
std::vector<int> findItems(std::vector<T> const &v, int target) {
    std::vector<int> indices;
    auto it = v.begin();
    while ((it = std::find_if(it, v.end(), [&] (T const &e) { return e == target; }))
        != v.end())
    {
        indices.push_back(std::distance(v.begin(), it));
        it++;
    }
    return indices;
}

CellHandoverItem_t* create_handover_item(long ueId, long destinationCellId){
    CellHandoverItem_t* control_message = (CellHandoverItem_t *) calloc(1, sizeof(CellHandoverItem_t));
    control_message->ueId = ueId;
    control_message->destinationCellId = destinationCellId;
    return control_message;
}

CellHandoverItemList_t* create_handover_item_list(std::list<CellHandoverItem_t*> handoverItems){
    CellHandoverItemList_t* cellHandoverList = (CellHandoverItemList_t *) calloc(1, sizeof(CellHandoverItemList_t));
    for (auto it = handoverItems.begin(); it != handoverItems.end(); ++it){
        ASN_SEQUENCE_ADD(&cellHandoverList->list, (*it));
    }
    return cellHandoverList;
}

int e2ap_asn1c_encode_handover_item(CellHandoverItem_t* pdu, unsigned char **buffer)
{
    int len;

    *buffer = NULL;
    assert(pdu != NULL);
    assert(buffer != NULL);

    return aper_encode_to_new_buffer(&asn_DEF_CellHandoverItem, 0, pdu, (void **)buffer);

    // len = aper_encode_to_new_buffer(&asn_DEF_CellHandoverItem, 0, pdu, (void **)buffer);

    if (len < 0) {
        // mdclog_write(MDCLOG_INFO,"[E2AP ASN] Unable to aper encode");
    } else {
        // mdclog_write(MDCLOG_INFO, "[E2AP ASN] Encoded succesfully, encoded size = %d", len);
        xer_fprint(stderr, &asn_DEF_CellHandoverItem, pdu);
    }

    // ASN_STRUCT_RESET(asn_DEF_CellHandoverItem, pdu);
    // ASN_STRUCT_FREE_CONTENTS_ONLY(asn_DEF_CellHandoverItem, pdu);

    // return len;
}

int e2ap_asn1c_encode_all_handovers_item_list(CellHandoverItemList_t* pdu, unsigned char **buffer){
    int len;

    *buffer = NULL;
    assert(pdu != NULL);
    assert(buffer != NULL);

    len = aper_encode_to_new_buffer(&asn_DEF_CellHandoverItemList, 0, pdu, (void **)buffer);

    if (len < 0) {
        // mdclog_write(MDCLOG_INFO,"[E2AP ASN] Unable to aper encode");
    } else {
        // mdclog_write(MDCLOG_INFO, "[E2AP ASN] Encoded succesfully, encoded size = %d", len);
        xer_fprint(stderr, &asn_DEF_CellHandoverItemList, pdu);
    }

    return len;
}

int e2ap_asn1c_encode_cell_handovers(CellHandoverList_t* pdu, unsigned char **buffer)
{
    int len;

    *buffer = NULL;
    assert(pdu != NULL);
    assert(buffer != NULL);

    len = aper_encode_to_new_buffer(&asn_DEF_CellHandoverList, 0, pdu, (void **)buffer);

    if (len < 0) {
        // mdclog_write(MDCLOG_INFO,"[E2AP ASN] Unable to aper encode");
    } else {
        // mdclog_write(MDCLOG_INFO, "[E2AP ASN] Encoded succesfully, encoded size = %d", len);
        xer_fprint(stderr, &asn_DEF_CellHandoverList, pdu);
    }

    return len;
}

int e2ap_asn1c_encode_all_handovers(AllHandoversList_t* pdu, unsigned char **buffer)
{
    int len;

    *buffer = NULL;
    assert(pdu != NULL);
    assert(buffer != NULL);

    len = aper_encode_to_new_buffer(&asn_DEF_AllHandoversList, 0, pdu, (void **)buffer);

    if (len < 0) {
        // mdclog_write(MDCLOG_INFO,"[E2AP ASN] Unable to aper encode");
    } else {
        // mdclog_write(MDCLOG_INFO, "[E2AP ASN] Encoded succesfully, encoded size = %d", len);
        xer_fprint(stderr, &asn_DEF_AllHandoversList, pdu);
    }

    return len;
}

struct asn_dec_rval_s e2ap_asn1c_decode_handover_item(CellHandoverItem_t *pdu, enum asn_transfer_syntax syntax, unsigned char *buffer, int len) {
    asn_dec_rval_t dec_ret;
    assert(buffer != NULL);

    dec_ret = asn_decode(NULL, syntax, &asn_DEF_CellHandoverItem, (void **) &pdu, buffer, len);
    if (dec_ret.code != RC_OK) {
        // mdclog_write(MDCLOG_ERR,"[E2AP ASN] Failed to decode pdu");
        // exit(EXIT_FAILURE);
    } else {
        // mdclog_write(MDCLOG_INFO, "[E2AP ASN] Decoded successfully");
        return dec_ret;
    }
    return dec_ret;
}

std::map<long, std::map<long, long>> 
extract_handover_map(AllHandoversList_t* pdu){
    std::map<long, std::map<long, long>> allHandovers;
    for(int idx = 0; idx < pdu->list.count; ++idx){
        CellHandoverList* cellHandoverList = (CellHandoverList*) pdu->list.array[idx];
        long cellId = cellHandoverList->sourceCellId;
        CellHandoverItemList_t* cellHandoverListItems = (CellHandoverItemList_t*) cellHandoverList->cellHandoverItemList;
        std::map <long, long> ueIdDestinationCellMap;
        for(int cellHandoverIdx = 0; cellHandoverIdx < cellHandoverListItems->list.count; ++cellHandoverIdx){
            CellHandoverItem_t* singleHandover  = (CellHandoverItem_t*)cellHandoverListItems->list.array[cellHandoverIdx];
            long ueId = singleHandover->ueId;
            long relayRnti = singleHandover->destinationCellId;
            ueIdDestinationCellMap.insert(std::pair<long, long>(ueId, relayRnti));
        }
        allHandovers.insert(std::pair<long, std::map<long, long>> (cellId, ueIdDestinationCellMap));
    }

    return allHandovers;
}

std::map<long, std::map<long, long>> 
decode_handover_control_message(uint8_t* buffer, size_t buffSize){
    E2SM_RC_ControlMessage_t* rcControlMessage = new E2SM_RC_ControlMessage_t;
    auto retval = asn_decode(nullptr, ATS_ALIGNED_BASIC_PER, &asn_DEF_E2SM_RC_ControlMessage, (void **) &rcControlMessage, (void *)buffer, buffSize);
    std::map<long, std::map<long, long>> allHandovers;
    // auto retval = asn_decode(nullptr, ATS_ALIGNED_BASIC_PER, &asn_DEF_AllHandoversList, (void **) &pdu, (void *)buffer, buffSize);
    if (retval.code == RC_OK) {
        if(rcControlMessage->present = E2SM_RC_ControlMessage_PR_handoverMessage_Format){
            AllHandoversList_t* pdu = rcControlMessage->choice.handoverMessage_Format;
            allHandovers = extract_handover_map(pdu);
        }
    }
    delete rcControlMessage;
    return allHandovers;
}

sctp_buffer_t* gnerate_e2ap_encode_handover_control_message(uint16_t* ue_id, uint16_t* start_position, uint16_t* optimized, size_t size){
    
    std::vector<long> ue_id_vec (size);
    std::vector<long> start_position_vec (size);
    std::vector<long> optimized_vec (size);
    for(int _ind = 0; _ind<size; ++_ind){
        ue_id_vec[_ind] = (ue_id[_ind]);
        start_position_vec[_ind] = (start_position[_ind]);
        optimized_vec[_ind] = (optimized[_ind]);
    }

    std::set<long> sourceCellIdSet;
    for (long x: start_position_vec){
        sourceCellIdSet.insert(x);
    }

    AllHandoversList_t* allHandoversList = (AllHandoversList_t *) calloc(1, sizeof(AllHandoversList_t));

    for (long sourceCellId: sourceCellIdSet){
        CellHandoverList_t* cellHandovers = (CellHandoverList_t *) calloc(1, sizeof(CellHandoverList_t));
        cellHandovers->sourceCellId = sourceCellId;
        // find items in the starting vec from the set
        std::vector<int> indices = findItems(start_position_vec, sourceCellId);
        std::list<CellHandoverItem_t*> handoverItems;
        for (int index : indices){
            long _ue_ind = ue_id_vec.at(index);
            long _dst_cell_id = optimized_vec.at(index);
            CellHandoverItem_t* control_message = create_handover_item(_ue_ind, _dst_cell_id);
            handoverItems.push_back(control_message);
        }
        CellHandoverItemList_t* cellHandoverList = create_handover_item_list(handoverItems);
        cellHandovers->cellHandoverItemList = cellHandoverList;
        ASN_SEQUENCE_ADD(&allHandoversList->list, cellHandovers);
    }

    // create E2SM
    // this is to keeep compatability with decoding in ns3
    E2SM_RC_ControlMessage_t* rcControlMessage = (E2SM_RC_ControlMessage_t *) calloc(1, sizeof(E2SM_RC_ControlMessage_t));
    rcControlMessage->present = E2SM_RC_ControlMessage_PR_handoverMessage_Format;
    rcControlMessage->handoverMessage_Format = allHandoversList;

    uint8_t *buf;
    sctp_buffer_t* data = (sctp_buffer_t *) calloc(1, sizeof(sctp_buffer_t));

    // data->length = e2ap_asn1c_encode_all_handovers_item_list(cellHandoverList, &buf);
    // data->length = e2ap_asn1c_encode_cell_handovers(cellHandovers, &buf);
    // data->length = e2ap_asn1c_encode_all_handovers(allHandoversList, &buf);
    data->length = e2ap_asn1c_encode_control_message(rcControlMessage, &buf);
    // printf( "Data length %d", data->length);
    data->buffer = (uint8_t *) calloc(1, data->length);
    memcpy(data->buffer, buf, std::min(data->length, MAX_SCTP_BUFFER));
    // data->buffer = buf;
    // printf( "Data length %d", data->length);

    delete allHandoversList;
    return data;
}

}

}
