#include "ns3/core-module.h"
#include "ns3/oran-interface.h"

extern "C" {
  // #include "OCUCP-PF-Container.h"
  #include "OCTET_STRING.h"
  #include "asn_application.h"
  // #include "E2SM-KPM-IndicationMessage.h"
  // #include "FQIPERSlicesPerPlmnListItem.h"
  // #include "E2SM-KPM-RANfunction-Description.h"
  // #include "E2SM-KPM-IndicationHeader-Format1.h"
  // #include "E2SM-KPM-IndicationHeader.h"
  // #include "Timestamp.h"
  #include "E2AP-PDU.h"
  #include "RICsubscriptionRequest.h"
  #include "RICsubscriptionResponse.h"
  #include "RICactionType.h"
  #include "ProtocolIE-Field.h"
  #include "ProtocolIE-SingleContainer.h"
  #include "InitiatingMessage.h"
  #include "E2SM-RC-ControlMessage-Format1.h"

    #include "RANParameter-ValueType.h"
    #include "RANParameter-ELEMENT.h"
    #include "RANParameter-LIST.h"
    #include "RANParameter-STRUCTURE.h"
    #include "INTEGER.h"
}

#include "e2sim.hpp"

// #include "ns3/cell-handover-control-message.h"

using namespace ns3;

RANParameter_ValueType_t *
GetItemValueType(unsigned long _value){
    RANParameter_ELEMENT_t* elem = (RANParameter_ELEMENT_t *) calloc(1, sizeof(RANParameter_ELEMENT_t));
    elem->keyFlag = 0;
    elem->ranParameter_Value.present = RANParameter_Value_PR_valueInt;
    elem->ranParameter_Value.choice.valueInt = _value;

    RANParameter_ValueType_t * valueType = (RANParameter_ValueType_t *) calloc(1, sizeof(RANParameter_ValueType_t));
    valueType->present = RANParameter_ValueType_PR_ranParameter_Element;
    valueType->choice.ranParameter_Element = elem;
    
    return valueType;
}

RANParameter_Item_t *
GetItem(unsigned long _value, unsigned long ranId){

    RANParameter_ValueType_t * valueType = GetItemValueType(_value);

    auto *item = (RANParameter_Item_t *) calloc(1, sizeof(RANParameter_Item_t));
    item->ranParameterItem_ID = ranId;
    item->ranParameterItem_valueType = valueType;

    return item;
}


RANParameter_STRUCTURE_t*
GetUeIdDestinationCellIdStructure(unsigned long ueId, unsigned long destCellId){
    // ue id & destination cell id
    RANParameter_Item_t * ueIdItem = GetItem(ueId, 12);
    RANParameter_Item_t * cellIdItem = GetItem(destCellId, 10);
    // inserting in a structure
    RANParameter_STRUCTURE_t* ueIdDestCellStructure = (RANParameter_STRUCTURE_t* )calloc(1, sizeof(RANParameter_STRUCTURE_t));
    
    ASN_SEQUENCE_ADD(&ueIdDestCellStructure->sequence_of_ranParameters.list, ueIdItem);
    ASN_SEQUENCE_ADD(&ueIdDestCellStructure->sequence_of_ranParameters.list, cellIdItem);

    return ueIdDestCellStructure;
}

RANParameter_STRUCTURE_t*
GetSingleCellHandoverMessages(unsigned long sourceCellId, std::map<unsigned long, unsigned long> ueIdDestiCellIdMap){
    RANParameter_Item_t * sourceCellItem = GetItem(sourceCellId, 10);
    RANParameter_STRUCTURE_t* sourceCellStructure = (RANParameter_STRUCTURE_t* )calloc(1, sizeof(RANParameter_STRUCTURE_t));
    ASN_SEQUENCE_ADD(&sourceCellStructure->sequence_of_ranParameters.list, sourceCellItem);

    // RANParameter_ValueType_t * sourceCellIdValueType = GetItemValueType(sourceCellId);
    for (auto mapIt = ueIdDestiCellIdMap.begin(); mapIt!= ueIdDestiCellIdMap.end(); ++mapIt){
        RANParameter_STRUCTURE_t* ueDestCellStruct = GetUeIdDestinationCellIdStructure(mapIt->first, mapIt->second);
        RANParameter_ValueType_t* ueDestCellStructValueType = (RANParameter_ValueType_t* )calloc(1, sizeof(RANParameter_ValueType_t));
        ueDestCellStructValueType->present = RANParameter_ValueType_PR_ranParameter_Structure;
        ueDestCellStructValueType->choice.ranParameter_Structure = ueDestCellStruct;
        RANParameter_Item_t* ueDestCellStructItem = (RANParameter_Item_t* )calloc(1, sizeof(RANParameter_Item_t));
        ueDestCellStructItem->ranParameterItem_ID = 10;
        ueDestCellStructItem->ranParameterItem_valueType = ueDestCellStructValueType;
        ASN_SEQUENCE_ADD(&sourceCellStructure->sequence_of_ranParameters.list, ueDestCellStructItem);
    }
    return sourceCellStructure;
}

int 
main_old (int argc, char *argv[])
{

    E2SM_RC_ControlMessage_t* controlMessage = new E2SM_RC_ControlMessage_t;

    E2SM_RC_ControlMessage_Format1_t* controlMessageFormat1 = new E2SM_RC_ControlMessage_Format1_t;
    
    controlMessage->present = E2SM_RC_ControlMessage_PR_controlMessage_Format1;
    controlMessage->choice.controlMessage_Format1 = controlMessageFormat1;

    controlMessageFormat1->ranParameters_List = (E2SM_RC_ControlMessage_Format1_t::E2SM_RC_ControlMessage_Format1__ranParameters_List*) 
                                   calloc (1, sizeof (E2SM_RC_ControlMessage_Format1_t::E2SM_RC_ControlMessage_Format1__ranParameters_List));
    

    auto *ranParameter_Element = (RANParameter_ELEMENT_t *) calloc(1, sizeof(RANParameter_ELEMENT_t));
    ranParameter_Element->keyFlag=0;
    ranParameter_Element->ranParameter_Value.present = RANParameter_Value_PR_valueInt;
    ranParameter_Element->ranParameter_Value.choice.valueInt = 100;

    std::map<unsigned long, unsigned long> ueIdDestiCellIdMap;
    ueIdDestiCellIdMap.insert(std::pair(1,2));
    // ueIdDestiCellIdMap.insert(std::pair(3,4));
    // ueIdDestiCellIdMap.insert(std::pair(6,7));

    auto ueIdDestCellStructure = GetSingleCellHandoverMessages(10, ueIdDestiCellIdMap);
    // auto ueIdDestCellStructure = GetUeIdDestinationCellIdStructure(10, 8);

    auto *ranParameterValueType = (RANParameter_ValueType_t *) calloc(1, sizeof(RANParameter_ValueType_t));
    ranParameterValueType->present = RANParameter_ValueType_PR_ranParameter_Structure;
    ranParameterValueType->choice.ranParameter_Structure = ueIdDestCellStructure;

    auto *ranParameterItem = (RANParameter_Item_t *) calloc(1, sizeof(RANParameter_Item_t));
    ranParameterItem->ranParameterItem_ID = 12;
    ranParameterItem->ranParameterItem_valueType = ranParameterValueType;

    ASN_SEQUENCE_ADD(&controlMessageFormat1->ranParameters_List->list, ranParameterItem);

    // int ret = asn_check_constraints(&asn_DEF_E2SM_RC_ControlMessage_Format1, controlMessage, errbuff, &errlen);
    
    xer_fprint(stderr, &asn_DEF_E2SM_RC_ControlMessage, controlMessage);
    // xer_fprint(stderr, &asn_DEF_E2SM_RC_ControlMessage_Format1, controlMessageFormat1);
    // xer_fprint(stderr, &asn_DEF_RANParameter_Item, ranParameterItem);
    // xer_fprint(stderr, &asn_DEF_RANParameter_ELEMENT, ranParameter_Element);
    // xer_fprint(stderr, &asn_DEF_RANParameter_ValueType, ranParameterValueType);
    return 0;
}

int 
main (int argc, char *argv[])
{
    // std::string bytes = "\0\x06\x80\x01d\x81\x01\n";
    // std::string bytes = "\0068001d8101";
    // std::string bytes = "06801d8101";
    std::string bytes = "01010102";
    // std::string bytes("\x06\x80\x01d\x81\x01");
    std::cout << bytes << std::endl;

    // char myChar[] = { 0x06, 0x80,  0x1d, 0x81, 0x01};

    // INTEGER_t ueId = 

    // INTEGER_t *ueId = (INTEGER_t *) calloc(1, sizeof(INTEGER_t));
    // ueId

    CellHandoverItem_t *pdu = new CellHandoverItem_t;
    pdu->ueId = 10;
    pdu->destinationCellId = 10;

    // E2SM_KPM_RANfunction_Description_t *descriptor = new E2SM_KPM_RANfunction_Description_t ();

    asn_codec_ctx_t *opt_cod = 0; // disable stack bounds checking
  // encode the structure into the e2smbuffer
    asn_encode_to_new_buffer_result_s encodedMsg = asn_encode_to_new_buffer (
      opt_cod, ATS_ALIGNED_BASIC_PER, &asn_DEF_CellHandoverItem, pdu);
    
    std::cout << "Encoded bytes " << encodedMsg.result.encoded << std::endl;
    // }
    // printf("%d", encodedMsg.buffer);
    // printf("%p\n",  encodedMsg.buffer);
    uint8_t *b = (uint8_t*) encodedMsg.buffer;
    int j;
    for(j = 0; j < encodedMsg.result.encoded; ++j)
        printf("%02x\n", ((uint8_t*) b)[j]);
    // for (int i =0; i < encodedMsg.result.encoded; i++){
    //     std::cout << encodedMsg.buffer[i];
    // }

	// auto retval = asn_decode(nullptr, ATS_ALIGNED_BASIC_PER, &asn_DEF_CellHandoverItem, (void **) &pdu, (void *)myChar, bytes.size());
	// auto retval = asn_decode(nullptr, ATS_ALIGNED_BASIC_PER, &asn_DEF_CellHandoverItem, (void **) &pdu, (void *)bytes.c_str(), bytes.size());
    // print decoded payload
	// if (retval.code == RC_OK) {
    //     std::cout << "decod ok " << std::endl;
    //     // std::cout << "ue id " <<  pdu->ueId << std::endl;
    //     // std::cout << "destination cell id " <<  pdu->destinationCellId << std::endl;
    //     std::cout << "ue id " <<  pdu->ueId.size << std::endl;
    //     std::cout << "ue id " <<  (*pdu->ueId.buf) << std::endl;

    // }
}