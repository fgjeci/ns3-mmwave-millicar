/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2022 Northeastern University
 * Copyright (c) 2022 Sapienza, University of Rome
 * Copyright (c) 2022 University of Padova
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation;
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 * Author: Andrea Lacava <thecave003@gmail.com>
 *		   Tommaso Zugno <tommasozugno@gmail.com>
 *		   Michele Polese <michele.polese@gmail.com>
 */
 
#ifndef RIC_CONTROL_MESSAGE_H
#define RIC_CONTROL_MESSAGE_H

#include "ns3/object.h"
#include <ns3/asn1c-types.h>

extern "C" {
  #include "E2AP-PDU.h"
  #include "E2SM-RC-ControlHeader.h"
  #include "E2SM-RC-ControlMessage.h"
  #include "E2SM-RC-ControlHeader-Format1.h"
  #include "E2SM-RC-ControlMessage-Format1.h"
  #include "RICcontrolRequest.h"
  #include "ProtocolIE-Field.h"
  #include "InitiatingMessage.h"
  #include "CellGlobalID.h"
  #include "NRCGI.h"
 }

namespace ns3 {

  class RicControlMessage : public SimpleRefCount<RicControlMessage>
  {
  public:
    enum ControlMessageRequestIdType { TS = 1001, QoS = 1002 };
    RicControlMessage (E2AP_PDU_t *pdu);
    ~RicControlMessage ();

    ControlMessageRequestIdType m_requestType;
    
    static std::vector<RANParameterItem> ExtractRANParametersFromControlMessage (
      E2SM_RC_ControlMessage_Format1_t *e2SmRcControlMessageFormat1);
    
    std::vector<RANParameterItem> m_valuesExtracted;
    RANfunctionID_t m_ranFunctionId;
    RICrequestID_t m_ricRequestId;
    RICcallProcessID_t m_ricCallProcessId;
    E2SM_RC_ControlHeader_Format1_t *m_e2SmRcControlHeaderFormat1;
    std::string GetSecondaryCellIdHO ();

    // modified
    std::map<long, std::map<long, long>> m_allHandoverList;
    std::string m_plmnString;
    enum ControlMessageFormatType {ControlMessage_PR_controlMessage_Format1, ControlMessage_PR_handoverMessage_Format};
    ControlMessageFormatType m_messageFormatType {ControlMessage_PR_handoverMessage_Format};
    // end modification

  private:
    /**
    * Decodes the RIC Control message .
    *
    * \param pdu PDU passed by the RIC
    */
    void DecodeRicControlMessage (E2AP_PDU_t *pdu);
    std::string m_secondaryCellId;
  };
}

#endif /* RIC_CONTROL_MESSAGE_H */
