

#ifndef	_CELL_ALL_HANDOVERS_H_
#define	_CELL_ALL_HANDOVERS_H_

extern "C" {
#include "asn_application.h"
#include "asn_SEQUENCE_OF.h"
#include "constr_SEQUENCE_OF.h"
#include "constr_SEQUENCE.h"
#include "NativeInteger.h"

}

#include "handover_list.h"

namespace ns3{
namespace ric_control {

// #ifdef __cplusplus
// extern "C" {
// #endif

	/*forward declaration*/
	// struct CellHandoverItemList_t;

	typedef struct CellHandoverList {
		long sourceCellId;
		CellHandoverItemList_t* cellHandoverItemList;
		
		/* Context for parsing across buffer boundaries */
		asn_struct_ctx_t _asn_ctx;
	} CellHandoverList_t;

	/* Implementation */
	// extern asn_TYPE_descriptor_t asn_DEF_CellHandoverList;
	// extern asn_SEQUENCE_specifics_t asn_SPC_CellHandoverList_specs_1;
	// extern asn_TYPE_member_t asn_MBR_CellHandoverList_1[2];

	/* Implementation */
	extern asn_TYPE_descriptor_t asn_DEF_CellHandoverList;
	extern asn_SEQUENCE_specifics_t asn_SPC_CellHandoverList_specs_1;
	extern asn_TYPE_member_t asn_MBR_CellHandoverList_1[2];
	// extern asn_per_constraints_t asn_PER_type_CellHandoverList_constr_1;

	// asn_struct_free_f CellHandoverList_free;
	// asn_struct_print_f CellHandoverList_print;


// #ifdef __cplusplus
// }
// #endif

}

}

#endif	/* _CELL_ALL_HANDOVERS_H_ */
#include <asn_internal.h>
