#include "cell_handovers_list.h"
#include "handover_list.h"

namespace ns3{

namespace ric_control{

asn_TYPE_member_t asn_MBR_CellHandoverList_1[] = {
    { ATF_NOFLAGS, 0, offsetof(struct CellHandoverList, sourceCellId),
        (ASN_TAG_CLASS_CONTEXT | (0 << 2)),
        0,	/* IMPLICIT tag at current level */
        &asn_DEF_NativeInteger,
        0,
        {0,0,0}, 
        // { &asn_OER_memb_CellHandoverItem_constr_2, &asn_PER_memb_CellHandoverItem_constr_2, 0 },
        0, 0, /* No default value */
        "sourceCellId"
		// ""
    },
	{ ATF_POINTER, 1, offsetof(struct CellHandoverList, cellHandoverItemList),
		(ASN_TAG_CLASS_CONTEXT | (1 << 2)),
		0,
		&asn_DEF_CellHandoverItemList,
		0,
		{ 0, 0, 0 },
		0, 0, /* No default value */
		"cellHandovers"
		// ""
    },
};
static const ber_tlv_tag_t asn_DEF_CellHandoverList_tags_1[] = {
	(ASN_TAG_CLASS_UNIVERSAL | (16 << 2))
};
// asn_SET_OF_specifics_t asn_SPC_CellHandoverList_specs_1 = {
// 	sizeof(struct CellHandoverList),
// 	offsetof(struct CellHandoverList, _asn_ctx),
// 	0,	/* XER encoding is XMLDelimitedItemList */
// };
static const asn_TYPE_tag2member_t asn_MAP_CellHandoverList_tag2el_1[] = {
    { (ASN_TAG_CLASS_CONTEXT | (0 << 2)), 0, 0, 0 }, /* cell-object-ID */
    { (ASN_TAG_CLASS_CONTEXT | (1 << 2)), 1, 0, 0 } /* cell-global-ID */
};
asn_SEQUENCE_specifics_t asn_SPC_CellHandoverList_specs_1 = {
	sizeof(struct CellHandoverList),
	offsetof(struct CellHandoverList, _asn_ctx),
	asn_MAP_CellHandoverList_tag2el_1,
	2,	/* Count of tags in the map */
	0, 0, 0,	/* Optional elements (not needed) */
	2,	/* First extension addition */
};
asn_TYPE_descriptor_t asn_DEF_CellHandoverList = {  
	"CellHandoverList",
	"CellHandoverList",
	&asn_OP_SEQUENCE,
	asn_DEF_CellHandoverList_tags_1,
	sizeof(asn_DEF_CellHandoverList_tags_1)
		/sizeof(asn_DEF_CellHandoverList_tags_1[0]), /* 1 */
	asn_DEF_CellHandoverList_tags_1,	/* Same as above */
	sizeof(asn_DEF_CellHandoverList_tags_1)
		/sizeof(asn_DEF_CellHandoverList_tags_1[0]), /* 1 */
	{ 0, 0, SEQUENCE_OF_constraint },
	asn_MBR_CellHandoverList_1,
	2,	/* Single element */
	&asn_SPC_CellHandoverList_specs_1	/* Additional specs */
};

}
}
