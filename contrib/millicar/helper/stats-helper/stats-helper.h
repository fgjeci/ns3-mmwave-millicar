#ifndef EF_STATS_HELPER_H
#define EF_STATS_HELPER_H

#include <ns3/core-module.h>
#include "sinr-report-stats.h"
#include "send-packet-stats.h"
#include "ef-scheduling-stats.h"
#include <ns3/network-module.h>
#include "ef-mac-rlc-buffer-status.h"
#include "ns3/mmwave-mac-sched-sap.h"
#include <ns3/mmwave-sidelink-mac.h>

namespace ns3 {

using namespace millicar;
using namespace mmwave;

// namespace mmwave{

class MmWaveSidelinkMac;
struct SlSchedulingCallback;

// class MmWaveSidelinkMac::SlSchedulingCallback;

class EfStatsHelper : public Object{

public:

	EfStatsHelper(void);

	virtual ~EfStatsHelper(void);

//	ElephantFlowHelper(NetDeviceContainer * ueNetDev, NetDeviceContainer * ueTcpNetDev);
	//	ElephantFlowHelper(std::string dirPath, Time startEF, Time endEF, Time periodicityEfCheck,
	//	Time durationEFCheck, Time samplingPeriodicityEF, NetDeviceContainer * ueNetDev,
	//	NetDeviceContainer * ueTcpNetDev);

	void DoInitialize(void);

	/**
	   * \brief Get the type ID.
	   * \return the object TypeId
	   */
	static TypeId GetTypeId (void);


	static void 
	SinrReportCallback (mmwave::SinrReportStats *stats, uint16_t sourceRnti, uint16_t rnti, uint8_t numSym, uint32_t tbSize,  double sinr);

	static void 
	AllPeersSinrReportCallback (mmwave::SinrReportStats *stats, uint16_t sourceRnti, uint64_t rnti,  double sinr);

	static void 
	SendPacketReportCallback (mmwave::SendPacketStats *stats, Ptr<Packet> packet, uint16_t sourceRnti, 
								uint16_t destRnti, uint16_t intermediateRnti, uint16_t localRnti
								, Vector pos// ,const ns3::Address & from, const ns3::Address & to
								);

	static void 
	RelayPacketReportCallback (mmwave::SendPacketStats *stats, Ptr<Packet> packet, uint16_t localRnti
                                        , Vector pos// ,const ns3::Address & from, const ns3::Address & to
                                        );

	static void
	ReportMacBsr(mmwave::MacBsrStats *macSRStats, mmwave::SfnSf sfnsf, uint16_t cellId, mmwave::MmWaveMacSchedSapProvider::SchedDlRlcBufferReqParameters schedParams);

	static void
	ReportSchedulingInfo(mmwave::DlSchedulingStats *enbStats, millicar::SlSchedulingCallback schedParams);
};

// }

}


#endif /* ELEPHANT_FLOW_HELPER_H */