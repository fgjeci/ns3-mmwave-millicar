/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */

#include "parameters-config.h"
#include <sys/stat.h>
// #include <ns3/mc-ue-net-device.h>
#include <ns3/mmwave-vehicular-5g-net-device.h>
#include <ns3/lte-enb-net-device.h>
#include <ns3/seq-ts-size-header.h>
#include <ns3/stats-helper.h>


namespace ns3 {

namespace mmwave
{

NS_LOG_COMPONENT_DEFINE ("ParametersConfig");

NS_OBJECT_ENSURE_REGISTERED (ParametersConfig);

ParametersConfig::ParametersConfig (void){
	NS_LOG_FUNCTION (this);
}


ParametersConfig::~ParametersConfig ()
{
  NS_LOG_FUNCTION (this);
}

TypeId
ParametersConfig::GetTypeId (void)
{
	// we can add attributes as well
  static TypeId tid = TypeId ("ns3::ParametersConfig")
    .SetParent<Object> ()
    .SetGroupName ("ParametersConfig")
    .AddConstructor<ParametersConfig> ()
	;
  return tid;
}

bool
Params::Validate (void) const
{
	NS_ABORT_MSG_IF (simulator != "5GLENA",
	                   "Cannot do the parameters config with the simulator " << simulator);
	return true;
}

void 
ParametersConfig::DisableTraces()
{
	LogLevel logLevel = (LogLevel)(LOG_PREFIX_FUNC | LOG_PREFIX_TIME | LOG_INFO | LOG_DEBUG | LOG_LEVEL_ALL);
	LogComponentDisableAll(logLevel);
}

void ParametersConfig::BulkApplicationTxPacketLteCoordinator(BulkSenderStruct* bulkSenderStruct, 
	Ptr<NetDevice> reportingEnbDevice, Ptr<const Packet> packet){
	// NS_LOG_UNCOND("Bulk application");
	Ptr<Ipv4> ipv4 = bulkSenderStruct->servertcpIpv4;
	Ptr<BulkSendApplication> bulkSenderApplication = DynamicCast<BulkSendApplication>(bulkSenderStruct->bulkSenderApplication);
	Ptr<OnOffApplication> onoffApplication = DynamicCast<OnOffApplication>(bulkSenderStruct->bulkSenderApplication);
	
	Ptr<OutputStreamWrapper> stream	= bulkSenderStruct->stream;
	uint32_t *remainingDataPtr = bulkSenderStruct->pointerToSize;
	
	Ptr<Node> node;
	uint32_t nodeId  = 0;
	uint64_t imsi = 0;
	uint16_t rnti = 0;
	uint16_t cellId = 0;
	uint32_t _intIndex = 0;
	uint32_t left = static_cast<uint32_t> (*remainingDataPtr);

	// we find the interface for which the imsi is not zero
 	for(uint32_t _ind = 0; _ind< ipv4->GetNInterfaces(); ++_ind){
		Ptr<MmWaveMillicarUeNetDevice> tmpUeNrUeNetDev = ipv4->GetNetDevice(_ind)->GetObject<MmWaveMillicarUeNetDevice> ();
		if(tmpUeNrUeNetDev){
			node = tmpUeNrUeNetDev->GetNode();
			if(tmpUeNrUeNetDev->GetRnti ()!=0){
				_intIndex = _ind;
				break;
			}
		}else{
			if (bulkSenderApplication!=nullptr){
				node = DynamicCast<Node>(bulkSenderApplication->GetSocket()->GetNode());
			}else{
				node = DynamicCast<Node>(onoffApplication->GetNode());
			}
			
		}
	}
	if (node!= nullptr){
		nodeId  = node->GetId();
	}

	Ptr<NetDevice> nodeNetDevice = ipv4->GetNetDevice(_intIndex);
	Ptr<MmWaveMillicarUeNetDevice> ueNrUeNetDev;
	if(nodeNetDevice){
		ueNrUeNetDev = DynamicCast < MmWaveMillicarUeNetDevice > (nodeNetDevice->GetObject<MmWaveMillicarUeNetDevice>());
		rnti = ueNrUeNetDev->GetRnti ();
	}

	uint32_t amountSent = packet->GetSize();
	uint64_t packetId = packet->GetUid();

	Vector _pos = node->GetObject<MobilityModel> ()->GetPosition ();
	uint32_t availableTx = 0;
	if (bulkSenderApplication!=nullptr){
		availableTx = bulkSenderApplication->GetSocket()->GetTxAvailable ();
	}else{
		availableTx = onoffApplication->GetSocket()->GetTxAvailable();
	}

	*stream->GetStream() << rnti << "\t" << imsi << "\t"  << nodeId << "\t" << cellId << "\t" 
		<< left - amountSent <<  "\t" << packet->GetSize() << "\t" << availableTx
		<< "\t" << Simulator::Now().GetSeconds() << "\t" << _pos.x << "\t" << _pos.y << "\t" << _pos.z <<  std::endl;
	(*remainingDataPtr) -= amountSent;
}

void ParametersConfig::BulkApplicationTxPacketLteCoordinatorAddress(BulkSenderStruct* bulkSenderStruct, 
		Ptr<NetDevice> reportingEnbDevice, Ptr<const Packet> packet,
		const ns3::Address & from, const ns3::Address & to//, SendPacketStats *stats
		){
	// NS_LOG_UNCOND("Bulk application");
	Ptr<Ipv4> ipv4 = bulkSenderStruct->servertcpIpv4;
	Ptr<BulkSendApplication> bulkSenderApplication = DynamicCast<BulkSendApplication>(bulkSenderStruct->bulkSenderApplication);
	Ptr<OnOffApplication> onoffApplication = DynamicCast<OnOffApplication>(bulkSenderStruct->bulkSenderApplication);
	Ptr<UdpClient> udpApplication = DynamicCast<UdpClient>(bulkSenderStruct->bulkSenderApplication);
	
	Ptr<OutputStreamWrapper> stream	= bulkSenderStruct->stream;
	uint32_t *remainingDataPtr = bulkSenderStruct->pointerToSize;

	SeqTsHeader seqTsHeader;
	packet->PeekHeader(seqTsHeader);
	
	Ptr<Node> node;
	uint32_t nodeId  = 0;
	uint64_t imsi = 0;
	uint16_t rnti = 0;
	uint16_t cellId = 0;
	uint32_t _intIndex = 0;
	uint32_t left = static_cast<uint32_t> (*remainingDataPtr);

	// we find the interface for which the imsi is not zero
 	for(uint32_t _ind = 0; _ind< ipv4->GetNInterfaces(); ++_ind){
		Ptr<MmWaveMillicarUeNetDevice> tmpUeNrUeNetDev = ipv4->GetNetDevice(_ind)->GetObject<MmWaveMillicarUeNetDevice> ();
		if(tmpUeNrUeNetDev){
			node = tmpUeNrUeNetDev->GetNode();
			if(tmpUeNrUeNetDev->GetRnti ()!=0){
				_intIndex = _ind;
				break;
			}
		}else{
			if (bulkSenderApplication!=nullptr){
				node = DynamicCast<Node>(bulkSenderApplication->GetSocket()->GetNode());
			}else if (onoffApplication!=nullptr){
				node = DynamicCast<Node>(onoffApplication->GetNode());
			}else if (udpApplication!=nullptr){
				node = DynamicCast<Node>(udpApplication->GetNode());
			}
			
		}
	}
	if (node!= nullptr){
		nodeId  = node->GetId();
	}

	Ptr<NetDevice> nodeNetDevice = ipv4->GetNetDevice(_intIndex);
	Ptr<MmWaveMillicarUeNetDevice> ueNrUeNetDev;
	if(nodeNetDevice){
		ueNrUeNetDev = DynamicCast < MmWaveMillicarUeNetDevice > (nodeNetDevice->GetObject<MmWaveMillicarUeNetDevice>());
		rnti = ueNrUeNetDev->GetRnti ();
	}

	uint32_t amountSent = packet->GetSize();
	uint64_t packetId = packet->GetUid();

	Vector _pos = node->GetObject<MobilityModel> ()->GetPosition ();
	uint32_t availableTx = 0;
	if (bulkSenderApplication!=nullptr){
		availableTx = bulkSenderApplication->GetSocket()->GetTxAvailable ();
	}else if (onoffApplication!=nullptr){
		availableTx = onoffApplication->GetSocket()->GetTxAvailable();
	}
	// else if (udpApplication!=nullptr){
	// 	availableTx = udpApplication->GetSocket()->GetTxAvailable();
	// }

	// EfStatsHelper::SendPacketReportCallback(stats, packet, );

	// *stream->GetStream() << rnti << "\t" << imsi << "\t"  << nodeId << "\t" << cellId << "\t" 
	// 	<< seqTsHeader.GetSeq() <<  "\t" << packet->GetSize() << "\t" << seqTsHeader.GetTs().GetSeconds()
	// 	<< "\t" << Simulator::Now().GetSeconds() << "\t" << _pos.x << "\t" << _pos.y << "\t" << _pos.z <<  std::endl;
}

void 
ParametersConfig::RxTcp (Ptr<OutputStreamWrapper> stream, Ptr<ns3::Node> node, Ptr<Ipv4> ipv4, 
		Ptr<const Packet> p, const ns3::Address & from, const ns3::Address & to, 
		const ns3::SeqTsSizeHeader & header)
{
	// std::cout << "RxTxp " << std::endl;
	uint16_t rnti = 0;
	uint32_t nodeId  = node->GetId();
	uint32_t _intIndex = 0;

 	uint64_t packetId = p->GetUid();

	// we find the interface for which the imsi is not zero
 	for(uint32_t _ind = 0; _ind< ipv4->GetNInterfaces(); ++_ind){
		Ptr<MmWaveMillicarUeNetDevice> tmpUeNrUeNetDev = ipv4->GetNetDevice(_ind)->GetObject<MmWaveMillicarUeNetDevice> ();
		if(tmpUeNrUeNetDev){
			if(tmpUeNrUeNetDev->GetRnti ()!=0){
				_intIndex = _ind;
				rnti = tmpUeNrUeNetDev->GetRnti ();
				break;
			}
		}
	}

	Vector pos = node->GetObject<MobilityModel> ()->GetPosition ();

 	*stream->GetStream () << Simulator::Now ().GetSeconds () << "\t" << rnti << "\t" << p->GetSize() 
	<< "\t" << header.GetSeq() << "\t" << header.GetTs().GetSeconds()
	<< "\t" << pos.x << "\t" << pos.y << std::endl;
}

void 
ParametersConfig::RxUdp (Ptr<OutputStreamWrapper> stream, Ptr<ns3::Node> node, Ptr<Ipv4> ipv4, 
		Ptr<const Packet> p, const ns3::Address & from, const ns3::Address & to)
{
	// std::cout << "RxTxp " << std::endl;
	uint16_t rnti = 0;
	uint32_t nodeId  = node->GetId();
	uint32_t _intIndex = 0;

 	uint64_t packetId = p->GetUid();

	SeqTsHeader seqTsHeader;
	p->PeekHeader(seqTsHeader);

	// we find the interface for which the imsi is not zero
 	for(uint32_t _ind = 0; _ind< ipv4->GetNInterfaces(); ++_ind){
		Ptr<MmWaveMillicarUeNetDevice> tmpUeNrUeNetDev = ipv4->GetNetDevice(_ind)->GetObject<MmWaveMillicarUeNetDevice> ();
		if(tmpUeNrUeNetDev){
			if(tmpUeNrUeNetDev->GetRnti ()!=0){
				_intIndex = _ind;
				rnti = tmpUeNrUeNetDev->GetRnti ();
				break;
			}
		}
	}
	

	Vector pos = node->GetObject<MobilityModel> ()->GetPosition ();

 	*stream->GetStream () << Simulator::Now ().GetSeconds () << "\t" << rnti << "\t" << p->GetSize() 
	// << "\t" << seqTsHeader.GetSeq() 
	<< "\t" << packetId
	<< "\t" << seqTsHeader.GetTs().GetSeconds()
	<< "\t" << pos.x << "\t" << pos.y << std::endl;
}

void 
ParametersConfig::RxUdpVideo (Ptr<OutputStreamWrapper> stream, Ptr<ns3::Node> node, Ptr<Ipv4> ipv4, 
		Ptr<const Packet> p, const ns3::Address & from, const ns3::Address & to)
{
	// std::cout << "Rx RxUdpVideo " << std::endl;
	uint16_t rnti = 0;
	uint32_t nodeId  = node->GetId();
	uint32_t _intIndex = 0;

 	uint64_t packetId = p->GetUid();

	// SeqTsHeader seqTsHeader;
	// p->PeekHeader(seqTsHeader);

	// we find the interface for which the imsi is not zero
 	for(uint32_t _ind = 0; _ind< ipv4->GetNInterfaces(); ++_ind){
		Ptr<MmWaveMillicarUeNetDevice> tmpUeNrUeNetDev = ipv4->GetNetDevice(_ind)->GetObject<MmWaveMillicarUeNetDevice> ();
		if(tmpUeNrUeNetDev){
			if(tmpUeNrUeNetDev->GetRnti ()!=0){
				_intIndex = _ind;
				rnti = tmpUeNrUeNetDev->GetRnti ();
				break;
			}
		}
	}
	

	Vector pos = node->GetObject<MobilityModel> ()->GetPosition ();

 	*stream->GetStream () << Simulator::Now ().GetSeconds () << "\t" << rnti << "\t" << p->GetSize() 
	// << "\t" << seqTsHeader.GetSeq() 
	<< "\t" << packetId
	<< "\t" << -1
	<< "\t" << pos.x << "\t" << pos.y << std::endl;
}

std::vector<uint32_t>
ParametersConfig::GetEfsIndex(Params params){
	std::string _line;
	std::string _el;
	// Elephant flow users index
	std::vector<uint32_t> efsIndex;
	std::fstream efsIndexFile(params.outputDir + params.simTag + "/" + params.elephantFlowUsersIndexFilename, std::ios::in);
	if(!efsIndexFile.is_open()){
		std::cerr << "There was a problem opening the mean burst arrival time file"; exit(1);
	}else{
		while (efsIndexFile >> _line){std::istringstream is( _line ); while(std::getline( is, _el, ',' )){efsIndex.push_back(std::stoi( _el ));}}
		// double num = 0.0; while (efsIndexFile >> num){efsIndex.push_back(num);}
		efsIndexFile.close();
	}
	return efsIndex;
}

std::vector<double>
ParametersConfig::GetMeanBurstArrivalTime(Params params){
	// mean burst arrival time
	std::string _line;
	std::string _el;
	std::vector<double> meanBurstArrivalTime;
	std::fstream meanBurstArrivalTimeFile(params.outputDir + params.simTag + "/" + params.meanBurstArrivalTimeFilename, std::ios::in);
	if(!meanBurstArrivalTimeFile.is_open()){
		std::cerr << "There was a problem opening the mean burst arrival time file"; exit(1);
	}else{
		while (meanBurstArrivalTimeFile >> _line){std::istringstream is( _line ); while(std::getline( is, _el, ',' )){meanBurstArrivalTime.push_back(std::stod( _el ));}}
		// double num = 0.0; while (meanBurstArrivalTimeFile >> num){meanBurstArrivalTime.push_back(num);}
		meanBurstArrivalTimeFile.close();
	}
	return meanBurstArrivalTime;
}

std::vector<double>
ParametersConfig::GetMeanBurstDuration(Params params){
	// mean burst duration
	std::string _line;
	std::string _el;
	std::vector<double> meanBurstDuration;
	std::fstream meanBurstDurationFile(params.outputDir + params.simTag + "/" + params.meanBurstDurationFilename, std::ios::in);
	if(!meanBurstDurationFile.is_open()){
		std::cerr << "There was a problem opening the mean burst duration file"; exit(1);
	}else{
		while (meanBurstDurationFile >> _line){std::istringstream is( _line ); while(std::getline( is, _el, ',' )){meanBurstDuration.push_back(std::stod( _el ));}}
		// double num = 0.0; while (meanBurstDurationFile >> num){meanBurstDuration.push_back(num);}
		meanBurstDurationFile.close();
	}
	return meanBurstDuration;
}

std::vector<double>
ParametersConfig::GetMeanBurstIntensity(Params params){
	// burst intensity
	std::string _line;
	std::string _el;
	std::vector<double> burstIntensity;
	std::fstream burstIntensityFile(params.outputDir + params.simTag + "/" + params.burstIntensityFilename, std::ios::in);
	if(!burstIntensityFile.is_open()){
		std::cerr << "There was a problem opening the burst intensity file";exit(1);
	}else{
		while (burstIntensityFile >> _line){std::istringstream is( _line ); while(std::getline( is, _el, ',' )){burstIntensity.push_back(std::stod( _el ));}}
		// double num = 0.0; while (burstIntensityFile >> num){burstIntensity.push_back(num);}
		burstIntensityFile.close();
	}
	return burstIntensity;
}

std::vector<double>
ParametersConfig::GetHurst(Params params){

	// hurst parameter
	std::string _line;
	std::string _el;
	std::vector<double> hurst;
	std::fstream hurstFile(params.outputDir + params.simTag + "/" + params.hurstFilename, std::ios::in);
	if(!hurstFile.is_open()){
		std::cerr << "There was a problem opening the burst intensity file";exit(1);
	}else{
		while (hurstFile >> _line){std::istringstream is( _line ); while(std::getline( is, _el, ',' )){hurst.push_back(std::stod( _el ));}}
		hurstFile.close();
	}
	return hurst;
}

void 
ParametersConfig::EnableTraces()
{
	LogLevel logLevel = (LogLevel)(LOG_PREFIX_FUNC | LOG_PREFIX_TIME | LOG_INFO | LOG_DEBUG | LOG_LEVEL_ALL); //
	// LogComponentEnable ("EpcX2", LOG_LEVEL_ALL);
	// LogComponentEnable("LteEnbRrc", LOG_LEVEL_ALL);
	// LogComponentEnable("LteUeRrc", logLevel);
	// LogComponentEnable ("MmWaveMillicarHelper", LOG_LEVEL_ALL);
  	// LogComponentEnable ("MmWaveMillicarUeNetDevice", LOG_LEVEL_ALL);
	// LogComponentEnable ("MmWaveSidelinkMac", LOG_LEVEL_ALL);
	// LogComponentEnable ("MmWaveSidelinkPhy", LOG_LEVEL_ALL);
	LogComponentEnable ("Vehicular5G", LOG_LEVEL_ALL);

	// LogComponentEnableAll (logLevel);
//   LogComponentEnable ("RicControlMessage", LOG_LEVEL_ALL);
//   LogComponentEnable ("MmWaveFlexTtiMaxWeightMacScheduler", LOG_LEVEL_ALL);
// 	LogComponentEnable ("MmWaveFlexTtiMaxRateMacScheduler", LOG_LEVEL_ALL);
  
//   LogComponentEnable ("E2Termination", logLevel);

//   LogComponentEnable ("LteEnbNetDevice", logLevel);
//   LogComponentEnable ("MmWaveEnbNetDevice", logLevel);
//   LogComponentEnable ("MmWaveUeNetDevice", logLevel);
// LogComponentEnable("LteEnbRrc", LOG_DEBUG);
//   LogComponentEnable ("VideoStreamServerApplication", logLevel);
//   LogComponentEnable ("MmWaveHelper", logLevel);
//   LogComponentEnable ("MmWaveVehicularTracesHelper", LOG_LEVEL_ALL);
//   LogComponentEnable ("MmWaveBearerStatsConnector", LOG_LEVEL_ALL);
//   LogComponentEnable ("MmWavePhyMacCommon", LOG_LEVEL_ALL);

	// LogComponentEnable ("DeterministicVehicleChannelConditionModel", logLevel);
	// LogComponentEnable ("NodeV2vChannelConditionModel", logLevel);
	// LogComponentEnable("SpectrumModel", logLevel);

	// LogComponentEnable ("RandomWalk2dOutdoor", logLevel);
	// LogComponentEnable ("MmWaveEnbMac", logLevel);
	
	// LogComponentEnable ("ThreeGppV2vPropagationLossModel", logLevel);
	// LogComponentEnable("ThreeGppV2vUrbanBlockageChannelConditionModel", logLevel);
	// LogComponentEnable("ThreeGppPropagationLossModel", logLevel);
	// LogComponentEnable ("MultiModelSpectrumChannel", logLevel);

	// LogComponentEnable ("ThreeGppChannelConditionModel", logLevel);
	// LogComponentEnable ("BeamformingHelperBase", logLevel);
	// LogComponentEnable ("IdealBeamformingAlgorithm", logLevel);
	// LogComponentEnable ("PhasedArrayModel", logLevel);
	// LogComponentEnable ("PPBPApplication", logLevel);

	// LogComponentEnable ("LteRlcUm", logLevel);
	// LogComponentEnable ("LteRlcAm", logLevel);
	// LogComponentEnable ("LteRlcTm", logLevel);
	// LogComponentEnable ("LteRlc", logLevel);
	
	// LogComponentEnable("LtePdcp", logLevel);
	// LogComponentEnable ("BulkSendApplication", logLevel);

	// LogComponentEnable ("LteInterference", logLevel);
	// LogComponentEnable ("PacketSink", logLevel);

	// LogComponentEnable ("LteRlc", logLevel);
	// LogComponentEnable ("ComponentCarrierUe", logLevel);
	// LogComponentEnable ("SimpleUeComponentCarrierManager", logLevel);

	// LogComponentEnable ("PointToPointNetDevice", logLevel);
	
}

void
ParametersConfig::CreateTracesDir(Params params)
{
	struct stat outputDirInfo;

	if (stat(params.outputDir.c_str(), &outputDirInfo) != 0)
	{
		std::cout << "Cannot access directory. Exiting... " << std::endl;
		exit(1);
	}
	else if (outputDirInfo.st_mode & S_IFDIR)
	{
		// If the main directory exists, we create the other ones if they do not exist
		// std::cout<< "Output dir is valid " << std::endl;
		struct stat simTagDirInfo;
		if (stat((params.outputDir + params.simTag + "/").c_str(), &simTagDirInfo) != 0)
		{
			// std::cout << "Directory does not exists. Creating... " << std::endl;
			const int dir_err = mkdir((params.outputDir + params.simTag).c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
			if (dir_err == -1)
			{
				// std::cout << "Could not create directory. Exiting... " << std::endl;
				exit(1);
			}
			else
			{
				// check for the pcap directory
				// if this directory exist we go and check the pcap directory if it exists. if not we create it
				struct stat pcapDirInfo;
				if (stat((params.outputDir + params.simTag + "/pcap").c_str(), &pcapDirInfo) != 0)
				{
					// std::cout<<"Directory does not exists. Creating... " << std::endl;
					// Try to create also the pcap directory
					const int pcap_dir_err = mkdir((params.outputDir + params.simTag + "/pcap").c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
					if (pcap_dir_err == -1)
					{
						// std::cout << "Could not create directory. Exiting... " << std::endl;
						exit(1);
					}
				}
			}
		}
		else if (simTagDirInfo.st_mode & S_IFDIR)
		{
			// if this directory exist we go and check the pcap directory if it exists. if not we create it
			struct stat pcapDirInfo;
			// std::cout<< "Simulation dir exists. Checking pcap dir" << std::endl;
			if (stat((params.outputDir + params.simTag + "/pcap/").c_str(), &pcapDirInfo) != 0)
			{
				// std::cout<<"Pcap directory does not exists. Creating... " << std::endl;
				// Try to create also the pcap directory
				const int pcap_dir_err = mkdir((params.outputDir + params.simTag + "/pcap").c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
				if (pcap_dir_err == -1)
				{
					// std::cout << "Could not create directory. Exiting... " << std::endl;
					exit(1);
				}
			}
			else
			{
				// std::cout<<"Pcap directory exists" << std::endl;
			}
		}
		else
		{
			std::cout << "Cannot access directory. Exiting... " << std::endl;
			exit(1);
		}
	}
	else
	{
		std::cout << "Directory does not exists. Exiting... " << std::endl;
		exit(1);
	}
}

}
}


