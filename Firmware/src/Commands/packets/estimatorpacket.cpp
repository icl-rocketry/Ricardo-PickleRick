#include "Commands/packets/estimatorpacket.h"

EstimatorPacket::~EstimatorPacket()
{};

EstimatorPacket::EstimatorPacket():
RnpPacket(0,
          101,
          size())
{};

EstimatorPacket::EstimatorPacket(const RnpPacketSerialized& packet):
RnpPacket(packet,size())
{
    getSerializer().deserialize(*this,packet.getBody());
};

void EstimatorPacket::serialize(std::vector<uint8_t>& buf){
    RnpPacket::serialize(buf);
	size_t bufsize = buf.size();
	buf.resize(bufsize + size());
	std::memcpy(buf.data() + bufsize,getSerializer().serialize(*this).data(),size());
};