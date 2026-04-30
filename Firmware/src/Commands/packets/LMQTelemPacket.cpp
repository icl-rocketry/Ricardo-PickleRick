#include "Commands/packets/LMQTelemPacket.h"

#include <librnp/rnp_networkmanager.h>
#include <librnp/rnp_packet.h>

#include <vector>



LMQTelemPacket::~LMQTelemPacket()
{};

LMQTelemPacket::LMQTelemPacket():
RnpPacket(0,
          104,
          size())
{};

LMQTelemPacket::LMQTelemPacket(const RnpPacketSerialized& packet):
RnpPacket(packet,size())
{
    getSerializer().deserialize(*this,packet.getBody());
};

void LMQTelemPacket::serialize(std::vector<uint8_t>& buf){
    RnpPacket::serialize(buf);
	size_t bufsize = buf.size();
	buf.resize(bufsize + size());
	std::memcpy(buf.data() + bufsize,getSerializer().serialize(*this).data(),size());
};