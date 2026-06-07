#include "Commands/packets/rtkpacket.h"
#include <vector>



RTKPacket::~RTKPacket()
{};

RTKPacket::RTKPacket():
RnpPacket(0,
          108, // ask andrei about types
          size())
{};

RTKPacket::RTKPacket(const RnpPacketSerialized& packet):
RnpPacket(packet,size())
{
    getSerializer().deserialize(*this,packet.getBody());
};

void RTKPacket::deserializeBody(std::vector<uint8_t>& buf){
    getSerializer().deserialize(*this, buf);
}

void RTKPacket::serialize(std::vector<uint8_t>& buf){
    RnpPacket::serialize(buf);
	size_t bufsize = buf.size();
	buf.resize(bufsize + size());
	std::memcpy(buf.data() + bufsize,getSerializer().serialize(*this).data(),size());
};