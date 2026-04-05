#include "Commands/packets/rawmagpacket.h"

RawMagPacket::~RawMagPacket()
{};

RawMagPacket::RawMagPacket():
RnpPacket(0,
          101,
          size())
{};

RawMagPacket::RawMagPacket(const RnpPacketSerialized& packet):
RnpPacket(packet,size())
{
    getSerializer().deserialize(*this,packet.getBody());
};

void RawMagPacket::serialize(std::vector<uint8_t>& buf){
    RnpPacket::serialize(buf);
	size_t bufsize = buf.size();
	buf.resize(bufsize + size());
	std::memcpy(buf.data() + bufsize,getSerializer().serialize(*this).data(),size());
};