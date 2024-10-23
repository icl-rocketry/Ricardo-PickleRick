#include "RadioTestPacket.h"

#include <librnp/rnp_packet.h>
#include <librnp/rnp_serializer.h>

#include <vector>



RadioTestPacket::~RadioTestPacket()
{};

RadioTestPacket::RadioTestPacket():
RnpPacket(0,
          101,
          size())
{};

RadioTestPacket::RadioTestPacket(const RnpPacketSerialized& packet):
RnpPacket(packet,size())
{
    getSerializer().deserialize(*this,packet.getBody());
};

void RadioTestPacket::serialize(std::vector<uint8_t>& buf){
    RnpPacket::serialize(buf);
	size_t bufsize = buf.size();
	buf.resize(bufsize + size());
	std::memcpy(buf.data() + bufsize,getSerializer().serialize(*this).data(),size());
};