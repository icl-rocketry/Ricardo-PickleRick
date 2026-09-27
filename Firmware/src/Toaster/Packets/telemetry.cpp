#include "telemetry.h"

#include <librnp/rnp_packet.h>
#include <librnp/rnp_serializer.h>

#include <vector>

ToasterTelem::~ToasterTelem()
{};

ToasterTelem::ToasterTelem():
RnpPacket(0,
          101,
          size())
{};

ToasterTelem::ToasterTelem(const RnpPacketSerialized& packet):
RnpPacket(packet,size())
{
    getSerializer().deserialize(*this,packet.getBody());
};

void ToasterTelem::serialize(std::vector<uint8_t>& buf){
    RnpPacket::serialize(buf);
	size_t bufsize = buf.size();
	buf.resize(bufsize + size());
	std::memcpy(buf.data() + bufsize,getSerializer().serialize(*this).data(),size());
};

PIDTelem::~PIDTelem()
{};

PIDTelem::PIDTelem():
RnpPacket(0,
          101,
          size())
{};

PIDTelem::PIDTelem(const RnpPacketSerialized& packet):
RnpPacket(packet,size())
{
    getSerializer().deserialize(*this,packet.getBody());
};

void PIDTelem::serialize(std::vector<uint8_t>& buf){
    RnpPacket::serialize(buf);
	size_t bufsize = buf.size();
	buf.resize(bufsize + size());
	std::memcpy(buf.data() + bufsize,getSerializer().serialize(*this).data(),size());
};
