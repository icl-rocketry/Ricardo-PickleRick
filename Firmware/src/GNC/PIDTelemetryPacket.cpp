#include "PIDTelemetryPacket.h"
#include <vector>



PIDTelemetryPacket::~PIDTelemetryPacket()
{};

PIDTelemetryPacket::PIDTelemetryPacket():
RnpPacket(0,
          108, // ask andrei about types
          size())
{};

PIDTelemetryPacket::PIDTelemetryPacket(const RnpPacketSerialized& packet):
RnpPacket(packet,size())
{
    getSerializer().deserialize(*this,packet.getBody());
};

void PIDTelemetryPacket::deserializeBody(std::vector<uint8_t>& buf){
    getSerializer().deserialize(*this, buf);
}

void PIDTelemetryPacket::serialize(std::vector<uint8_t>& buf){
    RnpPacket::serialize(buf);
	size_t bufsize = buf.size();
	buf.resize(bufsize + size());
	std::memcpy(buf.data() + bufsize,getSerializer().serialize(*this).data(),size());
};