#include "GNC/ControllerTelemetryPacket.h"

ControllerTelemetryPacket::~ControllerTelemetryPacket()
{};

ControllerTelemetryPacket::ControllerTelemetryPacket():
RnpPacket(0,
          108, 
          size())
{};

ControllerTelemetryPacket::ControllerTelemetryPacket(const RnpPacketSerialized& packet):
RnpPacket(packet,size())
{
    getSerializer().deserialize(*this,packet.getBody());
};

void ControllerTelemetryPacket::deserializeBody(std::vector<uint8_t>& buf){
    getSerializer().deserialize(*this, buf);
}

std::string ControllerTelemetryPacket::stringify() const
{
    return getSerializer().stringify(*this) + "\n";
}

void ControllerTelemetryPacket::serialize(std::vector<uint8_t>& buf){
    RnpPacket::serialize(buf);
	size_t bufsize = buf.size();
	buf.resize(bufsize + size());
	std::memcpy(buf.data() + bufsize,getSerializer().serialize(*this).data(),size());
};
