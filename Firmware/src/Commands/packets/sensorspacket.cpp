#include "Commands/packets/sensorspacket.h"

SensorsPacket::~SensorsPacket()
{};

SensorsPacket::SensorsPacket():
RnpPacket(0,
          101,
          size())
{};

SensorsPacket::SensorsPacket(const RnpPacketSerialized& packet):
RnpPacket(packet,size())
{
    getSerializer().deserialize(*this,packet.getBody());
};

void SensorsPacket::serialize(std::vector<uint8_t>& buf){
    RnpPacket::serialize(buf);
	size_t bufsize = buf.size();
	buf.resize(bufsize + size());
	std::memcpy(buf.data() + bufsize,getSerializer().serialize(*this).data(),size());
};