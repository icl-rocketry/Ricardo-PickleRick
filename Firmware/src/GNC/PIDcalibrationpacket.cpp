#include "PIDcalibrationpacket.h"
#include <vector>



PIDcalibrationpacket::~PIDcalibrationpacket()
{};

PIDcalibrationpacket::PIDcalibrationpacket():
RnpPacket(0,
          105,
          size())
{};

PIDcalibrationpacket::PIDcalibrationpacket(const RnpPacketSerialized& packet):
RnpPacket(packet,size())
{
    getSerializer().deserialize(*this,packet.getBody());
};

void PIDcalibrationpacket::deserializeBody(std::vector<uint8_t>& buf){
    getSerializer().deserialize(*this, buf);
}

void PIDcalibrationpacket::serialize(std::vector<uint8_t>& buf){
    RnpPacket::serialize(buf);
	size_t bufsize = buf.size();
	buf.resize(bufsize + size());
	std::memcpy(buf.data() + bufsize,getSerializer().serialize(*this).data(),size());
};