#include "PIDCalibrationPacket.h"
#include <vector>



PIDCalibrationPacket::~PIDCalibrationPacket()
{};

PIDCalibrationPacket::PIDCalibrationPacket():
RnpPacket(0,
          105,
          size())
{};

PIDCalibrationPacket::PIDCalibrationPacket(const RnpPacketSerialized& packet):
RnpPacket(packet,size())
{
    getSerializer().deserialize(*this,packet.getBody());
};

void PIDCalibrationPacket::deserializeBody(std::vector<uint8_t>& buf){
    getSerializer().deserialize(*this, buf);
}

void PIDCalibrationPacket::serialize(std::vector<uint8_t>& buf){
    RnpPacket::serialize(buf);
	size_t bufsize = buf.size();
	buf.resize(bufsize + size());
	std::memcpy(buf.data() + bufsize,getSerializer().serialize(*this).data(),size());
};