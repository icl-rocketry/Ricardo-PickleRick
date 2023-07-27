#include "magcalcommandpacket.h"

#include <librnp/rnp_packet.h>
#include <librnp/rnp_serializer.h>




MagCalCommandPacket::~MagCalCommandPacket()
{};

MagCalCommandPacket::MagCalCommandPacket(uint8_t _command):
RnpPacket(0,
          10,
          size()),
command(_command)
{};

MagCalCommandPacket::MagCalCommandPacket(const RnpPacketSerialized& packet):
RnpPacket(packet,size())
{
    getSerializer().deserialize(*this,packet.getBody());
};

void MagCalCommandPacket::serialize(std::vector<uint8_t>& buf){
    RnpPacket::serialize(buf);
	size_t bufsize = buf.size();
	buf.resize(bufsize + size());
	std::memcpy(buf.data() + bufsize,getSerializer().serialize(*this).data(),size());
};

Eigen::Matrix3f MagCalCommandPacket::getA() 
{
    return Eigen::Matrix3f{{A11,A12,A13},{A21,A22,A23},{A31,A32,A33}};
};

Eigen::Vector3f MagCalCommandPacket::getB() 
{
    return Eigen::Vector3f{{b1,b2,b3}};
};


