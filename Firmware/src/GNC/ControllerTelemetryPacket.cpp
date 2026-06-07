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

std::string ControllerTelemetryPacket::csvHeader()
{
    return "q0,"
           "q1,"
           "q2,"
           "q3,"
           "total_error,"
           "pitch_error,"
           "yaw_error,"
           "roll_rate_input,"
           "pitch_rate_input,"
           "yaw_rate_input,"
           "pitch_output,"
           "yaw_output,"
           "thrust_top,"
           "thrust_bottom,"
           "fx_body,"
           "fy_body,"
           "fz_body,"
           "m_cmd_y,"
           "m_cmd_z,"
           "m_roll_mix,"
           "m_batt,"
           "m_pos_err_dbg_x,"
           "m_pos_err_dbg_y,"
           "m_pos_err_dbg_z,"
           "m_vel_err_dbg_x,"
           "m_vel_err_dbg_y,"
           "m_vel_err_dbg_z,"
           "m_pos_des_x,"
           "m_pos_des_y,"
           "m_pos_des_z,"
           "m_vel_des_x,"
           "m_vel_des_y,"
           "m_vel_des_z,"
           "m_acc_des_x,"
           "m_acc_des_y,"
           "m_acc_des_z,"
           "m_thrust_world_des_x,"
           "m_thrust_world_des_y,"
           "m_thrust_world_des_z,"
           "system_time,\n";
}

void ControllerTelemetryPacket::serialize(std::vector<uint8_t>& buf){
    RnpPacket::serialize(buf);
	size_t bufsize = buf.size();
	buf.resize(bufsize + size());
	std::memcpy(buf.data() + bufsize,getSerializer().serialize(*this).data(),size());
};
