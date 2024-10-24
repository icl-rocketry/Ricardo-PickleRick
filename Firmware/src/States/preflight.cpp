
#include "preflight.h"
#include "launch.h"


#include <librnp/rnp_default_address.h>
#include <librnp/rnp_routingtable.h>

#include "Sound/Melodies/melodyLibrary.h"

#include "Config/systemflags_config.h"
#include "Config/types.h"
#include "Config/commands_config.h"

#include "system.h"

Preflight::Preflight(System& system):
State(SYSTEM_FLAG::STATE_PREFLIGHT,system.systemstatus),
_system(system)
{};

void Preflight::initialize(){
    State::initialize();
    //enable commands
    _system.commandhandler.enableCommands({Commands::ID::Launch,
                                           Commands::ID::Set_Home,
                                           Commands::ID::Stop_Logging,
                                           Commands::ID::Print_Flash_filesystem,
                                           Commands::ID::Print_Sd_filesystem,
                                           Commands::ID::Play_Song,
                                           Commands::ID::Skip_Song,
                                           Commands::ID::Clear_Song_Queue,
                                           Commands::ID::Reset_Orientation,
                                           Commands::ID::Reset_Localization,
                                           Commands::ID::Set_Beta,
                                           Commands::ID::Calibrate_AccelGyro_Bias,
                                           Commands::ID::Calibrate_HighGAccel_Bias,
                                           Commands::ID::Calibrate_Mag_Full,
                                           Commands::ID::Calibrate_Baro,
                                           Commands::ID::Enter_Debug,
                                           Commands::ID::Radio_SetFreq,
                                           Commands::ID::Radio_SetBW,
                                           Commands::ID::Radio_SetPower,
                                           Commands::ID::Radio_SetSF,
                                           Commands::ID::Radio_SetSYNC});
   
    

    _system.tunezhandler.play(MelodyLibrary::zeldatheme,true);

};


Types::CoreTypes::State_ptr_t Preflight::update(){
    return nullptr;
};

void Preflight::exit(){
    State::exit();
    _system.commandhandler.resetCommands();
    _system.tunezhandler.clear();
};