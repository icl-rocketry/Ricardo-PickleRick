
#include "preflight.h"
#include "launch.h"


#include <librnp/rnp_default_address.h>
#include <librnp/rnp_routingtable.h>

#include "Sound/Melodies/melodyLibrary.h"

#include "Config/systemflags_config.h"
#include "Config/types.h"

#include "system.h"

Preflight::Preflight(System& system):
State(SYSTEM_FLAG::STATE_PREFLIGHT,system.systemstatus),
_system(system)
{};

void Preflight::initialize(){
    State::initialize();
    //load the rocket routing table

    RoutingTable flightRouting;
    flightRouting.setRoute((uint8_t)DEFAULT_ADDRESS::GROUNDSTATION_GATEWAY,Route{2,1,{}});
    flightRouting.setRoute((uint8_t)DEFAULT_ADDRESS::GROUNDSTATION,Route{2,2,{}});
    flightRouting.setRoute(5,Route{3,2,{}});
    flightRouting.setRoute(6,Route{3,2,{}});
    flightRouting.setRoute(7,Route{3,2,{}});
    flightRouting.setRoute(8,Route{3,2,{}});
    flightRouting.setRoute(9,Route{3,2,{}});
    flightRouting.setRoute(10,Route{3,2,{}});
    flightRouting.setRoute(11,Route{3,2,{}});
    flightRouting.setRoute(12,Route{3,2,{}});
    flightRouting.setRoute(13,Route{3,2,{}});
    flightRouting.setRoute(14,Route{3,2,{}});
    flightRouting.setRoute(15,Route{3,2,{}});
    flightRouting.setRoute(16,Route{3,2,{}});
    flightRouting.setRoute(17,Route{3,2,{}});
    flightRouting.setRoute(18,Route{3,2,{}});
    flightRouting.setRoute(19,Route{3,2,{}});
    flightRouting.setRoute(20,Route{3,2,{}});
    
    _system.networkmanager.setRoutingTable(flightRouting);
    _system.networkmanager.updateBaseTable(); // save the new base table

    _system.networkmanager.setAddress(static_cast<uint8_t>(DEFAULT_ADDRESS::ROCKET));
    
    _system.networkmanager.enableAutoRouteGen(false);
    _system.networkmanager.setNoRouteAction(NOROUTE_ACTION::BROADCAST,{1,3});
    

    _system.tunezhandler.play(MelodyLibrary::zeldatheme,true);

};


Types::CoreTypes::State_ptr_t Preflight::update(){
    return nullptr;
};

void Preflight::exit(){
    State::exit();
    _system.tunezhandler.clear();
};