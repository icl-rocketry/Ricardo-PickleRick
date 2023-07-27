
#include "preflight.h"
#include "launch.h"
#include "stateMachine.h"

#include "rnp_default_address.h"
#include "rnp_routingtable.h"

#include "Sound/Melodies/melodyLibrary.h"

#include "flags.h"

Preflight::Preflight(stateMachine* sm):
State(sm,SYSTEM_FLAG::STATE_PREFLIGHT)
{
    
};

void Preflight::initialise(){
    State::initialise();
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
    
    _sm->networkmanager.setRoutingTable(flightRouting);
    _sm->networkmanager.updateBaseTable(); // save the new base table

    _sm->networkmanager.setAddress(static_cast<uint8_t>(DEFAULT_ADDRESS::ROCKET));
    
    _sm->networkmanager.enableAutoRouteGen(false);
    _sm->networkmanager.setNoRouteAction(NOROUTE_ACTION::BROADCAST,{1,3});
    

    _sm->tunezhandler.play(MelodyLibrary::zeldatheme,true);

};


State* Preflight::update(){
    return this;
};

void Preflight::exitstate(){
    State::exitstate();
    _sm->tunezhandler.clear();
};