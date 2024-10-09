#pragma once

#include "melodyClass.h"

#include <tuple>

namespace MelodyLibrary {
    
    extern melody_t<8> c_scalez;
    extern melody_t<1186> Fireflies;
    extern melody_t<110> zeldatheme;
    extern melody_t<64> cantinaband;
    extern melody_t<130> gameofthrones;
    extern melody_t<286> miichannel;
    extern melody_t<340> nevergonnagiveyouup;
    extern melody_t<3> confirmation;
    extern melody_t<4> heartbeat;
    extern melody_t<4> beepbeep;
    extern melody_t<208> pirates;



    static const std::array<melody_base_t*,7> songLibrary{
        &Fireflies,
        &zeldatheme,
        &cantinaband,
        &gameofthrones,
        &miichannel,
        &nevergonnagiveyouup,
        &pirates
        };

};


