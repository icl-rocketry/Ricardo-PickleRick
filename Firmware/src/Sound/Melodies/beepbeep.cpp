
#include "melodyClass.h"
#include "melodyLibrary.h"
#include "pitches.h"

melody_t<4> MelodyLibrary::beepbeep = { {{ 
                    {NOTE_B5,50},
					{REST,100},
					{NOTE_B5,50},
                    {REST,400}
                    }}, //create array of melody note values
                    false // assign high priority to this
                    };