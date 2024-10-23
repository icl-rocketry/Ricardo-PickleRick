
#include "melodyClass.h"
#include "melodyLibrary.h"
#include "pitches.h"

melody_t<4> MelodyLibrary::heartbeat = { {{ 
                    {NOTE_A5,50},
					{REST,100},
					{NOTE_D5,50},
                    {REST,2000}
                    }}, //create array of melody note values
                    false // assign high priority to this
                    };