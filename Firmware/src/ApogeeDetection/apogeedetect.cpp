

#include "apogeedetect.h"
#include <iostream>
#include <vector>
#include <Eigen/Core>
#include <Arduino.h>

#include <libriccore/riccorelogging.h>

#include <sstream>
// #include "millis.h"

ApogeeDetect::ApogeeDetect(uint16_t sampleTime) : 
                                                _sampleTime(sampleTime),
                                                mlock(true), // intialise the mach lockout stuff
                                                _apogeeinfo({false, 0, 0})
{
}

// Function to update flight data values and return data for apogee prediction
const ApogeeInfo &ApogeeDetect::checkApogee(float altitude, float velocity, uint32_t time)
{
    if(!initialEntryTime)
    {
        initialEntryTime = time; // recording first time this method is called to scale the system better
    }

    

    if (millis() - prevCheckApogeeTime > _sampleTime)
    {
        uint32_t timeSinceEntry = time-initialEntryTime;

        uint32_t prevTime = time_array.pop_push_back(timeSinceEntry);
        float prevAltitude = altitude_array.pop_push_back(altitude);

        // Mach lock check:
        if (abs(velocity) >= mlock_speed)
        {
            if (!mlock){
                RicCoreLogging::log<RicCoreLoggingConfig::LOGGERS::SYS>("Mach Lock Triggered!");
            }
            mlock = true;
            prevMachLockTime = millis();

            // log time
        }
        else if ((millis() - prevMachLockTime) > mlock_time) // if more than a second has passed and vel is less than mlock_speed
        {
            mlock = false;
            RicCoreLogging::log<RicCoreLoggingConfig::LOGGERS::SYS>("Mach unlocked");
            // log the time
        }

        // Apogee detection:

        if (!(_apogeeinfo.reached))
        {
            // coeffs = poly2fit(time_array, altitude_array); // POLYFIT -> x(t), time in ms
            quadraticFit((float)prevTime/1000.0, (float)timeSinceEntry/1000.0, prevAltitude, altitude);

            _apogeeinfo.time = ((-coeffs(1) / (2 * coeffs(2)))*1000)+initialEntryTime; // maximum from polyinomial using derivative
            RicCoreLogging::log<RicCoreLoggingConfig::LOGGERS::SYS>(std::to_string(_apogeeinfo.time));
            RicCoreLogging::log<RicCoreLoggingConfig::LOGGERS::SYS>("coeffs: " + std::to_string(coeffs(0)) + " " + std::to_string(coeffs(1)) + " " + std::to_string(coeffs(2)));

            if ((millis() >= _apogeeinfo.time) && (coeffs(2) < 0) && (millis() > 0) && (altitude > alt_min) && !mlock)
            {
                _apogeeinfo.altitude = coeffs(0) - (std::pow(coeffs(1), 2) / (4 * coeffs(2)));
                RicCoreLogging::log<RicCoreLoggingConfig::LOGGERS::SYS>("predicted apogee" + std::to_string(_apogeeinfo.time));
                // coeffs(2) * std::pow(_apogeeinfo.time,2) + (coeffs(1) * _apogeeinfo.time) + coeffs(0); // evalute 2nd order polynomial
                RicCoreLogging::log<RicCoreLoggingConfig::LOGGERS::SYS>(std::to_string(altitude - _apogeeinfo.altitude));
                if ((altitude - _apogeeinfo.altitude) < alt_threshold) // if we have passed apogee and now decending, could put a bound on here too
                {
                    _apogeeinfo.reached = true;
                    // log apogee time and altitude
                }
            }
        }
        prevCheckApogeeTime = millis();
    }

    return _apogeeinfo;
};

void ApogeeDetect::updateSigmas(float oldTime, float newTime, float oldAlt, float newAlt)
{
    float newTime_2 = newTime*newTime;
    float newTime_3 = newTime_2*newTime;
    float newTime_4 = newTime_3*newTime;
    float oldTime_2 = oldTime*oldTime;
    float oldTime_3 = oldTime_2*oldTime;
    float oldTime_4 = oldTime_3*oldTime;


    sigmaTime += (newTime - oldTime);
    sigmaTime_2 += newTime_2 - oldTime_2;
    sigmaTime_3 += newTime_3 - oldTime_3;
    sigmaTime_4 += newTime_4 - oldTime_4;

    sigmaAlt += (newAlt - oldAlt);

    sigmaAltTime += ((newAlt * (newTime)) - (oldAlt * (oldTime)));
    sigmaAltTime_2 += ((newAlt * newTime_2) - (oldAlt * oldTime_2));

    RicCoreLogging::log<RicCoreLoggingConfig::LOGGERS::SYS>(std::to_string(sigmaTime) + "\t" + std::to_string(sigmaTime) + "\t" + std::to_string(sigmaTime_2) + "\t" + std::to_string(sigmaTime_3) + "\t" + std::to_string(sigmaTime_4) + "\t" + std::to_string(sigmaAlt) + "\t" +std::to_string(sigmaAltTime) + "\t" + std::to_string(sigmaAltTime_2) + "\t" );
};

/*Create a matrix, three simulatneos equations */
void ApogeeDetect::quadraticFit(float oldTime, float newTime, float oldAlt, float newAlt)
{
    updateSigmas(oldTime, newTime, oldAlt, newAlt); // update sigmas with new values and remove old values
    // re populate the arrays
    A << float(altitude_array.size()), sigmaTime, sigmaTime_2,
        sigmaTime, sigmaTime_2, sigmaTime_3,
        sigmaTime_2, sigmaTime_3, sigmaTime_4;
    
    std::stringstream a;
    a<<A;
    RicCoreLogging::log<RicCoreLoggingConfig::LOGGERS::SYS>(a.str());


    b << sigmaAlt, sigmaAltTime, sigmaAltTime_2;

    std::stringstream b_str;
    b_str<<b;
    RicCoreLogging::log<RicCoreLoggingConfig::LOGGERS::SYS>(b_str.str());
    // solve the system for the coefficents 
    coeffs = A.colPivHouseholderQr().solve(b);
}
