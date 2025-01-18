#pragma once

#include <vector> 
#include <queue>
#include <array>
#include <algorithm>
#include <array>
#include <Eigen/Dense>


/**
 * @brief Simple ringbuf implementation based on std::array
 * 
 * @tparam T 
 * @tparam LEN 
 */
template<typename T,size_t LEN>
class RingBuf : private std::array<T,LEN> {
    public:        
        RingBuf(T defaultValue = {}):
        idx(0),
        curr_size(0),
        m_defaultValue(defaultValue)
        {};

        /**
         * @brief inserts a new value at the front and returns the removed value. 
         * Returns zero if len != maxLen
         * 
         * @param val 
         * @return T 
         */
        T pop_push_back(T val){
            
            //retrieve old val
            T oldVal = (*this)[idx];
            // push new element on queue
            (*this)[idx] = val;
            
            //increment idx, implement rollover
            idx++;

            if (idx >= LEN) idx = 0;
        
            //increment curr_size up to max size (LEN)
            curr_size = std::min(LEN,curr_size+1);

            if (curr_size == LEN){
                return oldVal; // the next index represents the oldest value in the ring buffer
            } 

            return m_defaultValue;

        };

        size_t size()
        {
            return curr_size;
        }

    private:
    
        size_t idx;
        size_t curr_size;
        const T m_defaultValue;

};

struct ApogeeInfo{
    bool reached;
    float altitude;
    uint32_t time; // time in ms
};


class ApogeeDetect
{
public:
/**
 * @brief Construct a new Apogee Detect object
 * 
 * @param sampleTime  in millis
 */
    ApogeeDetect(uint16_t sampleTime);
    /**
     * @brief 
     * 
     * @param altitude expects up +ve be careful!
     * @param velocity +ve up!
     * @param time 
     * @return const ApogeeInfo& 
     */
    const ApogeeInfo& checkApogee(float altitude, float velocity, uint32_t time);       //create function in the memory address of the structure to estimate the apogee

private:
    // int len_time; // The length of the time
    static constexpr int arrayLen = 100;                  //polyval takes elements 3:5 to approximate the apogee, so 5 elements are required
    
    const uint16_t _sampleTime;
    uint32_t prevCheckApogeeTime{0};
    uint32_t initialEntryTime{0};


    RingBuf<uint32_t,arrayLen> time_array;           //create arrays to store recent flight history for apogee approximation
    RingBuf<float,arrayLen> altitude_array;


    /**
     * @brief Returns the coefficents of a 2nd order fitted polynomial
     * 
     * @param time_array 
     * @param altitude_array 
     * @return std::array<float,2> 
     */
    void quadraticFit(float oldTime, float newTime, float oldAlt, float newAlt);
    //sums for fitting polynomial
    float sigmaTime;
    float sigmaTime_2;
    float sigmaTime_3;
    float sigmaTime_4;
    float sigmaAlt;
    float sigmaAltTime;
    float sigmaAltTime_2;

    void updateSigmas(float oldTime, float newTime, float oldAlt, float newAlt);
    

    Eigen::Matrix3f A;
    Eigen::Vector3f b;
    Eigen::Vector3f coeffs;


    uint32_t prevMachLockTime;
    bool mlock;         // Mach lockout activated by default on launch
    static constexpr float mlock_speed = 30; //value chosen thru tuning -> depends on filter performance
    static constexpr int mlock_time = 1000; // lockout time in milliseconds

    static constexpr float alt_threshold = 0; //threshold to detect altitude descent
    static constexpr float alt_min = 100;   // Minimum altitude (m) before apogee detection algorithm works
      
    ApogeeInfo _apogeeinfo;                             //create the structure for ApogeeInfo

};

