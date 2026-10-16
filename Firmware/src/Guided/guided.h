#pragma once
#include <memory>
#include <functional>
#include <unordered_map>
#include <vector>
#include <utility>

#include <librnp/rnp_networkmanager.h>
#include <ArduinoJson.h>

#include "Controller/controllable.h"

#include <librnp/rnp_networkcallbackmap.h>
#include <librrc/Interface/rocketcomponent.h>

#include <libriccore/riccorelogging.h>

enum class GUIDED_RUN_STATE : uint8_t
{   
    IDLE,
    STOPPING,
    CONTROLLED,
    SOFT_ABORT,
    HARD_ABORT,
    ERROR,
    DEBUG // for demoing actuation
};

enum class GUIDED_CONNECTION_STATE : uint8_t
{
    CONNECTED,
    ERROR
};

struct GuidedState
{
    uint8_t runState = static_cast<uint8_t>(GUIDED_RUN_STATE::IDLE);
    uint8_t connectionState = static_cast<uint8_t>(GUIDED_CONNECTION_STATE::CONNECTED);
    uint32_t startTime = 0;
    uint32_t stopTime = 0;
    uint32_t softAbortTime = 0;
    uint32_t hardAbortTime = 0;
};
using addNetworkCallbackFunction_t = std::function<void(uint8_t, uint8_t, std::function<void(std::unique_ptr<RnpPacketSerialized>)>, bool)>;
class Guided : public Controllable
{
public:
    /**
     * @brief Construct a new Guided object, setup the guided, as the guided also could contain networked compoentns, a reference to he addNetworkCallback function is passed by reference so we can add
     * the appropriate callback allowing incomming packets to be routed to the correct component.
     *
     * @param id
     * @param guidedConfig
     * @param addNetworkCallbackF
     * @param networkmanager
     * @param handlerServiceID
     */
    // Guided(uint8_t id, [[maybe_unused]] JsonObjectConst guidedConfig, [[maybe_unused]] addNetworkCallbackFunction_t addNetworkCallbackF, RnpNetworkManager &networkmanager, uint8_t handlerServiceID, LogController &logcontroller) : _id(id),
    //                                                                                                                                                                                                                                   _networkmanager(networkmanager),
    //                                                                                                                                                                                                                                   _handlerServiceID(handlerServiceID),
    //                                                                                                                                                                                                                                   _logcontroller(logcontroller){};
    Guided(uint8_t id, RnpNetworkManager &networkmanager, uint8_t handlerServiceID) : _id(id),
                                                                                      _networkmanager(networkmanager),
                                                                                      _handlerServiceID(handlerServiceID){};

    /**
     * @brief Requests a a state update for all components in guided
     *
     */
    virtual void updateState() = 0;
    /**
     * @brief Performs Flight check for all components in the guided
     *
     * @return uint8_t is guided in error
     */
    uint8_t flightCheckGuided() { return flightCheck() ? 1 : 0; };
    /**
     * @brief Performs flight check on guided
     *
     * @return uint8_t number of guided components in error
     */
    virtual uint8_t flightCheck() = 0;

    virtual void armGuided() = 0;

    virtual void disarmGuided() = 0;

    virtual void update() = 0;

    /**
     * @brief Guided functor
     *          param = 0 is stop
     *          param = 1 is start
     *          param = 2 is soft abort
     *          param = 3 is hard abort
     *          param = 4 is reset
     *          param = n is defined by the derived implementation 
     *
     * @param func
     */
    virtual void execute(int32_t func);

    const GuidedState *getState() { return getStatePtr(); };

    virtual ~Guided();

    uint8_t getID() { return _id; };

protected:
    const uint8_t _id;

    RnpNetworkManager &_networkmanager;

    const uint8_t _handlerServiceID;

    virtual GuidedState *getStatePtr() = 0;

    // all guideds will have these methods
    virtual void stop();
    virtual void start();
    virtual void softAbort();
    virtual void hardAbort();
    virtual void reset();
    virtual void debug();
    virtual void endHardAbort(); // helper function to end hard abort after a set time, can be overridden in derived class if required

    /**
     * @brief helper function for logging guided status and error messages within any guided. Override within child class if required to change log target
     * or log message. 
     * 
     * @param message 
     */
    virtual void log(std::string_view message){
        RicCoreLogging::log<RicCoreLoggingConfig::LOGGERS::SYS>("Guided:" + std::to_string(getID()) + std::string(message));
    }

    /**
     * @brief Generate a log callback lambda, used to pass log function to flight components of guided
     * 
     * @return std::function<void(std::string_view)> 
     */
    std::function<void(std::string_view)> getLogCB()
    {
        return [this](std::string_view msg){this->log(msg);}; 
    }

    
    /**
     * @brief Base guided executor ID definition. Always check this when extending the executor
     * in a derived guided class!
     *
     */
    enum class GUIDED_EXECUTE : uint8_t
    {
        STOP = 0, // when we touch the ground
        START = 1,
        SOFT_ABORT = 2,
        HARD_ABORT = 3,
        RESET = 4,
        DEBUG = 5 // for demoing actuation
    };

};