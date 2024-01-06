#pragma once
/**
 * @file engine.h
 * @author Kiran de Silva
 * @brief Engine Base class
 * @version 0.1
 * @date 2022-04-01
 *
 * @copyright Copyright (c) 2022
 *
 */

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

enum class ENGINE_RUN_STATE : uint8_t
{
    SHUTDOWN,
    RUNNING,
    IGNITION,
    ERROR
};

enum class ENGINE_CONNECTION_STATE : uint8_t
{
    CONNECTED,
    ERROR
};

struct EngineState
{
    uint8_t runState;
    uint8_t connectionState;
    uint32_t ignitionTime;
    uint32_t shutdownTime;
};
using addNetworkCallbackFunction_t = std::function<void(uint8_t, uint8_t, std::function<void(std::unique_ptr<RnpPacketSerialized>)>, bool)>;
class Engine : public Controllable
{
public:
    /**
     * @brief Construct a new Engine object, setup the engine, as the engine also could contain networked compoentns, a reference to he addNetworkCallback function is passed by reference so we can add
     * the appropriate callback allowing incomming packets to be routed to the correct component.
     *
     * @param id
     * @param engineConfig
     * @param addNetworkCallbackF
     * @param networkmanager
     * @param handlerServiceID
     */
    // Engine(uint8_t id, [[maybe_unused]] JsonObjectConst engineConfig, [[maybe_unused]] addNetworkCallbackFunction_t addNetworkCallbackF, RnpNetworkManager &networkmanager, uint8_t handlerServiceID, LogController &logcontroller) : _id(id),
    //                                                                                                                                                                                                                                   _networkmanager(networkmanager),
    //                                                                                                                                                                                                                                   _handlerServiceID(handlerServiceID),
    //                                                                                                                                                                                                                                   _logcontroller(logcontroller){};
    Engine(uint8_t id, RnpNetworkManager &networkmanager, uint8_t handlerServiceID) : _id(id),
                                                                                      _networkmanager(networkmanager),
                                                                                      _handlerServiceID(handlerServiceID){};

    /**
     * @brief Requests a a state update for all components in engine
     *
     */
    virtual void updateState() = 0;
    /**
     * @brief Performs Flight check for all components in the engine
     *
     * @return uint8_t is engine in error
     */
    uint8_t flightCheckEngine() { return flightCheck() ? 1 : 0; };
    /**
     * @brief Performs flight check on engine
     *
     * @return uint8_t number of engine components in error
     */
    virtual uint8_t flightCheck() = 0;

    virtual void armEngine() = 0;

    virtual void update() = 0;

    /**
     * @brief Engine functor
     *          param = 0 is shutdown
     *          param = 1 is ignition
     *          param = n is defined by the derived implementation i.e in hypnos p3 = vent main ox
     *
     * @param func
     */
    virtual void execute(int32_t func);

    const EngineState *getState() { return getStatePtr(); };

    virtual ~Engine();

    uint8_t getID() { return _id; };

protected:
    const uint8_t _id;

    RnpNetworkManager &_networkmanager;

    const uint8_t _handlerServiceID;

    virtual EngineState *getStatePtr() = 0;

    // all engines will have these methods
    virtual void ignite();
    virtual void shutdown();

    /**
     * @brief helper function for logging engine status and error messages within any engine. Override within child class if required to change log target
     * or log message. 
     * 
     * @param message 
     */
    virtual void log(std::string_view message){
        RicCoreLogging::log<RicCoreLoggingConfig::LOGGERS::SYS>("Engine:" + std::to_string(getID()) + std::string(message));
    }

    /**
     * @brief Generate a log callback lambda, used to pass log function to flight components of engine
     * 
     * @return std::function<void(std::string_view)> 
     */
    std::function<void(std::string_view)> getLogCB()
    {
        return [this](std::string_view msg){this->log(msg);}; 
    }

    
    /**
     * @brief Base engine executor ID definition. Always check this when extending the executor
     * in a derived engine class!
     *
     */
    enum class ENGINE_EXECUTE : uint8_t
    {
        SHUTDOWN = 0,
        IGNITE = 1
    };
};