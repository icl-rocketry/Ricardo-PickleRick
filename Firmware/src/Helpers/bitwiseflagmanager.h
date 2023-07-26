/**
 * @file bitwiseflagmanager.h
 * @author Kiran de Silva
 * @brief Class manages the tracking of bitwise flags and also includes templated methods to check if multiple flgas are triggered
 * @version 0.1
 * @date 2022-07-23
 *
 * @copyright Copyright (c) 2022
 *
 */

#pragma once
#include <type_traits>

template<typename T,typename T_underlying = typename std::underlying_type<T>::type>
class BitwiseFlagManager
{
   static_assert(std::is_integral_v<T_underlying>,"Underlying Type must be int castable");

public:
    BitwiseFlagManager() : _statusString(static_cast<T_underlying>(0)){};

    BitwiseFlagManager(T initalStatus) : _statusString(static_cast<T_underlying>(initalStatus)){};

    BitwiseFlagManager(T_underlying initalStatus) : _statusString(initalStatus){};

    T_underlying getStatus() const { return _statusString; };

    virtual void newFlag(T flag)
    {
        _statusString |= static_cast<T_underlying>(flag);
    };
    virtual void deleteFlag(T flag)
    {
        _statusString &= ~static_cast<T_underlying>(flag);
    };

    bool flagSet(T flag) const
    {
        return flagSetOr(flag); 
    };

    /**
     * @brief Checks if multiple flags have been triggered e.g flag1 or flag2 or ...
     *
     * @param args
     * @return true
     * @return false
     */

    template <typename... Args>
    bool flagSetOr(Args... args) const
    {
        T_underlying flags = (... | static_cast<T_underlying>(args));
        return (_statusString & flags);
    };

        /**
     * @brief Checks if all multiple flags have been triggered e.g flag1 and flag2 and ...
     *
     * @param args
     * @return true
     * @return false
     */

    template <typename... Args>
    bool flagSetAnd(Args... args) const
    {
        T_underlying flags = (... | static_cast<T_underlying>(args));
        return ((_statusString & flags) >= flags);
    };

    virtual ~BitwiseFlagManager(){};

protected:
    T_underlying _statusString;
};



