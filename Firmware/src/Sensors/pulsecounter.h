/* To Do List Aprt from the code:
PCNT is in components/libricccore/src/libriccore/platform/esp32/PCNT.h
Put delecaration into system.cpp
*/

#pragma once
/**
 * @file pulsecounter.h
 * @author Wayne
 * @brief Hardware pulse counter for a GPIO sensor input line, plus an SD log frame/logger.
 * @version 0.1
 * @date 2026-10-17
 *
 * Counts one pulse per rising edge on a GPIO input and keeps a non-overflowing
 * 64-bit total. Counting is done by the ESP32-S3 PCNT peripheral so the CPU is
 * not involved per edge and no pulses are dropped.
 **/

#include <cstdint>
#include <exception>
#include <memory>
#include <string>
#include <vector>

#include <driver/pcnt.h>
#include <driver/gpio.h>
#include <esp_err.h>
#include <esp_attr.h>

#include <librnp/rnp_serializer.h>
#include <libriccore/storage/wrappedfile.h>
#include "Sensors/sensorStructs.h"
#include "Config/types.h"
/**
 * @brief Counts rising edges on one GPIO input line using the hardware PCNT unit.
 *
 * The PCNT counter is 16-bit and saturates at HW_LIMIT. A high-limit interrupt
 * rolls the hardware counter over into a 32-bit software wrap counter, so the
 * reported total never overflows or loses pulses regardless of flight duration.
 */
class PulseCounter
{
public:
    /**
     * @brief Hardware counter full scale. PCNT is a 16-bit signed register; with
     *        counter_l_lim set to 0 this gives a usable range of 0..32767.
     *
     *        Keep this at the maximum. Between reaching the limit and the ISR
     *        clearing it, the hardware counter is pinned and any further pulses
     *        are lost - so a larger limit means rollovers are rarer and a smaller
     *        fraction of pulses fall into that window.
     */
    static constexpr int16_t HW_LIMIT = 32767;

    /**
     * @brief PCNT input filter length, in APB_CLK cycles (80 MHz -> 12.5 ns).
     *        10-bit field, so 1023 is the maximum (~12.8 us).
     */
    static constexpr uint16_t FILTER_CYCLES = 1000; // ~12.5 us

    /**
     * @param unit    PCNT unit. Use a different unit per counter; the S3 has four.
     * @param signalPin GPIO the sensor signal wire is on. Pass a PinMap constant.
     * @param enablePin GPIO that = 1 when detection wanted
     */
    PulseCounter(pcnt_unit_t unit, gpio_num_t signalPin, gpio_num_t enablePin, Types::CoreTypes::SystemStatus_t& systemstatus):
        _unit(unit),             
        _pinSig(signalPin),
        _pinEna(enablePin),
        _wraps(0),               
        _lastWindow(0),          
        _initialized(false),
        _isrHandle(nullptr),
        _systemstatus(systemstatus)
    {}

    /**
     * @brief Configure the PCNT unit and the input pin. Call once from setup().
     * @return true on success, false if the PCNT unit could not be configured.
     */
    bool setup()
    {
        if (_initialized) { return true; }

        /* Field order below must match the pcnt_config_t declaration exactly.
         * lctrl/hctrl come before pos/neg in IDF 5.3 - see the header notes. */
        pcnt_config_t config = {};
        config.pulse_gpio_num = static_cast<int>(_pinSig);
        config.ctrl_gpio_num  = static_cast<int>(_pinEna);
        config.lctrl_mode     = PCNT_MODE_DISABLE;   // When ctrl pulled low, stop counting
        config.hctrl_mode     = PCNT_MODE_KEEP;      // When ctrl pulled high, continue counting
        config.pos_mode       = PCNT_COUNT_INC;      // rising edge increments
        config.neg_mode       = PCNT_COUNT_DIS;      // falling edge does nothing
        config.counter_h_lim  = HW_LIMIT;            // highest count
        config.counter_l_lim  = 0;
        config.unit           = _unit;
        config.channel        = PCNT_CHANNEL_0;      //IIIIIIIIIIIIDDDDDDDDDDDDKKKKKKKKKKKK

        if (pcnt_unit_config(&config) != ESP_OK)
        {
            _systemstatus.newFlag(SYSTEM_FLAG::ERROR_Errradi, "pulsecounter init failed 1");

            return false;
        }
        /* pcnt_unit_config() disables the L_LIM, H_LIM and ZERO events, so the
         * H_LIM event has to be re-enabled here or the wrap interrupt never fires. */

        /* Input pin electrical configuration. */
        gpio_set_direction(_pinSig, GPIO_MODE_INPUT);
        gpio_set_direction(_pinEna, GPIO_MODE_INPUT); 
        gpio_set_intr_type(_pinSig, GPIO_INTR_DISABLE);
        gpio_set_intr_type(_pinEna, GPIO_INTR_DISABLE); // counting is PCNT's not GPIO's job, so don't let GPIO interrupt

        /* Input glitch filter. Covers electrical noise only, not contact bounce. */
        pcnt_set_filter_value(_unit, FILTER_CYCLES);
        pcnt_filter_enable(_unit);

        /* Roll the hardware counter over into software on every H_LIM event. */
        if (pcnt_isr_register(_isrHandler, this, 0, &_isrHandle) != ESP_OK)
        {
            _systemstatus.newFlag(SYSTEM_FLAG::ERROR_Errradi, "pulsecounter init failed 2");

            return false;
        }
        pcnt_event_enable(_unit, PCNT_EVT_H_LIM);  //high limit reroll enabled
        pcnt_intr_enable(_unit);                   //interrupt enabled

        /* Clear any stale counter value before starting (known PCNT quirk). */
        pcnt_counter_pause(_unit);
        pcnt_counter_clear(_unit);
        pcnt_counter_resume(_unit);

        /* Drive the enable pin high so counting is always enabled. */
        gpio_set_direction(_pinEna, GPIO_MODE_OUTPUT);
        gpio_set_level(_pinEna, 1);

        _initialized = true;
        return true;
        
    }

    /**
     * @brief Refresh the window count and return the running total.
     *        Call periodically from the main loop; the call rate does not affect
     *        counting accuracy since the hardware counts every edge regardless.
     * @return Total rising edges seen since setup()/reset().
     */
    void update(SensorStructs::PULSE_COUNTER_t data)
    {
        _lastWindow = readRaw();
        data.counts += static_cast<uint64_t>(_lastWindow);
    }

private:
    const pcnt_unit_t   _unit;      // Which Pulse Counter it exactly is
    const gpio_num_t    _pinSig;
    const gpio_num_t    _pinEna;

    volatile uint32_t _wraps;       // number of HW_LIMIT rollovers (i.e. num of iterations); written only by the ISR
    volatile uint32_t _lastWindow;  // pulses in the last update() window (between last update and this); main loop only

    bool _initialized;
    pcnt_isr_handle_t _isrHandle;
    Types::CoreTypes::SystemStatus_t& _systemstatus;


    /**
     * @brief Read the hardware counter, clamped to a sane range.
     */
    int16_t readRaw() const
    {
        int16_t raw = 0;
        pcnt_get_counter_value(_unit, &raw);
        return raw;
    }

    /**
     * @brief H_LIM interrupt: bank one full HW_LIMIT of pulses and restart the
     *        hardware counter.
     *
     * Kept ISR-safe: only a 32-bit add and PCNT register access. No floating
     * point, no 64-bit atomics (not lock-free on a 32-bit core), no logging, no
     * allocation.
     *
     * This handler is deliberately NOT placed in IRAM (no IRAM_ATTR): the PCNT
     * calls inside it are flash-resident because CONFIG_PCNT_ISR_IRAM_SAFE is
     * disabled in sdkconfig, and placing the handler itself in IRAM makes the
     * linker emit "literal placed after use" relocations (the literal pool for
     * an IRAM function lands in flash, which sits above IRAM). That is safe
     * because the ISR is registered with intr_alloc_flags = 0 rather than
     * ESP_INTR_FLAG_IRAM, so it is never executed while the flash cache is
     * disabled.
     */
    static void _isrHandler(void *arg)  // static means it belongs to the whole PCounter, not specific object
    {
        PulseCounter *self = static_cast<PulseCounter *>(arg);

        uint32_t status = 0;
        pcnt_get_event_status(self->_unit, &status);
        if (status & PCNT_EVT_H_LIM)
        {
            self->_wraps = self->_wraps + 1;
            pcnt_counter_clear(self->_unit);
        }
    }
};


