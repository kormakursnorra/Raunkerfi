
#pragma once

#ifndef INCLUDED_ENCODER_HPP
#define INCLUDED_ENCODER_HPP 1

#include <atomic>
#include <cstdint>
#include <oneapi/tbb.h>
#include <lgpio.h>

/* To order events of type `lgGpioReport_t` by their timestamp. */
class TimeStampOrder {
public:
	inline bool operator()(const lgGpioReport_t &first, const lgGpioReport_t &second) const noexcept {
    		return first.timestamp > second.timestamp;
	}
};


typedef enum : uint_fast8_t {
    QDECODER_STATE_IDLE,
    QDECODER_STATE_CCW1,
    QDECODER_STATE_CCW2,
    QDECODER_STATE_CCW3,
    QDECODER_STATE_CW1,
    QDECODER_STATE_CW2,
    QDECODER_STATE_CW3,
} state_t;

class encoder_state {
public:
    uint_fast8_t last_symbol;
    state_t qdecoder_state;
    const uint8_t pin1, pin2;
    std::atomic_int64_t steps;
    std::atomic_uint64_t timestamp;
    oneapi::tbb::concurrent_priority_queue<lgGpioReport_t, TimeStampOrder> inputs;

    // We do not want to have the copy constructor, copy assignment constructor, and move assignment operator.
    encoder_state() = delete;
    encoder_state(uint8_t chip, uint8_t _pin1, uint8_t _pin2) noexcept;
    ~encoder_state() = default;
    encoder_state(encoder_state&) = delete;
    encoder_state& operator=(const encoder_state&) = delete;
    encoder_state& operator=(encoder_state&&) = delete;

    uint_fast8_t symbol(const lgGpioReport_t&) const noexcept;
    void step(const lgGpioReport_t&) noexcept;
    void run(const int steps);
};

#endif
