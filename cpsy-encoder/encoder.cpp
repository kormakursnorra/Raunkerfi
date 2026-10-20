#include <cassert>

#include "encoder.hpp"



void afunc(int e, lgGpioAlert_p evt, void *data);


/*
 * We describe the quadrature decoder by a Mealy machine. `state_t` defines the states,
 * `transitions` defines the transition matrix (first index is a state and the second one the symbol).
 * The function output defines the output symbol (-1, 0, and 1) which we use to increment the step counter.
 */
const state_t transitions[7][4] = {
    { QDECODER_STATE_IDLE, QDECODER_STATE_CCW1, QDECODER_STATE_CW1, QDECODER_STATE_IDLE },  // IDLE
    { QDECODER_STATE_IDLE, QDECODER_STATE_CCW1, QDECODER_STATE_CCW3, QDECODER_STATE_CCW2 }, // CCW1
    { QDECODER_STATE_IDLE, QDECODER_STATE_CCW1, QDECODER_STATE_CCW3, QDECODER_STATE_CCW2 }, // CCW2
    { QDECODER_STATE_IDLE, QDECODER_STATE_IDLE, QDECODER_STATE_CCW3, QDECODER_STATE_CCW2 }, // CCW3
    { QDECODER_STATE_IDLE, QDECODER_STATE_CW3, QDECODER_STATE_CW1, QDECODER_STATE_CW2 },    // CW1
    { QDECODER_STATE_IDLE, QDECODER_STATE_CW3, QDECODER_STATE_CW1, QDECODER_STATE_CW2 },    // CW2
    { QDECODER_STATE_IDLE, QDECODER_STATE_CW3, QDECODER_STATE_IDLE, QDECODER_STATE_CW2 },   // CW3
};


/* We describe the output of the transition as a function instead of a table. */
inline int output(const state_t from, const uint_fast8_t symbol)
{
    if (QDECODER_STATE_CCW3 == from && 0 == symbol)
        return -1;
    if (QDECODER_STATE_CW3 == from && 0 == symbol)
        return 1;
    return 0;
}


encoder_state::encoder_state(uint8_t chip, uint8_t _pin1, uint8_t _pin2) noexcept : last_symbol(0), qdecoder_state(QDECODER_STATE_IDLE), pin1(_pin1), pin2(_pin2), steps(0), timestamp(0), inputs(255)
{
    lgGpioSetAlertsFunc(chip, pin1, afunc, this);
    lgGpioSetAlertsFunc(chip, pin2, afunc, this);
    lgGpioClaimAlert(chip, LG_SET_PULL_UP, LG_BOTH_EDGES, pin1, -1);
    lgGpioClaimAlert(chip, LG_SET_PULL_UP, LG_BOTH_EDGES, pin2, -1);
}

/* Obtain the symbol from an event and the current state.
 *
 * Each event records the change of a single pin, so we change the last known symbol,
 * a pair of pin values, to calculate the next pin.
 */
uint_fast8_t
encoder_state::symbol(const lgGpioReport_t &report) const noexcept
{
    assert((pin1 == report.gpio) || (pin2 == report.gpio));
    if (report.gpio == pin1)
        return (last_symbol & ~1) | static_cast<uint_fast8_t>(report.level);
    return (last_symbol & ~2) | (static_cast<uint_fast8_t>(report.level) << 1);
}


/* Ubdate the state of the quadrature decoder machine. */
inline void
encoder_state::step(const lgGpioReport_t &event) noexcept
{
#if 0
    const uint64_t last_ts = timestamp.load();
    if (event.timestamp < last_ts) {
	const double lateness = static_cast<double>(last_ts - event.timestamp) /* ns */ / 1000.0;
        printf("Event %" PRIu64 " processed %10.3fus late\n", event.timestamp, lateness);
    }
    timestamp.store(std::max(last_ts, event.timestamp));
#endif
    state_t from = qdecoder_state;
    last_symbol = symbol(event);
    qdecoder_state = transitions[from][last_symbol];
    steps += output(from, last_symbol);
#if 0
    printf("   [%d ---%d/%d---> %d] steps=%" PRId64 "\n", state.qdecoder_state, state.symbol, output(from, state.qdecoder_state), state.qdecoder_state, state.steps);
#endif
}


/* Run the state machine for `steps` steps, or as long as the input queue is empty.
 * Choose the number of steps to empty the queue in most of the situations.
 */
void
encoder_state::run(const int steps)
{
    lgGpioReport_t event;
    for (int i = 0; i < steps; i++) {
        if (!inputs.try_pop(event))
            return;
        step(event);
    }
}

/* Callback function.
 * When the value of a pin changes, this function is called to enqueue the event into our
 * priotrity queue.
 *
 * This function is thread-safe.
 */
void afunc(int e, lgGpioAlert_p evt, void *data)
{
    encoder_state *state = reinterpret_cast<encoder_state *>(data);

    for (int i=0; i<e; i++) {
        const lgGpioReport_t &report = evt[i].report;
        if (report.flags != 0) {
            continue;
        }
        state->inputs.push(report);
    }
}

