#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cinttypes>
#include <signal.h>

#include "encoder.hpp"

/* The configuration is currently hard coded. Redefine these macros to the values
 * You need or implement a configuration file mechanism. */
constexpr uint8_t GPIO_A1 = 6;
constexpr uint8_t GPIO_A2 = 5;
constexpr uint8_t GPIO_B1 = 24;
constexpr uint8_t GPIO_B2 = 23;

constexpr int RATE =  40; /* Once every 25ms */
constexpr int ITERS = RATE / 4;
// We expect about 2.75 revolutions per second, or 2.75 * 700 * 4 pin changes. We add a few more for slack.
constexpr int STEPS = 3 * 700 * 4 / RATE;


/* Global variables. */
int chip;
encoder_state *left, *right;

/* When we exit. */
void exit_handler()
{
    int status = lgGpiochipClose(chip);
    if (status < 0) {
        printf("Failed to close gpiochip %d\n", chip);
    }
    printf("left = %" PRId64 " right = %" PRId64 "\n", left->steps.load(), right->steps.load());
}

void sigterm_handler(int signum)
{
    printf("Received SIGTERM.\n");
    exit(EXIT_SUCCESS);
}

int main()
{
    struct sigaction sa;
    sa.sa_handler = sigterm_handler;
    sigemptyset(&sa.sa_mask);
    sa.sa_flags = 0;

    if (sigaction(SIGTERM, &sa, NULL) == -1) {
        perror("sigaction");
        exit(EXIT_FAILURE);
    }

    chip = lgGpiochipOpen(0);
    if (chip < 0) {
        // TODO: Handle error
        exit(EXIT_FAILURE);
    }
    atexit(exit_handler);

    encoder_state _left(chip, GPIO_A1, GPIO_A2), _right(chip, GPIO_B1, GPIO_B2);
    left = &_left;
    right = &_right;

    for (;;) {
        for (int i = 0; i < ITERS; i++) {
            lguSleep(1.0/static_cast<double>(RATE));
            _left.run(STEPS);
            _right.run(STEPS);
        }
        printf("left = %" PRId64 " right = %" PRId64 "\n", _left.steps.load(), _right.steps.load());
    }
}
