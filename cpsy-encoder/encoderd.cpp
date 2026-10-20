
#include <cassert>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cinttypes>
#include <algorithm>
#include <thread>
#include <sys/socket.h>
#include <sys/stat.h>
#include <sys/un.h>

#include "encoder.hpp"

#define SOCKET_NAME "/tmp/encoder.sock"

constexpr uint8_t GPIO_A1 = 6;
constexpr uint8_t GPIO_A2 = 5;
constexpr uint8_t GPIO_B1 = 24;
constexpr uint8_t GPIO_B2 = 23;

constexpr int RATE  = 40;
constexpr int STEPS = (4 * 3 * 700) / RATE;

uint8_t chip;
encoder_state *left, *right;


void exit_handler()
{
    int status = lgGpiochipClose(chip);
    if (status < 0) {
        // TODO: Log error
    }
    remove(SOCKET_NAME);
}


void server(void)
{
    int ret;
    char buffer[64];
    int connection = socket(AF_UNIX, SOCK_DGRAM, 0);
    if (connection == -1) {
        perror("socket");
        exit(EXIT_FAILURE);
    }
    struct sockaddr_un name{};
    remove(SOCKET_NAME);
    name.sun_family = AF_UNIX;
    strncpy(name.sun_path, SOCKET_NAME, sizeof(name.sun_path) - 1);
    ret = bind(connection, reinterpret_cast<const struct sockaddr *>(&name), sizeof(name));
    if (ret == -1) {
        perror("bind");
        exit(EXIT_FAILURE);
    }
    ret = chmod(SOCKET_NAME, S_IRUSR | S_IWUSR | S_IRGRP | S_IWGRP | S_IROTH | S_IWOTH);
    if (ret == -1) {
        perror("chmod");
        exit(EXIT_FAILURE);
    }


    for (;;) {
        struct sockaddr_un client;
        socklen_t length = sizeof(client);

        ret = recvfrom(connection, buffer, sizeof(buffer), 0, reinterpret_cast<struct sockaddr *>(&client), &length);
        if (ret == -1) {
            perror("recvfrom");
            exit(EXIT_FAILURE);
        }

        int len = snprintf(buffer, sizeof(buffer), "%" PRId64 ",%" PRId64, left->steps.load(), right->steps.load());
        ret = sendto(connection, buffer, len, 0, reinterpret_cast<struct sockaddr *>(&client), length);
        if (ret == -1) {
            perror("sendto");
            exit(EXIT_FAILURE);
        }
    }
}

int main()
{
    chip = lgGpiochipOpen(0);
    if (chip < 0) {
        // TODO: Handle error
        exit(1);
    }
    atexit(exit_handler);

    encoder_state _left{chip, GPIO_A1, GPIO_A2}, _right{chip, GPIO_B1, GPIO_B2};
    left = &_left;
    right = &_right;

    std::thread server_thread{server};
    server_thread.detach();

    for (;;) {
        lguSleep(1.0/static_cast<double>(RATE));
        _left.run(STEPS);
        _right.run(STEPS);
    }
}
