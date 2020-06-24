#include "Pong.hpp"

#include <cstdio>

void Pong::start() {
    tickOnMessage(rx_trigger());
}

void Pong::tick() {
    auto proto = rx_trigger().getProto();
    const std::string message = proto.getMessage();

    const int num_beeps = get_count();
    std::printf("%s:", message.c_str());
    for (int i = 0; i < num_beeps ; i++){
        std::printf(" nah!");
    }
    if (num_beeps > 0){
        std::printf("\n");
    }
}