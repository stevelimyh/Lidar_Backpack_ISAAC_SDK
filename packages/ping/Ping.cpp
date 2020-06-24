#include "Ping.hpp"
void Ping::start() {
    tickPeriodically();
}
void Ping::tick() {
    auto proto = tx_ping().initProto();
    proto.setMessage(get_message());
    tx_ping().publish();
}
void Ping::stop() {}