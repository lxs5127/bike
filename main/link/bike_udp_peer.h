#ifndef BIKE_UDP_PEER_H
#define BIKE_UDP_PEER_H

#include <stdbool.h>
#include <stdint.h>

/* One UDP controller endpoint. IP/port stored in network byte order. */
typedef struct {
    uint32_t address;
    uint16_t port;
    uint64_t last_seen_us;
    bool registered;
} bike_udp_peer_t;

bool bike_udp_peer_alive(const bike_udp_peer_t *peer, uint64_t now_us);
bool bike_udp_peer_hello(bike_udp_peer_t *peer, uint32_t address, uint16_t port, uint64_t now_us);
bool bike_udp_peer_command(bike_udp_peer_t *peer, uint32_t address, uint16_t port, uint64_t now_us);

#endif
