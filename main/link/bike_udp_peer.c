#include "bike_udp_peer.h"
#include "app_config.h"

bool bike_udp_peer_alive(const bike_udp_peer_t *p, uint64_t now_us)
{
    return p->registered && now_us >= p->last_seen_us &&
           now_us - p->last_seen_us < BIKE_PEER_LEASE_US;
}

bool bike_udp_peer_hello(bike_udp_peer_t *p, uint32_t address, uint16_t port, uint64_t now_us)
{
    if (bike_udp_peer_alive(p, now_us) && (p->address != address || p->port != port)) return false;
    p->address = address;
    p->port = port;
    p->last_seen_us = now_us;
    p->registered = true;
    return true;
}

bool bike_udp_peer_command(bike_udp_peer_t *p, uint32_t address, uint16_t port, uint64_t now_us)
{
    if (!bike_udp_peer_alive(p, now_us) || p->address != address || p->port != port) return false;
    p->last_seen_us = now_us;
    return true;
}
