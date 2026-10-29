#ifndef BIKE_LINK_GUARD_H
#define BIKE_LINK_GUARD_H

#include "bike_protocol.h"

typedef struct {
    uint32_t boot_id;
    uint32_t sequence[4];
    uint64_t timestamp[4];
    unsigned next;
    uint32_t last_sequence, round_trip_us;
    uint64_t source_timestamp_us;
    float torque_nm;
    bool seen_command, valid;
} bike_link_guard_t;

void bike_link_guard_init(bike_link_guard_t *guard, uint32_t boot_id);
void bike_link_guard_sent(bike_link_guard_t *guard, uint32_t sequence, uint64_t timestamp);
bool bike_link_guard_accept(bike_link_guard_t *guard, uint32_t sequence,
                            const bike_wire_command_t *command, uint64_t now_us);
bool bike_link_guard_alive(const bike_link_guard_t *guard, uint64_t now_us);

#endif
