#include "bike_link_guard.h"

#include <math.h>
#include <string.h>
#include "app_config.h"
#include "bike_safety.h"

void bike_link_guard_init(bike_link_guard_t *g, uint32_t boot_id)
{
    memset(g, 0, sizeof(*g));
    g->boot_id = boot_id;
}

void bike_link_guard_sent(bike_link_guard_t *g, uint32_t sequence, uint64_t timestamp)
{
    g->sequence[g->next] = sequence;
    g->timestamp[g->next] = timestamp;
    g->next = (g->next + 1U) % 4U;
}

bool bike_link_guard_accept(bike_link_guard_t *g, uint32_t sequence,
                            const bike_wire_command_t *c, uint64_t now_us)
{
    if (c->boot_id != g->boot_id || !isfinite(c->torque_nm) ||
        fabsf(c->torque_nm) > BIKE_TORQUE_LIMIT_NM ||
        (c->flags & ~BIKE_COMMAND_STOP) != 0 || c->state_timestamp_us == 0 ||
        c->state_timestamp_us > now_us ||
        now_us - c->state_timestamp_us > BIKE_COMMAND_MAX_AGE_US) return false;
    uint32_t difference = sequence - g->last_sequence;
    if (g->seen_command && (difference == 0 || difference >= 0x80000000U)) return false;
    bool matched = false;
    for (unsigned i = 0; i < 4; ++i) {
        if (g->sequence[i] == sequence && g->timestamp[i] == c->state_timestamp_us) {
            matched = true;
        }
    }
    if (!matched) return false;
    g->seen_command = true;
    g->last_sequence = sequence;
    g->source_timestamp_us = c->state_timestamp_us;
    g->round_trip_us = (uint32_t)(now_us - c->state_timestamp_us);
    g->torque_nm = c->flags & BIKE_COMMAND_STOP ? 0.0f : c->torque_nm;
    g->valid = (c->flags & BIKE_COMMAND_STOP) == 0;
    return true;
}

bool bike_link_guard_alive(const bike_link_guard_t *g, uint64_t now_us)
{
    return g->valid && now_us >= g->source_timestamp_us &&
           now_us - g->source_timestamp_us < BIKE_LINK_TIMEOUT_US;
}
