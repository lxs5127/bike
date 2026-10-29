#include "bike_protocol.h"

#include <string.h>

static const uint8_t magic[4] = {'B', 'K', 'R', '1'};

static void put_u32(uint8_t *p, uint32_t value)
{
    for (unsigned i = 0; i < 4; ++i) p[i] = (uint8_t)(value >> (8 * i));
}

static uint32_t get_u32(const uint8_t *p)
{
    return (uint32_t)p[0] | (uint32_t)p[1] << 8 | (uint32_t)p[2] << 16 |
           (uint32_t)p[3] << 24;
}

static void put_u64(uint8_t *p, uint64_t value)
{
    put_u32(p, (uint32_t)value);
    put_u32(p + 4, (uint32_t)(value >> 32));
}

static uint64_t get_u64(const uint8_t *p)
{
    return get_u32(p) | (uint64_t)get_u32(p + 4) << 32;
}

static void put_float(uint8_t *p, float value)
{
    uint32_t bits;
    memcpy(&bits, &value, sizeof(bits));
    put_u32(p, bits);
}

static float get_float(const uint8_t *p)
{
    uint32_t bits = get_u32(p);
    float value;
    memcpy(&value, &bits, sizeof(value));
    return value;
}

uint32_t bike_crc32(const uint8_t *bytes, size_t size)
{
    uint32_t crc = UINT32_MAX;
    for (size_t i = 0; i < size; ++i) {
        crc ^= bytes[i];
        for (unsigned bit = 0; bit < 8; ++bit) {
            crc = (crc >> 1) ^ (0xedb88320U & (0U - (crc & 1U)));
        }
    }
    return ~crc;
}

static void header(uint8_t *p, uint8_t type, uint16_t size, uint32_t sequence)
{
    memcpy(p, magic, sizeof(magic));
    p[4] = 1;
    p[5] = type;
    p[6] = (uint8_t)size;
    p[7] = (uint8_t)(size >> 8);
    put_u32(p + 8, sequence);
}

bool bike_validate_packet(const uint8_t *p, size_t size, uint8_t type)
{
    size_t payload_size;
    if (type == BIKE_PACKET_STATE) payload_size = BIKE_STATE_PAYLOAD_SIZE;
    else if (type == BIKE_PACKET_COMMAND) payload_size = BIKE_COMMAND_PAYLOAD_SIZE;
    else if (type == BIKE_PACKET_HELLO) payload_size = 0;
    else return false;
    if (size != 16 + payload_size || memcmp(p, magic, 4) != 0 ||
        p[4] != 1 || p[5] != type || p[6] != payload_size || p[7] != 0) return false;
    return get_u32(p + size - 4) == bike_crc32(p, size - 4);
}

size_t bike_encode_state(uint8_t *p, uint32_t sequence, const bike_wire_state_t *s)
{
    header(p, BIKE_PACKET_STATE, BIKE_STATE_PAYLOAD_SIZE, sequence);
    put_u32(p + 12, s->boot_id);
    put_u64(p + 16, s->timestamp_us);
    put_u32(p + 24, s->imu_age_us);
    put_u32(p + 28, s->encoder_age_us);
    put_float(p + 32, s->roll_rad);
    put_float(p + 36, s->roll_rate_rad_s);
    put_float(p + 40, s->flywheel_rad_s);
    put_u32(p + 44, s->fault);
    put_u32(p + 48, s->flags);
    put_u32(p + 52, s->acknowledged_sequence);
    put_u32(p + 56, s->command_age_us);
    put_u32(p + 60, s->round_trip_us);
    put_u32(p + 64, bike_crc32(p, 64));
    return BIKE_PACKET_MAX_SIZE;
}

size_t bike_encode_command(uint8_t *p, uint32_t sequence, const bike_wire_command_t *c)
{
    header(p, BIKE_PACKET_COMMAND, BIKE_COMMAND_PAYLOAD_SIZE, sequence);
    put_u32(p + 12, c->boot_id);
    put_u64(p + 16, c->state_timestamp_us);
    put_float(p + 24, c->torque_nm);
    put_u32(p + 28, c->flags);
    put_u32(p + 32, bike_crc32(p, 32));
    return 36;
}

static void discard_byte(bike_command_parser_t *p)
{
    --p->used;
    memmove(p->bytes, p->bytes + 1, p->used);
}

bool bike_parse_command_byte(bike_command_parser_t *p, uint8_t byte,
                             uint32_t *sequence, bike_wire_command_t *c)
{
    if (p->used == sizeof(p->bytes)) discard_byte(p);
    p->bytes[p->used++] = byte;
    while (p->used >= 4) {
        if (memcmp(p->bytes, magic, 4) != 0) { discard_byte(p); continue; }
        if (p->used < 12) return false;
        if (p->bytes[4] != 1 || p->bytes[5] != BIKE_PACKET_COMMAND ||
            p->bytes[6] != BIKE_COMMAND_PAYLOAD_SIZE || p->bytes[7] != 0) {
            discard_byte(p);
            continue;
        }
        if (p->used < 36) return false;
        if (get_u32(p->bytes + 32) != bike_crc32(p->bytes, 32)) {
            discard_byte(p);
            continue;
        }
        *sequence = get_u32(p->bytes + 8);
        c->boot_id = get_u32(p->bytes + 12);
        c->state_timestamp_us = get_u64(p->bytes + 16);
        c->torque_nm = get_float(p->bytes + 24);
        c->flags = get_u32(p->bytes + 28);
        p->used -= 36;
        memmove(p->bytes, p->bytes + 36, p->used);
        return true;
    }
    return false;
}
