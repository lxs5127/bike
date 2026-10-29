#ifndef BIKE_PROTOCOL_H
#define BIKE_PROTOCOL_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/* Wire format: BKR1, version u8, type u8, payload length u16, sequence u32,
 * payload, CRC32(header+payload). Little endian, IEEE754 float32, no C padding. */
#define BIKE_PACKET_STATE 1U
#define BIKE_PACKET_COMMAND 2U
#define BIKE_PACKET_HELLO 3U
#define BIKE_STATE_VALID 1U
#define BIKE_STATE_DRIVE_READY 2U
#define BIKE_STATE_MOCK 4U
#define BIKE_STATE_MOTOR_DISABLED 8U
#define BIKE_STATE_COMMAND_VALID 16U
#define BIKE_COMMAND_STOP 1U
#define BIKE_STATE_PAYLOAD_SIZE 52U
#define BIKE_COMMAND_PAYLOAD_SIZE 20U
#define BIKE_PACKET_MAX_SIZE 68U

typedef struct {
    uint32_t boot_id;
    uint64_t timestamp_us;
    uint32_t imu_age_us, encoder_age_us;
    float roll_rad, roll_rate_rad_s, flywheel_rad_s;
    uint32_t fault, flags, acknowledged_sequence, command_age_us, round_trip_us;
} bike_wire_state_t;

typedef struct {
    uint32_t boot_id;
    uint64_t state_timestamp_us;
    float torque_nm;
    uint32_t flags;
} bike_wire_command_t;

typedef struct {
    uint8_t bytes[BIKE_PACKET_MAX_SIZE];
    size_t used;
} bike_command_parser_t;

uint32_t bike_crc32(const uint8_t *bytes, size_t size);
/* UDP: exactly one full packet per datagram. HELLO has an empty payload. */
bool bike_validate_packet(const uint8_t *bytes, size_t size, uint8_t type);
size_t bike_encode_state(uint8_t *bytes, uint32_t sequence, const bike_wire_state_t *state);
size_t bike_encode_command(uint8_t *bytes, uint32_t sequence, const bike_wire_command_t *command);
/* Feed one byte. Bad CRC/header/log noise gets discarded and resynchronized. */
bool bike_parse_command_byte(bike_command_parser_t *parser, uint8_t byte,
                             uint32_t *sequence, bike_wire_command_t *command);

#endif
