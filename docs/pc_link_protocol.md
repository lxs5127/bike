# Wi-Fi / UDP protocol v1

This format is for the legacy PC link test. Enable `Run legacy PC link test instead of ODrive control` in the root firmware's `Balance bike framework` menu before running a UDP 3333 client.

Little endian. Floats are IEEE754 float32. No struct padding on the wire.
ESP32 SoftAP `BalanceBike-ESP32`, IPv4 192.168.4.1, UDP port 3333.
Each UDP datagram contains exactly one full packet; fragments/concatenation are
rejected. The PC uses a connected IPv4 UDP socket to filter other source endpoints.

| Header field | Size |
|---|---:|
| magic `BKR1` | 4 |
| version `1` | 1 |
| type: state=1, command=2, hello=3 | 1 |
| payload length | 2 |
| sequence | 4 |
| payload | 52 / 20 / 0 |
| CRC32 of header + payload, same as Python zlib.crc32 | 4 |

## HELLO, type 3, 16 bytes total

Empty payload, header sequence 0. PC sends every 250 ms to register/renew its
return IP/port. This never refreshes actuator command validity. The ESP32 accepts
one active endpoint; another endpoint must wait for the 1 second idle lease to
expire. Valid same-endpoint command packets also renew the endpoint lease.
Wi-Fi client disconnect clears registration. A missing endpoint is normal startup
waiting, not a permanently latched failure.

## State, type 1, 68 bytes total

| Payload order | Type | Meaning |
|---|---|---|
| boot_id | u32 | ESP32 boot nonce; resets policy memory on PC |
| timestamp_us | u64 | ESP32 state snapshot time, NOT laptop clock |
| imu_age_us | u32 | actual acquisition age; UINT32_MAX if invalid |
| encoder_age_us | u32 | actual acquisition age |
| roll_rad | f32 | body-X roll |
| roll_rate_rad_s | f32 | body-X gyro |
| flywheel_rad_s | f32 | signed flywheel speed relative to frame |
| fault | u32 | SENSOR=1, STALE=2, TIMING=4, POLICY=8, FALL=16, DRIVE=32, LINK=64 |
| flags | u32 | VALID=1, DRIVE_READY=2, MOCK=4, MOTOR_DISABLED=8, COMMAND_VALID=16 |
| acknowledged_sequence | u32 | last accepted reply, not measured/applied motor torque |
| command_age_us | u32 | now - source snapshot of last accepted command; UINT32_MAX if none |
| round_trip_us | u32 | last accepted reply time - its source snapshot time |

Sequence increments at each 60 Hz state publication. It wraps at u32. Boot ID is
not an authentication token. Same-axis signs/units must match training conventions.

## Command, type 2, 36 bytes total

Header sequence echoes the source STATE sequence (not an independent PC counter).

| Payload order | Type | Meaning |
|---|---|---|
| boot_id | u32 | echo source state |
| state_timestamp_us | u64 | echo source state exactly |
| torque_nm | f32 | flywheel joint torque in N·m |
| flags | u32 | 0=normal candidate, 1=STOP; other bits rejected |

C guard keeps the last four published sequence/timestamp pairs. Accept requires:
valid frame CRC/header, matching boot and an actual published pair, newer sequence
than last accepted command, finite bounded torque, known flags, age in [0,25 ms].
Accepted STOP immediately clears command validity. Normal command expires 50 ms
after its source snapshot. There is no command to enable the motor.

The control task handles RX notifications immediately; periodic 60 Hz events also
check sensor health/age, tilt, link expiry and actuator envelope. Scheduling can
delay the check; this is not an independent hardware emergency-stop guarantee.
Non-link local faults latch until MCU reset. Link waiting/expiry can recover only
as a disabled dry-run communication session; there is no automatic motor arming.

Normal diagnostic text remains on USB UART; binary state/commands use Wi-Fi only.
The PC verifies each datagram, drains a bounded burst and selects the newest
ESP32 timestamp. Duplicate/older timestamps and recently retired boot IDs are
ignored. Sequence gaps reset integration and send STOP for that cycle instead of
applying a backlog of old actions. Local send errors invalidate the command;
Wi-Fi absence/expiry is transient in the disabled prototype.

USB remains for flashing and power. IDF Monitor and the Wi-Fi runner can operate
at the same time. Wi-Fi latency must be measured separately from prior USB timing.
There is no guarantee that Wi-Fi is faster, nor an application-level UDP retry of
old commands. CRC is integrity checking, not controller authentication.
