#!/usr/bin/env python3
"""Run the existing RL policy on this laptop; ESP32 supplies 60 Hz state."""
import argparse
import os
from pathlib import Path
import sys
import time

# Tiny networks do not benefit from spinning up a large BLAS thread pool.
# Set before importing NumPy, only within this process.
for name in ("OPENBLAS_NUM_THREADS", "MKL_NUM_THREADS", "OMP_NUM_THREADS"):
    os.environ[name] = "1"

from controller import Controller, load_policy, DEFAULT_POLICY, DEFAULT_TRAINING_ROOT
from udp_link import UDPLink
from protocol import State, VALID, DRIVE_READY, MOCK, MOTOR_DISABLED, COMMAND_VALID
from protocol import encode_command


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--esp-ip", default="192.168.4.1", help="ESP32 hotspot IPv4 address")
    parser.add_argument("--udp-port", type=int, default=3333)
    parser.add_argument("--policy", type=Path, default=DEFAULT_POLICY)
    parser.add_argument("--training-root", type=Path, default=DEFAULT_TRAINING_ROOT)
    parser.add_argument("--self-test", action="store_true", help="Run laptop inference only; no network access")
    args = parser.parse_args()
    controller = Controller(load_policy(args.policy, args.training_root))
    if args.self_test:
        samples = []
        for i in range(121):
            state = State(i + 1, 1, 100000 + i * 16667, 1000, 1000, .0034906585,
                          0, 0, 0, VALID | DRIVE_READY | MOCK | MOTOR_DISABLED, 0, 0, 0)
            start = time.perf_counter_ns()
            action, torque = controller.step(state)
            elapsed = (time.perf_counter_ns() - start) / 1000
            if i:
                samples.append(elapsed)
        print(f"Laptop inference only: mean={sum(samples)/len(samples):.1f}us "
              f"max={max(samples):.1f}us action={action:.5f} torque={torque:.5f}Nm")
        print("No ESP32 connected/tested; this does not measure Wi-Fi round-trip latency.")
        return 0
    stream = UDPLink(args.esp_ip, args.udp_port)
    last_state = None
    last_receive = last_log = time.monotonic()
    count = 0
    policy_stops = 0
    max_infer_us = 0
    print(f"PC inference | Wi-Fi UDP {args.esp_ip}:{args.udp_port} | "
          "state-triggered 60Hz | MOTOR_OUTPUT=OFF", flush=True)
    try:
        while True:
            state = stream.read_state()
            now = time.monotonic()
            if state is None:
                if now - last_receive > .1:
                    controller.reset()
                if now - last_log > 1:
                    print("Waiting for ESP32 Wi-Fi state; connect to BalanceBike-ESP32 "
                          "and check ESP32 IP/firmware.", flush=True)
                    last_log = now
                continue
            last_state = state
            last_receive = now
            start = time.perf_counter_ns()
            reason = ""
            try:
                action, torque = controller.step(state)
            except ValueError as exc:
                action, torque, reason = 0.0, 0.0, str(exc)
            infer_us = (time.perf_counter_ns() - start) // 1000
            max_infer_us = max(max_infer_us, infer_us)
            if infer_us >= 16667:
                controller.reset()
                action, torque, reason = 0.0, 0.0, "Laptop computation exceeded one policy period"
            stream.write(encode_command(state, torque, stop=bool(reason)))
            policy_stops += bool(reason)
            count += 1
            if now - last_log >= .5:
                link = "OK" if state.flags & COMMAND_VALID and state.fault == 0 else "WAIT/FAULT"
                rtt = f"{state.round_trip_us/1000:.2f}ms" if state.acknowledged_sequence else "--"
                print(f"n={count} seq={state.sequence} link={link} "
                      f"command={'STOP' if reason else 'RUN'} mock={bool(state.flags & MOCK)} "
                      f"infer={infer_us}us max={max_infer_us}us rtt={rtt} "
                      f"ack={state.acknowledged_sequence} fault=0x{state.fault:02x} "
                      f"imu_age={state.imu_age_us}us enc_age={state.encoder_age_us}us "
                      f"cmd_age={state.command_age_us}us "
                      f"roll={state.roll_rad:.4f} rate={state.roll_rate_rad_s:.4f} "
                      f"speed={state.flywheel_rad_s:.2f} "
                      f"action={action:.4f} torque={torque:.4f}Nm "
                      f"crc_bad={stream.bad_packets} old={stream.old_packets} "
                      f"gaps={stream.gaps} coalesced={stream.coalesced} "
                      f"policy_stops={policy_stops} {reason}", flush=True)
                last_log = now
    except KeyboardInterrupt:
        print("Stopped laptop inference; ESP32 command deadline also stops the link.")
    finally:
        try:
            if last_state is not None:
                stream.write(encode_command(last_state, 0.0, stop=True))
        except OSError:
            pass
        stream.close()
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except (OSError, ValueError, KeyError) as exc:
        print(f"Cannot run policy: {exc}", file=sys.stderr)
        sys.exit(1)
