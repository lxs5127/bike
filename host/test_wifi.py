#!/usr/bin/env python3
"""Check changing mock inputs and command timeout/recovery; motor output stays OFF."""
import argparse
import json
import math
import os
from pathlib import Path
import sys
import time

for name in ("OPENBLAS_NUM_THREADS", "MKL_NUM_THREADS", "OMP_NUM_THREADS"):
    os.environ[name] = "1"

from controller import Controller, load_policy
from protocol import MOCK, MOTOR_DISABLED, COMMAND_VALID, LINK_FAULT, encode_command
from udp_link import UDPLink


def run_test(stream, controller, clock=time.monotonic, emit=print,
             warmup_s=5.0, pause_s=.3, recovery_s=3.0, wait_timeout_s=10.0):
    """Pause COMMAND only; keep reading states/HELLO so timeout remains observable."""
    started = clock()
    first = None
    boot_id = None
    phase = None
    last_state = None
    last_log = float("-inf")
    rolls, actions, rtts = [], [], []
    healthy = {"RUN": 0, "RECOVER": 0}
    expired_ages = []
    max_infer_us = 0
    policy_stops = 0
    pause_started = None
    pause_sequence = pause_source_us = pause_baseline_age_us = None
    last_sent = None
    previous_state = None
    ready_streak = 0
    try:
        while True:
            state = stream.read_state()
            now = clock()
            if state is None:
                if now - (first if first is not None else started) > (
                        warmup_s + wait_timeout_s + pause_s + recovery_s
                        if first is not None else wait_timeout_s):
                    raise ValueError("State stream missing/incomplete; check hotspot and stop other runners")
                continue
            last_state = state
            if first is None:
                first, boot_id = now, state.boot_id
            if state.boot_id != boot_id:
                raise ValueError("ESP32 rebooted during test")
            if state.flags & (MOCK | MOTOR_DISABLED) != MOCK | MOTOR_DISABLED:
                raise ValueError("This test requires mock feedback and MOTOR_OUTPUT=OFF")
            if state.fault & ~LINK_FAULT:
                raise ValueError(f"Latched ESP32 fault 0x{state.fault:02x}; inspect serial fault log")
            elapsed = now - first
            if pause_started is not None and now >= pause_started + pause_s + recovery_s:
                break
            alive = bool(state.flags & COMMAND_VALID)
            fresh_ack = (
                alive and state.fault == 0 and last_sent is not None and not last_sent[2]
                and state.acknowledged_sequence == last_sent[0]
                and state.timestamp_us - state.command_age_us == last_sent[1]
                and state.command_age_us < 25000
                and previous_state is not None
                and (state.sequence - previous_state.sequence) & 0xffffffff == 1
                and 8333 <= state.timestamp_us - previous_state.timestamp_us <= 25000)
            previous_state = state
            if pause_started is None:
                ready_streak = ready_streak + 1 if fresh_ack else 0
                if elapsed >= warmup_s and ready_streak >= 3:
                    pause_started = now
                    pause_sequence, pause_source_us = last_sent[:2]
                    pause_baseline_age_us = state.command_age_us
                    emit(f"PAUSE_BASELINE valid=1 ack={pause_sequence} "
                         f"cmd_age={pause_baseline_age_us}us; last RUN command acknowledged")
                elif elapsed > warmup_s + wait_timeout_s:
                    raise ValueError("No fresh acknowledged RUN baseline; cannot verify command expiry")
            next_phase = ("RUN" if pause_started is None else
                          "PAUSE" if now < pause_started + pause_s else "RECOVER")
            if next_phase != phase:
                phase = next_phase
                controller.reset()
                emit(f"PHASE={phase} t={elapsed:.2f}s MOTOR_OUTPUT=OFF")
                if phase == "PAUSE":
                    emit(f"Withholding COMMAND for {pause_s*1000:.0f}ms; "
                         "continuing HELLO and state reception (no STOP sent)")
            if alive and state.command_age_us >= 50000:
                raise ValueError("Expired command still marked valid at >=50ms")
            if phase == "PAUSE":
                # Last sent RUN was acknowledged before the pause. No later
                # commands exist: require that exact command to age and expire.
                if (state.acknowledged_sequence != pause_sequence or
                        state.command_age_us != state.timestamp_us - pause_source_us):
                    raise ValueError("Acknowledged command changed while COMMAND was withheld")
                if state.command_age_us < 50000:
                    if not alive or state.fault != 0:
                        raise ValueError("Command stopped before its deadline; timeout test is inconclusive")
                else:
                    if alive or state.fault != LINK_FAULT:
                        raise ValueError("Withheld commands did not expire")
                    expired_ages.append(state.command_age_us)
                action = torque = 0.0
                command_kind = "HELD"
            else:
                begin = time.perf_counter_ns()
                reason = ""
                try:
                    action, torque = controller.step(state)
                except ValueError as exc:
                    action = torque = 0.0
                    reason = str(exc)
                infer_us = (time.perf_counter_ns() - begin) // 1000
                max_infer_us = max(max_infer_us, infer_us)
                if infer_us >= 16667:
                    controller.reset()
                    action = torque = 0.0
                    reason = "Inference exceeded one policy period"
                stream.write(encode_command(state, torque, stop=bool(reason)))
                last_sent = (state.sequence, state.timestamp_us, bool(reason))
                command_kind = "STOP" if reason else "RUN"
                policy_stops += bool(reason)
                if alive and state.fault == 0 and not reason:
                    healthy[phase] += 1
                    rtts.append(state.round_trip_us)
                    if phase == "RUN":
                        rolls.append(math.degrees(state.roll_rad))
                        actions.append(action)
            if now - last_log >= .5 or phase == "PAUSE":
                emit(f"phase={phase} link={'OK' if alive and state.fault == 0 else 'STOP'} "
                     f"command={command_kind} "
                     f"fault=0x{state.fault:02x} cmd_age={state.command_age_us}us "
                     f"roll={math.degrees(state.roll_rad):+.2f}deg "
                     f"rate={state.roll_rate_rad_s:+.4f}rad/s action={action:+.4f} "
                     f"torque={torque:+.4f}Nm rtt={state.round_trip_us/1000:.2f}ms "
                     f"coalesced={stream.coalesced} policy_stops={policy_stops}")
                last_log = now
        if healthy["RUN"] < 30 or healthy["RECOVER"] < 10:
            raise ValueError("Not enough healthy command acknowledgements before/after timeout")
        if not expired_ages:
            raise ValueError("No expired-command state observed during pause")
        if min(rolls) > -1.5 or max(rolls) < 1.5 or max(actions) - min(actions) < .01:
            raise ValueError("Dynamic roll/action not observed; flash wifi-udp-v4 with mock motion enabled")
        report = {
            "result": "PASS", "motor_output_enabled": False, "mock_input_only": True,
            "vehicle_closed_loop_tested": False, "boot_id": boot_id,
            "healthy_run_states": healthy["RUN"], "healthy_recovery_states": healthy["RECOVER"],
            "roll_degrees_min": min(rolls), "roll_degrees_max": max(rolls),
            "action_min": min(actions), "action_max": max(actions),
            "observed_rtt_min_ms": min(rtts)/1000, "observed_rtt_max_ms": max(rtts)/1000,
            "max_infer_us": max_infer_us, "withheld_command_seconds": pause_s,
            "expired_command_states": len(expired_ages),
            "first_observed_expired_command_age_us": expired_ages[0],
            "pause_baseline_acknowledged_run": True,
            "pause_baseline_sequence": pause_sequence,
            "pause_baseline_command_age_us": pause_baseline_age_us,
            "pause_command_identity_unchanged": True,
            "crc_bad": stream.bad_packets, "old": stream.old_packets, "gaps": stream.gaps,
            "coalesced": stream.coalesced, "policy_stops_outside_pause": policy_stops,
        }
        if policy_stops:
            emit(f"NOTE: {policy_stops} policy STOP cycles outside planned pause; "
                 "this PASS does not establish a continuous 60Hz control stream")
        emit("PASS: changing input/action, command expiry while HELLO continues, and link recovery")
        return report
    finally:
        # Applies to success, failure and Ctrl+C. If this datagram is lost,
        # ESP32 still expires the last accepted command by its own clock.
        try:
            if last_state is not None:
                stream.write(encode_command(last_state, 0, stop=True))
        except OSError:
            pass
        stream.close()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--esp-ip", default="192.168.4.1")
    parser.add_argument("--udp-port", type=int, default=3333)
    parser.add_argument("--report", type=Path,
                        default=Path(__file__).resolve().parents[1] / "docs/wifi_test_latest.json")
    args = parser.parse_args()
    controller = Controller(load_policy())
    stream = UDPLink(args.esp_ip, args.udp_port)
    print("Wi-Fi dry-run test v2: >=5s RUN (wait for fresh RUN ACK) -> "
          "0.3s PAUSE COMMAND -> 3s RECOVER; MOTOR_OUTPUT=OFF", flush=True)
    try:
        report = run_test(stream, controller, emit=lambda s: print(s, flush=True))
    except (OSError, ValueError, KeyboardInterrupt) as exc:
        report = {"result": "FAIL", "reason": str(exc) or "Interrupted", "motor_output_enabled": False}
        print("FAIL: " + report["reason"], file=sys.stderr)
    args.report.write_text(json.dumps(report, indent=2) + "\n")
    print(f"Report: {args.report}", flush=True)
    return 0 if report["result"] == "PASS" else 1


if __name__ == "__main__":
    sys.exit(main())
