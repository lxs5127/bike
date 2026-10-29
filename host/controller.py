"""Laptop inference; imports the existing simulation's actor without changing it."""
import importlib.util
import math
from pathlib import Path

import numpy as np
from protocol import VALID, DRIVE_READY, MOTOR_DISABLED, LINK_FAULT

ROOT = Path(__file__).resolve().parents[1]
DEFAULT_TRAINING_ROOT = Path("/home/lxs/work_place/balance_bike")
DEFAULT_POLICY = DEFAULT_TRAINING_ROOT / "deploy/policies/best_turn.npz"
DT = 1 / 60
TORQUE_LIMIT = 4.9
INERTIA = .00739


def load_policy(policy_path=DEFAULT_POLICY, training_root=DEFAULT_TRAINING_ROOT):
    source = Path(training_root) / "sim2sim/policy.py"
    spec = importlib.util.spec_from_file_location("bike_numpy_actor", source)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    actor = module.NumpyPolicy(policy_path)
    metadata = actor.metadata
    if metadata["observation_size"] != 4 or not metadata["policy_memory"]:
        raise ValueError("This runner requires four observations plus integral memory")
    config = metadata["config"]
    if config.get("flywheel_unload_kw", 0) or config.get("flywheel_momentum_ki", 0):
        raise ValueError("External momentum/unload controller is not implemented")
    for name, shape in [("feature.0.weight", (128, 5)), ("feature.0.bias", (128,)),
                        ("feature.2.weight", (128, 128)), ("feature.2.bias", (128,)),
                        ("actor_mean.weight", (1, 128)), ("actor_mean.bias", (1,))]:
        value = actor.weights[name]
        if value.shape != shape or not np.isfinite(value).all():
            raise ValueError("Invalid actor weight: " + name)
    if (not math.isclose(config["max_flywheel_speed"], 250) or
            not math.isclose(config["fall_angle_degrees"], 10) or
            not 0 < config["action_smoothing_alpha"] <= 1):
        raise ValueError("Policy limits differ from the ESP32's simulation envelope")
    return actor


class Controller:
    def __init__(self, policy):
        self.policy = policy
        self.reset()

    def reset(self):
        self.boot_id = None
        self.previous_timestamp = None
        self.previous_sequence = None
        self.previous_speed = 0.0
        self.rotation = 0.0
        self.previous_action = 0.0

    def step(self, state):
        """Return (action, torque_nm); use ESP32 time, never the laptop wall clock."""
        required = VALID | DRIVE_READY | MOTOR_DISABLED
        if (state.flags & required != required or state.fault & ~LINK_FAULT or
                max(state.imu_age_us, state.encoder_age_us) > 20000 or
                not all(math.isfinite(v) for v in
                        (state.roll_rad, state.roll_rate_rad_s, state.flywheel_rad_s)) or
                abs(state.roll_rad) > math.radians(10)):
            self.reset()
            raise ValueError("Invalid/stale sensor state or latched ESP32 fault")
        if self.boot_id != state.boot_id:
            self.reset()
            self.boot_id = state.boot_id
        if self.previous_timestamp is not None:
            dt = (state.timestamp_us - self.previous_timestamp) * 1e-6
            delta = (state.sequence - self.previous_sequence) & 0xffffffff
            if not DT / 2 <= dt <= DT * 1.5 or delta != 1:
                self.reset()
                raise ValueError(f"State gap/duplicate: seq_step={delta} dt={dt*1e6:.0f}us; "
                                 "reset policy memory, stop this cycle")
            self.rotation += .5 * (self.previous_speed + state.flywheel_rad_s) * dt
        self.previous_timestamp = state.timestamp_us
        self.previous_sequence = state.sequence
        self.previous_speed = state.flywheel_rad_s
        observation = [math.sin(state.roll_rad), math.cos(state.roll_rad),
                       np.clip(state.roll_rate_rad_s, -5, 5),
                       np.clip(state.flywheel_rad_s / 420, -1.5, 1.5)]
        action = self.policy.action(observation, self.rotation)
        alpha = self.policy.metadata["config"]["action_smoothing_alpha"]
        action = alpha * action + (1 - alpha) * self.previous_action
        span = max(.2 * 250, 2 * TORQUE_LIMIT * DT / INERTIA)
        speed = state.flywheel_rad_s
        action *= np.clip((250 - speed * np.sign(action)) / span, 0, 1)
        brake = np.clip((abs(speed) - 250) / span, 0, 1)
        if speed > 250:
            action = min(action, -brake)
        elif speed < -250:
            action = max(action, brake)
        if not math.isfinite(action) or abs(action) > 1:
            self.reset()
            raise ValueError("Invalid policy output")
        self.previous_action = float(action)
        return float(action), float(TORQUE_LIMIT * action)
