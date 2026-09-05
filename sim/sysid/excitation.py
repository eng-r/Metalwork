"""
Excitation Signal Generators for Laboratory SysID and DoE Campaigns.
Generates multi-level PRBS, frequency chirps, and stepped cutting test sequences.
"""

from typing import List, Tuple
import math
import numpy as np


def generate_prbs(
    duration: float,
    dt: float,
    clock_period: float,
    val_low: float,
    val_high: float,
    seed: int = 42,
) -> np.ndarray:
    """
    Generate a Pseudo-Random Binary Sequence (PRBS) signal.
    """
    np.random.seed(seed)
    n_samples = int(math.ceil(duration / dt))
    steps_per_clock = max(1, int(round(clock_period / dt)))
    n_clocks = int(math.ceil(n_samples / steps_per_clock))

    random_bits = np.random.choice([val_low, val_high], size=n_clocks)
    signal = np.repeat(random_bits, steps_per_clock)[:n_samples]
    return signal


def generate_frequency_chirp(
    duration: float,
    dt: float,
    f_start: float = 0.1,
    f_end: float = 10.0,
    amplitude: float = 500.0,
    offset: float = 2500.0,
) -> np.ndarray:
    """
    Generate a logarithmic/linear frequency chirp for motor drive identification.
    """
    t = np.arange(0.0, duration, dt)
    # Instantaneous frequency: f(t) = f_start + (f_end - f_start) * (t / duration)
    phase = 2.0 * math.pi * (f_start * t + 0.5 * (f_end - f_start) * (t ** 2) / duration)
    signal = offset + amplitude * np.sin(phase)
    return signal


def generate_stepped_pump_profile(
    duration: float = 20.0,
    dt: float = 0.001,
) -> Tuple[np.ndarray, np.ndarray]:
    """
    Generate stepped pump RPM excitation to characterize deadhead pressure,
    H-Q curves, and bypass orifice relaxation.
    Returns: (time_array, rpm_array)
    """
    t = np.arange(0.0, duration, dt)
    rpm = np.zeros_like(t)

    for idx, t_val in enumerate(t):
        if t_val < 2.0:
            rpm[idx] = 1000.0
        elif t_val < 6.0:
            rpm[idx] = 2500.0
        elif t_val < 10.0:
            rpm[idx] = 4000.0
        elif t_val < 14.0:
            # Idle pump to measure pressure relaxation through bypass orifice
            rpm[idx] = 0.0
        elif t_val < 17.0:
            rpm[idx] = 3000.0
        else:
            rpm[idx] = 0.0

    return t, rpm
