"""
Dynamic soft Weight-On-Bit (WOB) observer.

The observer explicitly acknowledges that static seal friction is set-valued and therefore not
uniquely observable from pressure + displacement alone. A calibrated forward-friction prior is
used during stick; moving-rod friction uses the dynamic seal model.
"""

from sim.common.contracts import ProcessVariables
from sim.common.interfaces import ISoftSensor
from sim.plant.mechanics import MechanicsParameters, MechanicsSubsystem


class SoftWOBObserver(ISoftSensor):
    def __init__(
        self,
        piston_area: float = 2.0e-3,
        effective_mass: float = 45.0,
        atmospheric_pressure: float = 1.01325e5,
        mech_params: MechanicsParameters = MechanicsParameters(),
    ) -> None:
        self.piston_area = piston_area
        self.effective_mass = effective_mass
        self.atmospheric_pressure = atmospheric_pressure
        self.friction_model = MechanicsSubsystem(mech_params)

        self.pos_est = 0.0
        self.vel_est = 0.0
        self.accel_est = 0.0
        self.wob_filtered = 0.0

        # Conservative tracking gains for a noisy displacement transducer at 100 Hz.
        self.alpha = 0.32
        self.beta = 0.045
        self.gamma = 0.0012

    def reset(self) -> None:
        self.pos_est = 0.0
        self.vel_est = 0.0
        self.accel_est = 0.0
        self.wob_filtered = 0.0

    def update(self, dt: float, sensors: ProcessVariables) -> float:
        # Alpha-beta-gamma position tracker.
        pos_pred = (
            self.pos_est
            + self.vel_est * dt
            + 0.5 * self.accel_est * dt * dt
        )
        vel_pred = self.vel_est + self.accel_est * dt
        residual = sensors.rod_displacement - pos_pred

        self.pos_est = pos_pred + self.alpha * residual
        self.vel_est = vel_pred + (self.beta / max(dt, 1.0e-9)) * residual
        self.accel_est += (
            self.gamma / max(0.5 * dt * dt, 1.0e-12)
        ) * residual

        # Blend with the plant's sensor-equivalent velocity channel if present. In hardware this
        # can be the same displacement signal differentiated/filtered by firmware.
        self.vel_est = 0.75 * self.vel_est + 0.25 * sensors.rod_velocity_est

        gauge_p = max(0.0, sensors.pressure_hyd - self.atmospheric_pressure)
        f_hyd = gauge_p * self.piston_area
        f_fric_est = self.friction_model.estimate_forward_seal_friction(
            gauge_p,
            self.vel_est,
        )
        f_inertia = self.effective_mass * self.accel_est

        wob_raw = max(0.0, f_hyd - f_fric_est - f_inertia)
        alpha_lpf = dt / (0.060 + dt)
        self.wob_filtered += alpha_lpf * (wob_raw - self.wob_filtered)
        return self.wob_filtered
