"""
Integrated Physical Plant: CentrifugalHydraulicMillingPlant implementing IPlant with RK4 ODE integration.
"""

from typing import Any, Dict, Optional, Tuple
import math
import random

from sim.common.contracts import ControlCommands, ProcessVariables, TruthDiagnostics
from sim.common.interfaces import IPlant
from sim.plant.cutting import CuttingParameters, MechanisticCuttingSubsystem
from sim.plant.hydraulics import HydraulicParameters, HydraulicSubsystem
from sim.plant.mechanics import MechanicsParameters, MechanicsSubsystem
from sim.plant.motors import PMSMPumpDrive, PMSMSpindleDrive


class CentrifugalHydraulicMillingPlant(IPlant):
    """
    High-fidelity physical truth plant combining:
    - Variable-speed centrifugal pump H-Q curves
    - Hydraulic chamber fluid continuity with entrained air bulk modulus
    - Stiff calibrated bypass orifice and seal leakage
    - Pressure-dependent Stribeck cylinder seal friction
    - Inconel 718 mechanistic milling against spherical workpiece
    - PMSM drives with FOC Iq current loops and resolver tracking
    """

    def __init__(
        self,
        hyd_params: Optional[HydraulicParameters] = None,
        mech_params: Optional[MechanicsParameters] = None,
        cut_params: Optional[CuttingParameters] = None,
        fidelity_level: str = "averaged",        # "averaged" (Level A) or "tooth_resolved" (Level B)
        seed: int = 42,
    ) -> None:
        self.hyd_params = hyd_params or HydraulicParameters()
        self.mech_params = mech_params or MechanicsParameters()
        self.cut_params = cut_params or CuttingParameters()
        self.fidelity_level = fidelity_level
        self.seed = seed
        self.rng = random.Random(seed)

        # Subsystems
        self.hydraulics = HydraulicSubsystem(self.hyd_params)
        self.mechanics = MechanicsSubsystem(self.mech_params)
        self.cutting = MechanisticCuttingSubsystem(self.cut_params)
        self.spindle_drive = PMSMSpindleDrive(seed=seed)
        self.pump_drive = PMSMPumpDrive()

        # Cached state
        self.sim_time = 0.0
        self.f_axial_true = 0.0
        self.t_cutting_true = 0.0
        self.mrr_true = 0.0
        self.chip_h_true = 0.0
        self.f_friction_true = 0.0
        self.q_pump_true = 0.0
        self.q_bypass_true = 0.0
        self.q_leak_true = 0.0
        self.rod_accel_true = 0.0
        self.pressure_sensor_val = self.hyd_params.atmospheric_pressure
        self.displacement_sensor_val = 0.0
        self.spindle_speed_res = 0.0
        self.pump_speed_res = 0.0
        self.spindle_torque_est = 0.0

    def reset(self, initial_state: Optional[Dict[str, Any]] = None) -> None:
        """Reset all plant states to initial conditions."""
        init_p = initial_state.get("pressure", self.hyd_params.atmospheric_pressure) if initial_state else self.hyd_params.atmospheric_pressure
        init_x = initial_state.get("position", 0.0) if initial_state else 0.0

        self.rng = random.Random(self.seed)
        self.hydraulics.reset(initial_pressure=init_p)
        self.mechanics.reset(initial_position=init_x, initial_velocity=0.0)
        self.cutting.reset()
        self.spindle_drive.reset()
        self.pump_drive.reset()

        self.sim_time = 0.0
        self.f_axial_true = 0.0
        self.t_cutting_true = 0.0
        self.mrr_true = 0.0
        self.chip_h_true = 0.0
        self.f_friction_true = 0.0
        self.q_pump_true = 0.0
        self.q_bypass_true = 0.0
        self.q_leak_true = 0.0
        self.rod_accel_true = 0.0
        self.pressure_sensor_val = init_p
        self.displacement_sensor_val = init_x
        self.spindle_speed_res = 0.0
        self.pump_speed_res = 0.0
        self.spindle_torque_est = 0.0

    def _state_derivatives(
        self,
        p_curr: float,
        x_curr: float,
        v_curr: float,
        omega_p: float,
        f_axial: float,
    ) -> Tuple[float, float, float, float, float, float, float]:
        """
        Compute state derivatives: [dp_dt, dx_dt, dv_dt] and instantaneous forces/flows.
        """
        dp_dt, q_pump, q_bypass, q_leak = self.hydraulics.compute_pressure_derivative(
            pressure=p_curr,
            rod_position=x_curr,
            rod_velocity=v_curr,
            omega_pump=omega_p,
        )

        gauge_p = p_curr - self.hyd_params.atmospheric_pressure
        accel, f_fric = self.mechanics.compute_acceleration(
            gauge_pressure=gauge_p,
            piston_area=self.hyd_params.piston_area,
            axial_cutting_force=f_axial,
            position=x_curr,
            velocity=v_curr,
        )

        return dp_dt, v_curr, accel, f_fric, q_pump, q_bypass, q_leak

    def step(self, dt: float, commands: ControlCommands) -> None:
        """
        Step physical states by dt using 4th-Order Runge-Kutta (RK4) integration.
        """
        # 1. Step PMSM motor drives
        pump_w_cmd = commands.pump_speed_cmd_rads if commands.enable_pump else 0.0
        spindle_w_cmd = commands.spindle_speed_cmd_rads if commands.enable_spindle else 0.0

        self.pump_speed_res = self.pump_drive.step(dt, pump_w_cmd)
        spindle_w_true, self.spindle_speed_res, self.spindle_torque_est = self.spindle_drive.step(
            dt, spindle_w_cmd, self.t_cutting_true
        )

        omega_pump_mech = self.pump_drive.mechanical_speed

        # 2. Compute current cutting reaction forces
        if self.fidelity_level == "tooth_resolved":
            f_ax, t_cut, mrr, h_chip = self.cutting.step_level_b_tooth_resolved(
                dt, self.mechanics.position, self.mechanics.velocity, spindle_w_true
            )
        else:
            f_ax, t_cut, mrr, h_chip = self.cutting.step_level_a_averaged(
                dt, self.mechanics.position, self.mechanics.velocity, spindle_w_true
            )

        self.f_axial_true = f_ax
        self.t_cutting_true = t_cut
        self.mrr_true = mrr
        self.chip_h_true = h_chip

        # 3. Continuous RK4 ODE integration for [Pressure, Position, Velocity]
        p0 = self.hydraulics.pressure
        x0 = self.mechanics.position
        v0 = self.mechanics.velocity

        # k1
        dp1, dx1, dv1, f_fric1, q_p1, q_b1, q_l1 = self._state_derivatives(
            p0, x0, v0, omega_pump_mech, self.f_axial_true
        )

        # k2
        p_k2 = p0 + 0.5 * dt * dp1
        x_k2 = x0 + 0.5 * dt * dx1
        v_k2 = v0 + 0.5 * dt * dv1
        dp2, dx2, dv2, _, _, _, _ = self._state_derivatives(
            p_k2, x_k2, v_k2, omega_pump_mech, self.f_axial_true
        )

        # k3
        p_k3 = p0 + 0.5 * dt * dp2
        x_k3 = x0 + 0.5 * dt * dx2
        v_k3 = v0 + 0.5 * dt * dv2
        dp3, dx3, dv3, _, _, _, _ = self._state_derivatives(
            p_k3, x_k3, v_k3, omega_pump_mech, self.f_axial_true
        )

        # k4
        p_k4 = p0 + dt * dp3
        x_k4 = x0 + dt * dx3
        v_k4 = v0 + dt * dv3
        dp4, dx4, dv4, _, _, _, _ = self._state_derivatives(
            p_k4, x_k4, v_k4, omega_pump_mech, self.f_axial_true
        )

        # Weighted RK4 update
        p_new = p0 + (dt / 6.0) * (dp1 + 2.0 * dp2 + 2.0 * dp3 + dp4)
        x_new = x0 + (dt / 6.0) * (dx1 + 2.0 * dx2 + 2.0 * dx3 + dx4)
        v_new = v0 + (dt / 6.0) * (dv1 + 2.0 * dv2 + 2.0 * dv3 + dv4)

        # Apply state updates
        self.hydraulics.pressure = max(self.hyd_params.atmospheric_pressure, p_new)
        self.mechanics.position = max(0.0, min(self.mech_params.stroke_max, x_new))
        self.mechanics.velocity = v_new
        self.rod_accel_true = dv1
        self.f_friction_true = f_fric1
        self.q_pump_true = q_p1
        self.q_bypass_true = q_b1
        self.q_leak_true = q_l1

        self.sim_time += dt

        # 4. Sensor emulation with small measurement noise
        # Hydraulic line pressure sensor (+- 0.15 bar noise)
        p_noise = self.rng.gauss(0.0, 1.5e4)
        self.pressure_sensor_val = max(self.hyd_params.atmospheric_pressure, self.hydraulics.pressure + p_noise)

        # Linear displacement transducer (+- 5 microns noise)
        x_noise = self.rng.gauss(0.0, 5.0e-6)
        self.displacement_sensor_val = self.mechanics.position + x_noise

    def get_sensor_readings(self) -> ProcessVariables:
        """Expose controller-visible telemetry."""
        return ProcessVariables(
            timestamp=self.sim_time,
            pressure_hyd=self.pressure_sensor_val,
            spindle_speed_res=self.spindle_speed_res,
            pump_speed_res=self.pump_speed_res,
            iq_current=self.spindle_drive.iq_current,
            spindle_torque_est=self.spindle_torque_est,
            rod_displacement=self.displacement_sensor_val,
            rod_velocity_est=self.mechanics.velocity,
            wob_soft_sensor=0.0,                # Controller/SoftSensor populates this
        )

    def get_truth_diagnostics(self) -> TruthDiagnostics:
        """Expose complete internal physical truth for diagnostics and UI."""
        _, a_contact = self.cutting.compute_engagement_geometry(self.mechanics.position)
        return TruthDiagnostics(
            timestamp=self.sim_time,
            rod_position_true=self.mechanics.position,
            rod_velocity_true=self.mechanics.velocity,
            rod_acceleration_true=self.rod_accel_true,
            chamber_pressure_true=self.hydraulics.pressure,
            pump_flow_rate=self.q_pump_true,
            bypass_flow_rate=self.q_bypass_true,
            leakage_flow_rate=self.q_leak_true,
            seal_friction_force=self.f_friction_true,
            axial_cutting_force_true=self.f_axial_true,
            spindle_cutting_torque_true=self.t_cutting_true,
            penetration_depth=self.cutting.penetration_depth,
            contact_area=a_contact,
            material_removal_rate=self.mrr_true,
            cumulative_volume_removed=self.cutting.cumulative_volume_removed,
            spindle_speed_true=self.spindle_drive.mechanical_speed,
            pump_speed_true=self.pump_drive.mechanical_speed,
            chip_thickness=self.chip_h_true,
            total_disturbance_f=0.0,
            leso_disturbance_est=0.0,
            operating_mode="RUNNING",
        )
