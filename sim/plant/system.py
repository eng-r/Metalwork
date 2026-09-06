"""
Integrated physical plant for the centrifugal-hydraulic milling simulator.

Numerical integration uses RK4 for pressure/rod states. Crucially, the cutting/contact
reaction is re-evaluated at every RK stage using the stage-specific rod state while slow
material/chip states are frozen across that 1 ms integration interval. That keeps the
stiff contact coupling consistent instead of holding one axial force across all RK stages.
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
    def __init__(
        self,
        hyd_params: Optional[HydraulicParameters] = None,
        mech_params: Optional[MechanicsParameters] = None,
        cut_params: Optional[CuttingParameters] = None,
        fidelity_level: str = "averaged",
        seed: int = 42,
    ) -> None:
        self.hyd_params = hyd_params or HydraulicParameters()
        self.mech_params = mech_params or MechanicsParameters()
        self.cut_params = cut_params or CuttingParameters()
        self.fidelity_level = fidelity_level
        self.seed = seed
        self.rng = random.Random(seed)

        self.hydraulics = HydraulicSubsystem(self.hyd_params)
        self.mechanics = MechanicsSubsystem(self.mech_params)
        self.cutting = MechanisticCuttingSubsystem(self.cut_params, seed=seed)
        self.spindle_drive = PMSMSpindleDrive(seed=seed)
        self.pump_drive = PMSMPumpDrive()

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
        init_p = (
            initial_state.get("pressure", self.hyd_params.atmospheric_pressure)
            if initial_state
            else self.hyd_params.atmospheric_pressure
        )
        init_x = initial_state.get("position", 0.0) if initial_state else 0.0
        init_v = initial_state.get("velocity", 0.0) if initial_state else 0.0

        self.rng = random.Random(self.seed)
        self.hydraulics.reset(initial_pressure=init_p)
        self.mechanics.reset(initial_position=init_x, initial_velocity=init_v)
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
        omega_pump: float,
        omega_spindle: float,
    ) -> Tuple[float, float, float, float, float, float, float]:
        """Evaluate the coupled hydraulic/contact mechanics at one RK stage."""
        f_axial = self.cutting.evaluate_axial_force(
            rod_position=x_curr,
            rod_velocity=v_curr,
            spindle_speed=omega_spindle,
        )

        dp_dt, q_pump, q_bypass, q_leak = (
            self.hydraulics.compute_pressure_derivative(
                pressure=p_curr,
                rod_position=x_curr,
                rod_velocity=v_curr,
                omega_pump=omega_pump,
            )
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
        # 1. Actuator dynamics. The spindle sees the previous 1 ms cutting torque; this is a
        # physically negligible transport delay and avoids algebraic motor/cutting coupling.
        pump_w_cmd = commands.pump_speed_cmd_rads if commands.enable_pump else 0.0
        spindle_w_cmd = (
            commands.spindle_speed_cmd_rads if commands.enable_spindle else 0.0
        )

        self.pump_speed_res = self.pump_drive.step(dt, pump_w_cmd)
        (
            spindle_w_true,
            self.spindle_speed_res,
            self.spindle_torque_est,
        ) = self.spindle_drive.step(
            dt,
            spindle_w_cmd,
            self.t_cutting_true,
        )
        omega_pump_mech = self.pump_drive.mechanical_speed

        # 2. RK4 for hydraulic pressure + pusher position/velocity. Contact force is evaluated
        # at each RK stage using the current stage geometry.
        p0 = self.hydraulics.pressure
        x0 = self.mechanics.position
        v0 = self.mechanics.velocity

        dp1, dx1, dv1, f_fric1, q_p1, q_b1, q_l1 = self._state_derivatives(
            p0, x0, v0, omega_pump_mech, spindle_w_true
        )

        p2 = p0 + 0.5 * dt * dp1
        x2 = x0 + 0.5 * dt * dx1
        v2 = v0 + 0.5 * dt * dv1
        dp2, dx2, dv2, _, _, _, _ = self._state_derivatives(
            p2, x2, v2, omega_pump_mech, spindle_w_true
        )

        p3 = p0 + 0.5 * dt * dp2
        x3 = x0 + 0.5 * dt * dx2
        v3 = v0 + 0.5 * dt * dv2
        dp3, dx3, dv3, _, _, _, _ = self._state_derivatives(
            p3, x3, v3, omega_pump_mech, spindle_w_true
        )

        p4 = p0 + dt * dp3
        x4 = x0 + dt * dx3
        v4 = v0 + dt * dv3
        dp4, dx4, dv4, _, _, _, _ = self._state_derivatives(
            p4, x4, v4, omega_pump_mech, spindle_w_true
        )

        p_new = p0 + dt / 6.0 * (dp1 + 2.0 * dp2 + 2.0 * dp3 + dp4)
        x_new = x0 + dt / 6.0 * (dx1 + 2.0 * dx2 + 2.0 * dx3 + dx4)
        v_new = v0 + dt / 6.0 * (dv1 + 2.0 * dv2 + 2.0 * dv3 + dv4)

        self.hydraulics.pressure = max(
            self.hyd_params.atmospheric_pressure,
            p_new,
        )

        if x_new <= 0.0 and v_new < 0.0:
            x_new = 0.0
            v_new = 0.0
        elif x_new >= self.mech_params.stroke_max and v_new > 0.0:
            x_new = self.mech_params.stroke_max
            v_new = 0.0

        self.mechanics.position = max(
            0.0,
            min(self.mech_params.stroke_max, x_new),
        )
        self.mechanics.velocity = v_new

        self.rod_accel_true = dv1
        self.f_friction_true = f_fric1
        self.q_pump_true = q_p1
        self.q_bypass_true = q_b1
        self.q_leak_true = q_l1

        # 3. Advance the slow cutting/material states once after the mechanical state update.
        if self.fidelity_level == "tooth_resolved":
            f_ax, t_cut, mrr, h_chip = self.cutting.step_level_b_tooth_resolved(
                dt,
                self.mechanics.position,
                self.mechanics.velocity,
                spindle_w_true,
            )
        else:
            f_ax, t_cut, mrr, h_chip = self.cutting.step_level_a_averaged(
                dt,
                self.mechanics.position,
                self.mechanics.velocity,
                spindle_w_true,
            )

        self.f_axial_true = f_ax
        self.t_cutting_true = t_cut
        self.mrr_true = mrr
        self.chip_h_true = h_chip
        self.sim_time += dt

        # 4. Sensor emulation. Noise is small compared with process excursions; high-frequency
        # truth is anti-aliased before the 30 Hz UI stream in sim/server.py.
        p_noise = self.rng.gauss(0.0, 5.0e3)  # 0.05 bar sigma
        self.pressure_sensor_val = max(
            self.hyd_params.atmospheric_pressure,
            self.hydraulics.pressure + p_noise,
        )

        x_noise = self.rng.gauss(0.0, 5.0e-6)
        self.displacement_sensor_val = self.mechanics.position + x_noise

    def get_sensor_readings(self) -> ProcessVariables:
        return ProcessVariables(
            timestamp=self.sim_time,
            pressure_hyd=self.pressure_sensor_val,
            spindle_speed_res=self.spindle_speed_res,
            pump_speed_res=self.pump_speed_res,
            iq_current=self.spindle_drive.iq_current,
            spindle_torque_est=self.spindle_torque_est,
            rod_displacement=self.displacement_sensor_val,
            rod_velocity_est=self.mechanics.velocity,
            wob_soft_sensor=0.0,
        )

    def get_truth_diagnostics(self) -> TruthDiagnostics:
        _, contact_area = self.cutting.compute_engagement_geometry(
            self.mechanics.position
        )
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
            surface_recession_depth=self.cutting.surface_recession_depth,
            engagement_depth=self.cutting.engagement_depth,
            contact_area=contact_area,
            material_removal_rate=self.mrr_true,
            cumulative_volume_removed=self.cutting.cumulative_volume_removed,
            physical_rop=self.cutting.physical_rop_m_s,
            demo_acceleration=self.cutting.params.demo_acceleration,
            equivalent_process_time=self.cutting.equivalent_process_time,
            disturbance_event=self.cutting.disturbance_event,
            spindle_speed_true=self.spindle_drive.mechanical_speed,
            pump_speed_true=self.pump_drive.mechanical_speed,
            chip_thickness=self.chip_h_true,
            total_disturbance_f=0.0,
            leso_disturbance_est=0.0,
            operating_mode="RUNNING",
        )
