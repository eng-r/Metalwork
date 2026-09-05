"""
Staged Grey-Box Parameter Identification Engine using Nonlinear Least-Squares.
Calibrates pump H-Q curves, pressure-dependent Stribeck friction, and Inconel 718 cutting forces.
"""

from typing import Dict, List, Optional, Tuple
import math
import numpy as np
from scipy.optimize import least_squares

from sim.plant.cutting import CuttingParameters, MechanisticCuttingSubsystem
from sim.plant.hydraulics import HydraulicParameters, HydraulicSubsystem
from sim.plant.mechanics import MechanicsParameters, MechanicsSubsystem


def identify_pump_and_orifice_parameters(
    time_series: np.ndarray,
    pump_rpm_series: np.ndarray,
    measured_pressure_bar: np.ndarray,
    piston_area: float = 2.0e-3,
    dead_volume: float = 1.5e-4,
) -> Dict[str, float]:
    """
    Stage 2 Identification: Fit pump head coefficient a0 and bypass orifice area C_bypass
    from no-contact deadhead pressure step tests.
    """
    p_meas_pa = measured_pressure_bar * 1.0e5
    dt = float(np.mean(np.diff(time_series)))

    # Initial parameter guess: theta = [a0, C_bypass]
    theta_0 = [0.40, 1.0e-8]
    bounds = ([0.1, 1.0e-10], [1.5, 1.0e-6])

    def residuals(theta: List[float]) -> np.ndarray:
        a0_cand, c_bypass_cand = theta
        params = HydraulicParameters(
            pump_a0=a0_cand,
            bypass_orifice_cd_a=c_bypass_cand,
            piston_area=piston_area,
            dead_volume=dead_volume,
        )
        hyd = HydraulicSubsystem(params)
        p_sim = np.zeros_like(p_meas_pa)
        p_sim[0] = p_meas_pa[0]
        curr_p = p_meas_pa[0]

        for i in range(1, len(time_series)):
            omega_p = pump_rpm_series[i-1] * math.pi / 30.0
            # Rod velocity is 0 during deadhead test
            dp_dt, _, _, _ = hyd.compute_pressure_derivative(curr_p, 0.0, 0.0, omega_p)
            curr_p = max(params.atmospheric_pressure, curr_p + dp_dt * dt)
            p_sim[i] = curr_p

        # Scaled residual (in bar)
        return (p_sim - p_meas_pa) / 1.0e5

    result = least_squares(residuals, theta_0, bounds=bounds, method="trf")
    a0_opt, c_bypass_opt = result.x

    rmse_bar = float(np.sqrt(np.mean((result.fun) ** 2)))
    return {
        "pump_a0": float(a0_opt),
        "bypass_orifice_cd_a": float(c_bypass_opt),
        "rmse_bar": rmse_bar,
        "success": bool(result.success),
    }


def identify_cutting_coefficients(
    measured_penetration_mm: np.ndarray,
    measured_feed_velocity_mm_s: np.ndarray,
    measured_spindle_rpm: np.ndarray,
    measured_spindle_torque_nm: np.ndarray,
    measured_axial_force_n: np.ndarray,
    cutter_radius_mm: float = 10.0,
    target_sphere_radius_mm: float = 50.0,
) -> Dict[str, float]:
    """
    Stage 4 Identification: Fit mechanistic specific cutting energy and thrust coefficients
    from steady cutting test data on Inconel 718.
    """
    cutter_r_m = cutter_radius_mm / 1000.0
    sphere_r_m = target_sphere_radius_mm / 1000.0

    # theta = [averaged_specific_energy, axial_thrust_coeff, rubbing_torque_coeff]
    theta_0 = [3.0e9, 1.5e7, 1000.0]
    bounds = ([1.0e9, 1.0e6, 100.0], [8.0e9, 5.0e7, 10000.0])

    d_m = measured_penetration_mm / 1000.0
    v_m = measured_feed_velocity_mm_s / 1000.0
    omega_rads = measured_spindle_rpm * (2.0 * math.pi / 60.0)

    # Compute geometric contact areas
    a_contact = np.zeros_like(d_m)
    for i, d in enumerate(d_m):
        if d > 0:
            a_geom = math.pi * (2.0 * sphere_r_m * d - d * d)
            a_contact[i] = min(a_geom, math.pi * (cutter_r_m ** 2))

    mrr = a_contact * np.maximum(0.0, v_m)

    def residuals(theta: List[float]) -> np.ndarray:
        k_cut, k_ax, t_rub = theta
        # Simulated torque: T = k_cut * MRR / omega + t_rub * A
        t_sim = (k_cut * mrr) / np.maximum(1.0, omega_rads) + t_rub * a_contact
        # Simulated axial force: F_ax = k_ax * A
        f_ax_sim = k_ax * a_contact

        err_t = (t_sim - measured_spindle_torque_nm) / 2.0  # normalize
        err_f = (f_ax_sim - measured_axial_force_n) / 200.0

        return np.concatenate([err_t, err_f])

    result = least_squares(residuals, theta_0, bounds=bounds, method="trf")
    k_cut_opt, k_ax_opt, t_rub_opt = result.x

    return {
        "averaged_specific_energy": float(k_cut_opt),
        "axial_thrust_coeff": float(k_ax_opt),
        "rubbing_torque_coeff": float(t_rub_opt),
        "success": bool(result.success),
    }


def main() -> None:
    print("[SysID] Generating synthetic calibration dataset...")
    t = np.linspace(0.0, 10.0, 1000)
    # Synthetic stepped cut on Inconel 718
    d_mm = np.clip((t - 2.0) * 1.5, 0.0, 8.0)
    v_mm_s = np.where(t > 2.0, 1.5, 0.0)
    rpm = np.full_like(t, 3500.0)

    # True parameters: 3.2e9, 1.8e7, 1200.0
    r_sphere_m = 0.050
    r_cut_m = 0.010
    a_cont = np.array([min(math.pi * (r_cut_m**2), math.pi * (2*r_sphere_m*(d/1000) - (d/1000)**2)) if d > 0 else 0 for d in d_mm])
    mrr = a_cont * (v_mm_s / 1000.0)
    w_rads = rpm * (math.pi / 30.0)

    torque_true = (3.2e9 * mrr) / w_rads + 1200.0 * a_cont + np.random.normal(0, 0.05, len(t))
    force_true = 1.8e7 * a_cont + np.random.normal(0, 10.0, len(t))

    print("[SysID] Fitting Inconel 718 mechanistic cutting parameters via Levenberg-Marquardt...")
    fit = identify_cutting_coefficients(
        measured_penetration_mm=d_mm,
        measured_feed_velocity_mm_s=v_mm_s,
        measured_spindle_rpm=rpm,
        measured_spindle_torque_nm=torque_true,
        measured_axial_force_n=force_true,
    )

    print("[SysID] Parameter Identification Results:")
    print(f"  Specific Cutting Energy: {fit['averaged_specific_energy'] / 1e6:.1f} MPa (True: 3200.0 MPa)")
    print(f"  Axial Thrust Coeff:      {fit['axial_thrust_coeff'] / 1e6:.2f} MPa (True: 18.00 MPa)")
    print(f"  Rubbing Torque Coeff:    {fit['rubbing_torque_coeff']:.1f} N*m/m^2 (True: 1200.0 N*m/m^2)")
    print(f"  Optimization Converged:  {fit['success']}")


if __name__ == "__main__":
    main()

