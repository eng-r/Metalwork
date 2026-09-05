"""
Headless Batch Simulation Runner and CLI Interface.
Executes deterministic virtual-time simulations, comparative benchmarks, and exports figures/parquet.
"""

import argparse
import os
from typing import Dict, List, Optional
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

from sim.common.contracts import ControlCommands, ProcessVariables, TruthDiagnostics
from sim.controller.adrc import CascadeLADRCController
from sim.controller.baselines import BaselinePIDController
from sim.kernel.backplane import ProcessBackplane, TelemetryFrame
from sim.kernel.scheduler import VirtualTimeScheduler
from sim.plant.cutting import CuttingParameters
from sim.plant.hydraulics import HydraulicParameters
from sim.plant.mechanics import MechanicsParameters
from sim.plant.system import CentrifugalHydraulicMillingPlant


def create_scenario_plant(scenario: str, fidelity: str = "averaged") -> CentrifugalHydraulicMillingPlant:
    """Factory creating plant configured for a specific scenario."""
    cut_params = CuttingParameters()
    mech_params = MechanicsParameters()
    hyd_params = HydraulicParameters()

    if scenario == "benchmark_overload":
        # Harder Inconel 718 with 40% higher specific cutting energy
        cut_params.averaged_specific_energy = 4.5e9
        cut_params.k_tc = 4200.0e6
        cut_params.axial_thrust_coeff = 2.5e7
    elif scenario == "relaxation_test":
        # Stiff orifice to highlight relaxation dependency on material removal feed
        hyd_params.bypass_orifice_cd_a = 0.8e-8

    return CentrifugalHydraulicMillingPlant(
        hyd_params=hyd_params,
        mech_params=mech_params,
        cut_params=cut_params,
        fidelity_level=fidelity,
    )


def run_simulation(
    controller_type: str = "adrc",
    scenario: str = "step_engagement",
    duration: float = 12.0,
    dt_plant: float = 0.001,
    dt_ctrl: float = 0.010,
    fidelity: str = "averaged",
) -> List[TelemetryFrame]:
    """Execute headless simulation run and return historical frames."""
    plant = create_scenario_plant(scenario, fidelity)

    if controller_type.lower() == "pid":
        controller = BaselinePIDController(target_pressure_bar=35.0, spindle_rpm_nominal=3500.0)
    else:
        controller = CascadeLADRCController(target_pressure_bar=35.0, spindle_rpm_nominal=3500.0)

    backplane = ProcessBackplane(history_len=int(duration / dt_plant) + 500)
    scheduler = VirtualTimeScheduler(
        plant=plant,
        controller=controller,
        backplane=backplane,
        dt_plant=dt_plant,
        dt_ctrl=dt_ctrl,
    )

    scheduler.reset()
    scheduler.run_until(duration)

    return backplane.get_recent_history()


def frames_to_dataframe(frames: List[TelemetryFrame]) -> pd.DataFrame:
    """Convert telemetry frames into a structured Pandas DataFrame."""
    data = []
    for f in frames:
        data.append({
            "timestamp": f.timestamp,
            "pressure_bar": f.sensors.pressure_bar,
            "pressure_true_bar": f.truth.chamber_pressure_true / 1.0e5,
            "spindle_rpm": f.sensors.spindle_rpm,
            "pump_rpm": f.sensors.pump_rpm,
            "pump_cmd_rpm": f.commands.pump_speed_cmd_rpm,
            "spindle_torque_est": f.sensors.spindle_torque_est,
            "spindle_torque_true": f.truth.spindle_cutting_torque_true,
            "wob_soft_sensor": f.sensors.wob_soft_sensor,
            "axial_cutting_force_true": f.truth.axial_cutting_force_true,
            "rod_position_mm": f.truth.rod_position_true * 1000.0,
            "rod_velocity_mm_s": f.truth.rod_velocity_true * 1000.0,
            "penetration_depth_mm": f.truth.penetration_depth * 1000.0,
            "seal_friction_n": f.truth.seal_friction_force,
            "material_removal_rate_mm3_s": f.truth.material_removal_rate * 1.0e9,
            "cumulative_volume_mm3": f.truth.cumulative_volume_removed * 1.0e9,
            "operating_mode": f.truth.operating_mode,
        })
    return pd.DataFrame(data)


def plot_single_run(df: pd.DataFrame, output_path: str, title_prefix: str = "Cascade LADRC") -> None:
    """Generate publication-ready 4-panel vector figure."""
    os.makedirs(os.path.dirname(os.path.abspath(output_path)), exist_ok=True)

    plt.style.use("seaborn-v0_8-whitegrid" if "seaborn-v0_8-whitegrid" in plt.style.available else "default")
    fig, axs = plt.subplots(4, 1, figsize=(10, 11), sharex=True)

    # Panel 1: Pressures & WOB
    axs[0].plot(df["timestamp"], df["pressure_true_bar"], label="Chamber Pressure P (bar)", color="#2563eb", lw=2)
    axs[0].plot(df["timestamp"], df["pressure_bar"], label="Sensor P_hyd (bar)", color="#93c5fd", lw=1, alpha=0.7)
    axs[0].set_ylabel("Pressure [bar]", fontweight="bold")
    axs[0].set_title(f"{title_prefix}: Asymmetric Hydraulic Milling of Inconel 718", fontsize=13, fontweight="bold", pad=12)
    axs[0].legend(loc="upper right", framealpha=0.9)
    axs[0].grid(True, linestyle="--", alpha=0.5)

    # Panel 2: Spindle Torque and Axial WOB
    ax2_twin = axs[1].twinx()
    l1 = axs[1].plot(df["timestamp"], df["spindle_torque_true"], label="Spindle Torque True (N*m)", color="#dc2626", lw=2)
    l2 = axs[1].plot(df["timestamp"], df["spindle_torque_est"], label="FOC Iq Estimated Torque (N*m)", color="#f87171", linestyle="--", lw=1.5)
    l3 = ax2_twin.plot(df["timestamp"], df["axial_cutting_force_true"], label="Axial Reaction / WOB True (N)", color="#16a34a", lw=1.8)
    l4 = ax2_twin.plot(df["timestamp"], df["wob_soft_sensor"], label="Soft WOB Observer (N)", color="#86efac", linestyle=":", lw=1.5)
    axs[1].set_ylabel("Torque [N·m]", color="#dc2626", fontweight="bold")
    ax2_twin.set_ylabel("WOB Force [N]", color="#16a34a", fontweight="bold")
    lines = l1 + l2 + l3 + l4
    axs[1].legend(lines, [line.get_label() for line in lines], loc="upper right", framealpha=0.9)
    axs[1].grid(True, linestyle="--", alpha=0.5)

    # Panel 3: Speeds (Spindle & Pump)
    axs[2].plot(df["timestamp"], df["spindle_rpm"], label="Spindle Speed (RPM)", color="#475569", lw=2)
    axs[2].plot(df["timestamp"], df["pump_cmd_rpm"], label="Pump Cmd (RPM)", color="#d97706", linestyle="--", lw=1.8)
    axs[2].plot(df["timestamp"], df["pump_rpm"], label="Pump Actual (RPM)", color="#f59e0b", lw=1.5)
    axs[2].set_ylabel("Speed [RPM]", fontweight="bold")
    axs[2].legend(loc="upper right", framealpha=0.9)
    axs[2].grid(True, linestyle="--", alpha=0.5)

    # Panel 4: Penetration & Material Removal
    ax4_twin = axs[3].twinx()
    p1 = axs[3].plot(df["timestamp"], df["penetration_depth_mm"], label="Crater Depth d (mm)", color="#7c3aed", lw=2)
    p2 = ax4_twin.plot(df["timestamp"], df["material_removal_rate_mm3_s"], label="MRR (mm³/s)", color="#0891b2", lw=1.8, linestyle="-.")
    axs[3].set_xlabel("Virtual Simulation Time [s]", fontweight="bold")
    axs[3].set_ylabel("Penetration [mm]", color="#7c3aed", fontweight="bold")
    ax4_twin.set_ylabel("MRR [mm³/s]", color="#0891b2", fontweight="bold")
    p_lines = p1 + p2
    axs[3].legend(p_lines, [line.get_label() for line in p_lines], loc="upper right", framealpha=0.9)
    axs[3].grid(True, linestyle="--", alpha=0.5)

    plt.tight_layout()
    plt.savefig(output_path, dpi=300)
    plt.close()
    print(f"[Runner] Saved publication-quality plot to: {output_path}")


def plot_comparison(df_adrc: pd.DataFrame, df_pid: pd.DataFrame, output_path: str) -> None:
    """Generate side-by-side benchmark comparison figure between ADRC and Baseline PID."""
    os.makedirs(os.path.dirname(os.path.abspath(output_path)), exist_ok=True)
    fig, axs = plt.subplots(3, 1, figsize=(10, 9), sharex=True)

    # Panel 1: Chamber Pressure Comparison
    axs[0].plot(df_adrc["timestamp"], df_adrc["pressure_true_bar"], label="Cascade LADRC: Pressure", color="#2563eb", lw=2)
    axs[0].plot(df_pid["timestamp"], df_pid["pressure_true_bar"], label="Baseline PID: Pressure", color="#dc2626", lw=1.8, linestyle="--")
    axs[0].set_ylabel("Pressure [bar]", fontweight="bold")
    axs[0].set_title("Controller Benchmark: Cascade LADRC vs Baseline PID on Inconel 718", fontsize=13, fontweight="bold")
    axs[0].legend(loc="upper right")
    axs[0].grid(True, linestyle="--", alpha=0.5)

    # Panel 2: Spindle Cutting Torque Comparison
    axs[1].plot(df_adrc["timestamp"], df_adrc["spindle_torque_true"], label="Cascade LADRC: Torque", color="#2563eb", lw=2)
    axs[1].plot(df_pid["timestamp"], df_pid["spindle_torque_true"], label="Baseline PID: Torque", color="#dc2626", lw=1.8, linestyle="--")
    axs[1].axhline(y=7.5, color="#f59e0b", linestyle=":", label="Torque Overload Threshold (7.5 N·m)")
    axs[1].set_ylabel("Torque [N·m]", fontweight="bold")
    axs[1].legend(loc="upper right")
    axs[1].grid(True, linestyle="--", alpha=0.5)

    # Panel 3: Spindle Speed Comparison (Highlighting Stall Immunity)
    axs[2].plot(df_adrc["timestamp"], df_adrc["spindle_rpm"], label="Cascade LADRC: Spindle RPM", color="#2563eb", lw=2)
    axs[2].plot(df_pid["timestamp"], df_pid["spindle_rpm"], label="Baseline PID: Spindle RPM", color="#dc2626", lw=1.8, linestyle="--")
    axs[2].set_xlabel("Virtual Simulation Time [s]", fontweight="bold")
    axs[2].set_ylabel("Spindle Speed [RPM]", fontweight="bold")
    axs[2].legend(loc="lower right")
    axs[2].grid(True, linestyle="--", alpha=0.5)

    plt.tight_layout()
    plt.savefig(output_path, dpi=300)
    plt.close()
    print(f"[Runner] Saved benchmark comparison to: {output_path}")


def main() -> None:
    parser = argparse.ArgumentParser(description="Deterministic Asymmetric Milling Simulator CLI")
    parser.add_argument("--scenario", type=str, default="step_engagement", choices=["step_engagement", "benchmark_overload", "relaxation_test"])
    parser.add_argument("--controller", type=str, default="adrc", choices=["adrc", "pid"])
    parser.add_argument("--duration", type=float, default=12.0)
    parser.add_argument("--fidelity", type=str, default="averaged", choices=["averaged", "tooth_resolved"])
    parser.add_argument("--output", type=str, default="figures/simulation_results.png")
    parser.add_argument("--compare", action="store_true", help="Run side-by-side LADRC vs PID benchmark")
    parser.add_argument("--export-csv", type=str, default=None)
    parser.add_argument("--export-parquet", type=str, default=None)

    args = parser.parse_args()

    if args.compare:
        print(f"[Runner] Executing comparative benchmark: LADRC vs PID for {args.duration}s...")
        frames_adrc = run_simulation(controller_type="adrc", scenario=args.scenario, duration=args.duration, fidelity=args.fidelity)
        frames_pid = run_simulation(controller_type="pid", scenario=args.scenario, duration=args.duration, fidelity=args.fidelity)

        df_adrc = frames_to_dataframe(frames_adrc)
        df_pid = frames_to_dataframe(frames_pid)

        plot_comparison(df_adrc, df_pid, args.output)

        if args.export_csv:
            df_adrc.to_csv(args.export_csv, index=False)
            print(f"[Runner] Exported ADRC CSV to: {args.export_csv}")
    else:
        print(f"[Runner] Executing simulation ({args.controller.upper()}) on scenario '{args.scenario}' for {args.duration}s...")
        frames = run_simulation(controller_type=args.controller, scenario=args.scenario, duration=args.duration, fidelity=args.fidelity)
        df = frames_to_dataframe(frames)

        title = "Cascade LADRC" if args.controller == "adrc" else "Baseline PID"
        plot_single_run(df, args.output, title_prefix=title)

        if args.export_csv:
            df.to_csv(args.export_csv, index=False)
            print(f"[Runner] Exported CSV to: {args.export_csv}")
        if args.export_parquet:
            df.to_parquet(args.export_parquet, index=False)
            print(f"[Runner] Exported Parquet to: {args.export_parquet}")


if __name__ == "__main__":
    main()
