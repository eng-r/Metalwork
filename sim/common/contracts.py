"""
Data contracts for process variables, control commands, and physical truth diagnostics.
"""

from dataclasses import dataclass


@dataclass(slots=True)
class ProcessVariables:
    """Sensor-equivalent telemetry exposed to the controller."""

    timestamp: float = 0.0
    pressure_hyd: float = 1.01325e5
    spindle_speed_res: float = 0.0
    pump_speed_res: float = 0.0
    iq_current: float = 0.0
    spindle_torque_est: float = 0.0
    rod_displacement: float = 0.0
    rod_velocity_est: float = 0.0
    wob_soft_sensor: float = 0.0

    @property
    def pressure_bar(self) -> float:
        return self.pressure_hyd / 1.0e5

    @property
    def spindle_rpm(self) -> float:
        return self.spindle_speed_res * 30.0 / 3.141592653589793

    @property
    def pump_rpm(self) -> float:
        return self.pump_speed_res * 30.0 / 3.141592653589793


@dataclass(slots=True)
class ControlCommands:
    """Actuator commands latched via zero-order hold."""

    timestamp: float = 0.0
    pump_speed_cmd_rpm: float = 0.0
    spindle_speed_cmd_rpm: float = 0.0
    enable_pump: bool = True
    enable_spindle: bool = True

    @property
    def pump_speed_cmd_rads(self) -> float:
        return self.pump_speed_cmd_rpm * 3.141592653589793 / 30.0

    @property
    def spindle_speed_cmd_rads(self) -> float:
        return self.spindle_speed_cmd_rpm * 3.141592653589793 / 30.0


@dataclass(slots=True)
class TruthDiagnostics:
    """Internal physical truth. Controllers must never inspect these fields."""

    timestamp: float = 0.0
    rod_position_true: float = 0.0
    rod_velocity_true: float = 0.0
    rod_acceleration_true: float = 0.0
    chamber_pressure_true: float = 1.01325e5
    pump_flow_rate: float = 0.0
    bypass_flow_rate: float = 0.0
    leakage_flow_rate: float = 0.0
    seal_friction_force: float = 0.0
    axial_cutting_force_true: float = 0.0
    spindle_cutting_torque_true: float = 0.0

    # Geometry / removal truth.
    penetration_depth: float = 0.0          # geometric tool intrusion from first contact [m]
    surface_recession_depth: float = 0.0    # actual removed/crater advance [m]
    engagement_depth: float = 0.0             # instantaneous unremoved interference [m]
    contact_area: float = 0.0
    material_removal_rate: float = 0.0      # physical MRR [m^3/s]
    cumulative_volume_removed: float = 0.0
    physical_rop: float = 0.0               # physical rate of penetration [m/s]
    demo_acceleration: float = 1.0          # material-evolution time-compression factor
    equivalent_process_time: float = 0.0    # accumulated equivalent cutting time [s]
    disturbance_event: str = "FREE"

    spindle_speed_true: float = 0.0
    pump_speed_true: float = 0.0
    chip_thickness: float = 0.0
    total_disturbance_f: float = 0.0
    leso_disturbance_est: float = 0.0
    operating_mode: str = "INITIALIZING"
