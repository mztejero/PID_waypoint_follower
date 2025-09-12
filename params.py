from dataclasses import dataclass

@dataclass
class CarParams:
    L: float = 3.0
    W: float = 2.0
    R_lidar: float = 5.0
    dt: float = 0.05

@dataclass
class FrameParams:
    xmin: float = -20
    xmax: float = 20
    ymin: float = -20
    ymax: float = 20

@dataclass
class IC:
    v_long: float = 0.0
    x: float = 0.0
    y: float = 0.0
    yaw: float = 0.0

    vr: float = 0.0
    vl: float = 0.0

    v_ref: float = 20.0

@dataclass
class PIDParams:
    Kp_speed: float = 0.5
    Ki_speed: float = 0.1
    Kd_speed: float = 0.01

    Kp_steer: float = 4
    Ki_steer: float = 0.1
    Kd_steer: float = 0.01

    K_scalling: float = 0.4