# PDSMC hover controller — port nguyen si tu run_PDSMC.m (MATLAB)
# Quy dao: x=0, y=0, z=3m, psi=0 (hover tai cho)
# PDSMC gains: kp1=100, kd1=40, H1=160, lam1=100 (dung MATLAB goc)
# Khong co ESO, co nhieu sin nho

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np


@dataclass
class PDSMCGains:
    """PDSMC gains — giong huong dan trong run_PDSMC.m."""
    kp1: float = 100.0
    kd1: float = 40.0
    H1: float = 160.0
    lam1: float = 100.0
    kp2: float = 10.0
    kd2: float = 2.0
    H2: float = 0.5
    lam2: float = 5.0
    kp3: float = 10.0
    kd3: float = 2.0
    H3: float = 0.5
    lam3: float = 5.0
    kp4: float = 10.0
    kd4: float = 2.0
    H4: float = 0.5
    lam4: float = 5.0
    kpx: float = 1.5
    kdx: float = 1.0
    Hx: float = 0.5
    lamx: float = 0.5
    kpy: float = 1.5
    kdy: float = 1.0
    Hy: float = 0.5
    lamy: float = 0.5


@dataclass
class QuadPlantParams:
    """Thong so vat ly — giong MATLAB."""
    m: float = 0.8
    g: float = 9.81
    Ix: float = 0.005
    Iy: float = 0.005
    Iz: float = 0.009


# === THAY DOI 1: Class quy dao hinh so 8 ===
# Phuong trinh so 8 (port tu MATLAB):
#   xd  = A*sin(w*t)     yd  = A*sin(2*w*t)     zd  = z_const
#   w = 2*pi / T  (T = chu ky, 1 vong so 8 = T/2 giay)
#   1 vong so 8 hoan chinh = T giay (vi sin(2w*t) co chu ky T/2)
#
# THAY DOI QUY DAO: Chi sua tham so A, T, z_const ben duoi.
# Muon quy dao khac? Viet class moi nhu Figure8Trajectory roi doi
# Trajectory class ben duoi.


class Figure8Trajectory:
    """
    Quy dao hinh so 8 — port nguyen si tu MATLAB.

    Phuong trinh (MATLAB):
        A  = 3;  w = 2*pi/100;
        xd_f    = A*sin(w*t)
        yd_f    = A*sin(2*w*t)
        zd_f    = 3*ones
        xd_d_f  = A*w*cos(w*t)
        yd_d_f  = 2*A*w*cos(2*w*t)
        xd_dd_f = -A*w^2*sin(w*t)
        yd_dd_f = -4*A*w^2*sin(2*w*t)

    Tham so (THAY DOI TAI DAY):
        A      = bien do (m)          -> thay doi KICH THUOC hinh so 8
        T      = chu ky (giay)        -> thay doi TOC DO bay
        z_const = do cao (m)           -> thay doi DO CAO bay
    """

    # === THAY DOI THAM SO QUY DAO TAI DAY ===
    A: float = 3.0       # bien do (m) — ban kinh hinh so 8 theo x
    T: float = 100.0     # chu ky (s) — 1 vong so 8 hoan chinh = T giay
    z_const: float = 3.0  # do cao (m)

    def __init__(self):
        self.w = 2.0 * math.pi / self.T   # tan so goc

    def at(self, t: float) -> dict:
        w = self.w
        A = self.A
        return {
            # Vi tri
            "xd":    A * math.sin(w * t),
            "yd":    A * math.sin(2.0 * w * t),
            "zd":    self.z_const,
            "psid":  0.0,
            # Van toc
            "xd_d":    A * w * math.cos(w * t),
            "yd_d":    2.0 * A * w * math.cos(2.0 * w * t),
            "zd_d":    0.0,
            "psid_d":  0.0,
            # Gia toc
            "xd_dd":   -A * w * w * math.sin(w * t),
            "yd_dd":   -4.0 * A * w * w * math.sin(2.0 * w * t),
            "zd_dd":   0.0,
            "psid_dd": 0.0,
        }

    def period(self) -> float:
        """1 vong so 8 hoan chinh = T giay."""
        return self.T


class HoverTrajectory:
    """Quy dao hover: x=0, y=0, z=z_const, psi=0."""

    def __init__(self, z_const: float = 3.0):
        self.z_const = float(z_const)

    def at(self, t: float):  # noqa: ARG002
        return {
            "xd":     0.0,
            "yd":     0.0,
            "zd":     self.z_const,
            "psid":   0.0,
            "xd_d":   0.0,
            "yd_d":   0.0,
            "zd_d":   0.0,
            "psid_d": 0.0,
            "xd_dd":  0.0,
            "yd_dd":  0.0,
            "zd_dd":  0.0,
            "psid_dd": 0.0,
        }


class CircleTrajectory:
    """
    Quy dao hinh tron — tâm tại (xc, yc), bán kính R, độ cao z_const.

    Phuong trinh:
        xd  = xc + R * cos(theta)
        yd  = yc + R * sin(theta)
        zd  = z_const
        theta = w * t
    """

    def __init__(
        self,
        xc: float = 0.0,
        yc: float = 0.0,
        R: float = 5.0,
        w: float = 0.3,
        z_const: float = 3.0,
    ):
        self.xc = float(xc)
        self.yc = float(yc)
        self.R = float(R)
        self.w = float(w)
        self.z_const = float(z_const)

    def at(self, t: float) -> dict:
        theta = self.w * t
        ct = math.cos(theta)
        st = math.sin(theta)
        return {
            "xd":    self.xc + self.R * ct,
            "yd":    self.yc + self.R * st,
            "zd":    self.z_const,
            "psid":  0.0,
            "xd_d":  -self.R * self.w * st,
            "yd_d":   self.R * self.w * ct,
            "zd_d":   0.0,
            "psid_d": 0.0,
            "xd_dd":  -self.R * self.w * self.w * ct,
            "yd_dd":  -self.R * self.w * self.w * st,
            "zd_dd":   0.0,
            "psid_dd": 0.0,
        }

    def period(self) -> float:
        """1 vong tron hoan chinh = 2*pi / w giay."""
        if abs(self.w) < 1e-9:
            return float("inf")
        return 2.0 * math.pi / abs(self.w)


def build_xd_v12(tr: dict) -> np.ndarray:
    """Vector xd_v 12x1 giong MATLAB: [xd, xd_d, yd, yd_d, zd, zd_d, 0,0,0,0,0,0]."""
    return np.array(
        [
            tr["xd"],
            tr["xd_d"],
            tr["yd"],
            tr["yd_d"],
            tr["zd"],
            tr["zd_d"],
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
        ],
        dtype=float,
    )


def pdsmc_step(
    x: np.ndarray,
    traj,      # HoverTrajectory | Figure8Trajectory | CircleTrajectory
    gains: PDSMCGains,
    plant: QuadPlantParams,
    t: float = 0.0,
    psi: float = 0.0,
) -> dict:
    """
    Mot buoc PDSMC — port tu ham ctrl_PDSMC trong run_PDSMC.m.

    x: (12,) — [x, xd, y, yd, z, zd, phi, phid, theta, thetad, psi, psid]
    traj: HoverTrajectory | Figure8Trajectory
    t: thoi gian tu luc bat dau dieu khien (s) — de tinh quy dao phu thuoc t
    tra ve dict chua U1..U4 va gia tri trung gian.

    LUU Y ve 2 fix so voi MATLAB goc:
    1. Gravity feedforward: U1 += m*g (MATLAB khong co, boi vi trong mo phong
       MATLAB, dieu kien dau z0=0 nen PID "thay" duoc luc hut, nhung thuc te ArduPilot
       GUIDED giữ z nen U1 phai ít nhất bang m*g.)
    2. Chuan hoa sinphi/sintheta theo (U1/m) — cung ly do.
    """
    assert x.shape == (12,)

    tr = traj.at(t)
    xd_v = build_xd_v12(tr)

    kp1 = gains.kp1;  kd1 = gains.kd1;  H1 = gains.H1;  lam1 = gains.lam1
    kp2 = gains.kp2;  kd2 = gains.kd2;  H2 = gains.H2;  lam2 = gains.lam2
    kp3 = gains.kp3;  kd3 = gains.kd3;  H3 = gains.H3;  lam3 = gains.lam3
    kp4 = gains.kp4;  kd4 = gains.kd4;  H4 = gains.H4;  lam4 = gains.lam4
    kpx = gains.kpx;  kdx = gains.kdx;  Hx = gains.Hx;  lamx = gains.lamx
    kpy = gains.kpy;  kdy = gains.kdy;  Hy = gains.Hy;  lamy = gains.lamy

    m = plant.m

    # --- PDSMC altitude (z) — kenh U1 ---
    # Giong MATLAB: U1 = kp1*ez + kd1*ez_d + H1*tanh(ez_d + lam1*ez)
    # Chi them gravity feedforward + clamp (xem docstring)
    ez   = xd_v[4] - x[4]     # xd_v[4] = zd = z_const
    ez_d = xd_v[5] - x[5]     # xd_v[5] = 0

    U1_raw = (
        plant.m * plant.g
        + kp1 * ez
        + kd1 * ez_d
        + H1 * math.tanh(ez_d + lam1 * ez)
    )
    U1 = max(0.5 * plant.m * plant.g, U1_raw)

    # --- PDSMC yaw — kenh U4 ---
    # psid=0 tu quy dao -> khong co yaw rate tham chieu
    # Tra ve U4=0 de ArduPilot GIAO LUON giu yaw (khong can thiep)
    psi_ref = tr["psid"]         # = 0
    epsi   = psi_ref - x[10]    # = 0 - psi
    epsi_d = xd_v[11] - x[11]   # = 0
    U4 = 0.0                     # ArduPilot GUIDED giu yaw, khong can U4

    # --- PDSMC x,y — kenh Ux, Uy ---
    ex   = xd_v[0] - x[0]
    ex_d = xd_v[1] - x[1]
    ey   = xd_v[2] - x[2]
    ey_d = xd_v[3] - x[3]

    Ux = kpx * ex + kdx * ex_d + Hx * math.tanh(ex_d + lamx * ex)
    Uy = kpy * ey + kdy * ey_d + Hy * math.tanh(ey_d + lamy * ey)

    # --- Phan bo Ux,Uy -> goc roll/pitch (phides, thetades) ---
    # Giong MATLAB (sau khi da fix chuan hoa):
    #   sinphi   = (Ux*sin(psi) - Uy*cos(psi)) / (U1/m)
    #   sintheta = (Ux*cos(psi) + Uy*sin(psi)) / ((U1/m)*cos(phi))
    T_norm = max(U1 / m, 1e-6)
    sinphi = (Ux * math.sin(psi) - Uy * math.cos(psi)) / T_norm
    phides = math.asin(float(np.clip(sinphi, -1.0, 1.0)))
    sintheta = (Ux * math.cos(psi) + Uy * math.sin(psi)) / (
        T_norm * max(math.cos(phides), 1e-6)
    )
    thetades = math.asin(float(np.clip(sintheta, -1.0, 1.0)))

    # --- PDSMC attitude — kenh U2 (phi), U3 (theta) ---
    # phides da tinh o tren, xd_v[7]=xd_v[9]=0
    ephi   = phides - x[6]
    ephi_d = xd_v[7] - x[7]  # = 0 - phid

    etheta   = thetades - x[8]
    etheta_d = xd_v[9] - x[9]  # = 0 - thetad

    U2 = kp2 * ephi + kd2 * ephi_d + H2 * math.tanh(ephi_d + lam2 * ephi)
    U3 = kp3 * etheta + kd3 * etheta_d + H3 * math.tanh(etheta_d + lam3 * etheta)

    # --- Gia tri trung gian tra ve (chi debug, khong dung trong tinh toan chinh) ---
    cph = math.cos(phides)
    cth = math.cos(thetades)
    ax_des = -plant.g + cph * cth * U1 / m
    ay_des = 0.0
    az_des = cph * cth * U1 / m

    return {
        "U1":         U1,
        "U2":         U2,
        "U3":         U3,
        "U4":         U4,
        "Ux":         Ux,
        "Uy":         Uy,
        "phides":     phides,
        "thetades":   thetades,
        "psi":         psi,
        "psid_ref":    psi_ref,
        "ax_des":     ax_des,
        "ay_des":     ay_des,
        "az_des":     az_des,
        "xd_d":       tr["xd_d"],
        "yd_d":       tr["yd_d"],
        "zd_d":       tr["zd_d"],
        "xd_ref":     tr["xd"],
        "yd_ref":     tr["yd"],
        "zd_ref":     tr["zd"],
    }
