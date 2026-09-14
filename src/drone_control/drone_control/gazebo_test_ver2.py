
import csv
import math
import os
import time

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from geometry_msgs.msg import PoseStamped, TwistStamped
from nav_msgs.msg import Path
from sensor_msgs.msg import Imu
from mavros_msgs.msg import State, ActuatorControl
from mavros_msgs.srv import CommandBool, SetMode, CommandLong


# ============================================================
# QUADROTOR
# IMPROVED PDSMC
# + 2-FREQUENCY DISTURBANCE OBSERVER
# + INPUT SATURATION
# + ANTI-SATURATION
# + ROTOR SPEED ALLOCATION
# + MAVROS ACTUATOR_CONTROL controls[8]
# ============================================================


class PDSMCController:

    def __init__(self):

        # ====================================================
        # QUADROTOR PARAMETERS (from MATLAB)
        # ====================================================
        self.m = 0.8

        self.Ix = 0.005
        self.Iy = 0.005
        self.Iz = 0.009

        self.g = 9.81

        # a1 = (Iy-Iz)/Ix
        # a3 = (Iz-Ix)/Iy
        # a5 = (Ix-Iy)/Iz
        self.a1 = (self.Iy - self.Iz) / self.Ix
        self.a3 = (self.Iz - self.Ix) / self.Iy
        self.a5 = (self.Ix - self.Iy) / self.Iz

        self.b1 = 1.0 / self.Ix
        self.b2 = 1.0 / self.Iy
        self.b3 = 1.0 / self.Iz

        # ====================================================
        # PDSMC PARAMETERS (from MATLAB)
        # ====================================================
        self.kp1 = 100.0
        self.kp2 = 10.0
        self.kp3 = 10.0
        self.kp4 = 10.0

        self.kd1 = 100.0
        self.kd2 = 2.0
        self.kd3 = 2.0
        self.kd4 = 2.0

        self.H1 = 50.0
        self.H2 = 1.5
        self.H3 = 1.5
        self.H4 = 1.5

        self.lambda1 = 100.0
        self.lambda2 = 5.0
        self.lambda3 = 5.0
        self.lambda4 = 5.0

        # ====================================================
        # ANTI-SATURATION (MATLAB: p=10, q=15)
        # ====================================================
        self.p = 10.0
        self.q = 15.0

        # beta1 ... beta8
        self.beta = np.zeros(8)

        # ====================================================
        # DISTURBANCE OBSERVER (from MATLAB)
        # ====================================================
        self.w11 = 0.5
        self.w12 = 1.2

        self.k1 = 200.0
        self.k2 = 200.0
        self.k3 = 200.0
        self.k4 = 200.0

        # 16 observer states
        # phi   : z[0:4]
        # theta : z[4:8]
        # psi   : z[8:12]
        # z     : z[12:16]
        self.zobs = np.zeros(16)

        # ====================================================
        # INPUT SATURATION (MATLAB: satU1=20, satU2/3/4=0.2)
        # ====================================================
        self.U1_MAX = 20.0
        self.U2_MAX = 0.2
        self.U3_MAX = 0.2
        self.U4_MAX = 0.2

        # ====================================================
        # ROTOR PARAMETERS
        # MUST match Gazebo model SDF:
        #   motor_constant   -> kf
        #   moment_constant  -> km
        #   max_rot_velocity -> OMEGA_MAX
        # ====================================================
        self.arm      = 0.165       # arm length [m]
        self.kf       = 5.567e-4   # thrust coefficient
        self.km       = 1.0e-5     # drag/yaw coefficient
        self.OMEGA_MAX = 1200.0    # max rotor speed [rad/s]

    # ========================================================
    # SATURATION
    # ========================================================
    @staticmethod
    def sat(u, umax):
        return max(-umax, min(u, umax))

    # ========================================================
    # CONTROLLER
    #
    # Quỹ đạo tham chiếu được truyền từ bên ngoài (node):
    #   xd, xd_dot, xd_ddot  — tham chiếu x và đạo hàm
    #   yd, yd_dot, yd_ddot  — tham chiếu y và đạo hàm
    #   zd, zd_dot, zd_ddot  — tham chiếu z và đạo hàm
    # ========================================================
    def compute(
        self,
        t,
        x,
        vx,
        y,
        vy,
        z,
        vz,
        phi,
        p_rate,
        theta,
        q_rate,
        psi,
        r_rate,
        dt,
        xd,
        xd_dot,
        xd_ddot,
        yd,
        yd_dot,
        yd_ddot,
        zd,
        zd_dot,
        zd_ddot,
    ):

        psi_d = 0.0

        # ====================================================
        # ERRORS
        # ====================================================
        ex = xd - x
        ey = yd - y
        ez = zd - z

        ex_dot = xd_dot - vx
        ey_dot = yd_dot - vy
        ez_dot = zd_dot - vz

        # ====================================================
        # DISTURBANCE ESTIMATION
        #
        # Khop MATLAB:
        # phi_dis = z_d(1) + k1*x(8) + z_d(3) + k3*x(8)
        # (2-frequency observer output)
        # ====================================================

        dphi = (
            self.zobs[0]
            + self.k1 * p_rate
            + self.zobs[2]
            + self.k3 * p_rate
        )

        dtheta = (
            self.zobs[4]
            + self.k1 * q_rate
            + self.zobs[6]
            + self.k3 * q_rate
        )

        dpsi = (
            self.zobs[8]
            + self.k1 * r_rate
            + self.zobs[10]
            + self.k3 * r_rate
        )

        dz = (
            self.zobs[12]
            + self.k1 * vz
            + self.zobs[14]
            + self.k3 * vz
        )

        # ====================================================
        # U1  (MATLAB lines 220-222)
        # ====================================================

        cphi   = math.cos(phi)
        ctheta = math.cos(theta)

        denominator = cphi * ctheta

        # Bao ve so hoc: tranh chia cho 0 khi lat
        if abs(denominator) < 0.05:
            denominator = (
                0.05 if denominator >= 0.0
                else -0.05
            )

        anti_z = (
            -15.0 * self.beta[1]
            - self.beta[0]
            - 10.0 * (
                -10.0 * self.beta[0]
                + self.beta[1]
            )
        )

        U1 = (
            self.kp1 * ez
            + self.kd1 * ez_dot
            + self.H1 * math.tanh(
                ez_dot + self.lambda1 * ez
            )
            - self.m / denominator * anti_z
            - self.m / denominator * dz
        )

        # ====================================================
        # VIRTUAL CONTROL Ux, Uy  (MATLAB lines 227-235)
        # ====================================================

        sx = 5.0 * ex + ex_dot
        sy = 5.0 * ey + ey_dot

        if abs(U1) < 1.0e-8:

            Ux = 0.0
            Uy = 0.0

        else:

            Ux = (
                self.m / U1
                * (
                    xd_ddot
                    + 5.0 * ex_dot
                    + 2.0 * math.tanh(sx)
                )
            )

            Uy = (
                self.m / U1
                * (
                    yd_ddot
                    + 5.0 * ey_dot
                    + 2.0 * math.tanh(sy)
                )
            )

        # ====================================================
        # DESIRED PHI
        # psi_d=0 => sin(psi_d)=0, cos(psi_d)=1
        # => sin_phi_d = -Uy
        # ====================================================

        sin_phi_d = (
            Ux * math.sin(psi_d)
            - Uy * math.cos(psi_d)
        )

        sin_phi_d = max(-1.0, min(1.0, sin_phi_d))

        phi_d = math.asin(sin_phi_d)

        # ====================================================
        # DESIRED THETA
        # psi_d=0 => sin_theta_d = Ux / cos(phi_d)
        # ====================================================

        cos_phi_d = math.cos(phi_d)

        if abs(cos_phi_d) < 1.0e-6:
            cos_phi_d = 1.0e-6

        sin_theta_d = (
            Ux * math.cos(psi_d)
            + Uy * math.sin(psi_d)
        ) / cos_phi_d

        sin_theta_d = max(-1.0, min(1.0, sin_theta_d))

        theta_d = math.asin(sin_theta_d)

        # ====================================================
        # ATTITUDE ERRORS
        # ====================================================

        ephi   = phi_d   - phi
        etheta = theta_d - theta
        epsi   = psi_d   - psi

        # ====================================================
        # U2 — ROLL  (MATLAB lines 245-247)
        # kd2*(xd(8)-x(8)) = kd2*(0-p_rate) = kd2*(-p_rate)
        # ====================================================

        anti_phi = (
            -15.0 * self.beta[3]
            - self.beta[2]
            - 10.0 * (
                -10.0 * self.beta[2]
                + self.beta[3]
            )
        )

        U2 = (
            self.kp2 * ephi
            + self.kd2 * (-p_rate)
            + self.H2 * math.tanh(
                -p_rate
                + self.lambda2 * ephi
            )
            - (1.0 / self.b1) * anti_phi
            - (1.0 / self.b1) * dphi
        )

        # ====================================================
        # U3 — PITCH  (MATLAB lines 242-244)
        # kd3*(xd(10)-x(10)) = kd3*(0-q_rate) = kd3*(-q_rate)
        # ====================================================

        anti_theta = (
            -15.0 * self.beta[5]
            - self.beta[4]
            - 10.0 * (
                -10.0 * self.beta[4]
                + self.beta[5]
            )
        )

        U3 = (
            self.kp3 * etheta
            + self.kd3 * (-q_rate)
            + self.H3 * math.tanh(
                -q_rate
                + self.lambda3 * etheta
            )
            - (1.0 / self.b2) * anti_theta
            - (1.0 / self.b2) * dtheta
        )

        # ====================================================
        # U4 — YAW  (MATLAB lines 223-225)
        # kd4*(xd(12)-x(12)) = kd4*(0-r_rate) = kd4*(-r_rate)
        # ====================================================

        anti_psi = (
            -15.0 * self.beta[7]
            - self.beta[6]
            - 10.0 * (
                -10.0 * self.beta[6]
                + self.beta[7]
            )
        )

        U4 = (
            self.kp4 * epsi
            + self.kd4 * (-r_rate)
            + self.H4 * math.tanh(
                -r_rate
                + self.lambda4 * epsi
            )
            - (1.0 / self.b3) * anti_psi
            - (1.0 / self.b3) * dpsi
        )

        # ====================================================
        # SATURATION  (MATLAB lines 249-254)
        # ====================================================

        satU1 = self.sat(U1, self.U1_MAX)
        satU2 = self.sat(U2, self.U2_MAX)
        satU3 = self.sat(U3, self.U3_MAX)
        satU4 = self.sat(U4, self.U4_MAX)

        # ====================================================
        # ANTI-SATURATION  (MATLAB lines 257-264)
        # ====================================================

        beta_dot = np.zeros(8)

        beta_dot[0] = (
            -self.p * self.beta[0]
            + self.beta[1]
        )

        beta_dot[1] = (
            -self.q * self.beta[1]
            - self.beta[0]
            + cphi * ctheta
            * (satU1 - U1) / self.m
        )

        beta_dot[2] = (
            -self.p * self.beta[2]
            + self.beta[3]
        )

        beta_dot[3] = (
            -self.q * self.beta[3]
            - self.beta[2]
            + self.b1 * (satU2 - U2)
        )

        beta_dot[4] = (
            -self.p * self.beta[4]
            + self.beta[5]
        )

        beta_dot[5] = (
            -self.q * self.beta[5]
            - self.beta[4]
            + self.b2 * (satU3 - U3)
        )

        beta_dot[6] = (
            -self.p * self.beta[6]
            + self.beta[7]
        )

        beta_dot[7] = (
            -self.q * self.beta[7]
            - self.beta[6]
            + self.b3 * (satU4 - U4)
        )

        # Euler integration
        self.beta += beta_dot * dt

        # ====================================================
        # OBSERVER DYNAMICS  (MATLAB lines 272-295)
        # alpha_i dung satU (khong phai U tho)
        # ====================================================

        alpha_phi = (
            q_rate * r_rate * self.a1
            + self.b1 * satU2
        )

        alpha_theta = (
            p_rate * r_rate * self.a3
            + self.b2 * satU3
        )

        alpha_psi = (
            p_rate * q_rate * self.a5
            + self.b3 * satU4
        )

        alpha_z = (
            -self.g
            + cphi * ctheta * satU1 / self.m
        )

        # ----------------------------------------------------
        # PHI OBSERVER  (kenh x(8) = p_rate)
        # ----------------------------------------------------

        z1_dot_phi = (
            self.w11 * self.zobs[1]
            - self.k1 * (alpha_phi + dphi)
            + self.w11 * self.k2 * p_rate
        )

        z2_dot_phi = (
            -self.w11 * self.zobs[0]
            - self.k2 * (alpha_phi + dphi)
            - self.w11 * self.k1 * p_rate
        )

        z3_dot_phi = (
            self.w12 * self.zobs[3]
            - self.k3 * (alpha_phi + dphi)
            + self.w12 * self.k4 * p_rate
        )

        z4_dot_phi = (
            -self.w12 * self.zobs[2]
            - self.k4 * (alpha_phi + dphi)
            - self.w12 * self.k3 * p_rate
        )

        # ----------------------------------------------------
        # THETA OBSERVER  (kenh x(10) = q_rate)
        # ----------------------------------------------------

        z1_dot_theta = (
            self.w11 * self.zobs[5]
            - self.k1 * (alpha_theta + dtheta)
            + self.w11 * self.k2 * q_rate
        )

        z2_dot_theta = (
            -self.w11 * self.zobs[4]
            - self.k2 * (alpha_theta + dtheta)
            - self.w11 * self.k1 * q_rate
        )

        z3_dot_theta = (
            self.w12 * self.zobs[7]
            - self.k3 * (alpha_theta + dtheta)
            + self.w12 * self.k4 * q_rate
        )

        z4_dot_theta = (
            -self.w12 * self.zobs[6]
            - self.k4 * (alpha_theta + dtheta)
            - self.w12 * self.k3 * q_rate
        )

        # ----------------------------------------------------
        # PSI OBSERVER  (kenh x(12) = r_rate)
        # ----------------------------------------------------

        z1_dot_psi = (
            self.w11 * self.zobs[9]
            - self.k1 * (alpha_psi + dpsi)
            + self.w11 * self.k2 * r_rate
        )

        z2_dot_psi = (
            -self.w11 * self.zobs[8]
            - self.k2 * (alpha_psi + dpsi)
            - self.w11 * self.k1 * r_rate
        )

        z3_dot_psi = (
            self.w12 * self.zobs[11]
            - self.k3 * (alpha_psi + dpsi)
            + self.w12 * self.k4 * r_rate
        )

        z4_dot_psi = (
            -self.w12 * self.zobs[10]
            - self.k4 * (alpha_psi + dpsi)
            - self.w12 * self.k3 * r_rate
        )

        # ----------------------------------------------------
        # Z OBSERVER  (kenh x(6) = vz)
        # ----------------------------------------------------

        z1_dot_z = (
            self.w11 * self.zobs[13]
            - self.k1 * (alpha_z + dz)
            + self.w11 * self.k2 * vz
        )

        z2_dot_z = (
            -self.w11 * self.zobs[12]
            - self.k2 * (alpha_z + dz)
            - self.w11 * self.k1 * vz
        )

        z3_dot_z = (
            self.w12 * self.zobs[15]
            - self.k3 * (alpha_z + dz)
            + self.w12 * self.k4 * vz
        )

        z4_dot_z = (
            -self.w12 * self.zobs[14]
            - self.k4 * (alpha_z + dz)
            - self.w12 * self.k3 * vz
        )

        # Observer Euler update
        z_dot = np.array([
            z1_dot_phi,
            z2_dot_phi,
            z3_dot_phi,
            z4_dot_phi,

            z1_dot_theta,
            z2_dot_theta,
            z3_dot_theta,
            z4_dot_theta,

            z1_dot_psi,
            z2_dot_psi,
            z3_dot_psi,
            z4_dot_psi,

            z1_dot_z,
            z2_dot_z,
            z3_dot_z,
            z4_dot_z
        ])

        self.zobs += z_dot * dt

        return (
            U1,
            U2,
            U3,
            U4,
            satU1,
            satU2,
            satU3,
            satU4,
            phi_d,
            theta_d
        )

    # ========================================================
    # 4U -> 4 ROTOR ANGULAR VELOCITIES
    #
    # Quad-X mixing matrix.
    # QUAN TRONG: Kiem tra thu tu rotor va dau voi SDF Gazebo.
    # ========================================================
    def U_to_omega(
        self,
        satU1,
        satU2,
        satU3,
        satU4
    ):
        # Chi nhan gia tri DA bao hoa (satU), khong dung U tho.
        # Duoc goi voi satU1..satU4 tu control_loop.

        kf = self.kf
        km = self.km
        L  = self.arm

        w1_sq = (
            satU1 / (4.0 * kf)
            + satU3 / (2.0 * L * kf)
            + satU4 / (4.0 * km)
        )

        w2_sq = (
            satU1 / (4.0 * kf)
            - satU2 / (2.0 * L * kf)
            - satU4 / (4.0 * km)
        )

        w3_sq = (
            satU1 / (4.0 * kf)
            - satU3 / (2.0 * L * kf)
            + satU4 / (4.0 * km)
        )

        w4_sq = (
            satU1 / (4.0 * kf)
            + satU2 / (2.0 * L * kf)
            - satU4 / (4.0 * km)
        )

        w1 = math.sqrt(max(0.0, w1_sq))
        w2 = math.sqrt(max(0.0, w2_sq))
        w3 = math.sqrt(max(0.0, w3_sq))
        w4 = math.sqrt(max(0.0, w4_sq))

        return w1, w2, w3, w4

    # ========================================================
    # OMEGA -> controls[8]  (chuan hoa ve [-1, +1])
    # ========================================================
    def omega_to_control(
        self,
        omega1,
        omega2,
        omega3,
        omega4
    ):

        def normalize(w):
            u = w / self.OMEGA_MAX
            # ArduPilot motor range: 0.0 (off) -> 1.0 (full)
            # PX4 dung [-1, 1], ArduPilot dung [0, 1]
            return max(0.0, min(1.0, u))

        # EXACTLY 8 ELEMENTS
        controls = [
            normalize(omega1),
            normalize(omega2),
            normalize(omega3),
            normalize(omega4),
            0.0,
            0.0,
            0.0,
            0.0
        ]

        return controls


# ============================================================
# ROS2 NODE
# ============================================================

class PDSMCGazeboNode(Node):

    # ====================================================
    # MISSION STATES  (giong CircleMission)
    # ====================================================
    WAIT_CONN     = 0
    SEND_GUIDED   = 1
    WAIT_GUIDED   = 2
    SEND_ARM      = 3
    WAIT_ARM      = 4
    WAIT_STABLE   = 15   # cho FCU xac nhan armed + delay on dinh
    SEND_TAKEOFF  = 5
    WAIT_TAKEOFF  = 6
    CLIMBING      = 7    # PDSMC giu vi tri, ArduPilot leo cao
    FLY_TO_START  = 8    # PDSMC bay den diem dau vong tron
    CIRCLE        = 9    # PDSMC bam quy dao vong tron
    FLY_TO_CENTER = 10   # PDSMC bay ve tam vong tron
    SEND_LAND     = 11
    WAIT_LAND     = 12
    WAIT_DISARM   = 13
    DONE          = 14

    def __init__(self):

        super().__init__('pdsmc_gazebo_controller')

        # Su dung thoi gian mo phong cua Gazebo (/clock topic)
        self.set_parameters([
            rclpy.parameter.Parameter(
                'use_sim_time',
                rclpy.Parameter.Type.BOOL,
                True
            )
        ])

        # ====================================================
        # PARAMETERS
        # ====================================================
        self.declare_parameter('save_log', True)
        self.save_log = self.get_parameter('save_log').get_parameter_value().bool_value

        # ====================================================
        # DT (50 Hz control loop)
        # ====================================================
        self.DT = 0.02

        # ====================================================
        # VEHICLE STATE
        # ====================================================

        self.connected = False
        self.armed     = False
        self.mode      = ''

        # Position
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0

        # Linear velocity
        self.vx = 0.0
        self.vy = 0.0
        self.vz = 0.0

        # Euler angles [rad]
        self.phi   = 0.0
        self.theta = 0.0
        self.psi   = 0.0

        # Body angular rates [rad/s]
        self.p = 0.0
        self.q = 0.0
        self.r = 0.0

        # Simulation time - dong bo tu Gazebo clock qua pose_callback
        self.sim_time = 0.0

        # ====================================================
        # MISSION STATE MACHINE
        # ====================================================
        self.mission_state  = self.WAIT_CONN
        self.pending_future = None

        # Arm / stabilisation
        self.arm_time       = None
        self.ARM_STABLE_SEC = 2.0   # giay cho sau khi FCU xac nhan armed

        # Circle / mission parameters
        self.radius   = 3.0         # ban kinh vong tron [m]
        self.w        = 0.3         # toc do goc [rad/s]
        self.x0       = None        # tam vong tron (dat sau khi leo cao xong)
        self.y0       = None
        self.takeoff_x = 0.0        # vi tri cat canh (giu trong CLIMBING)
        self.takeoff_y = 0.0
        self.takeoff_z = 0.0
        self.target_z  = 0.0        # do cao muc tieu
        self.circle_start_time = None  # thoi diem bat dau vong tron

        # ====================================================
        # CONTROLLER
        # ====================================================
        self.controller = PDSMCController()

        # ====================================================
        # QoS
        # ====================================================
        # Sensor topics: BEST_EFFORT (giam overhead, khong can reliable)
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # State topic: RELIABLE + TRANSIENT_LOCAL
        # MAVROS publish /mavros/state voi Transient Local
        # => subscriber PHAI khop, neu khong se khong nhan duoc ban tin nao!
        state_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=10
        )

        # ====================================================
        # SUBSCRIBERS
        # ====================================================

        self.pose_sub = self.create_subscription(
            PoseStamped,
            '/mavros/local_position/pose',
            self.pose_callback,
            sensor_qos
        )

        self.velocity_sub = self.create_subscription(
            TwistStamped,
            '/mavros/local_position/velocity_local',
            self.velocity_callback,
            sensor_qos
        )

        self.imu_sub = self.create_subscription(
            Imu,
            '/mavros/imu/data',
            self.imu_callback,
            sensor_qos
        )

        self.state_sub = self.create_subscription(
            State,
            '/mavros/state',
            self.state_callback,
            state_qos
        )

        # ====================================================
        # PUBLISHERS
        # ====================================================

        self.actuator_pub = self.create_publisher(
            ActuatorControl,
            '/mavros/actuator_control',
            10
        )

        # Path publishers (planned / actual trajectory)
        self.planned_path_pub = self.create_publisher(
            Path,
            '/planned_path',
            10
        )

        self.actual_path_pub = self.create_publisher(
            Path,
            '/actual_path',
            10
        )

        self.planned_path = Path()
        self.planned_path.header.frame_id = 'map'

        self.actual_path = Path()
        self.actual_path.header.frame_id = 'map'

        # ====================================================
        # SERVICE CLIENTS
        # ====================================================

        self.arming_client = self.create_client(
            CommandBool,
            '/mavros/cmd/arming'
        )

        self.set_mode_client = self.create_client(
            SetMode,
            '/mavros/set_mode'
        )

        self.takeoff_client = self.create_client(
            CommandLong,
            '/mavros/cmd/command'
        )

        # Cho service san sang
        while not self.set_mode_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for set_mode service...')
        while not self.arming_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for arming service...')
        while not self.takeoff_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for takeoff service...')

        # ====================================================
        # CONTROL TIMER (50 Hz)
        # ====================================================

        self.timer = self.create_timer(
            self.DT,
            self.control_loop
        )

        self.get_logger().info(
            'PDSMC + CircleMission controller started. '
            'Waiting for ArduPilot/MAVROS connection...'
        )

    # ========================================================
    # SET STREAM RATE (MAV_CMD_SET_MESSAGE_INTERVAL)
    # ========================================================
    def set_stream_rate(self, stream_id, rate):
        req = CommandLong.Request()
        req.command = 511   # MAV_CMD_SET_MESSAGE_INTERVAL
        req.param1  = float(stream_id)
        req.param2  = float(1_000_000 // rate)  # microseconds
        self.takeoff_client.call_async(req)

    # ========================================================
    # QUATERNION -> EULER (ZYX convention)
    # ========================================================
    def quaternion_to_euler(self, qx, qy, qz, qw):

        # Roll (phi)
        sinr_cosp = 2.0 * (qw * qx + qy * qz)
        cosr_cosp = 1.0 - 2.0 * (qx * qx + qy * qy)
        roll = math.atan2(sinr_cosp, cosr_cosp)

        # Pitch (theta)
        sinp = 2.0 * (qw * qy - qz * qx)
        if abs(sinp) >= 1.0:
            pitch = math.copysign(math.pi / 2.0, sinp)
        else:
            pitch = math.asin(sinp)

        # Yaw (psi)
        siny_cosp = 2.0 * (qw * qz + qx * qy)
        cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        return roll, pitch, yaw

    # ========================================================
    # POSE CALLBACK
    # ========================================================
    def pose_callback(self, msg):

        # Lay thoi gian mo phong tu Gazebo header
        stamp = msg.header.stamp
        self.sim_time = float(stamp.sec) + float(stamp.nanosec) * 1.0e-9

        self.x = msg.pose.position.x
        self.y = msg.pose.position.y
        self.z = msg.pose.position.z

        q = msg.pose.orientation
        (self.phi, self.theta, self.psi) = self.quaternion_to_euler(
            q.x, q.y, q.z, q.w
        )

        # --- Ghi actual path (tu khi CLIMBING den het WAIT_DISARM) ---
        if self.CLIMBING <= self.mission_state <= self.WAIT_DISARM:
            pose = PoseStamped()
            pose.header = msg.header
            pose.header.frame_id = 'map'
            pose.pose = msg.pose
            self.actual_path.poses.append(pose)
            self.actual_path.header.stamp = msg.header.stamp
            self.actual_path_pub.publish(self.actual_path)

    # ========================================================
    # VELOCITY CALLBACK
    # ========================================================
    def velocity_callback(self, msg):
        self.vx = msg.twist.linear.x
        self.vy = msg.twist.linear.y
        self.vz = msg.twist.linear.z

    # ========================================================
    # IMU CALLBACK
    # ========================================================
    def imu_callback(self, msg):
        self.p = msg.angular_velocity.x
        self.q = msg.angular_velocity.y
        self.r = msg.angular_velocity.z

    # ========================================================
    # MAVROS STATE CALLBACK
    # ========================================================
    def state_callback(self, msg):
        self.connected = msg.connected
        self.armed     = msg.armed
        self.mode      = msg.mode

    # ========================================================
    # PUBLISH ACTUATOR CONTROL
    # ========================================================
    def publish_actuator_control(self, controls):

        msg = ActuatorControl()
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.header.frame_id = ''
        msg.controls = [
            float(controls[0]),
            float(controls[1]),
            float(controls[2]),
            float(controls[3]),
            0.0,
            0.0,
            0.0,
            0.0
        ]
        msg.group_mix = 0
        self.actuator_pub.publish(msg)

    # ========================================================
    # ADD PLANNED WAYPOINT
    # ========================================================
    def add_planned(self, x, y, z):
        pose = PoseStamped()
        pose.header.stamp    = self.get_clock().now().to_msg()
        pose.header.frame_id = 'map'
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = z
        pose.pose.orientation.w = 1.0
        self.planned_path.poses.append(pose)
        self.planned_path.header.stamp = pose.header.stamp
        self.planned_path_pub.publish(self.planned_path)

    # ========================================================
    # SAVE PATHS TO CSV
    # ========================================================
    def save_paths_to_csv(self):
        home = os.path.expanduser('~')
        ts   = time.strftime('%Y%m%d_%H%M%S')

        planned_file = os.path.join(home, f'planned_path_{ts}.csv')
        with open(planned_file, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['time_sec', 'time_nsec', 'x', 'y', 'z'])
            for pose in self.planned_path.poses:
                writer.writerow([
                    pose.header.stamp.sec,
                    pose.header.stamp.nanosec,
                    pose.pose.position.x,
                    pose.pose.position.y,
                    pose.pose.position.z
                ])
        self.get_logger().info(
            f'Planned path saved: {planned_file} ({len(self.planned_path.poses)} points)'
        )

        actual_file = os.path.join(home, f'actual_path_{ts}.csv')
        with open(actual_file, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['time_sec', 'time_nsec', 'x', 'y', 'z'])
            for pose in self.actual_path.poses:
                writer.writerow([
                    pose.header.stamp.sec,
                    pose.header.stamp.nanosec,
                    pose.pose.position.x,
                    pose.pose.position.y,
                    pose.pose.position.z
                ])
        self.get_logger().info(
            f'Actual path saved: {actual_file} ({len(self.actual_path.poses)} points)'
        )

    # ========================================================
    # RUN PDSMC WITH GIVEN REFERENCE TRAJECTORY
    # ========================================================
    def _run_pdsmc(
        self,
        xd, xd_dot, xd_ddot,
        yd, yd_dot, yd_ddot,
        zd, zd_dot, zd_ddot
    ):
        """Tinh PDSMC, chuyen doi sang omega, publish ActuatorControl."""
        (
            U1, U2, U3, U4,
            satU1, satU2, satU3, satU4,
            phi_d, theta_d
        ) = self.controller.compute(
            self.sim_time,
            self.x, self.vx,
            self.y, self.vy,
            self.z, self.vz,
            self.phi, self.p,
            self.theta, self.q,
            self.psi, self.r,
            self.DT,
            xd, xd_dot, xd_ddot,
            yd, yd_dot, yd_ddot,
            zd, zd_dot, zd_ddot,
        )

        omega1, omega2, omega3, omega4 = self.controller.U_to_omega(
            satU1, satU2, satU3, satU4
        )

        controls = self.controller.omega_to_control(
            omega1, omega2, omega3, omega4
        )

        self.publish_actuator_control(controls)

        # Debug log (~moi 1 giay)
        if int(self.sim_time * 10) % 10 == 0:
            self.get_logger().info(
                'T=%.2f | state=%d | '
                'pos=[%.2f %.2f %.2f] | ref=[%.2f %.2f %.2f] | '
                'satU=[%.3f %.3f %.3f %.3f]' %
                (
                    self.sim_time, self.mission_state,
                    self.x, self.y, self.z,
                    xd, yd, zd,
                    satU1, satU2, satU3, satU4,
                )
            )

        return satU1, satU2, satU3, satU4, omega1, omega2, omega3, omega4

    # ========================================================
    # CONTROL LOOP (50 Hz) — MISSION STATE MACHINE
    # ========================================================
    def control_loop(self):

        # --- WAIT_CONN: cho MAVROS ket noi ---
        if self.mission_state == self.WAIT_CONN:
            if self.connected:
                self.get_logger().info('MAVROS connected')
                # LOCAL_POSITION_NED (#32) at 20 Hz
                self.set_stream_rate(32, 20)
                self.mission_state = self.SEND_GUIDED
            else:
                self.publish_actuator_control(
                    [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
                )
            return

        # --- SEND_GUIDED ---
        if self.mission_state == self.SEND_GUIDED:
            req = SetMode.Request()
            req.custom_mode = 'GUIDED'
            self.pending_future = self.set_mode_client.call_async(req)
            self.get_logger().info('Switching to GUIDED mode...')
            self.mission_state = self.WAIT_GUIDED
            self.publish_actuator_control(
                [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
            )
            return

        # --- WAIT_GUIDED ---
        if self.mission_state == self.WAIT_GUIDED:
            self.publish_actuator_control(
                [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
            )
            if not self.pending_future.done():
                return
            resp = self.pending_future.result()
            if resp and resp.mode_sent:
                self.get_logger().info('GUIDED mode accepted')
                self.mission_state = self.SEND_ARM
            else:
                self.get_logger().warn('GUIDED mode rejected, retrying...')
                self.mission_state = self.SEND_GUIDED
            return

        # --- SEND_ARM ---
        if self.mission_state == self.SEND_ARM:
            req = CommandBool.Request()
            req.value = True
            self.pending_future = self.arming_client.call_async(req)
            self.get_logger().info('Arming vehicle...')
            self.mission_state = self.WAIT_ARM
            self.publish_actuator_control(
                [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
            )
            return

        # --- WAIT_ARM ---
        if self.mission_state == self.WAIT_ARM:
            self.publish_actuator_control(
                [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
            )
            if not self.pending_future.done():
                return
            resp = self.pending_future.result()
            if resp and resp.success:
                self.get_logger().info(
                    'Arming accepted, waiting for FCU arm confirmation + stabilisation...'
                )
                self.arm_time = None
                self.mission_state = self.WAIT_STABLE
            else:
                self.get_logger().warn('Arming rejected, retrying...')
                self.mission_state = self.SEND_ARM
            return

        # --- WAIT_STABLE: cho FCU xac nhan armed + delay on dinh ---
        if self.mission_state == self.WAIT_STABLE:
            self.publish_actuator_control(
                [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
            )
            if self.armed:
                if self.arm_time is None:
                    self.arm_time = self.get_clock().now()
                    self.get_logger().info(
                        f'FCU armed confirmed, stabilising for {self.ARM_STABLE_SEC:.1f}s...'
                    )
                elapsed = (self.get_clock().now() - self.arm_time).nanoseconds / 1e9
                if elapsed >= self.ARM_STABLE_SEC:
                    self.get_logger().info('Stabilisation done, proceeding to takeoff')
                    self.mission_state = self.SEND_TAKEOFF
            else:
                # FCU chua armed — reset timer
                self.arm_time = None
            return

        # --- SEND_TAKEOFF ---
        if self.mission_state == self.SEND_TAKEOFF:
            req = CommandLong.Request()
            req.command = 22        # MAV_CMD_NAV_TAKEOFF
            req.param1  = 0.0
            req.param2  = 0.0
            req.param3  = 0.0
            req.param4  = float('nan')
            req.param5  = 0.0
            req.param6  = 0.0
            req.param7  = 3.0       # do cao cat canh [m]
            self.pending_future = self.takeoff_client.call_async(req)
            self.get_logger().info('Sending takeoff command...')
            self.mission_state = self.WAIT_TAKEOFF
            self.publish_actuator_control(
                [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
            )
            return

        # --- WAIT_TAKEOFF ---
        if self.mission_state == self.WAIT_TAKEOFF:
            self.publish_actuator_control(
                [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
            )
            if not self.pending_future.done():
                return
            resp = self.pending_future.result()
            if resp and resp.success:
                self.takeoff_x = self.x
                self.takeoff_y = self.y
                self.takeoff_z = self.z
                self.target_z  = self.takeoff_z + 3.0
                self.get_logger().info(
                    f'Takeoff accepted, climbing to {self.target_z:.1f} m...'
                )
                # Ghi planned path: diem leo cao
                self.add_planned(self.takeoff_x, self.takeoff_y, self.target_z)
                self.mission_state = self.CLIMBING
            else:
                self.get_logger().warn('Takeoff rejected, retrying...')
                self.mission_state = self.SEND_TAKEOFF
            return

        # --- CLIMBING: PDSMC giu vi tri cat canh, leo den target_z ---
        # ArduPilot da xu ly MAV_CMD_NAV_TAKEOFF; PDSMC bam vi tri ngang
        # va leo theo thoi gian thuc khi altitude thay doi
        if self.mission_state == self.CLIMBING:
            # Quy dao tham chieu: giu vi tri cat canh, target_z
            xd = self.takeoff_x
            yd = self.takeoff_y
            zd = self.target_z
            self._run_pdsmc(
                xd, 0.0, 0.0,
                yd, 0.0, 0.0,
                zd, 0.0, 0.0
            )
            if self.z >= self.target_z - 0.1:
                self.x0 = self.x
                self.y0 = self.y
                self.get_logger().info(
                    f'Altitude reached ({self.z:.2f} m), '
                    f'center=({self.x0:.2f}, {self.y0:.2f}), '
                    f'flying to circle start...'
                )
                self.mission_state = self.FLY_TO_START
            return

        # --- FLY_TO_START: bay den (x0+R, y0) ---
        if self.mission_state == self.FLY_TO_START:
            tx = self.x0 + self.radius
            ty = self.y0
            dx = tx - self.x
            dy = ty - self.y
            dist = math.sqrt(dx * dx + dy * dy)

            # Dat xd/yd mac tieu, de PDSMC tu tinh toc do
            xd = tx
            yd = ty
            zd = self.target_z
            self._run_pdsmc(
                xd, 0.0, 0.0,
                yd, 0.0, 0.0,
                zd, 0.0, 0.0
            )
            self.add_planned(tx, ty, self.target_z)

            if dist < 0.3:
                self.circle_start_time = self.sim_time
                self.get_logger().info('Reached circle start, beginning circle')
                self.mission_state = self.CIRCLE
            return

        # --- CIRCLE: PDSMC bam quy dao vong tron ---
        if self.mission_state == self.CIRCLE:
            t     = self.sim_time - self.circle_start_time
            angle = self.w * t

            if angle >= 2.0 * math.pi:
                # Het vong tron — planned landing trajectory
                steps = int(self.target_z / 0.1)
                for z_step in [self.target_z - i * 0.1 for i in range(steps)]:
                    self.add_planned(self.x0, self.y0, z_step)
                self.add_planned(self.x0, self.y0, self.target_z)
                self.get_logger().info('Circle completed, flying back to center...')
                # Dung PDSMC tai vi tri hien tai trong khi chuyen trang thai
                self._run_pdsmc(
                    self.x, 0.0, 0.0,
                    self.y, 0.0, 0.0,
                    self.target_z, 0.0, 0.0
                )
                self.mission_state = self.FLY_TO_CENTER
                return

            # Quy dao vong tron voi dao ham chinh xac
            xd      = self.x0 + self.radius * math.cos(angle)
            yd      = self.y0 + self.radius * math.sin(angle)
            zd      = self.target_z

            xd_dot  = -self.radius * self.w * math.sin(angle)
            yd_dot  =  self.radius * self.w * math.cos(angle)
            zd_dot  = 0.0

            xd_ddot = -self.radius * self.w * self.w * math.cos(angle)
            yd_ddot = -self.radius * self.w * self.w * math.sin(angle)
            zd_ddot = 0.0

            self._run_pdsmc(
                xd, xd_dot, xd_ddot,
                yd, yd_dot, yd_ddot,
                zd, zd_dot, zd_ddot
            )
            self.add_planned(xd, yd, zd)
            return

        # --- FLY_TO_CENTER: ve tam vong tron truoc khi ha canh ---
        if self.mission_state == self.FLY_TO_CENTER:
            dx   = self.x0 - self.x
            dy   = self.y0 - self.y
            dist = math.sqrt(dx * dx + dy * dy)

            xd = self.x0
            yd = self.y0
            zd = self.target_z
            self._run_pdsmc(
                xd, 0.0, 0.0,
                yd, 0.0, 0.0,
                zd, 0.0, 0.0
            )

            if dist < 0.3:
                self.get_logger().info('Reached center, landing...')
                self.mission_state = self.SEND_LAND
            return

        # --- SEND_LAND ---
        if self.mission_state == self.SEND_LAND:
            req = SetMode.Request()
            req.custom_mode = 'LAND'
            self.pending_future = self.set_mode_client.call_async(req)
            self.get_logger().info('Sending LAND command...')
            self.mission_state = self.WAIT_LAND
            self.publish_actuator_control(
                [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
            )
            return

        # --- WAIT_LAND ---
        if self.mission_state == self.WAIT_LAND:
            self.publish_actuator_control(
                [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
            )
            if not self.pending_future.done():
                return
            resp = self.pending_future.result()
            if resp and resp.mode_sent:
                self.get_logger().info('LAND mode accepted, waiting for drone to land...')
                self.mission_state = self.WAIT_DISARM
            else:
                self.get_logger().warn('LAND mode rejected, retrying...')
                self.mission_state = self.SEND_LAND
            return

        # --- WAIT_DISARM: cho den khi ha canh xong va disarm ---
        if self.mission_state == self.WAIT_DISARM:
            self.publish_actuator_control(
                [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
            )
            if not self.armed:
                self.get_logger().info('Drone disarmed, landing complete.')
                if self.save_log:
                    self.get_logger().info('Saving paths...')
                    self.save_paths_to_csv()
                self.mission_state = self.DONE
            return

        # --- DONE ---
        if self.mission_state == self.DONE:
            self.publish_actuator_control(
                [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
            )
            return


# ============================================================
# MAIN
# ============================================================

def main(args=None):

    rclpy.init(args=args)

    node = PDSMCGazeboNode()

    try:

        rclpy.spin(node)

    except KeyboardInterrupt:

        pass

    finally:

        # Gui lenh zero khi tat de an toan
        try:
            msg = ActuatorControl()
            msg.header.stamp    = node.get_clock().now().to_msg()
            msg.header.frame_id = ''
            msg.controls = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
            msg.group_mix = 0
            node.actuator_pub.publish(msg)
        except Exception:
            pass

        # Luu CSV khi tat (neu save_log = True va chua luu)
        if node.save_log and node.mission_state != node.DONE:
            try:
                node.get_logger().info('Saving trajectory logs...')
                node.save_paths_to_csv()
            except Exception as e:
                node.get_logger().error(f'Failed to save logs: {e}')

        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
