
import math
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from geometry_msgs.msg import PoseStamped, TwistStamped
from sensor_msgs.msg import Imu
from mavros_msgs.msg import State, ActuatorControl
from mavros_msgs.srv import CommandBool, SetMode


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
        dt
    ):

        # ====================================================
        # DESIRED TRAJECTORY
        #
        # Khop hoan toan voi MATLAB:
        #
        # xd  = [0.5*sin(1.5t);  0.75*cos(1.5t);   % x, xdot
        #         0.5*cos(1.5t); -0.75*sin(1.5t);   % y, ydot
        #         0.5+0.5*sin(1.5t); 0.75*cos(1.5t); % z, zdot
        #         0; 0; 0; 0; 0; 0]
        #
        # xdd = [0.75*cos;  -0.375*sin;   % xdd(1,2)
        #        -0.75*sin; -0.375*cos;   % xdd(3,4)
        #         0.75*cos; -0.375*sin;   % xdd(5,6)
        #         0; 0; 0; 0; 0; 0]
        #
        # psi_d = 0
        # ====================================================

        xd = 0.5 * math.sin(1.5 * t)            # xd(1)
        yd = 0.5 * math.cos(1.5 * t)            # xd(3)  FIX: 0.5 khop MATLAB
        zd = 0.5 + 0.5 * math.sin(1.5 * t)      # xd(5)

        # First derivatives — xd(2), xd(4), xd(6)
        xd_dot = 0.75 * math.cos(1.5 * t)
        yd_dot = -0.75 * math.sin(1.5 * t)      # d/dt[0.5*cos(1.5t)]
        zd_dot = 0.75 * math.cos(1.5 * t)

        # Second derivatives — xdd(2), xdd(4), xdd(6) tu MATLAB
        xd_ddot = -0.375 * math.sin(1.5 * t)    # MATLAB xdd(2)
        yd_ddot = -0.375 * math.cos(1.5 * t)    # MATLAB xdd(4)
        zd_ddot = -0.375 * math.sin(1.5 * t)    # MATLAB xdd(6)

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
        # STARTUP STATE MACHINE
        #
        # PX4 yeu cau setpoint stream phai hoat dong TRUOC
        # khi chuyen sang OFFBOARD mode.
        # Stream lenh zero trong STARTUP_CYCLES chu ky truoc.
        # ====================================================
        self._startup_counter    = 0
        self._STARTUP_CYCLES     = 10    # 10 * 0.02s = 0.2 giay (ArduPilot khong yeu cau pre-stream dai)
        self._offboard_requested = False
        self._arm_requested      = False

        # ====================================================
        # CONTROLLER
        # ====================================================
        self.controller = PDSMCController()

        # ====================================================
        # QoS
        # ====================================================
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # ====================================================
        # SUBSCRIBERS
        # ====================================================

        self.pose_sub = self.create_subscription(
            PoseStamped,
            '/mavros/local_position/pose',
            self.pose_callback,
            qos
        )

        self.velocity_sub = self.create_subscription(
            TwistStamped,
            '/mavros/local_position/velocity_local',
            self.velocity_callback,
            qos
        )

        self.imu_sub = self.create_subscription(
            Imu,
            '/mavros/imu/data',
            self.imu_callback,
            qos
        )

        self.state_sub = self.create_subscription(
            State,
            '/mavros/state',
            self.state_callback,
            qos
        )

        # ====================================================
        # ACTUATOR CONTROL PUBLISHER
        # ====================================================

        self.actuator_pub = self.create_publisher(
            ActuatorControl,
            '/mavros/actuator_control',
            10
        )

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

        # ====================================================
        # CONTROL TIMER (50 Hz)
        # ====================================================

        self.timer = self.create_timer(
            self.DT,
            self.control_loop
        )

        self.get_logger().info(
            'Improved PDSMC controller started. '
            'Waiting for ArduPilot/MAVROS connection...'
        )

    # ========================================================
    # REQUEST OFFBOARD MODE
    # ========================================================
    def _request_offboard(self):

        if not self.set_mode_client.service_is_ready():
            self.get_logger().warn(
                'SetMode service not ready, retrying...'
            )
            return

        req = SetMode.Request()
        req.base_mode   = 0
        req.custom_mode = 'GUIDED'

        future = self.set_mode_client.call_async(req)
        future.add_done_callback(self._offboard_response_cb)

        self._offboard_requested = True
        self.get_logger().info('Requesting OFFBOARD mode...')

    def _offboard_response_cb(self, future):
        try:
            resp = future.result()
            if resp.mode_sent:
                self.get_logger().info('OFFBOARD mode set successfully.')
            else:
                self.get_logger().warn(
                    'OFFBOARD mode request denied. Will retry.'
                )
                self._offboard_requested = False
        except Exception as e:
            self.get_logger().error(
                f'SetMode service call failed: {e}'
            )
            self._offboard_requested = False

    # ========================================================
    # REQUEST ARM
    # ========================================================
    def _request_arm(self):

        if not self.arming_client.service_is_ready():
            self.get_logger().warn(
                'Arming service not ready, retrying...'
            )
            return

        req = CommandBool.Request()
        req.value = True

        future = self.arming_client.call_async(req)
        future.add_done_callback(self._arm_response_cb)

        self._arm_requested = True
        self.get_logger().info('Requesting arm...')

    def _arm_response_cb(self, future):
        try:
            resp = future.result()
            if resp.success:
                self.get_logger().info('Vehicle armed successfully.')
            else:
                self.get_logger().warn(
                    'Arming request denied. Will retry.'
                )
                self._arm_requested = False
        except Exception as e:
            self.get_logger().error(
                f'Arming service call failed: {e}'
            )
            self._arm_requested = False

    # ========================================================
    # QUATERNION -> EULER (ZYX convention)
    # ========================================================
    def quaternion_to_euler(
        self,
        qx,
        qy,
        qz,
        qw
    ):

        # Roll (phi)
        sinr_cosp = (
            2.0 * (
                qw * qx
                + qy * qz
            )
        )

        cosr_cosp = (
            1.0
            - 2.0 * (
                qx * qx
                + qy * qy
            )
        )

        roll = math.atan2(sinr_cosp, cosr_cosp)

        # Pitch (theta)
        sinp = (
            2.0 * (
                qw * qy
                - qz * qx
            )
        )

        if abs(sinp) >= 1.0:
            pitch = math.copysign(
                math.pi / 2.0,
                sinp
            )
        else:
            pitch = math.asin(sinp)

        # Yaw (psi)
        siny_cosp = (
            2.0 * (
                qw * qz
                + qx * qy
            )
        )

        cosy_cosp = (
            1.0
            - 2.0 * (
                qy * qy
                + qz * qz
            )
        )

        yaw = math.atan2(siny_cosp, cosy_cosp)

        return roll, pitch, yaw

    # ========================================================
    # POSE CALLBACK
    # Dong bo sim_time voi Gazebo clock qua header.stamp
    # ========================================================
    def pose_callback(self, msg):

        # Lay thoi gian mo phong tu Gazebo header
        stamp = msg.header.stamp
        self.sim_time = float(stamp.sec) + float(stamp.nanosec) * 1.0e-9

        self.x = msg.pose.position.x
        self.y = msg.pose.position.y
        self.z = msg.pose.position.z

        q = msg.pose.orientation

        (
            self.phi,
            self.theta,
            self.psi
        ) = self.quaternion_to_euler(
            q.x,
            q.y,
            q.z,
            q.w
        )

    # ========================================================
    # VELOCITY CALLBACK
    # ========================================================
    def velocity_callback(self, msg):

        self.vx = msg.twist.linear.x
        self.vy = msg.twist.linear.y
        self.vz = msg.twist.linear.z

    # ========================================================
    # IMU CALLBACK
    # Angular velocity dung lam p, q, r trong bo dieu khien
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
    # header.stamp bat buoc de PX4/MAVROS chap nhan lenh
    # ========================================================
    def publish_actuator_control(self, controls):

        msg = ActuatorControl()

        # Timestamp - PX4 kiem tra timeout neu khong co stamp
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.header.frame_id = ''

        # Dung 8 phan tu: 4 rotor + 4 zero
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
    # CONTROL LOOP (50 Hz)
    # ========================================================
    def control_loop(self):

        # ----------------------------------------------------
        # Cho MAVROS ket noi voi FCU
        # ----------------------------------------------------
        if not self.connected:
            return

        # ----------------------------------------------------
        # STARTUP PHASE
        # Stream lenh zero truoc de kich hoat setpoint stream
        # TRUOC khi chuyen sang OFFBOARD (yeu cau cua PX4)
        # ----------------------------------------------------
        if self._startup_counter < self._STARTUP_CYCLES:
            self.publish_actuator_control(
                [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
            )
            self._startup_counter += 1

            if self._startup_counter == self._STARTUP_CYCLES:
                self.get_logger().info(
                    'Startup complete. Requesting OFFBOARD mode...'
                )
            return

        # ----------------------------------------------------
        # Yeu cau chuyen sang OFFBOARD (mot lan)
        # ----------------------------------------------------
        if self.mode != 'GUIDED' and not self._offboard_requested:
            self._request_offboard()
            self.publish_actuator_control(
                [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
            )
            return

        # ----------------------------------------------------
        # Yeu cau ARM (sau khi da vao OFFBOARD)
        # ----------------------------------------------------
        if (
            self.mode == 'GUIDED'
            and not self.armed
            and not self._arm_requested
        ):
            self._request_arm()

        # ----------------------------------------------------
        # Cho du dieu kien: OFFBOARD + ARMED
        # ----------------------------------------------------
        if not self.armed or self.mode != 'GUIDED':
            self.publish_actuator_control(
                [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
            )
            return

        # ----------------------------------------------------
        # PDSMC CONTROLLER
        # ----------------------------------------------------

        (
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
        ) = self.controller.compute(

            self.sim_time,

            self.x,
            self.vx,

            self.y,
            self.vy,

            self.z,
            self.vz,

            self.phi,
            self.p,

            self.theta,
            self.q,

            self.psi,
            self.r,

            self.DT
        )

        # ----------------------------------------------------
        # Dung satU (khong phai U tho) cho phan phoi rotor
        # ----------------------------------------------------

        omega = self.controller.U_to_omega(
            satU1,
            satU2,
            satU3,
            satU4
        )

        omega1, omega2, omega3, omega4 = omega

        # ----------------------------------------------------
        # Chuan hoa omega -> controls[8]
        # ----------------------------------------------------

        controls = self.controller.omega_to_control(
            omega1,
            omega2,
            omega3,
            omega4
        )

        # ----------------------------------------------------
        # Gui ACTUATOR_CONTROL_TARGET
        # ----------------------------------------------------

        self.publish_actuator_control(controls)

        # ----------------------------------------------------
        # Debug log (~moi 1 giay)
        # ----------------------------------------------------

        if int(self.sim_time * 10) % 10 == 0:

            self.get_logger().info(
                'T=%.2f | '
                'pos=[%.3f %.3f %.3f] | '
                'att=[%.3f %.3f %.3f] | '
                'satU=[%.3f %.3f %.3f %.3f] | '
                'omega=[%.1f %.1f %.1f %.1f]' %
                (
                    self.sim_time,

                    self.x, self.y, self.z,

                    self.phi, self.theta, self.psi,

                    satU1, satU2, satU3, satU4,

                    omega1, omega2, omega3, omega4
                )
            )


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

        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
