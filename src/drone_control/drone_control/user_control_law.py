# Copyright 2026 — mẫu port BĐK từ MATLAB sang Python (NumPy).
# Thay nội dung trong UserControlLaw bằng ma trận / hàm của bạn.

from __future__ import annotations

import numpy as np


class UserControlLaw:
    """
    Gọi lớp này mỗi bước lấy mẫu Ts (giống for-loop trong MATLAB).

    Cách port nhanh từ MATLAB
    ---------------------------
    1) Mô hình liên tục x_dot = A*x + B*u → trong MATLAB dùng c2d(A,B,Ts)
       → Ad, Bd. Trong Python:
           from scipy.signal import cont2discrete
           Ad, Bd, _, _, _ = cont2discrete((A, B, np.eye(n), np.zeros((n,m))), Ts, method='zoh')
       (Nếu không có scipy, bạn có thể dán sẵn Ad, Bd tính từ MATLAB.)

    2) Luật u = -K*x  (hoặc u = Kr*r - K*x) → copy K vào self.K.

    3) Nếu MATLAB của bạn ra *gia tốc* hay *góc nghiêng*, cần bản đồ sang
       vận tộc setpoint cho MAVROS (ví dụ tích phân nhỏ u_a → v_cmd), hoặc
       đổi sang topic setpoint khác của MAVROS — hiện mẫu này xuất **vận tộc**.

    4) Vector trạng thái x phải cùng thứ tự trong MATLAB và Python.

    Mặc định bên dưới: PD khớp kênh dọc độc lập cho (x,y) → vận tộc yêu cầu.
    """

    def __init__(self, Ts: float) -> None:
        self.Ts = float(Ts)
        # --- PD mặc định (thay bằng K của bạn nếu cần) ---
        self.Kp_xy = np.array([0.75, 0.75])
        self.Kd_xy = np.array([0.55, 0.55])

        # --- Ví dụ state-feedback: u = -K @ x (2 output: vx, vy), x dim 4 ---
        # self.use_state_feedback = True
        # self.K = np.array([...])  # paste từ MATLAB, shape (2, 4)

        self.use_state_feedback = False
        self.K: np.ndarray | None = None

        # Tham chiếu quỹ đạo (độ) — có thể chỉnh từ node ROS qua setter
        self.ref_x = 0.0
        self.ref_y = 0.0

    def set_reference(self, ref_x: float, ref_y: float) -> None:
        self.ref_x = float(ref_x)
        self.ref_y = float(ref_y)

    def reset(self) -> None:
        """Gọi khi bắt đầu phase điều khiển."""
        pass

    def compute_velocity_setpoint_xy(
        self,
        x: float,
        y: float,
        vx: float,
        vy: float,
        t: float,
    ) -> tuple[float, float]:
        """
        Trả về (vx_cmd, vy_cmd) trong frame đồng nhất với /mavros/local_position/pose
        (thường dùng ENU ngang cho xy).

        t: thời gian từ lúc bắt đầu điều khiển [s] — dùng cho tham chiếu phụ thuộc thời gian.
        """
        if self.use_state_feedback and self.K is not None:
            ex = x - self.ref_x
            ey = y - self.ref_y
            state = np.array([ex, ey, vx, vy], dtype=float)
            u = -self.K @ state
            return float(u[0]), float(u[1])

        ex = self.ref_x - x
        ey = self.ref_y - y
        vx_cmd = self.Kp_xy[0] * ex - self.Kd_xy[0] * vx
        vy_cmd = self.Kp_xy[1] * ey - self.Kd_xy[1] * vy
        return float(vx_cmd), float(vy_cmd)


def example_circle_reference(center_x: float, center_y: float, R: float, omega: float, t: float):
    """Tham chiếu điểm trên vòng tròn (đạo hàm không dùng ở đây; chỉ vị trí mục tiêu)."""
    cx = center_x + R * np.cos(omega * t)
    cy = center_y + R * np.sin(omega * t)
    return float(cx), float(cy)
