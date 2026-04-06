# X500 Drone Flip Bug — Root Cause Analysis & Fix

## Triệu chứng

Drone x500 bị **flip (lật)** ngay sau khi takeoff trong Gazebo SITL simulation.
Hướng flip thay đổi ngẫu nhiên (trái/phải) giữa các lần chạy.

## Bối cảnh

- Model x500 được port từ PX4 Gazebo sang ArduPilot SITL
- PX4 x500 dùng plugin `MulticopterMotorModel` (thrust = motorConstant × ω²)
- ArduPilot iris dùng plugin `LiftDrag` + `ArduPilotPlugin` PID controller
- X500 được cấu hình theo pattern của iris nhưng với thông số vật lý khác

## Root Cause: IMU Sensor Frame Convention

**Nguyên nhân gốc (gây flip):**

ArduPilotPlugin gửi dữ liệu IMU (accel, gyro) **trực tiếp** từ Gazebo sensor đến
ArduPilot SITL mà không transform frame.

ArduPilot expects body frame **z-DOWN** (aircraft convention), nhưng Gazebo sensor
mặc định output theo frame **z-UP**.

| | Iris (hoạt động đúng) | X500 (bị flip) |
|---|---|---|
| IMU sensor pose | `<pose degrees="true">0 0 0 180 0 0</pose>` | **Không có** |
| Accel z khi đứng yên | -9.81 (z-down ✓) | +9.81 (z-up ✗) |
| ArduPilot hiểu | Drone đứng thẳng | **Drone lộn ngược!** |
| Phản ứng | Hover ổn định | **Lật ngay lập tức** |

**Fix:** Thêm `<pose degrees="true">0 0 0 180 0 0</pose>` vào IMU sensor (xoay 180° quanh X).

## Các Fix Phụ (ổn định bay)

### 1. PID p_gain: 0.20 → 0.0088

X500 rotor Izz = 2.65×10⁻⁵ (nhỏ hơn iris 6.3×), stability limit = 2×Izz/dt = 0.053.
p_gain=0.20 (của iris) vượt giới hạn → rotor oscillation.

### 2. Joint damping: 0.004 → 0.0001

Với P-only controller (i_gain=0), damping tạo steady-state error:
- damping=0.004 + p_gain=0.0088 → motor chỉ đạt 69% RPM target → thiếu lực nâng
- damping=0.0001 → motor đạt ~99% RPM target

### 3. Attitude Rate PID Gains (ArduPilot params)

X500 body inertia nhỏ hơn iris → nhạy hơn với torque:
- Roll/Pitch: 2.09× nhạy hơn → scale 0.479×
- Yaw: 3.93× nhạy hơn → scale 0.254×

```
ATC_RAT_RLL_P    0.065    (iris 0.135 × 0.479)
ATC_RAT_RLL_I    0.065
ATC_RAT_RLL_D    0.0015   (iris 0.003 × 0.479)
ATC_RAT_PIT_P    0.065
ATC_RAT_PIT_I    0.065
ATC_RAT_PIT_D    0.0015
ATC_RAT_YAW_P    0.046    (iris 0.18 × 0.254)
ATC_RAT_YAW_I    0.0046
MOT_THST_HOVER   0.48
```

### 4. Compass Enable: COMPASS_USE 0 → 1

Cần compass cho EKF heading reference khi hover tại chỗ.

### 5. ARMING_CHECK=0, ApplyJointForce removed

- ARMING_CHECK=0: bỏ qua pre-arm checks cho SITL
- ApplyJointForce plugins xung đột với ArduPilotPlugin trong Dart physics

## Files Modified

| File | Changes |
|---|---|
| `src/ardupilot_gazebo/models/x500/model_standalone.sdf` | IMU pose 180°, damping 0.0001, p_gain 0.0088, removed ApplyJointForce |
| `src/ardupilot_gazebo/config/gazebo-x500.parm` | Scaled PID gains, COMPASS_USE=1, ARMING_CHECK=0 |

## So sánh X500 vs Iris vs F450 thật

| | Iris (sim) | X500 (sim) | F450 (thật) |
|---|---|---|---|
| Thrust model | LiftDrag | LiftDrag | Aerodynamic thật |
| Motor control | PID → JointForce | PID → JointForce | ESC closed-loop |
| Joint damping | 0.004 | 0.0001 | N/A |
| IMU frame | Sensor pose Rx(180°) | Sensor pose Rx(180°) | Hardware mounting |
| Flip bug | Không | **Có** (trước fix) | Không bao giờ |

**F450 thật không bị flip vì:** ESC có closed-loop speed control, propeller tạo
thrust theo aerodynamic thực tế, và IMU hardware output đã đúng convention.
Bug flip chỉ tồn tại trong simulation do cấu hình sensor frame sai.
