# Flow architecture: `uwb_gz_plugin` → ArduPilot

Tài liệu này mô tả luồng dữ liệu đầy đủ từ Gazebo UWB plugin tới EKF3 của
ArduPilot SITL, gồm các topic, frame, đơn vị và các bước xử lý trung gian.

> **Thay đổi so với bản cũ:** vị trí được gửi sang ArduPilot bằng MAVLink
> `ODOMETRY` (id 331) thay vì protocol Pozyx (`AP_Beacon`). Toàn bộ tài liệu
> dưới đây mô tả pipeline mới. Xem `UWB_POZYX_PIPELINE.md` cho lịch sử
> pipeline cũ.

## 1. Tổng quan

```
┌────────────────────────────────────────────────────────────────────────────┐
│ Gazebo — uwb_runway.sdf                                                    │
    <plugin filename="uwb_gz_plugin" name="uwb_gz_plugin::systems::UwbSystem">│
│     update_rate = 20 Hz, tag_model_name = x500, tag_link = base_link      │
│     anchors (ENU, m): (0,0,2) (0,10,2) (10,10,2) (10,0,2)                │
│     (sinh động theo layout: X×Y, xoay α)                                  │
└───────────────────────────────┬────────────────────────────────────────────┘
                                │ PostUpdate() mỗi 1/20 s
                                │ tìm tag link → tính range tới 4 anchor
                                ▼
        gz-transport topic: /uwb/ranging/tag   (gz::msgs::Pose, 1 msg/anchor)
        gz-transport topic: /uwb/tag_pose/tag  (gz::msgs::Pose, vị trí tag)
        gz-transport topic: /uwb/anchors/tag   (gz::msgs::Marker, màu theo LOS)
                                │
                                ▼
┌────────────────────────────────────────────────────────────────────────────┐
│ uwb_gz_to_ros_bridge  (C++, node trong package uwb_gz_plugin)             │
│   Subscribe gz → Publish ROS 2                                            │
│   /uwb/ranging/tag    →  /uwb/ranging_ros   (geometry_msgs/Pose)          │
│   /uwb/tag_pose/tag   →  /uwb/tag_pose_ros  (geometry_msgs/Pose)          │
│   QoS: best_effort / volatile, depth 10                                   │
└───────────────────────────────┬────────────────────────────────────────────┘
                                │
                                ▼
┌────────────────────────────────────────────────────────────────────────────┐
│ odometry_node  (Python, package uwb_pozyx_bridge)                          │
│                                                                             │
│  1. decode_ranging_pose()  : Pose → RangeMeasurement                       │
│  2. solve_position()       : WLS 3D chọn nhánh → vị trí tag (Anchor Frame) │
│  3. AnchorLayout.to_ned_3d(): xoay α + dịch z → NED (tương đối EKF Origin)│
│  4. build_odometry_message(): MAVLink ODOMETRY (id 331)                    │
│  5. MavlinkSender.send()   : ghi MAVLink frame vào TCP 127.0.0.1:5761      │
│                                                                             │
│     Timer 20 Hz (send_rate)                                                 │
│  Publish thêm: /uwb/pose  (PoseStamped, frame "world") để debug           │
└───────────────────────────────┬────────────────────────────────────────────┘
                                │ MAVLink v2 frame
                                │ [0xFD][len][seq][sysid][compid][msgid=331][payload][crc]
                                ▼
┌────────────────────────────────────────────────────────────────────────────┐
│ ArduPilot SITL                                                            │
│   arducopter --serial1=tcp:5761:wait                                       │
│   SERIAL1_PROTOCOL = 1  (SerialProtocol_MAVLink)                           │
│   SERIAL1_BAUD = 115, BRD_SER1_RTSCTS = 0                                  │
└───────────────────────────────┬────────────────────────────────────────────┘
                                │ GCS_MAVLINK::handle_odometry()
                                ▼
┌────────────────────────────────────────────────────────────────────────────┐
│ GCS_MAVLINK::handle_odometry()                                             │
│   • yêu cầu frame_id = MAV_FRAME_LOCAL_FRD (20)                           │
│     và child_frame_id = MAV_FRAME_BODY_FRD (12) — nếu sai thì bị bỏ qua   │
│   • posErr = cbrtf(cov[0]² + cov[6]² + cov[11]²)  (từ pose_covariance)   │
│   • velocity được xoay bằng quaternion q: vel = q * vel (body FRD)       │
│   → AP_VisualOdom_MAV::handle_pose_estimate()                             │
└───────────────────────────────┬────────────────────────────────────────────┘
                                │ writeExtNavData()
                                ▼
┌────────────────────────────────────────────────────────────────────────────┐
│ EKF3 (EXTNAV)                                                             │
│   EK3_SRC1_POSXY = 6 (EXTNAV), EK3_SRC1_POSZ = 1 (BARO)                   │
│   velPosObs[3] = x (North), velPosObs[4] = y (East)  ← fuse nguyên xi      │
│   R_OBS[3] = sq(constrain(posErr, 0.01, 10))                               │
└────────────────────────────────────────────────────────────────────────────┘
```



## 2. Tầng 1 — Gazebo UWB plugin (`uwb_gz_plugin`)

Plugin được attach vào **world** (không phải model), vì vậy nó tick mỗi bước
mô phỏng và tự tìm model xe theo tên.

### Cấu hình trong `uwb_runway.sdf`

File này là **template**. Launch file sinh lại vị trí anchor theo layout
(`layout_size_x_m`, `layout_size_y_m`, `layout_alpha_deg`) trước khi khởi động
Gazebo, xem `uwb_pozyx_bridge/uwb_world.py::render_world`. Ở layout mặc định
(hình vuông 10×10 m, α = 0):

```xml
<plugin filename="uwb_gz_plugin" name="uwb_gz_plugin::systems::UwbSystem">
  <update_rate>20.0</update_rate>
  <tag_id>0</tag_id>
  <tag_model_name>x500</tag_model_name>
  <tag_link>base_link</tag_link>
  <all_los>true</all_los>
  <max_range>50.0</max_range>
  <los_std>0.05</los_std>
  <anchor name="uwb_anchor0"><pos>0 0 2.0</pos></anchor>
  <anchor name="uwb_anchor1"><pos>0 10 2.0</pos></anchor>
  <anchor name="uwb_anchor2"><pos>10 10 2.0</pos></anchor>
  <anchor name="uwb_anchor3"><pos>10 0 2.0</pos></anchor>
</plugin>
```



### Vòng lặp `PostUpdate()`

1. **Tìm tag** — `EntityByName(tag_model_name)` cho model, rồi `FindLinkEntity()`
  đệ quy xuống các model con (vì `x500_with_ardupilot` `<include>` `x500_base`,
   nên `base_link` nằm trong model lồng nhau).
2. **Lấy vị trí world** — ưu tiên component `WorldPose`; nếu không có thì
  `LocalToWorldPose()` cộng dồn `Pose` theo chuỗi cha–con.
3. **Cập nhật anchor** — `RefreshAnchorPositions()` đọc `WorldPose`/`Pose` của
  từng anchor (anchor có entity thật thì lấy vị trí động, ngược lại giữ vị
   trí lúc `Configure()`).
4. **Tính range** — `ranging_->Compute(tag_world, dt)` → một `RangingResult`
  cho mỗi anchor, kèm `range`, `rss`, `los`.
5. **Publish** — xem mục 3.



### Vị trí tag: `tag_z_offset_`

Giá trị mặc định là `0.0`, tức tag nằm đúng ở `base_link`. Trong world này
anchor ở `z = 2.0 m` còn xe ở `z ≈ 0.2 m`, nên range ngang ~7.07 m.

## 3. Encoding trên gz-transport

Ba topic, đều là `gz::msgs::Pose` (trừ marker):


| gz topic            | Loại               | Nội dung                          |
| ------------------- | ------------------ | --------------------------------- |
| `/uwb/ranging/tag`  | `gz::msgs::Pose`   | 1 message / anchor / chu kỳ       |
| `/uwb/tag_pose/tag` | `gz::msgs::Pose`   | vị trí world của tag              |
| `/uwb/anchors/tag`  | `gz::msgs::Marker` | hình cầu tại anchor, màu theo LOS |




### Encoding của `/uwb/ranging/tag`

Đây là **encoding tùy biến**, không phải pose thật — các field bị dùng lại:


| Field           | Ý nghĩa                                         |
| --------------- | ----------------------------------------------- |
| `position.x`    | range tới anchor, **mét**                       |
| `position.y`    | RSS, **dBm**                                    |
| `position.z`    | `anchor_id` (float)                             |
| `orientation.x` | LOS code: `0`=LOS, `1`=NLOS soft, `2`=NLOS hard |
| `orientation.w` | `1.0` (identity)                                |
| `name`          | tên anchor                                      |


Hai chi tiết quan trọng:

- `<pos>` của `<anchor>` phải là **child element**, không phải attribute
(`<anchor name="..." pos="x y z"/>` sẽ bị bỏ qua).
- LOS code **không** được encode thành quaternion roll: giá trị `1` sẽ thành
`sin(0.5) = 0.479` và bị Python round về `0`, mất cờ NLOS. Vì vậy plugin ghi
thẳng vào `orientation.x`.



## 4. Tầng 2 — Bridge gz → ROS 2

`uwb_gz_to_ros_bridge` (`src/uwb_gz_plugin/src/ros_gz_bridge_node.cc`) là node
C++ độc lập, dùng `gz::transport::Node` để subscribe và `rclcpp` để publish.

**Vì sao không dùng** `ros_gz_bridge` **có sẵn?** Bản `ros_gz_bridge` 0.244 trong
workspace này bị lỗi factory lookup khi bridge `gz.msgs.Pose` →
`geometry_msgs/msg/Pose`, nên phải tự viết node.

```cpp
auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort().durability_volatile();
```

QoS là **best-effort / volatile** để subscriber dùng profile mặc định của
rclpy vẫn nhận được, không bị lỗi "incompatible QoS".

## 5. Tầng 3 — `odometry_node` (Python)

Node Python nhận ranging, giải vị trí, đóng gói MAVLink `ODOMETRY`, gửi đi.

### 5.1 Tham số quan trọng


| Tham số                                       | Mặc định                 | Ý nghĩa                                    |
| --------------------------------------------- | ------------------------ | ------------------------------------------ |
| `input_topic`                                 | `/uwb/ranging_ros`       | topic ranging đầu vào                      |
| `output_pose_topic`                           | `/uwb/pose`              | topic debug vị trí giải được               |
| `serial_host` / `serial_port`                 | `127.0.0.1` / `5761`     | endpoint TCP tới SITL                      |
| `send_rate`                                   | `20.0`                   | nhịp gửi message (Hz)                      |
| `anchor_count`                                | `4`                      | số anchor                                  |
| `layout_size_x_m`                             | `10.0`                   | cạnh `a0 → a1` của layout (Anchor Frame), m  |
| `layout_size_y_m`                             | `10.0`                   | cạnh `a0 → a3` của layout (Anchor Frame), m  |
| `layout_alpha_deg`                            | `0.0`                    | góc lệch α của layout so với NED (°)        |
| `anchor_plane_height_m`                       | `2.0`                    | chiều cao mặt phẳng anchor so với mặt đất, m |
| `position_std_m`                              | `0.05`                   | **độ lệch chuẩn** của estimate vị trí, mét |
| `quality`                                     | `50`                     | quality gửi kèm (0–100)                    |
| `reset_jump_threshold_m`                      | `2.0`                    | ngưỡng nhảy vị trí để bump reset counter   |
| `assume_constant_z`                           | `False`                  | giữ altitude cố định (mặc định: giải 3D)    |
| `initial_guess`                               | `[5.0, 5.0, 1.0]`        | hạt giống cho solver (Anchor Frame)          |
| `min_anchors`                                 | `3`                      | số anchor tối thiểu để có lời giải         |
| `max_range`                                   | `50.0`                   | loại bỏ range vượt ngưỡng                  |
| `los_std` / `nlos_soft_std` / `nlos_hard_std` | `0.10` / `0.30` / `0.60` | nhiễu theo LOS (m)                         |




### 5.2 Vòng lặp gửi (`_send_cycle`, 20 Hz)

```
_on_ranging_pose(msg)              # callback: decode + buffer theo anchor_id
        │
        ▼
solve_position(anchors, ranges)    # WLS trilateration
        │  ├─ _linear_seed()        # seed khép kín, chỉ giải XY nếu assume_z
        │  ├─ _gauss_newton()       # lặp WLS tối đa 10 vòng, tol 1e-4
        │  └─ _estimate_covariance()
        ▼
_publish_pose(result)               # → /uwb/pose (PoseStamped, frame "world")
        │
        ▼
north, east, down = layout.to_ned_3d(x, y, z)  # Anchor Frame → NED
        │
        ▼
_detect_reset(...)                 # bump reset_counter nếu nhảy > threshold
        │
        ▼
build_odometry_message(...)        # MAVLink ODOMETRY (id 331)
        │
        ▼
MavlinkSender.send(message)        # 1 lần write, ~245 byte
```



### 5.3 Trọng số WLS

`RangeMeasurement.weight()` suy ra variance từ LOS code cộng thêm một hạng tử
phụ thuộc RSS:

```python
if self.los == 0:   sigma = los_std          # LOS
elif self.los == 1: sigma = nlos_soft_std    # NLOS soft
else:               sigma = nlos_hard_std    # NLOS hard
rss_term = 1.0 + max(0.0, (-70.0 - self.rss_dbm)) * 0.05   # RSS yếu → variance lớn
sigma *= rss_term
return 1.0 / (sigma * sigma)
```



### 5.4 Vì sao mặc định giải 3D chọn nhánh (`assume_constant_z = False`)

Anchor nằm cùng một mặt phẳng (Anchor Frame `z = 0`) ⇒ chỉ từ khoảng cách đo
được thì **không xác định được** tag nằm ở phía nào của mặt phẳng: một tag 1 m
*dưới* mặt phẳng cho ra đúng những khoảng cách như một tag 1 m *trên* mặt phẳng.

Cách cũ (giữ Z cố định) **không trung lập**: khoảng cách đo được sẽ không nhất
quán với giả định về độ cao, và nghiệm ngang bị lệch. Đo thực nghiệm trên
hình chữ nhật 10×10 m:

| Chiều cao thật | Sai số ngang (2D, Z cố định) | Sai số ngang (3D, chọn nhánh) |
| :-- | :-- | :-- |
| 0.30 m | 0.074 m / p99 0.190 m | 0.045 m / p99 0.113 m |
| 0.80 m | 0.067 m / p99 0.170 m | 0.046 m / p99 0.112 m |
| 1.50 m | 0.050 m / p99 0.124 m | 0.047 m / p99 0.118 m |

⇒ Solver 3D chọn nhánh giữ sai số ngang ở mức nhiễu đo (~0.05 m) **bất kể độ
cao thật**.

**Cơ chế chọn nhánh** (`trilateration.py` → `_solve_3d_branch`):

1. Giải Gauss-Newton 3D từ **hai hạt giống**, một ở mỗi phía mặt phẳng anchor.
2. Chọn hạt giống cho **nghiệm gần nghiệm trước** (`previous_position`) nhất ⇒
   giữ đúng nhánh khi tag bay xuyên qua mặt phẳng.
3. Nếu chưa có nghiệm trước, chọn theo residual nhỏ nhất.

Node giữ `self._last_solution.position` và truyền vào solver làm `previous_position`.

> Cờ `assume_constant_z = True` vẫn còn để dùng khi layout **không** đồng phẳng
> (ví dụ anchor gắn trên trần và dưới sàn). Với layout đồng phẳng mặc định của
> project, để `False`.

## 6. Tầng 4 — MAVLink `ODOMETRY` (id 331)



### 6.1 Hai ràng buộc frame của ArduPilot

`GCS_MAVLINK::handle_odometry()` **bỏ qua âm thầm** message nếu frame không khớp:

```cpp
if (m.frame_id != MAV_FRAME_LOCAL_FRD ||
    m.child_frame_id != MAV_FRAME_BODY_FRD) {
    // only support local FRD frame data
    return;
}
```


| Trường           | Hằng số               | Giá trị |
| ---------------- | --------------------- | ------- |
| `frame_id`       | `MAV_FRAME_LOCAL_FRD` | 20      |
| `child_frame_id` | `MAV_FRAME_BODY_FRD`  | 12      |


Hai giá trị này được hard-code trong `mavlink_odometry.py` và **không** cấu
hình được.

### 6.2 Quy ước vị trí: NED, không xoay

Khác với velocity, **vị trí trong** `ODOMETRY` **không bị ArduPilot xoay** — nó
được fuse thẳng vào EKF3, mà EKF3 làm việc trong hệ NED:

```cpp
// AP_NavEKF3_PosVelFusion.cpp
velPosObs[3] = extNavDataDelayed.pos.x;   // North
velPosObs[4] = extNavDataDelayed.pos.y;   // East
```

Vì vậy position phải mang **NED**: `x` = North, `y` = East, `z` = Down.

Gazebo world là ENU (x East, y North, z Up), còn solver giải trong **Anchor
Frame** (`x` hướng `a1`, `y` hướng `a3`, `z` hướng xuống). Việc chuyển frame
được gom vào `uwb_layout.py`:

```python
def rotate_ned(x: float, y: float, alpha_rad: float) -> tuple[float, float]:
    cos_a = math.cos(alpha_rad)
    sin_a = math.sin(alpha_rad)
    return (x * cos_a - y * sin_a, x * sin_a + y * cos_a)
```

và `AnchorLayout.to_ned_3d()`:

```python
north, east = self.to_ned(x, y)          # xoay α + offset gốc
return (north, east, z - self.plane_height_m)
```

> ⚠️ Đây là **điểm duy nhất** được phép đổi frame trong codebase. Nếu hoán
> đổi sai, vị trí EKF bị phản chiếu qua đường chéo và drone bay tới vị trí
> đối xứng. `tools/inspect_frames.py` có gate check riêng cho việc này.



### 6.3 Quy ước velocity: body FRD

Velocity thì **có** bị ArduPilot xoay:

```cpp
// convert velocity vector from FRD to NED frame
Vector3f vel{m.vx, m.vy, m.vz};
vel = q * vel;
```

Tức `vx`/`vy`/`vz` phải là theo **body FRD** (x forward, y right, z down). UWB
trilateration không cho velocity estimate, nên node gửi **`NaN`** cho cả ba và
`EK3_SRC1_VELXY` để `0` (NONE) — EKF không fuse velocity từ nguồn này.

> `NaN` là cách **đúng**, không phải lỗi: ArduPilot reject mẫu NaN
> (`AP_NavEKF3_Measurements.cpp`: `if (vel.is_nan() || isnan(err)) return;`) ⇒
> `useExtNavVel` **không** được set ⇒ EKF3 tự tích phân vận tốc bằng IMU.
>
> Nếu gửi `0.0` thì EKF sẽ fuse "vận tốc bằng 0" — sai. Với
> `EK3_SRC1_POSZ = 1` (BARO) thì thiệt hại chưa thấy ngay, nhưng nếu ai đó đổi
> `EK3_SRC1_POSZ` sang `EXTNAV` thì drone bị **đóng băng vận tốc Z**.
>
> `velocity_covariance[0]` cũng đặt `NaN` — đúng theo MAVLink spec
> *"If unknown, assign NaN value to first element in the array"*.



### 6.4 Covariance → measurement noise

ArduPilot không có trường "accuracy" riêng; nó suy ra noise từ covariance:

```cpp
posErr = cbrtf(sq(m.pose_covariance[0]) + sq(m.pose_covariance[6]) + sq(m.pose_covariance[11]));
```

Chỉ ba phần tử chéo của khối position trong ma trận 6×6 được đọc:
`[0]` = var(x), `[6]` = var(y), `[11]` = var(z). Mỗi phần tử là **variance**
(\sigma^2), không phải stddev.

Do công thức lại **bình phương** giá trị đã là variance, `posErr` thu được
nhỏ hơn stddev thật. Vì vậy `VISO_POS_M_NSE` là **sàn** quan trọng:

```cpp
// AP_VisualOdom_MAV::handle_pose_estimate()
posErr = constrain_float(posErr, _frontend.get_pos_noise(), 100.0f);
```

Node gửi `pose_covariance = position_std_m²` cho cả ba trục; với
`position_std_m = 0.05` thì variance là `0.0025`, và `posErr` bị clamp lên
đúng sàn `VISO_POS_M_NSE = 0.05`. Giá trị này thành
`R_OBS[3] = sq(constrain(posErr, 0.01, 10))`.

### 6.5 Timestamp

`time_usec` là **Unix epoch** (microsecond). ArduPilot dùng `JitterCorrection`
để map nó sang đồng hồ nội bộ:

```cpp
const uint32_t timestamp_ms = correct_offboard_timestamp_usec_to_ms(m.time_usec, ...);
```

Bộ sửa này ước lượng độ trễ của link và bù vào, nên gửi wall-clock time là
đúng. `VISO_DELAY_MS` (mặc định đặt `50`) là độ trễ vận chuyển từ lúc fix
được tạo ra tới lúc EKF dùng nó.

### 6.6 Frame wire

MAVLink v2:

```
[0xFD][len][seq][sysid][compid][msgid=331][payload][crc][crc]
```

- `sysid = 255` (khác `SYSID_THISMAV = 1` của ArduPilot), `compid = 191`
  (`MAV_COMP_ID_ONBOARD_COMPUTER` — khớp ESP32 tag).
- Payload ODOMETRY dài 222 byte → frame ~245 byte.
- Sequence number tăng dần, wrap ở 256.
- `estimator_type = MAV_ESTIMATOR_TYPE_NAIVE` (1) — khớp ESP32 tag (solver
  trilateration là một estimator *naive*, không phải VIO).

> ArduPilot chấp nhận packet khi `msg.sysid == SYSID_MYGCS` (mặc định 255)
> hoặc khi `SYSID_ENFORCE = 0`. `sysid = 255` khớp `SYSID_MYGCS` nên
> message được nhận kể cả khi `SYSID_ENFORCE` bật.



## 7. Tầng 5 — ArduPilot SITL



### 7.1 Port serial

```bash
arducopter --model json --speedup 1 --slave 0 --sim-address=127.0.0.1 \
           --instance 0 --defaults .../x500_uwb.parm \
           --synthetic-clock --serial1 tcp:5761:wait --sysid 1
```

`:wait` nghĩa là SITL **block** cho tới khi có client kết nối, nên không bỏ
lỡ message đầu tiên.

### 7.2 Tham số liên quan


| Tham số                   | Giá trị   | Ý nghĩa                                               |
| ------------------------- | --------- | ----------------------------------------------------- |
| `BCN_TYPE`                | `0`       | backend beacon **tắt** (thay bằng VISO)               |
| `VISO_TYPE`               | `1`       | `AP_VisualOdom_MAV` — backend MAVLink (nhận ODOMETRY) |
| `VISO_DELAY_MS`           | `50`      | độ trễ transport, ms                                  |
| `VISO_POS_M_NSE`          | `0.05`    | sàn measurement noise vị trí, m                       |
| `VISO_QUAL_MIN`           | `0`       | chấp nhận cả quality=0 (unknown)                      |
| `SERIAL1_PROTOCOL`        | `1`       | `SerialProtocol_MAVLink`                              |
| `SERIAL1_BAUD`            | `115`     | baud rate                                             |
| `BRD_SER1_RTSCTS`         | `0`       | không dùng flow control phần cứng                     |
| `EK3_SRC1_POSXY`          | `6`       | nguồn vị trí XY = **EXTNAV**                          |
| `EK3_SRC1_POSZ`           | `1`       | nguồn độ cao = BARO                                   |
| `EK3_SRC1_VELXY` / `VELZ` | `0` / `0` | không có velocity estimate                            |
| `EK3_SRC1_YAW`            | `1`       | yaw từ compass                                        |
| `AHRS_EKF_TYPE`           | `3`       | dùng EKF3                                             |
| `GPS1_TYPE` / `GPS2_TYPE` | `0`       | indoor, tắt GPS                                       |
| `ARMING_CHECK`            | `0`       | tắt toàn bộ pre-arm check                             |


> `VISO_TYPE` **phải** là `1`. `EK3_SRC1_POSXY = EXTNAV` khi
> `VISO_TYPE = 0` sẽ không có backend nào cấp dữ liệu, EKF không bao giờ
> chuyển sang `AID_ABSOLUTE`.

> `EK3_SRC1_POSZ` **phải** là `BARO`. Nếu để `EXTNAV` thì EKF lấy độ cao từ
> ODOMETRY, mà node gửi `z` cố định bằng 0 — drone sẽ không giữ được cao độ.



### 7.3 Điều ArduPilot làm với message nhận được

```
GCS_MAVLINK::update_receive() → packetReceived() → handle_message()
  → handle_odometry()
      ├─ kiểm tra frame_id / child_frame_id (20 / 12)
      ├─ posErr từ pose_covariance
      ├─ visual_odom->handle_pose_estimate(...)
      │     → AP::ahrs().writeExtNavData(pos, attitude, posErr, angErr, ...)
      └─ visual_odom->handle_vision_speed_estimate(...)
            → AP::ahrs().writeExtNavVelData(...)
  → NavEKF3_core::writeExtNavData()
      → storedExtNav buffer
      → fusePosData: velPosObs[3] = pos.x, velPosObs[4] = pos.y
```

Log xác nhận:

```
EKF3 IMU0 is using external nav data
EKF3 IMU0 initial pos NED = 3.0,4.0,-0.0 (m)
```



## 8. Định nghĩa topic và frame


| Topic               | Kiểu                        | Publisher                  | QoS                   |
| ------------------- | --------------------------- | -------------------------- | --------------------- |
| `/uwb/ranging/tag`  | `gz::msgs::Pose`            | `UwbSystem` (gz-transport) | gz mặc định           |
| `/uwb/tag_pose/tag` | `gz::msgs::Pose`            | `UwbSystem`                | gz mặc định           |
| `/uwb/anchors/tag`  | `gz::msgs::Marker`          | `UwbSystem`                | gz mặc định           |
| `/uwb/ranging_ros`  | `geometry_msgs/Pose`        | `uwb_gz_to_ros_bridge`     | best-effort, volatile |
| `/uwb/tag_pose_ros` | `geometry_msgs/Pose`        | `uwb_gz_to_ros_bridge`     | best-effort, volatile |
| `/uwb/pose`         | `geometry_msgs/PoseStamped` | `odometry_node`            | best-effort, volatile |


Frame bị ảnh hưởng bởi hệ UWB:

- `world` — frame world của Gazebo, dùng cho `/uwb/pose` và cho vị trí anchor
trong trilateration.
- `odom` — frame odom của Gazebo (odometry publisher, `odom_frame=odom`,
`robot_base_frame=base_link`), được RViz dùng  làm fixed frame.
- `base_link` — frame của link gốc xe, là vị trí gắn tag UWB.

> Lưu ý: frame `odom` do Gazebo sinh ra **không** phải frame `odom` của EKF3
> ArduPilot. Chúng là hai frame độc lập, không có transform nối giữa. Khi
> external nav đã fuse, vị trí EKF3 (qua MAVROS) và vị trí `/uwb/pose` mới
> xấp xỉ trùng nhau.



## 9. Chuỗi khởi động

Thứ tự khởi động trong `x500_uwb_runway.launch.py` có chủ đích:


| Mốc thời gian | Hành động                                         | Lý do                         |
| ------------- | ------------------------------------------------- | ----------------------------- |
| t = 0         | Gazebo server + GUI, RViz                         | world và anchor phải có trước |
| t = 0         | `uwb_gz_to_ros_bridge`                            | subscribe gz topic            |
| t ≈ 0         | SITL (`arducopter`) với `--serial1 tcp:5761:wait` | block chờ MAVLink client      |
| t = 8 s       | `sitl_extnav_monitor` (mặc định **tắt**)          | chỉ dùng để set param runtime |
| t = 14 s      | `odometry_node`                                   | SITL đã bind port 5761        |


Ngoài ra launch file kiểm tra trước khi start:

- **Port 5760 và 5761 phải free** — nếu SITL instance cũ còn chạy, SITL mới
exit với `bind failed on port 5760` và MAVProxy retry vô hạn, trông như lỗi
telemetry thay vì process cũ.
- `world_name` **phải khớp** `<world name="...">` trong SDF — bridge Gazebo
subscribe `/world/<world_name>/model/<robot_name>/joint_state`. Nếu sai,
joint state không tới ⇒ RSP không xuất transform `base_link → rotor_0` ⇒
RViz báo `No transform from [rotor_0] to [odom]`.



## 10. Luồng lỗi và triệu chứng


| Triệu chứng                                                | Nguyên nhân                                        | Cách xử lý                                                  |
| ---------------------------------------------------------- | -------------------------------------------------- | ----------------------------------------------------------- |
| RViz: `No transform from [rotor_0] to [odom]`              | `world_name` không khớp SDF ⇒ không có joint state | truyền `world_name` đúng, ví dụ `uwb_runway`                |
| `odometry_node` chết với `PackageNotFoundError`            | build `uwb_pozyx_bridge` lỗi, mất package metadata | `colcon build --packages-up-to uwb_pozyx_bridge`            |
| MAVProxy lặp `link 1 down`                                 | SITL cũ còn chiếm port 5760                        | `pkill -f arducopter; pkill -f mavproxy.py`                 |
| EKF không báo `is using external nav data`                 | `VISO_TYPE` = 0, hoặc node chưa kết nối port 5761  | kiểm tra `VISO_TYPE 1` và log `messages_sent` của node      |
| EKF fuse nhưng vị trí lệch gương (đối xứng qua đường chéo) | Anchor Frame → NED bị hoán đổi sai trục     | dùng `AnchorLayout.to_ned_3d()`: `x` = North, `y` = East        |
| EKF không lấy độ cao                                       | `EK3_SRC1_POSZ` bị đặt sai                         | đặt `EK3_SRC1_POSZ 1` (BARO)                                |
| EKF bám quá chặt / innovation test fail                    | `VISO_POS_M_NSE` quá nhỏ                           | tăng `VISO_POS_M_NSE` (ví dụ `0.05` → `0.2`)                |
| Drone bị "lật" khi bay vòng                                | position setpoint lệch vì EKF lệch frame           | kiểm tra `/uwb/pose` so với `LOCAL_POSITION_NED`            |
| LOS flag bị mất                                            | encode LOS thành quaternion roll                   | ghi thẳng `orientation.x`                                   |
| `Arm: Need Position Estimate`                              | EKF mất external nav sau khi SITL restart          | restart lại toàn bộ stack, chờ `is using external nav data` |




## 11. Diễn giải protocol trong Python

`mavlink_odometry.py` là **điểm duy nhất** được phép đóng gói message. Mọi thứ
khác giữ nguyên hệ Anchor Frame.

```python
north_m, east_m, down_m = layout.to_ned_3d(x, y, z)   # Anchor Frame → NED
message = build_odometry_message(
    north_m, east_m, down_m,
    time_usec=int(time.time() * 1e6),
    position_std_m=0.05,
    quality=50,
)
self._sender.send(message)
```

`build_odometry_message()` đặt sẵn `velocity_ned = (NaN, NaN, NaN)` và
`velocity_covariance[0] = NaN`, `estimator_type = MAV_ESTIMATOR_TYPE_NAIVE` —
đúng như ESP32 tag gửi.

`MavlinkSender` cố ý **tolerant**: nếu socket rớt (SITL restart) thì reconnect
nền ở chu kỳ sau thay vì raise, và message không gửi được thì **drop** chứ
không queue — với EKF thì dữ liệu cũ còn tệ hơn không có dữ liệu.

Sender cũng có một thread drain nhận dữ liệu từ ArduPilot: autopilot gửi
telemetry ngược lại trên cùng port, nếu không đọc thì buffer nhận của kernel
đầy và phía autopilot bị block.

## 12. Công cụ kiểm tra


| Tool                             | Việc                                                                        |
| -------------------------------- | --------------------------------------------------------------------------- |
| `tools/inspect_frames.py`        | gate check frame: in message ODOMETRY sẽ gửi, kiểm tra frame id và khớp NED |
| `tools/sitl_extnav_monitor.py`   | đọc/ghi param VISO + EK3, theo dõi `EKF_STATUS_REPORT`                      |
| `tools/check_extnav_e2e.py`      | gửi ODOMETRY cố định, xác nhận EKF báo `is using external nav data`         |
| `tools/check_extnav_tracking.py` | gửi ramp vị trí, xác nhận EKF bám theo                                      |
| `tools/compare_positions.py`     | so sánh vị trí Gazebo / trilateration / EKF                                 |
| `tools/verify_fdm_coupling.py`   | kiểm tra SITL thực sự điều khiển mô phỏng Gazebo                            |
| `tools/run_pipeline_verify.sh`   | chạy cả ba gate trên trong một lần                                          |


```bash
# Kiểm tra nhanh không cần Gazebo
python3 src/uwb_pozyx_bridge/tools/check_extnav_e2e.py
```

