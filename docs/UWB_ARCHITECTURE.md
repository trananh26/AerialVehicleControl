# Flow architecture: `uwb_gz_plugin` → ArduPilot

Tài liệu này mô tả luồng dữ liệu đầy đủ từ Gazebo UWB plugin tới EKF3 của
ArduPilot SITL, gồm các topic, frame, đơn vị và các bước xử lý trung gian.

## 1. Tổng quan

```
┌────────────────────────────────────────────────────────────────────────────┐
│ Gazebo — uwb_runway.sdf                                                    │
│   <plugin filename="uwb_gz_plugin" name="uwb_gz_plugin::systems::UwbSystem">│
│     update_rate = 20 Hz, tag_model_name = x500, tag_link = base_link      │
│     anchors (ENU, m): (0,0,2) (10,0,2) (10,10,2) (0,10,2)                │
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
│ pozyx_node  (Python, package uwb_pozyx_bridge) — đóng vai Arduino Uno     │
│                                                                             │
│  1. decode_ranging_pose()  : Pose → RangeMeasurement                       │
│  2. solve_position()       : WLS trilateration → vị trí tag (ENU, m)       │
│  3. encode_*()             : đóng gói 3 loại frame Pozyx                  │
│  4. PozyxSender.send()     : ghi bytes vào TCP 127.0.0.1:5761             │
│                                                                             │
│  Timer 20 Hz (send_rate)                                                   │
│  Publish thêm: /uwb/pose  (PoseStamped, frame "world") để debug           │
└───────────────────────────────┬────────────────────────────────────────────┘
                                │ binary frames
                                │ [0x01][msg_id][len][payload][checksum]
                                ▼
┌────────────────────────────────────────────────────────────────────────────┐
│ ArduPilot SITL                                                            │
│   arducopter --serial1=tcp:5761:wait                                       │
│   SERIAL1_PROTOCOL = 13  (SerialProtocol_Beacon)                           │
│   SERIAL1_BAUD = 115, BRD_SER1_RTSCTS = 0                                  │
└───────────────────────────────┬────────────────────────────────────────────┘
                                │ AP_Beacon_Pozyx::update() → parse_buffer()
                                ▼
┌────────────────────────────────────────────────────────────────────────────┐
│ AP_Beacon_Pozyx  (đặt vị trí, khoảng cách, vị trí xe)                     │
│   set_beacon_position(id, pos)    ← MSGID_BEACON_CONFIG (0x02) × 4         │
│   set_beacon_distance(id, dist)   ← MSGID_BEACON_DIST   (0x03) × 4         │
│   set_vehicle_position(pos, err)  ← MSGID_POSITION      (0x04) × 1         │
│                                                                             │
│   ENU mm → wire (x=North, y=East) → NED m                                   │
└───────────────────────────────┬────────────────────────────────────────────┘
                                ▼
┌────────────────────────────────────────────────────────────────────────────┐
│ AP_Beacon frontend → AP_DAL_Beacon → EKF3                                  │
│   EKF3_SRC1_POSXY = 4 (BEACON), EKF3_SRC1_POSZ = 1 (BARO)                  │
│   readRngBcnData() → FuseRngBcn() → EKF lấy origin từ beacon system        │
└────────────────────────────────────────────────────────────────────────────┘
```

## 2. Tầng 1 — Gazebo UWB plugin (`uwb_gz_plugin`)

Plugin được attach vào **world** (không phải model), vì vậy nó tick mỗi bước
mô phỏng và tự tìm model xe theo tên.

### Cấu hình trong `uwb_runway.sdf`

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
  <anchor name="uwb_anchor1"><pos>10 0 2.0</pos></anchor>
  <anchor name="uwb_anchor2"><pos>10 10 2.0</pos></anchor>
  <anchor name="uwb_anchor3"><pos>0 10 2.0</pos></anchor>
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

| gz topic | Loại | Nội dung |
| --- | --- | --- |
| `/uwb/ranging/tag` | `gz::msgs::Pose` | 1 message / anchor / chu kỳ |
| `/uwb/tag_pose/tag` | `gz::msgs::Pose` | vị trí world của tag |
| `/uwb/anchors/tag` | `gz::msgs::Marker` | hình cầu tại anchor, màu theo LOS |

### Encoding của `/uwb/ranging/tag`

Đây là **encoding tùy biến**, không phải pose thật — các field bị dùng lại:

| Field | Ý nghĩa |
| --- | --- |
| `position.x` | range tới anchor, **mét** |
| `position.y` | RSS, **dBm** |
| `position.z` | `anchor_id` (float) |
| `orientation.x` | LOS code: `0`=LOS, `1`=NLOS soft, `2`=NLOS hard |
| `orientation.w` | `1.0` (identity) |
| `name` | tên anchor |

Hai chi tiết quan trọng:

- `<pos>` của `<anchor>` phải là **child element**, không phải attribute
  (`<anchor name="..." pos="x y z"/>` sẽ bị bỏ qua).
- LOS code **không** được encode thành quaternion roll: giá trị `1` sẽ thành
  `sin(0.5) = 0.479` và bị Python round về `0`, mất cờ NLOS. Vì vậy plugin ghi
  thẳng vào `orientation.x`.

## 4. Tầng 2 — Bridge gz → ROS 2

`uwb_gz_to_ros_bridge` (`src/uwb_gz_plugin/src/ros_gz_bridge_node.cc`) là node
C++ độc lập, dùng `gz::transport::Node` để subscribe và `rclcpp` để publish.

**Vì sao không dùng `ros_gz_bridge` có sẵn?** Bản `ros_gz_bridge` 0.244 trong
workspace này bị lỗi factory lookup khi bridge `gz.msgs.Pose` →
`geometry_msgs/msg/Pose`, nên phải tự viết node.

```cpp
auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort().durability_volatile();
```

QoS là **best-effort / volatile** để subscriber dùng profile mặc định của
rclpy vẫn nhận được, không bị lỗi "incompatible QoS".

## 5. Tầng 3 — `pozyx_node` (Python)

Node Python đóng vai Arduino `IndoorLoiter.ino`: nhận ranging, giải vị trí,
đóng gói protocol Pozyx, gửi đi.

### 5.1 Tham số quan trọng

| Tham số | Mặc định | Ý nghĩa |
| --- | --- | --- |
| `input_topic` | `/uwb/ranging_ros` | topic ranging đầu vào |
| `output_pose_topic` | `/uwb/pose` | topic debug vị trí giải được |
| `serial_host` / `serial_port` | `127.0.0.1` / `5761` | endpoint TCP tới SITL |
| `send_rate` | `20.0` | nhịp gửi frame (Hz) |
| `anchor_count` | `4` | số anchor |
| `anchor_offsets_mm` | `(-5000,-5000,1900)`, … | vị trí anchor **tương đối beacon origin**, mm |
| `position_accuracy_m` | `0.05` | độ chính xác, **mét** (xem mục 6.3) |
| `assume_constant_z` | `True` | giữ altitude cố định |
| `min_anchors` | `3` | số anchor tối thiểu để có lời giải |
| `max_range` | `50.0` | loại bỏ range vượt ngưỡng |
| `los_std` / `nlos_soft_std` / `nlos_hard_std` | `0.10` / `0.30` / `0.60` | nhiễu theo LOS (m) |

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
frames = []
  + encode_beacon_config(...)   × 4 # chỉ gửi lại mỗi config_interval (1 s)
  + encode_beacon_distance(...) × 4 # 1 frame / anchor, giữ range cũ nếu chưa có
  + encode_vehicle_position(...)  × 1
        │
        ▼
PozyxSender.send(bytes(frames))  # 1 lần write, ~40 byte
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

### 5.4 Vì sao `assume_constant_z = True`

Anchor nằm cùng một mặt phẳng (z = 2.0 m). Nếu giải full-3D, bài toán
trilateration có **hai nghiệm** đối xứng qua mặt phẳng anchor, và solver có
thể nhảy sang nghiệm sai. Giữ z cố định loại bỏ luôn nguồn sai này.

## 6. Tầng 4 — Wire protocol Pozyx

### 6.1 Format chung

```
[0x01][msg_id][len][payload ...][checksum]
```

- `len` = `payload_len + 1` (bao gồm byte checksum)
- `checksum` = XOR của `msg_id`, `len` và mọi byte payload
- multi-byte field: **little-endian**
- payload tối đa 20 byte (giới hạn của `AP_BEACON_POZYX_MSG_LEN_MAX`)

### 6.2 Ba loại message

| msg_id | Tên | Payload | Ý nghĩa |
| --- | --- | --- | --- |
| `0x02` | `BEACON_CONFIG` | `<BBiii` = id, count, x, y, z (mm) | vị trí anchor tương đối beacon origin |
| `0x03` | `BEACON_DIST` | `<BI` = id, distance (mm) | khoảng cách tag ↔ anchor |
| `0x04` | `POSITION` | `<iiih` = x, y, z (mm), error | vị trí xe + sai số |

Tất cả vị trí được gửi theo **millimetre, tương đối beacon origin**, theo
convention của Pozyx: byte `x` = **NED North**, byte `y` = **NED East**,
byte `z` = **Up**.

> ⚠️ `AP_Beacon_Pozyx.cpp` **không** tự đảo trục ngang — nó đọc byte thô và
> gán thẳng vào hệ NED:
>
> ```cpp
> Vector3f beacon_pos(beacon_x * 0.001f, beacon_y * 0.001f, -beacon_z * 0.001f);
> ```
>
> nên byte `x` được hiểu là **North** và byte `y` là **East**. Node Python phải
> hoán đổi trục ngang so với Gazebo ENU trước khi gửi. Việc này do
> `enu_to_wire_mm()` trong `pozyx_protocol.py` đảm nhiệm — xem §11.

### 6.3 Vấn đề đơn vị `position_error` — đã workaround

`IndoorLoiter.ino` điền `pos_error.xy` (đơn vị **mm**) vào trường
`position_error`, nhưng `AP_Beacon_Pozyx::parse_buffer()` chuyển thẳng giá
trị này cho `set_vehicle_position(veh_pos, position_error)`, mà tham số
`accuracy_estimate` được tài liệu hoá là **mét**.

EKF3 so sánh với ngưỡng 1 m:

```cpp
if ((imuSampleTime_ms - rngBcn.last3DmeasTime_ms < 250)
    && (rngBcn.vehiclePosErr < 1.0f)
    && rngBcn.alignmentCompleted) {
```

Nếu gửi `50` (mm) thì EKF hiểu là `50 m` → **fix 3D bị từ chối**,
`goodToAlign` không bao giờ true, EKF không set origin từ beacon system.

**Workaround:** gửi sai số theo **mét** (`0.05` cho 50 mm). Giá trị vẫn nằm
trong cùng trường `int16`, chỉ khác cách diễn giải.

### 6.4 Anchor layout

Beacon origin = vị trí spawn của xe = `(5, 5, 0.1)`. Anchor nằm ở góc hình
vuông 10×10 m, `z = 2.0 m`. Vì vậy offset tương đối origin (mm):

| id | anchor | world (m) | offset từ origin (mm) |
| --- | --- | --- | --- |
| 0 | `uwb_anchor0` | (0, 0, 2.0) | (-5000, -5000, 1900) |
| 1 | `uwb_anchor1` | (10, 0, 2.0) | (5000, -5000, 1900) |
| 2 | `uwb_anchor2` | (10, 10, 2.0) | (5000, 5000, 1900) |
| 3 | `uwb_anchor3` | (0, 10, 2.0) | (-5000, 5000, 1900) |

Giá trị này là `DEFAULT_ANCHOR_OFFSETS_MM` trong `uwb_decoding.py` và được gửi
đi trong frame `BEACON_CONFIG`.

## 7. Tầng 5 — ArduPilot SITL

### 7.1 Port serial

```bash
arducopter --model json --speedup 1 --slave 0 --sim-address=127.0.0.1 \
           --instance 0 --defaults .../x500_uwb.parm \
           --synthetic-clock --serial1 tcp:5761:wait --sysid 1
```

`:wait` nghĩa là SITL **block** cho tới khi có client kết nối, nên không bỏ
lỡ frame đầu tiên.

### 7.2 Tham số liên quan

| Tham số | Giá trị | Ý nghĩa |
| --- | --- | --- |
| `BCN_TYPE` | `1` | dùng backend Pozyx (không phải simulator SITL tích hợp = 10) |
| `SERIAL1_PROTOCOL` | `13` | `SerialProtocol_Beacon` |
| `SERIAL1_BAUD` | `115` | baud rate |
| `BRD_SER1_RTSCTS` | `0` | không dùng flow control phần cứng |
| `EK3_SRC1_POSXY` | `4` | nguồn vị trí XY = BEACON |
| `EK3_SRC1_POSZ` | `1` | nguồn độ cao = BARO (không phải BEACON) |
| `EK3_SRC1_VELXY` / `VELZ` / `YAW` | `0` / `0` / `1` | velocity không lấy từ beacon, yaw từ compass |
| `GPS1_TYPE` / `GPS2_TYPE` | `0` | indoor, tắt GPS |
| `ARMING_CHECK` | `0` | tắt toàn bộ pre-arm check (beacon không đòi GPS) |

> `EK3_SRC1_POSZ` **phải** là `BARO`. Nếu để `BEACON` thì EKF dùng độ cao
> beacon cho cả vị trí 3D và fix 3D bị loại.

### 7.3 Điều ArduPilot làm với frame nhận được

```
AP_Beacon_Pozyx::update()
  → parse_buffer()
      MSGID_BEACON_CONFIG → set_beacon_position(id, pos)     # wire mm → NED m
      MSGID_BEACON_DIST   → set_beacon_distance(id, dist)    # mm → m
      MSGID_POSITION      → set_vehicle_position(pos, err)   # wire mm → NED m
  → AP_Beacon frontend
  → AP_DAL_Beacon
  → EKF3 readRngBcnData() → FuseRngBcn()
```

> Wire convention: `x` = NED **North**, `y` = NED **East**, `z` = **Up**.
> Node Python đã hoán đổi trục ngang từ Gazebo ENU sang convention này
> bằng `enu_to_wire_mm()`.

`set_beacon_distance()` set `healthy = true` và tăng `num_beacons`, nên EKF
cần **~200 measurement** liên tiếp trước khi bắt đầu fuse và set origin.

## 8. Định nghĩa topic và frame

| Topic | Kiểu | Publisher | QoS |
| --- | --- | --- | --- |
| `/uwb/ranging/tag` | `gz::msgs::Pose` | `UwbSystem` (gz-transport) | gz mặc định |
| `/uwb/tag_pose/tag` | `gz::msgs::Pose` | `UwbSystem` | gz mặc định |
| `/uwb/anchors/tag` | `gz::msgs::Marker` | `UwbSystem` | gz mặc định |
| `/uwb/ranging_ros` | `geometry_msgs/Pose` | `uwb_gz_to_ros_bridge` | best-effort, volatile |
| `/uwb/tag_pose_ros` | `geometry_msgs/Pose` | `uwb_gz_to_ros_bridge` | best-effort, volatile |
| `/uwb/pose` | `geometry_msgs/PoseStamped` | `pozyx_node` | best-effort, volatile |

Frame bị ảnh hưởng bởi hệ UWB:

- `world` — frame world của Gazebo, dùng cho `/uwb/pose` và cho vị trí anchor
  trong trilateration.
- `odom` — frame odom của Gazebo (odometry publisher, `odom_frame=odom`,
  `robot_base_frame=base_link`), được RViz dùng làm fixed frame.
- `base_link` — frame của link gốc xe, là vị trí gắn tag UWB.

> Lưu ý: frame `odom` do Gazebo sinh ra **không** phải frame `odom` của EKF3
> ArduPilot. Chúng là hai frame độc lập, không có transform nối giữa. Khi
> beacon đã fuse, vị trí EKF3 (qua MAVROS) và vị trí `/uwb/pose` mới xấp xỉ
> trùng nhau.

## 9. Chuỗi khởi động

Thứ tự khởi động trong `x500_uwb_runway.launch.py` có chủ đích:

| Mốc thời gian | Hành động | Lý do |
| --- | --- | --- |
| t = 0 | Gazebo server + GUI, RViz | world và anchor phải có trước |
| t = 0 | `uwb_gz_to_ros_bridge` | subscribe gz topic |
| t ≈ 0 | SITL (`arducopter`) với `--serial1 tcp:5761:wait` | block chờ Pozyx client |
| t = 8 s | `sitl_beacon_monitor` (mặc định **tắt**) | chỉ dùng để set param runtime |
| t = 14 s | `pozyx_node` | SITL đã bind port 5761 |

Ngoài ra launch file kiểm tra trước khi start:

- **Port 5760 và 5761 phải free** — nếu SITL instance cũ còn chạy, SITL mới
  exit với `bind failed on port 5760` và MAVProxy retry vô hạn, trông như lỗi
  telemetry thay vì process cũ.
- **`world_name` phải khớp** `<world name="...">` trong SDF — bridge Gazebo
  subscribe `/world/<world_name>/model/<robot_name>/joint_state`. Nếu sai,
  joint state không tới ⇒ RSP không xuất transform `base_link → rotor_0` ⇒
  RViz báo `No transform from [rotor_0] to [odom]`.

## 10. Luồng lỗi và triệu chứng

| Triệu chứng | Nguyên nhân | Cách xử lý |
| --- | --- | --- |
| RViz: `No transform from [rotor_0] to [odom]` | `world_name` không khớp SDF ⇒ không có joint state | truyền `world_name` đúng, ví dụ `uwb_runway` |
| `pozyx_node` chết với `PackageNotFoundError` | build `uwb_pozyx_bridge` lỗi, mất package metadata | `colcon build --packages-up-to uwb_pozyx_bridge` |
| MAVProxy lặp `link 1 down` | SITL cũ còn chiếm port 5760 | `pkill -f arducopter; pkill -f mavproxy.py` |
| EKF không fuse beacon, cảnh báo alignment | `position_error` sai đơn vị | gửi sai số theo **mét** (`position_accuracy_m`) |
| EKF không lấy độ cao từ beacon | `EK3_SRC1_POSZ` bị đặt sai | đặt `EK3_SRC1_POSZ 1` (BARO) |
| Fix 3D bị từ chối | EKF cần ~200 measurement liên tiếp | đợi, và đảm bảo range được gửi liên tục ở 20 Hz |
| LOS flag bị mất | encode LOS thành quaternion roll | ghi thẳng `orientation.x` |
| Drone bay lệch quỹ đạo, dễ lật | wire frame bị hoán đổi trục ngang | dùng `enu_to_wire_mm()`: byte `x` = North, byte `y` = East |
| `Arm: Need Position Estimate` | EKF mất beacon alignment sau khi SITL restart | restart lại toàn bộ stack, chờ `is using range beacons` |

## 11. Diễn giải protocol trong Python

`pozyx_protocol.py` encode **millimetre theo convention Pozyx** (`x` = North,
`y` = East). Việc đổi từ Gazebo ENU sang convention đó do `enu_to_wire_mm()`
đảm nhiệm — đây là **điểm duy nhất** được phép đổi frame trong codebase.

Các frame được nối tiếp rồi gửi **một lần write** qua socket, vì ArduPilot
đọc theo luồng byte chứ không cần ranh giới message.

```python
n_mm, e_mm, u_mm = enu_to_wire_mm(*anchor.position)
frames += encode_beacon_config(anchor.id, len(self._anchors), n_mm, e_mm, u_mm)
frames += encode_beacon_distance(...)
frames += encode_vehicle_position(n_mm, e_mm, u_mm, accuracy_m)
self._sender.send(bytes(frames))
```

`PozyxSender` cố ý ** tolerant**: nếu socket rớt (SITL restart) thì reconnect
nền ở chu kỳ sau thay vì raise, và frame không gửi được thì **drop** chứ
không queue — với EKF thì dữ liệu cũ còn tệ hơn không có dữ liệu.
