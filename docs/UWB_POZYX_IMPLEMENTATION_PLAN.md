# Plan triển khai pipeline UWB → Pozyx → ArduPilot SITL

> Ngày lập: 2026-10-27
> Tài liệu đánh giá chi tiết: [`UWB_POZYX_PIPELINE.md`](UWB_POZYX_PIPELINE.md)
> Commit ArduPilot: `55ae8d2825` (ArduCopter-stable-2)

---

## 0. Quyết định đã chốt

| # | Câu hỏi | Quyết định |
|---|---|---|
| Q1 | Vị trí spawn drone trong Gazebo | **`(5, 5, 0.1)`** — tâm 4 anchor (hình học tốt nhất cho trilateration) |
| Q2 | Model drone | **Dùng `x500_with_ardupilot` có sẵn** ở `src/ardupilot_gazebo/models/`; chỉ **port plugin** từ nhánh cũ, không giữ `x500_with_uwb` |
| Q3 | Unit bug `position_error` | **Giữ workaround phía sender** (gửi theo **mét**), **không sửa ArduPilot**; ghi rõ vào docs |

### Q1 — Hệ quả của spawn tại `(5, 5, 0.1)`

Beacon origin = vị trí spawn. Anchor trong world ở `(0,0,2) (10,0,2) (10,10,2) (0,10,2)`.

Offset tương đối tới beacon origin (đơn vị mm, ENU, z = **up**):

| Anchor | World (m) | Tương đối origin (mm) | Gửi cho ArduPilot |
|---|---|---|---|
| 0 | (0, 0, 2) | (-5000, -5000, +1900) | x=-5000, y=-5000, z=+1900 |
| 1 | (10, 0, 2) | (+5000, -5000, +1900) | x=+5000, y=-5000, z=+1900 |
| 2 | (10, 10, 2) | (+5000, +5000, +1900) | x=+5000, y=+5000, z=+1900 |
| 3 | (0, 10, 2) | (-5000, +5000, +1900) | x=-5000, y=+5000, z=+1900 |

> ⚠️ Pozyx backend tự đảo ENU→NED: `beacon_pos(x, y, -z)`.
> Vì anchor **cao hơn** origin 1.9 m, `z` gửi đi là **+1900** (không phải -1900).

---

## 1. Trạng thái hiện tại

### Đã xong ✅

| Hạng mục | Ghi chú |
|---|---|
| `uwb_pozyx_bridge/pozyx_protocol.py` | Encode 3 message type, 13 unit test |
| `uwb_pozyx_bridge/trilateration.py` | WLS solver, 11 unit test (tách khỏi ROS) |
| `config/beacon.parm` | Param ArduPilot cho Pozyx backend |
| `tools/fake_pozyx_sender.py` | Fake Arduino (giả lập IndoorLoiter.ino) |
| `tools/sitl_beacon_monitor.py` | MAVLink monitor + đọc param |
| `tools/ekf_diag.py` | Chẩn đoán trạng thái EKF |
| `tools/run_tier2_verify.sh` | Script orchestration Tier 2 |
| **Tier 2 verify** | **PASS** — `EKF3 IMU0 is using range beacons` |

Test hiện tại: **24/24 pass**.

### Chưa làm ❌

| Hạng mục | Phase |
|---|---|
| `launch/*.launch.py` | 5 |
| Nối Gazebo + node + SITL | 5 |
| Flight test | 6 |

### Đã làm trong Phase 4 ✅

| Hạng mục | Ghi chú |
|---|---|
| `src/uwb_gz_plugin/**` | Port từ `origin/feat/uwb-odometry`, **không** giữ `x500_with_uwb` (Q2) |
| `src/UwbSystem.cc` | 4 sửa đổi (tìm link lồng nhau, world pose chain, LOS encoding) |
| `src/ros_gz_bridge_node.cc` | Tách callback ranging / tag pose |
| `worlds/uwb_runway.sdf` | `<anchor>` tường minh + `PosePublisher` system |
| `tools/run_phase4_verify.sh` | Script verify Gazebo side |

**Kết quả Phase 4: PASS**
- `colcon build` ✅
- Plugin load, 3 topic `/uwb/*` ✅
- Anchor đúng vị trí ✅
- Tag spawn đúng `(5, 5, 0.104)` ✅
- Ranging đúng toán học: **7.30–7.42 m** vs kỳ vọng **7.34 m** ✅
- Bridge publish `/uwb/ranging_ros` với encoding chuẩn ✅


### Đã làm trong Phase 3 ✅

| Hạng mục | Ghi chú |
|---|---|
| `package.xml`, `setup.py`, `setup.cfg`, `pytest.ini` | ament_python, entry point `pozyx_node` |
| `uwb_pozyx_bridge/pozyx_sender.py` | TCP transport, reconnect, drop frame khi buffer đầy |
| `uwb_pozyx_bridge/pozyx_node.py` | Node ROS chính, đầy đủ params |
| `uwb_pozyx_bridge/uwb_decoding.py` | Decode ranging + tính anchor offset (test được, không phụ thuộc ROS) |
| `test/test_pozyx_node.py` | 11 test mới (tổng 35) |
| `tools/fake_ranging_source.py` | Fake ranging source (rclpy), thay Gazebo khi test node |
| `tools/run_tier3_verify.sh` | Orchestration SITL + node + fake source |

**Kết quả Phase 3: PASS**
- `colcon build` ✅ · `colcon test` ✅ **35/35**
- Integration: SITL nhận data thật, log:
  `EKF3 IMU0 origin set` → `EKF3 IMU0 is using range beacons` →
  `EKF3 IMU0 initial pos NE = -0.0,0.0 (m)` (đúng tâm anchors) →
  `initial beacon pos D offset = 0.1 (m)` (đúng spawn z = 0.1)

#### Phát hiện mới trong Phase 3

1. **`EK3_SRC1_POSZ` phải là BARO (1), không phải BEACON (4)** — với anchor
   coplanar, Z có ambiguity mirror nên EKF không fuse được beacon.
   Xem R2b trong [`UWB_POZYX_PIPELINE.md`](UWB_POZYX_PIPELINE.md).
2. **Workaround `position_error`** đã được xác nhận lại: gửi `0` (mét) →
   EKF align. Gửi `50` (mm theo sketch) → không align. Xem R2.


### Môi trường đã xác minh

- ROS 2 Humble (`/opt/ros/humble`)
- Gazebo `gz sim` 8.11.0 — có đủ `gz-sim8`, `gz-plugin2`, `gz-transport13`, `gz-msgs10`, `gz-common5`, `gz-math7`
- `ros_gz` 0.244.23
- Model: `src/ardupilot_gazebo/models/x500_with_ardupilot/model.sdf`
- World có sẵn: `src/ardupilot_gz/ardupilot_gz_gazebo/worlds/runway.sdf`
- Param có sẵn: `src/ardupilot_gazebo/config/gazebo-x500.parm`

---

## 2. Nguyên tắc làm việc

> **Sau mỗi Phase → DỪNG LẠI, chạy test, báo cáo kết quả, chờ user xác nhận mới sang Phase tiếp theo.**

Không tự ý chuyển Phase. Nếu test fail → sửa, không đi tiếp.

Mỗi Phase có **exit criteria** rõ ràng để user tự verify độc lập.

---

## Phase 3 — Đóng gói package + Node Python

**Mục tiêu:** package ROS 2 hoàn chỉnh, node đọc ranging và gửi Pozyx protocol qua TCP.
**Ước lượng:** 0.5 ngày · **Rủi ro:** Thấp

### 3.1 Hạ tầng package

| File | Nội dung |
|---|---|
| `package.xml` | ament_cmake_python; deps: rclcpp, geometry_msgs, std_msgs, python3-numpy |
| `setup.py` | entry_points: `pozyx_node = uwb_pozyx_bridge.pozyx_node:main` |
| `setup.cfg` | script_dir |
| `resource/uwb_pozyx_bridge` | marker file |

### 3.2 `pozyx_sender.py` — transport

- TCP client, reconnect với backoff
- `send(bytes)` non-blocking-ish, drop frame nếu buffer đầy (không block ROS executor)
- Thread riêng cho write, hoặc ghi thẳng (frame nhỏ ~40 byte, 20 Hz → OK)

### 3.3 `pozyx_node.py` — node chính

**Params** (tất cả có default hợp lý):

| Param | Default | Ý nghĩa |
|---|---|---|
| `input_topic` | `/uwb/ranging_ros` | topic Pose từ bridge node C++ |
| `output_pose_topic` | `/uwb/pose` | debug: vị trí trilateration |
| `serial_host` | `127.0.0.1` | SITL serial endpoint |
| `serial_port` | `5761` | TCP port |
| `anchor_count` | `4` | |
| `anchor_positions` | `[-5,-5,1.9, 5,-5,1.9, 5,5,1.9, -5,5,1.9]` | **mm, tương đối origin** |
| `beacon_origin` | `[0,0,0]` | offset (mm) của origin so với world — dùng để trừ khi tính anchor |
| `assume_constant_z` | `True` | 2D (x,y) — khớp `IndoorLoiter.ino` |
| `tag_altitude` | `0.0` | z của tag so với origin (mm) |
| `los_std` / `nlos_soft_std` / `nlos_hard_std` | `0.10 / 0.30 / 0.60` | m |
| `min_anchors` | `3` | |
| `position_accuracy` | `0.05` | **mét** — workaround R2 |
| `send_rate` | `20.0` | Hz |
| `max_range` | `50.0` | m |

**Luồng:**

1. Subscribe Pose (QoS **best_effort**, khớp bridge C++)
2. Decode → `RangeMeasurement`, buffer theo `anchor_id`
3. Timer 20 Hz:
   - `solve_position()` → vị trí ENU (m) tương đối beacon origin
   - `encode_beacon_distance()` × 4 (dùng last-known nếu ranging cũ)
   - `encode_vehicle_position()` (x, y, z mm + accuracy **mét**)
   - gửi qua TCP
4. Gửi `encode_beacon_config()` × 4 **mỗi chu kỳ 1 s** (để ArduPilot không mất config)
5. Publish vị trí lên `/uwb/pose` để debug

**Bắt buộc (R1):** gửi distance **liên tục** 20 Hz, kể cả khi không có ranging mới (giữ last-known).
EKF cần ~200 measurement liên tục mới align.

### 3.4 Test cho Phase 3

**Unit test** (không cần ROS):
- `test_pozyx_node.py`: decode Pose → RangeMeasurement đúng
- Anchor offset tính đúng từ world position (test Q1: `(0,0,2)` → `(-5000,-5000,+1900)`)
- Frame gửi ra đúng byte layout (integration với `pozyx_protocol`)

**Integration test (không cần Gazebo):**
- `tools/run_tier3_verify.sh`: chạy SITL + node với **fake ranging source** (Python script giả lập topic `/uwb/ranging_ros` bằng rclpy) → verify EKF "is using range beacons"
- Đây là bước quan trọng: tách rủi ro node khỏi rủi ro Gazebo

### Exit criteria Phase 3

- [x] `colcon build` thành công, không lỗi
- [x] `pytest` pass (≥ 30 test) — **35/35**
- [x] Script `run_tier3_verify.sh` chạy: SITL nhận data, EKF log `is using range beacons`
- [x] `EKF3 IMU0 origin set` xuất hiện (chứng minh workaround R2 hoạt động qua node thật)

---

## Phase 4 — Port Gazebo plugin

**Mục tiêu:** có `uwb_gz_plugin` build được, world publish ranging đúng.
**Ước lượng:** 0.5 ngày · **Rủi ro:** Thấp

### 4.1 Port từ nhánh cũ

```bash
git checkout origin/feat/uwb-odometry -- src/uwb_gz_plugin/
```

Port **nguyên vẹn**, chỉ sửa tối thiểu:

| File | Sửa gì |
|---|---|
| `src/UwbSystem.cc` | **4 sửa đổi** — xem bảng bên dưới |
| `src/UwbRanging.cc` | Giữ nguyên |
| `src/ros_gz_bridge_node.cc` | **Tách callback** ranging / tag pose (xem bên dưới) |
| `include/uwb_gz_plugin/*.hh` | Thêm khai báo `LocalToWorldPose` |
| `CMakeLists.txt` | Giữ nguyên (đã nhắm đúng GZ 8.11) |
| `models/uwb_anchor/**` | Giữ (để visualize anchor) |
| `models/x500_with_uwb/**` | **BỎ** — dùng `x500_with_ardupilot` có sẵn (Q2) |
| `worlds/uwb_runway.sdf` | Sửa (xem 4.2) |
| `worlds/uwb_plugin_test.sdf` | Sửa tương tự |

#### 4.1.1 Bốn sửa đổi trong `UwbSystem.cc`

| # | Vấn đề | Sửa |
|---|---|---|
| 1 | `FindLinkEntity` chỉ tìm link **trực tiếp** của model. Với `x500_with_ardupilot` (include `x500_base` → include `x500_base` lồng nhau), `base_link` nằm ở model **con** nên không tìm thấy → ranging không publish | Đệ quy vào child model |
| 2 | Đọc **local pose** của link (luôn = 0,0,0) thay vì world pose. Gazebo không tạo `WorldPose` component trừ khi có `PosePublisher` system | Thêm `LocalToWorldPose()`: dồn `Pose` qua chuỗi parent (`_ecm.ParentEntity`) |
| 3 | `<anchor>` trong SDF phải có `<pos>` là **child element**, không phải attribute | Sửa world (4.2) |
| 4 | LOS code đưa vào `Quaterniond(los, 0, 0)` → `orientation.x = sin(los/2)` = 0.479, Python `round()` về 0 → **mất cờ NLOS** | Đặt thẳng `orientation.x = los`, `orientation.w = 1.0` |

#### 4.1.2 Sửa trong `ros_gz_bridge_node.cc`

`OnPose` dùng chung cho cả ranging và tag pose → tag pose **lẫn** vào topic ranging
(đọc được `position.z = 0.11` thay vì anchor id). Tách thành `OnRanging` và
`OnTagPose`.

### 4.2 `uwb_runway.sdf` — sửa cho Q1

Dùng **`<anchor>` tường minh** (plugin hỗ trợ, ưu tiên hơn include model):

```xml
<plugin filename="uwb_gz_plugin" name="uwb_gz_plugin::systems::UwbSystem">
  <update_rate>20.0</update_rate>
  <tag_id>0</tag_id>
  <tag_model_name>x500</tag_model_name>
  <tag_link>base_link</tag_link>
  <all_los>true</all_los>
  <max_range>50.0</max_range>
  <los_std>0.05</los_std>

  <!-- <pos> là CHILD ELEMENT, không phải attribute -->
  <anchor name="uwb_anchor0"><pos>0 0 2.0</pos></anchor>
  <anchor name="uwb_anchor1"><pos>10 0 2.0</pos></anchor>
  <anchor name="uwb_anchor2"><pos>10 10 2.0</pos></anchor>
  <anchor name="uwb_anchor3"><pos>0 10 2.0</pos></anchor>
</plugin>
```

- `spherical_coordinates` giữ nguyên (`-35.3632621, 149.1652374`) → khớp SITL home
- Không include drone (spawn runtime)
- Thêm `gz-sim-pose-publisher-system` (xem 4.1.1 #2)

### 4.3 ⚠️ Spawn pose: `<pose>` phải nằm trên `<include>`

Đây là điểm dễ sai nhất. Ba cách đặt pose đều **không** cho kết quả mong muốn:

| Cách | Kết quả |
|---|---|
| `<model><pose>5 5 0.1</pose><include/></model>` | ❌ model ở (0,0,0.1) — `x500_base` có `<pose>0 0 .025</pose>` riêng ghi đè |
| `EntityFactory` request field `pose:` | ❌ bị bỏ qua |
| `ros2 run ros_gz_sim create -pose 5,5,0.1` | ❌ tạo model ở (0,0,0) |
| **`<model><include><pose>5 5 0.1</pose></include></model>`** | ✅ **(5, 5, 0.104)** |

Cách đúng (đã verify):

```bash
gz service -s /world/uwb_runway/create \
  --reqtype gz.msgs.EntityFactory --reptype gz.msgs.Boolean \
  --req 'name: "x500", sdf: "<sdf version=\"1.9\"><model name=\"x500\"><include><uri>model://x500_base</uri><pose>5 5 0.1 0 0 0</pose></include></model></sdf>"'
```

> Tên model **phải** là `x500` (khớp `<tag_model_name>`), nếu khác thì plugin
> không tìm thấy tag và ranging rỗng.

### 4.4 Test cho Phase 4

- [x] `colcon build` plugin thành công
- [x] `gz sim -r -s uwb_runway.sdf` chạy, plugin load không lỗi
- [x] Log GZ hiện: `Explicit anchor 'uwb_anchor0' at (0, 0, 2)` × 4
- [x] `gz topic -l` thấy `/uwb/ranging/tag`
- [x] Bridge node publish `/uwb/ranging_ros`
- [x] Giá trị decode đúng: `position.x` = range (m), `position.z` = anchor_id
- [x] **Range khớp toán học**: tag ở tâm → range = √(5² + 5² + 1.9²) = **7.34 m**
      (đo được 7.30–7.42 m, chênh do nhiễu `los_std = 0.05`)

Script: `src/uwb_gz_plugin/tools/run_phase4_verify.sh`

---

## Phase 5 — Nối Gazebo + Node + SITL (RỦI RO CAO NHẤT)

**Mục tiêu:** drone bay trong Gazebo, EKF3 dùng UWB làm nguồn vị trí.
**Ước lượng:** 1–1.5 ngày · **Rủi ro:** Cao (frame mapping)

> Phase này được tách thành **5 step nhỏ**. Mỗi step có **gate verify riêng**
> và phải **DỪNG LẠI** sau khi verify — không được sang step kế tiếp khi
> gate chưa xanh.

| Step | Nội dung | Rủi ro | Cổng |
|---|---|---|---|
| **5.1** | Gazebo → ROS 2 (bridge) | Thấp | Ranging đúng, 4 anchor, 20 Hz |
| **5.2** | ROS 2 → beacon-relative → Pozyx frame | Trung bình | Frame mapping đúng, khớp toán học |
| **5.3** | Pozyx frame → ArduPilot SITL | Trung bình | EKF "origin set" + "is using range beacons" |
| **5.4** | Ba nguồn vị trí khớp nhau | Cao | Gazebo truth ≈ EKF ≈ trilateration |
| **5.5** | Launch file tổng hợp | Thấp | `ros2 launch` một lệnh chạy trọn vẹn |

**Nguyên tắc:** mỗi step giả định step trước đã xanh, và **chỉ thêm đúng một
thứ**. Nếu gate đỏ → sửa step hiện tại, không đi tiếp.

**Trạng thái: cả 5 step đã PASS ✅**

| Step | Kết quả | Sai số chính |
|---|---|---|
| 5.1 | PASS | 19.9 Hz/anchor, range 7.30–7.31 m (kỳ vọng 7.34) |
| 5.2 | PASS | trilateration sai **0.027 m**; offset `(±5000, ±5000, +1900)` mm |
| 5.3 | PASS | `frames_sent=1201 dropped=0`, EKF flags `0x033f` healthy |
| 5.4 | PASS | EKF vs truth **0.189 m**, drift **0.004 m** |
| 5.5 | PASS | EKF vs truth **0.069 m**, drift **0.008 m** |

---

### Step 5.1 — Gazebo → ROS 2 (bridge)

**Mục tiêu:** xác nhận dữ liệu ranging từ Gazebo đi qua bridge vào ROS 2 với
encoding đúng, **chưa** đụng tới ArduPilot.

**Thay đổi:** không sửa code. Chỉ chạy Gazebo + bridge node.

**Cách chạy:**
```bash
# terminal 1
gz sim -r -s uwb_runway.sdf
# terminal 2
ros2 run ros_gz_sim create -topic /world/uwb_runway -name x500 \
  -file model://x500_base   # spawn tại trọng tâm, xem Step 5.2
ros2 run uwb_gz_plugin uwb_gz_to_ros_bridge
```

**Gate 5.1 — phải xanh trước khi sang 5.2:**

- [x] `ros2 topic list` thấy `/uwb/ranging_ros` và `/uwb/tag_pose_ros`
- [x] Tần suất **19.9 Hz/anchor** (79.6 Hz tổng cho 4 anchor) — plugin publish
      1 message/anchor/cycle
- [x] Mỗi message: `position.x` = range (7.30–7.31 m), `position.z` = anchor_id
      (0,1,2,3), `position.y` = RSS (−67.3 dBm), `orientation.x` = LOS (0)
- [x] `/uwb/tag_pose_ros` = (5.000, 5.000) — vị trí thật của drone
- [x] **Không** có message lẫn từ `tag_pose` vào `ranging_ros`
      (range spread chỉ 0.269 m)

**Công cụ gate:** `src/uwb_gz_plugin/tools/check_ranging_stream.py`
(chạy tự động, exit code khác 0 nếu fail)
**Script:** `src/uwb_gz_plugin/tools/run_step51_verify.sh`

---

### Step 5.2 — ROS 2 → beacon-relative → Pozyx frame

**Mục tiêu:** kiểm tra frame mapping ở **phía node**, chưa gửi gì đi.

**Các hệ toạ độ phải khớp:**

```
Gazebo world (ENU, m)     ← anchor ở đây, drone spawn ở đây
        ↓ trừ beacon origin (5,5,0.1)
Beacon-relative (ENU, m) ← anchor offset, tag position
        ↓ × 1000
Pozyx wire (mm, ENU)      ← những gì node gửi
        ↓ AP_Beacon_Pozyx tự đảo
ArduPilot (NED, m)        ← EKF làm việc ở đây
```

**Thay đổi:** thêm công cụ `tools/inspect_frames.py` — subscribe `/uwb/ranging_ros`,
in ra:
- range thô từ Gazebo
- anchor offset tương đối beacon origin (mm)
- vị trí tag sau trilateration (m và mm)
- frame Pozyx sẽ gửi (byte-level, hex)

**Checklist frame mapping:**
- [x] `anchor_offsets_mm` = anchor world − (5, 5, 0.1) × 1000
      → `(±5000, ±5000, +1900)`
- [x] `z` **dương** khi anchor **cao hơn** origin (anchor ở z=2.0, origin z=0.1)
- [x] Tag position: `x,y` từ trilateration, `z` = `tag_altitude` (0)
- [x] `BCN_ORIENT_YAW 0` (Gazebo world không xoay)

**Gate 5.2:**
- [x] Vị trí tag sau trilateration ≈ `(0, 0)` m khi drone ở tâm
      (vì beacon origin **chính là** điểm spawn)
- [x] Sai số trilateration vs vị trí thật = **0.027 m** (< 0.3 m) ✅
- [x] `anchor_offsets_mm` in ra đúng `(±5000, ±5000, +1900)`, khớp default
      của node
- [x] Frame Pozyx dựng lại **giống hệt** byte mà node sẽ gửi
- [x] `position_error` trong frame = **0** (mét, workaround R2)

**Kết quả thực tế:**

```
=== anchor offsets (beacon-relative, mm) ===
  anchor 0: (-5000, -5000, +1900)
  anchor 1: (+5000, -5000, +1900)
  anchor 2: (+5000, +5000, +1900)
  anchor 3: (-5000, +5000, +1900)
  -> matches the node's default anchor_offsets_mm

=== tag position ===
  trilateration (beacon-relative, m): (0.019, -0.019, 0.000)
  Gazebo truth   (beacon-relative, m): (0.000, 0.000)
  horizontal error: 0.027 m

=== Pozyx frames that would be sent ===
  msg 0x02  payload 00 04 78 ec ff ff 78 ec ff ff 6c 07 00 00   ← config 0
  msg 0x02  payload 01 04 88 13 00 00 78 ec ff ff 6c 07 00 00   ← config 1
  msg 0x02  payload 02 04 88 13 00 00 88 13 00 00 6c 07 00 00   ← config 2
  msg 0x02  payload 03 04 78 ec ff ff 88 13 00 00 6c 07 00 00   ← config 3
  msg 0x03  payload 00 b3 1c 00 00                              ← dist 0 = 7347 mm
  msg 0x03  payload 01 87 1c 00 00                              ← dist 1 = 7319 mm
  msg 0x03  payload 02 b4 1c 00 00                              ← dist 2 = 7348 mm
  msg 0x03  payload 03 bb 1c 00 00                              ← dist 3 = 7355 mm
  msg 0x04  payload 13 00 00 00 ed ff ff ff 00 00 00 00 00 00   ← pos, err=0
```

**Công cụ gate:** `src/uwb_pozyx_bridge/tools/inspect_frames.py`

---

### Step 5.3 — Pozyx frame → ArduPilot SITL

**Mục tiêu:** SITL thật nhận frame từ node và EKF3 fuse beacon.

**Thay đổi:** chạy SITL với `config/beacon.parm` + node gửi thật. Chưa cần
drone bay — vẫn dùng Gazebo làm nguồn ranging.

**Cách chạy:**
```bash
arducopter --model x500 --defaults config/beacon.parm --serial1=tcp:5761:wait
python3 tools/sitl_beacon_monitor.py --set-params
ros2 run uwb_pozyx_bridge pozyx_node
```

**Gate 5.3:**
- [x] SITL log: `EKF3 IMU0 origin set`
- [x] SITL log: `EKF3 IMU0 is using range beacons`
- [x] SITL log: `EKF3 IMU0 initial pos NE = -0.0,0.0 (m)`
- [x] SITL log: `EKF3 IMU0 initial beacon pos D offset = 0.1 (m)`
- [x] `ekf_status` flags = `0x033f` — có `PosHRel` + `PosHAbs`,
      **không còn** `UNINITIALIZED`
- [x] Node log: `frames_sent=1201 dropped=0` (không rơi frame nào)

**Kết quả thực tế:**

```
############ GATE CHECK ############
  [OK ] EKF3 origin set
  [OK ] EKF3 is using range beacons
  [OK ] initial pos NE is ~0
  [OK ] beacon pos D offset is 0.1
  [OK ] node frames_sent=1201 dropped=0
  [OK ] final EKF flags 0x033f (healthy)

STEP 5.3 GATE: PASSED
```

> **Hai lưu ý khi đọc log:**
> 1. `EKF_UNINITIALIZED` = **1024** (bit 10), không phải bit 8.
> 2. Các dòng `is using range beacons` / `initial pos NE` xuất hiện **vào
>    cuối** run, nên gate phải retry thay vì đọc log một lần.

> Nếu `origin set` có mà `is using range beacons` **không** có →
> kiểm tra `EK3_SRC1_POSZ` = 1 (Baro), xem R2b.

**Công cụ gate:** `src/uwb_pozyx_bridge/tools/run_step53_verify.sh`

---

### Step 5.4 — Ba nguồn vị trí khớp nhau

**Mục tiêu:** xác nhận EKF đang ở đúng chỗ, không chỉ "đang chạy".

**Thay đổi:** thêm `tools/compare_positions.py` — đọc song song:
- `/uwb/pose` (trilateration của node)
- Gazebo truth (gz topic pose của model `x500`)
- `LOCAL_POSITION_NED` (EKF) qua MAVLink

In bảng + sai số theo thời gian.

**Gate 5.4:**
- [x] |EKF − Gazebo truth| = **0.189 m** (< 0.5 m) ✅
- [x] |EKF − trilateration| = **0.180 m** (< 0.3 m) ✅
- [x] |trilateration − Gazebo truth| = **0.092 m** (< 0.3 m) ✅
- [x] Sai số **ổn định**: drift = **0.004 m**, worst error = 0.197 m

**Kết quả thực tế:**

```
=== sample counts ===
  truth (Gazebo)  : 122
  trilateration   : 122
  ekf             : 138

=== latest positions (beacon-relative ENU metres) ===
  truth       : (+0.000, +0.000, +0.093)
  trilateration: (+0.086, +0.034, +0.000)
  ekf         : (+0.124, -0.142, -0.082)

=== horizontal errors ===
  ekf vs truth                 = 0.189 m
  ekf vs trilateration         = 0.180 m
  trilateration vs truth       = 0.092 m

=== stability ===
  drift over the run: 0.004 m
  worst ekf vs truth: 0.197 m

STEP 5.4 GATE: PASSED
```

> **Lưu ý về trục z**: cổng so sánh chỉ kiểm tra **mặt phẳng ngang**. Trục
> đứng có ambiguity vì 4 anchor cùng phẳng (xem R2b), nên EKF dùng Baro cho
> z. Giá trị z chỉ mang tính chỉ báo.

**Công cụ gate:** `src/uwb_pozyx_bridge/tools/compare_positions.py`
**Script:** `src/uwb_pozyx_bridge/tools/run_step54_verify.sh`

> Đây là gate quan trọng nhất của Phase 5. Nếu EKF "dùng beacon" nhưng sai
> chỗ, thì đó chính là lỗi frame mapping — phải quay lại Step 5.2.

---

### Step 5.5 — Launch file tổng hợp

**Mục tiêu:** gom mọi thứ vào một `ros2 launch` để chạy trọn vẹn.

**Thay đổi:** `src/uwb_pozyx_bridge/launch/uwb_pozyx_sitl.launch.py`:
- Launch world `uwb_runway.sdf` (GZ Sim)
- Spawn `x500_with_ardupilot` tại `(5, 5, 0.1)`
  (> dùng `x500.launch.py` của `ardupilot_gz_bringup` với args `x`/`y`/`z`)
- Chạy `uwb_gz_to_ros_bridge`
- Chạy `pozyx_node`
- (SITL do `x500.launch.py` tự khởi động)

**Gate 5.5:**
- [x] `ros2 launch uwb_pozyx_bridge uwb_pozyx_sitl.launch.py` chạy trọn vẹn
- [x] Drone spawn đúng `(5, 5, 0.1)`
- [x] EKF "is using range beacons"
- [x] Vị trí khớp truth trong 0.5 m → thực tế **0.069 m**
- [x] Không cần chạy tay bất kỳ process nào khác

**Kết quả thực tế:**

```
=== latest positions (beacon-relative ENU metres) ===
  truth       : (+0.000, +0.000, +0.093)
  trilateration: (+0.011, -0.015, +0.000)
  ekf         : (+0.066, -0.019, -0.109)

=== horizontal errors ===
  ekf vs truth                 = 0.069 m
  ekf vs trilateration         = 0.055 m
  trilateration vs truth       = 0.018 m

=== stability ===
  drift over the run: 0.008 m
  worst ekf vs truth: 0.085 m

STEP 5.4 GATE: PASSED
```

**Bốn lỗi đã gặp khi làm Step 5.5** (đều là lỗi launch, không phải lỗi pipeline):

1. **`gz sim -s <name>` không tìm thấy world.** Phải dùng **absolute path**
   tới file `.sdf`; `GZ_SIM_RESOURCE_PATH` không được quét đệ quy.
2. **`GZ_SIM_SYSTEM_PLUGIN_PATH` phải trỏ tới thư mục `lib/`.** Trỏ nhầm
   sang `share/` làm Gazebo không nạp được UWB plugin (không có ranging nào).
3. **Cú pháp serial của SITL là `tcp:<port>:wait`, KHÔNG có host.**
   `tcp:127.0.0.1:5761:wait` bị parse thành `port=127` → SITL bind
   `base_port + 127 = 5887` thay vì 5761.
4. **SITL chỉ phục vụ MAVLink cho 1 client TCP.** Monitor phải nhả port 5760
   trước khi công cụ so sánh kết nối → thêm launch arg `monitor_duration`.

> Ngoài ra: node phải khởi động **sau** khi SITL bind SERIAL1 (TimerAction
> 14 s), nếu không sẽ drop ~40 frame đầu. Và monitor phải chạy `python3 -u`
> để stdout không bị buffer.

**Công cụ gate:** `src/uwb_pozyx_bridge/tools/run_step55_verify.sh`

---

## Phase 6 — Verify + Flight test

**Mục tiêu:** bay thật, giữ vị trí ổn định.
**Ước lượng:** 0.5 ngày · **Rủi ro:** Trung bình

### 6.1 Pre-arm

- [ ] Arm được (beacon OK, không cần GPS)
- [ ] Kiểm tra `ARMING_CHECK` — autotest tắt bit 3 (GPS). Cần không?

### 6.2 Bay

- [ ] Takeoff ổn định
- [ ] Position hold giữ vị trí, sai số < 0.5 m
- [ ] Không drift khi hover 60 s
- [ ] Landing ổn định

### 6.3 Logging

- [ ] Đọc `BEA` packet (LOG_BEACON_MSG) trong log: `dist0..3`, `posx/y/z` khớp
- [ ] So sánh EKF innovations: `ekf_status` innovation ratios < 1

---

## 3. Tổng hợp rủi ro

| # | Rủi ro | Mức | Xử lý |
|---|---|---|---|
| **R2** | **Unit bug upstream ArduPilot**: Pozyx truyền `position_error` (mm) thô; EKF so sánh `< 1.0` (m) → EKF **không bao giờ set origin** | 🔴 Cao | **Workaround đã verify thực nghiệm**: node gửi accuracy **mét** (0.05). `pos_err=0` → "origin set" xuất hiện; `pos_err=50` → không. **Không patch ArduPilot** (Q3). Ghi rõ docs. |
| **R3** | Beacon origin lệch → EKF sai vị trí | 🔴 Cao | `BCN_*` = SITL home; anchor offset tính từ spawn `(5,5,0.1)` |
| **R1** | EKF cần ~200 measurement **liên tục** | 🟡 TB | Node gửi distance liên tục 20 Hz, giữ last-known |
| **R7** | 4 anchor coplanar → **ambiguity chiều z** | 🟡 TB | **Đã verify bằng test**: tag z=1 vs z=3 cho range giống hệt. Node chạy **2D**, khớp `IndoorLoiter.ino` (z=0) |
| **R6** | `ros_gz_bridge` factory-lookup failure với `gz.msgs.Pose` | 🟢 Thấp | Giữ `ros_gz_bridge_node.cc` riêng |
| R8 | Thứ tự góc anchor khác Pozyx | 🟢 Thấp | Không ảnh hưởng — EKF dùng `beacon_ID` làm chỉ số |

---

## 4. Ước lượng tổng

| Phase | Công sức | Rủi ro | Còn lại |
|---|---|---|---|
| 3 — package + node | 0.5 ngày | Thấp | 0.5 |
| 4 — port plugin | 0.5 ngày | Thấp | 0.5 |
| 5 — nối + verify | 1–1.5 ngày | **Cao** | 1–1.5 |
| 6 — flight test | 0.5 ngày | TB | 0.5 |
| **Tổng** | **2.5–3 ngày** | | |

---

## 5. Log các lần test

| Ngày | Phase | Kết quả | Ghi chú |
|---|---|---|---|
| 2026-10-26 | Tier 2 | ✅ PASS | `EKF3 IMU0 is using range beacons`; phát hiện unit bug R2 |
| 2026-10-26 | Tier 1 | ✅ PASS | 24/24 unit test |
| 2026-10-27 | **Phase 3** | ✅ **PASS** | 35/35 test; `colcon build` OK; SITL + node + fake source → "is using range beacons", "initial pos NE = -0.0,0.0" |
| 2026-10-27 | **Phase 4** | ✅ **PASS** | Plugin port + build OK; ranging 7.30–7.42 m (kỳ vọng 7.34 m); 4 bug fix (xem 4.1.1) |
| | | | |
