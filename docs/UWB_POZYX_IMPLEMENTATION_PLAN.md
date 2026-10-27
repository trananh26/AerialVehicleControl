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
| `src/uwb_gz_plugin/**` (port từ nhánh cũ) | 4 |
| `launch/*.launch.py` | 5 |
| Nối Gazebo + node + SITL | 5 |
| Flight test | 6 |

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
| `src/UwbSystem.cc` | Giữ nguyên |
| `src/UwbRanging.cc` | Giữ nguyên |
| `src/ros_gz_bridge_node.cc` | Giữ nguyên (bắt buộc — `ros_gz_bridge` factory-lookup failure) |
| `include/uwb_gz_plugin/*.hh` | Giữ nguyên |
| `CMakeLists.txt` | Kiểm tra deps khớp GZ 8.11 (đã xác minh) |
| `models/uwb_anchor/**` | Giữ (để visualize anchor) |
| `models/x500_with_uwb/**` | **BỎ** — dùng `x500_with_ardupilot` có sẵn (Q2) |
| `worlds/uwb_runway.sdf` | Sửa (xem 4.2) |
| `worlds/uwb_plugin_test.sdf` | Sửa tương tự |

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
  <anchor_prefix>uwb_anchor</anchor_prefix>
  <anchor name="uwb_anchor0" pos="0 0 2"/>
  <anchor name="uwb_anchor1" pos="10 0 2"/>
  <anchor name="uwb_anchor2" pos="10 10 2"/>
  <anchor name="uwb_anchor3" pos="0 10 2"/>
</plugin>
```

- `spherical_coordinates` giữ nguyên (`-35.3632621, 149.1652374`) → khớp SITL home
- Không include drone (spawn runtime)

### 4.3 Test cho Phase 4

- [ ] `colcon build` plugin thành công
- [ ] `gz sim -r -s uwb_runway.sdf` chạy, không lỗi load plugin
- [ ] Log GZ hiện: `Explicit anchor 'uwb_anchor0' at (0, 0, 2)` × 4
- [ ] `gz topic -l` thấy `/uwb/ranging/tag`
- [ ] Bridge node publish `/uwb/ranging_ros`, `ros2 topic echo` thấy 4 anchor × 20 Hz
- [ ] Giá trị decode đúng: `position.x` = range (m), `position.z` = anchor_id

---

## Phase 5 — Nối Gazebo + Node + SITL (RỦI RO CAO NHẤT)

**Mục tiêu:** drone bay trong Gazebo, EKF3 dùng UWB làm nguồn vị trí.
**Ước lượng:** 1–1.5 ngày · **Rủi ro:** Cao (frame mapping)

### 5.1 Điểm khó: frame mapping

Ba hệ toạ độ phải khớp:

```
Gazebo world (ENU, m)     ← anchor ở đây, drone spawn ở đây
        ↓ trừ beacon origin (5,5,0.1)
Beacon-relative (ENU, m) ← anchor offset, tag position
        ↓ × 1000
Pozyx wire (mm, ENU)      ← những gì node gửi
        ↓ AP_Beacon_Pozyx tự đảo
ArduPilot (NED, m)        ← EKF làm việc ở đây
```

**Checklist khớp:**
- [ ] `anchor_positions` (mm) = anchor world − (5, 5, 0.1) × 1000
- [ ] `z` dương khi anchor **cao hơn** origin
- [ ] Tag position: `x,y` từ trilateration, `z` = `tag_altitude` (0)
- [ ] `BCN_LATITUDE/LONGITUDE/ALT` = SITL home (đã có trong `beacon.parm`)
- [ ] `BCN_ORIENT_YAW 0` (Gazebo world không xoay)

### 5.2 Launch file

`launch/uwb_pozyx_sitl.launch.py`:
- Launch `uwb_runway.sdf`
- Spawn `x500_with_ardupilot` tại `(-5.5, -5.5, 0.1)`?

> ⚠️ **Lưu ý vị trí spawn**: `x500.launch.py` của `ardupilot_gz_bringup` spawn drone tại vị trí mặc định.
> Cần override để đặt đúng `(5, 5, 0.1)`.

### 5.3 Test cho Phase 5

- [ ] Gazebo chạy, drone spawn đúng `(5, 5, 0.1)` — verify bằng `gz model --list` / topic pose
- [ ] `/uwb/ranging_ros` có data, range hợp lý (anchor cách drone ~7 m khi ở tâm)
- [ ] Node gửi frame, SITL nhận
- [ ] EKF log: `is using range beacons` + `origin set`
- [ ] **Vị trí EKF khớp vị trí thật của drone trong Gazebo** trong ~0.5 m
- [ ] **Vị trí EKF khớp trilateration** (publish `/uwb/pose`) trong ~0.3 m
- [ ] So sánh 2 nguồn: Gazebo truth vs EKF vs trilateration — cả 3 phải gần nhau

### 5.4 Công cụ so sánh

`tools/compare_positions.py`:
- Đọc `/uwb/pose` (trilateration)
- Đọc gz topic pose của drone (Gazebo truth)
- Đọc `LOCAL_POSITION_NED` (EKF) qua MAVLink
- In bảng so sánh + sai số

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
| | | | |
