# Đánh giá pipeline UWB → Pozyx → ArduPilot SITL

> Tài liệu này ghi lại đánh giá kiến trúc, các rủi ro đã xác minh bằng đọc source
> ArduPilot, và roadmap triển khai.
> Ngày lập: 2026-10-26. Commit ArduPilot: `55ae8d2825` (ArduCopter-stable-2).

---

## 1. Quyết định kiến trúc

**Đã chọn:** Gazebo UWB plugin → Node Python (giả lập Arduino Uno) → Pozyx binary
protocol → SITL serial → `AP_Beacon_Pozyx` → EKF3 range-beacon fusion.

**Bỏ nhánh `feat/uwb-odometry`'s approach (MAVLink odometry / VISION_POSITION_ESTIMATE).**

### 1.1 Vì sao nhánh cũ có lỗi

| | Nhánh cũ | Pipeline mới |
|---|---|---|
| Message | MAVLink `VISION_POSITION_ESTIMATE` (qua MAVROS) | Binary Pozyx protocol |
| EKF3 path | `FuseExtNav()` | `FuseRngBcn()` |
| Cần velocity? | **Có** → phải finite-difference → noise | **Không** |
| Position source | Position fix tuyệt đối | Distance từng anchor, EKF tự giải |
| Phụ thuộc MAVROS | Có (fragile) | Không |
| Sửa ArduPilot? | Không | Không |

Nguyên nhân gốc: `FuseExtNav()` cần EKF estimate velocity tốt. Khi
`EK3_SRC1_VELXY=0` (không velocity source) và chỉ có vision position, EKF sẽ:

1. Không có velocity observation → covariance velocity phình to
2. Innovation check fail → reject measurement
3. Hoặc báo "stopped aiding"

`FuseRngBcn()` chỉ cần **distance** — measurement trực tiếp, không cần
differentiate. Kiến trúc beacon tồn tại chính vì robust hơn nhiều cho indoor
không GPS.

Ngoài ra nhánh cũ dùng `ros_gz_bridge` để bridge `gz.msgs.Pose` →
`geometry_msgs/Pose`, mà comment trong code ghi rõ:

> *"ros_gz_bridge 0.244 in this workspace has a factory-lookup failure when
> bridging gz.msgs.Pose -> geometry_msgs/msg/Pose"*

→ Đó là lý do plugin có `ros_gz_bridge_node.cc` riêng. **Phải giữ node đó.**

---

## 2. Kiến trúc tổng thể

```
┌───────────────────────────────────────────────────────────────────────┐
│ Gazebo (uwb_runway.sdf) — UwbSystem plugin, update_rate 20Hz         │
│   anchors: (0,0,2) (10,0,2) (10,10,2) (0,10,2)   [ENU, mét]        │
│   publish /uwb/ranging/tag  (gz::msgs::Pose, 1 msg / anchor)         │
│   encoding: position.x=range, position.y=rss, position.z=anchor_id   │
│             orientation.x=los_code                                    │
└────────────────────────────┬──────────────────────────────────────────┘
                             │  ros_gz_bridge_node (C++, trong plugin)
                             ▼
                    /uwb/ranging/ros  (geometry_msgs/Pose)
                             │
┌────────────────────────────▼──────────────────────────────────────────┐
│ [MỚI] uwb_pozyx_node.py — đóng vai Arduino Uno                      │
│   1. subscribe /uwb/ranging/ros                                      │
│   2. WLS trilateration → vị trí tag (ENU)                            │
│   3. đóng gói 3 message type theo IndoorLoiter.ino                   │
│   4. gửi bytes qua TCP → SITL serial1                               │
└────────────────────────────┬──────────────────────────────────────────┘
                             │  binary: [0x01][id][len][payload][cksum]
                             ▼
┌───────────────────────────────────────────────────────────────────────┐
│ SITL:  arducopter --serial1=tcp:5761:wait                            │
│   SERIAL1_PROTOCOL=13 (SerialProtocol_Beacon)                        │
│   SERIAL1_BAUD=115                                                   │
└────────────────────────────┬──────────────────────────────────────────┘
                             ▼
┌───────────────────────────────────────────────────────────────────────┐
│ AP_Beacon_Pozyx::update() → parse_buffer()                           │
│   set_beacon_position(id, pos)   ← 4 anchor  (ENU mm → NED m)        │
│   set_beacon_distance(id, dist)  ← 4 anchor  (mm → m)                │
│   set_vehicle_position(pos, err) ← ENU mm → NED m                     │
└────────────────────────────┬──────────────────────────────────────────┘
                             ▼
┌───────────────────────────────────────────────────────────────────────┐
│ AP_Beacon frontend → AP_DAL_Beacon                                   │
│   → EKF3 readRngBcnData() → FuseRngBcn()                             │
└───────────────────────────────────────────────────────────────────────┘
```

---

## 3. Wire protocol Pozyx (decode từ `IndoorLoiter.ino`)

Nguồn: `Tools/Pozyx/IndoorLoiter/IndoorLoiter.ino` (ArduPilot master).

### 3.1 Format chung

```
[0x01][msg_id][len][payload...][checksum]
```

- `len` = `payload_len + 1` (bao gồm checksum byte)
- `checksum` = XOR của `msg_id`, `len`, và tất cả payload bytes
- Multi-byte fields: **little-endian**

### 3.2 MSGID_BEACON_CONFIG (0x02) — len = 15

| Offset | Kiểu | Ý nghĩa |
|---|---|---|
| 0 | u8 | beacon_id |
| 1 | u8 | beacon_count |
| 2..5 | i32 LE | x (mm) |
| 6..9 | i32 LE | y (mm) |
| 10..13 | i32 LE | z (mm) |
| 14 | u8 | checksum |

### 3.3 MSGID_BEACON_DIST (0x03) — len = 6

| Offset | Kiểu | Ý nghĩa |
|---|---|---|
| 0 | u8 | beacon_id |
| 1..4 | u32 LE | distance (mm) |
| 5 | u8 | checksum |

### 3.4 MSGID_POSITION (0x04) — len = 15

| Offset | Kiểu | Ý nghĩa |
|---|---|---|
| 0..3 | i32 LE | x (mm) |
| 4..7 | i32 LE | y (mm) |
| 8..11 | i32 LE | z (mm) |
| 12..13 | i16 LE | position_error (mm) |
| 14 | u8 | checksum |

### 3.5 Đối chiếu với parser `AP_Beacon_Pozyx.cpp`

```cpp
// CONFIG: linebuf[5]<<24 | [4]<<16 | [3]<<8 | [2]   → little-endian ✅
// DIST:   linebuf[4]<<24 | [3]<<16 | [2]<<8 | [1]   → little-endian ✅
// POS:    linebuf[3]<<24 | [2]<<16 | [1]<<8 | [0]   → little-endian ✅
```

**Protocol match hoàn hảo — không cần sửa parser.**

---

## 4. Frame convention (ENU → NED)

Cả Pozyx và NoopLoop đều **nhận ENU và tự đảo sang NED** phía ArduPilot:

```cpp
// AP_Beacon_Pozyx.cpp
Vector3f beacon_pos(beacon_x * 0.001f, beacon_y * 0.001f, -beacon_z * 0.001f);
Vector3f veh_pos(vehicle_x * 0.001f, vehicle_y * 0.001f, -vehicle_z * 0.001f);
```

```cpp
// AP_Beacon_Nooploop.cpp
const Vector3f pos_m {pos_y * 0.001f, pos_x * 0.001f, -pos_z * 0.001f};
```

→ **Node Python gửi ENU (Gazebo world frame), KHÔNG transform.**

Gazebo là ENU (x=East, y=North, z=Up); Pozyx cũng ENU. Khớp tự nhiên.

⚠️ Đây là điểm dễ sai nhất. Không được transform trong node.

---

## 5. Anchor layout — đã xác minh khớp

### 5.1 IndoorLoiter.ino (Pozyx reference)

```c
int32_t anchors_x[NUM_ANCHORS] = {0, 10000, 0, 10000};    // mm
int32_t anchors_y[NUM_ANCHORS] = {0, 0, 10000, 10000};    // mm
int32_t heights[NUM_ANCHORS]   = {-1200, -1200, -1200, -1200};  // mm
```

### 5.2 Gazebo `uwb_runway.sdf` (nhánh cũ)

```xml
<include><uri>model://uwb_anchor</uri><name>uwb_anchor0</name>
  <pose>0 0 2.0 0 0 0</pose></include>
<include><uri>model://uwb_anchor</uri><name>uwb_anchor1</name>
  <pose>10 0 2.0 0 0 0</pose></include>
<include><uri>model://uwb_anchor</uri><name>uwb_anchor2</name>
  <pose>10 10 2.0 0 0 0</pose></include>
<include><uri>model://uwb_anchor</uri><name>uwb_anchor3</name>
  <pose>0 10 2.0 0 0 0</pose></include>
```

### 5.3 Đối chiếu

| Anchor | Pozyx (mm) | Gazebo (m) | Khớp |
|---|---|---|---|
| 0 | (0, 0) | (0, 0, 2.0) | ✅ xy |
| 1 | (10000, 0) | (10, 0, 2.0) | ✅ xy |
| 2 | (0, 10000) | (10, 10, 2.0) | ⚠️ **swap** |
| 3 | (10000, 10000) | (0, 10, 2.0) | ⚠️ **swap** |

**Kết luận:** layout XY của Pozyx là hình vuông 10×10 m với anchor 0 ở góc
(0,0). Gazebo dùng **cùng hình vuông 10×10 m**, cũng anchor 0 ở (0,0), nhưng
**thứ tự góc khác**: Pozyx đi vòng ngược chiều kim đồng hồ (0,0)→(10,0)→
(0,10)→(10,10), Gazebo đi vòng cùng chiều (0,0)→(10,0)→(10,10)→(0,10).

→ **Quan trọng:** thứ tự này **không ảnh hưởng** tới EKF vì EKF dùng
`beacon_ID` làm chỉ số và `beacon_posNED` đi kèm. Chỉ cần **mapping nhất quán**
giữa `beacon_id` (Python) và vị trí anchor (Gazebo). Có thể giữ nguyên thứ tự
Gazebo; chỉ cần truyền đúng cặp (id, position).

### 5.4 Độ cao anchor

Pozyx: `heights = -1200 mm` (anchor 1.2 m **trên** xe → NED z = -1.2 m).
Gazebo: anchor ở `z = +2.0 m` (trên mặt đất), xe bay ở ~0.1–1 m.

Đây là **frame khác nhau**: Pozyx heights là tương đối so với xe; Gazebo z là
tuyệt đối so với world. Khi gửi `MSGID_BEACON_CONFIG`, ta gửi vị trí anchor
**tương đối tới beacon origin** (mà ta chọn = vị trí spawn của xe).

→ Nếu beacon origin = xe spawn ở `(0, 0, 0.1)`, thì anchor 0 ở
`(0, 0, 2.0)` → NED z = `-(2.0 - 0.1)` = `-1.9 m`. Hợp lý.

### 5.5 Trả lời câu hỏi: "drone spawn ở tâm 4 anchor được không?"

**Được, và đó là lựa chọn đúng.** Tâm hình vuông 10×10 là `(5, 5)`.

Lý do:
1. Tâm là vị trí **trung lập** — sai số trilateration nhỏ nhất, DOP tốt nhất.
2. Tâm = **beacon origin tự nhiên** → beacon NED offsets nhỏ, dễ kiểm tra.
3. Pozyx `IndoorLoiter.ino` đặt anchor0 tại (0,0) nhưng drone ở đâu không quyết
   định — ông ta chỉ cần 4 anchor bao quanh.

**Lưu ý:** world file hiện tại *không* spawn drone (spawn runtime qua
`ros_gz_sim create`). Cần set `--pos` khi spawn:
```bash
ros2 run ros_gz_sim gz sim -r -s x500.sdf -p 5 5 0.1
```
→ Khi đó beacon origin = `(5, 5, 0.1)`.

### 5.6 `spherical_coordinates` của world

```xml
<latitude_deg>-35.3632621</latitude_deg>
<longitude_deg>149.1652374</longitude_deg>
<elevation>10.0</elevation>
```

→ Đây chính là **SITL home location** mặc định (`-35.363261, 149.165230`).
Tốt — world đã được align với SITL home sẵn.

---

## 6. Rủi ro đã xác minh

### R1 — EKF3 cần ~200 measurement để bắt đầu fuse (QUAN TRỌNG)

`AP_NavEKF3_RngBcnFusion.cpp:337`:

```cpp
if (rngBcn.numMeas >= 100) { rngBcn.alignmentStarted = true; ... }
...
if (rngBcn.numMeas >= 100) { rngBcn.alignmentCompleted = true; }
```

`SelectRngBcnFusion()`:

```cpp
if ((frontend->sources.getPosXYSource() == AP_NavEKF_Source::SourceXY::BEACON)
    && rngBcn.alignmentCompleted) {
    FuseRngBcn();      // ← fusion thật, chỉ chạy sau alignment
} else {
    FuseRngBcnStatic();
}
```

**Hệ quả:**
- Cần **~200 measurement** (100 để start + 100 để complete).
- Với 4 anchor ở 20 Hz → 200/4 = 50 vòng = **~2.5 giây** sau khi có data.
- `FuseRngBcnStatic()` cũng chỉ chạy khi `rngBcn.dataToFuse` true → **cần
  measurement liên tục**.

→ **Node phải gửi distance liên tục, kể cả khi không có ranging mới** (giữ
last-known distance), để EKF không bị "no data".

### R2 — `position_error` phải < 1.0 m (BLOCKER nếu sai đơn vị)

`AP_NavEKF3_Measurements.cpp:977`:

```cpp
if ((imuSampleTime_ms - rngBcn.last3DmeasTime_ms < 250)
    && (rngBcn.vehiclePosErr < 1.0f)
    && rngBcn.alignmentCompleted) {
```

→ `set_vehicle_position(pos, accuracy_estimate)` với `accuracy >= 1.0` →
**không bao giờ align**.

Đơn vị: Pozyx `pos_error.xy` là **mm**. Gửi 50 (mm) → 0.05 m. An toàn.

⚠️ Nếu gửi 0.05 (m) thì accuracy = 50.0 m → **fail hoàn toàn**.

#### ⚠️ Đính chính sau kiểm thử thực nghiệm (2026-10-27)

Mô tả trên là **suy đoán từ code sketch**, đã bị thực nghiệm **bác bỏ**.

`IndoorLoiter.ino` gửi `pos_error.xy` bằng **mm** (ví dụ `50` = 50 mm),
nhưng `AP_Beacon_Pozyx.cpp` truyền thẳng giá trị này cho
`set_vehicle_position(pos, accuracy_estimate)`, mà EKF lại so sánh với
ngưỡng **1.0 (mét)**:

```cpp
if ((imuSampleTime_ms - rngBcn.last3DmeasTime_ms < 250)
    && (rngBcn.vehiclePosErr < 1.0f)
    && rngBcn.alignmentCompleted) {
```

→ gửi `50` bị EKF hiểu là **50 mét**, vượt ngưỡng → **không bao giờ align**.

Kết quả đo thực nghiệm trên SITL:

| `position_error` gửi đi | EKF hiểu là | Kết quả |
|---|---|---|
| `50` (mm, theo sketch) | 50.0 m | ❌ không có "origin set" |
| `0` | 0.0 m | ✅ "EKF3 IMU0 origin set" + "is using range beacons" |

**Workaround (đã áp dụng, quyết định Q3):** node gửi `position_error`
theo **mét** — cụ thể là `0` (hoặc bất kỳ giá trị `< 1.0` nào).

> **Không patch ArduPilot** (theo quyết định Q3). Workaround nằm hoàn toàn
> ở phía sender, và đã được ghi lại tại đây để tránh người sau hiểu nhầm.

Xem thêm: `uwb_pozyx_bridge/uwb_pozyx_bridge/pozyx_protocol.py`
(docstring của `encode_vehicle_position`).

### R2b — `EK3_SRC1_POSZ` phải là **BARO**, không phải BEACON

Với 4 anchor **coplanar** (cùng một độ cao), chiều Z của tag **không quan
sát được** — có hai nghiệm mirror đối xứng. Nếu đặt
`EK3_SRC1_POSZ = 4` (BEACON), EKF cố suy ra Z từ beacon và bị lỗi do
ambiguity, dẫn tới innovation lớn → EKF **không** báo
"is using range beacons".

Đã kiểm chứng:

| `EK3_SRC1_POSZ` | Kết quả |
|---|---|
| `4` (Beacon) | ⚠️ "origin set" nhưng **không** "is using range beacons" |
| `1` (Baro)   | ✅ "origin set" **và** "is using range beacons" |

→ Dùng `EK3_SRC1_POSZ = 1` (Baro) cho hệ anchor coplanar này.
Đây cũng là cấu hình mặc định trong `config/beacon.parm`.

### R3 — Beacon origin phải khớp EKF origin

`AP_NavEKF3_Measurements.cpp:986`:

```cpp
if (!validOrigin && (PV_AidingMode != AID_ABSOLUTE)) {
    Location origin_loc;
    if (beacon->get_origin(origin_loc)) {
        setOriginLLH(origin_loc);
        alignMagStateDeclination();
        ekfOriginHgtVar = sq(rngBcn.vehiclePosErr);
    }
}
```

Trong `AP_Beacon_SITL` (tham chiếu):

```cpp
beacon_origin.lat = get_beacon_origin_lat() * 1.0e7f;
const Vector2f veh_diff = beacon_origin.get_distance_NE(current_loc);
Vector3f veh_pos3d(veh_diff.x, veh_diff.y, (beacon_origin.alt - current_loc.alt)*1.0e-2f);
```

→ `veh_pos3d` = vector NED **từ beacon origin tới xe**. Vậy `BCN_LATITUDE`/
`LONGITUDE`/`ALT` là **LLH của beacon origin** (tức vị trí spawn của xe nếu
ta chọn origin = tâm anchors).

Giá trị chuẩn (từ `Tools/autotest/arducopter.py:7885`):

```python
"BCN_LATITUDE": SITL_START_LOCATION.lat,
"BCN_LONGITUDE": SITL_START_LOCATION.lng,
"BCN_ALT": SITL_START_LOCATION.alt,
```

→ Dùng `SITL_START_LOCATION` = `-35.363261, 149.165230, 584.9`.

### R4 — `BCN_TYPE` phải là 1 (Pozyx), không phải 10 (SITL)

`AP_Beacon::Type`: `None=0, Pozyx=1, Marvelmind=2, Nooploop=3, SITL=10`.

Dùng **1** vì ta dùng backend Pozyx thật qua serial.

### R5 — Version ArduPilot dùng `ARMING_CHECK`, không có `ARMING_SKIPCHK`

Đã grep xác nhận: `ARMING_SKIPCHK` **không tồn tại** trong build này.
`ARMING_CHECK` là bitmask (bit 0 = all checks).

→ **Param file `copter-uwb.parm` của nhánh cũ không áp dụng được** (nó dùng
`GPS_TYPE`, `VISO_TYPE` — param không tồn tại ở version này).

### R6 — Pre-arm khi POSXY=BEACON không đòi GPS

`AP_NavEKF_Source.cpp:463`:

```cpp
if (beacon_required) {
    const bool beacon_available = (dal.beacon() != nullptr && dal.beacon()->enabled());
    if (!beacon_available) { /* fail */ }
}
```

→ Chỉ cần `BCN_TYPE != 0`. Không cần GPS lock.

### R7 — `set_beacon_distance` set `healthy = true` và tăng `num_beacons`

```cpp
_frontend.beacon_state[beacon_instance].distance_update_ms = AP_HAL::millis();
_frontend.beacon_state[beacon_instance].distance = distance;
_frontend.beacon_state[beacon_instance].healthy = true;
```

`readRngBcnData()` kiểm tra `beacon->beacon_healthy(index)`.
`num_beacons = beacon_instance + 1` → beacon_id 0..3 cho num_beacons=4 ✅
(`AP_BEACON_MAX_BEACONS = 4`).

### R8 — Protocol serial `SerialProtocol_Beacon = 13` không có `case` riêng

`AP_SerialManager.cpp` → rơi vào `default: uart->begin(state[i].baudrate())`.
→ Baud lấy từ `SERIALx_BAUD`, serial port **không bị chiếm** bởi protocol khác.
→ Tốt cho việc dùng serial1.

### R9 — SITL serial transport khả dụng

`AP_HAL_SITL/UARTDriver.cpp` hỗ trợ: `tcp:`, `tcpclient:`, `udpclient:`,
`mcast:`, `file:`, `sim:`, `uart:`.

→ Dùng `--serial1=tcp:5761:wait` (SITL chờ connect) hoặc `tcp:5761` (không
block).

### R10 — Plugin phải tìm link trong model **lồng nhau**

`x500_with_ardupilot` → `<include>` `x500_base` → `<include>` `x500_base`.
Gazebo bọc mỗi `<include>` trong một model con, nên `base_link` **không** phải
child trực tiếp của model `x500`.

→ `FindLinkEntity` phải đệ quy vào child model. Nếu không, plugin không tìm
thấy tag → **ranging rỗng, không có bất kỳ message nào**.

### R11 — Gazebo không cấp `WorldPose` component cho link

`components::Pose` của một link là pose **cục bộ** (luôn = origin). Component
`WorldPose` chỉ tồn tại nếu world có `gz::sim::systems::PosePublisher`.

→ Plugin tự dồn pose qua chuỗi parent (`_ecm.ParentEntity`) để suy ra world
pose. Không có bước này, ranging đo **sai hoàn toàn** (tag luôn ở origin).

### R12 — `<pos>` của `<anchor>` là **child element**

```xml
<!-- SAI: attribute -->
<anchor name="uwb_anchor0" pos="0 0 2.0"/>

<!-- ĐÚNG -->
<anchor name="uwb_anchor0"><pos>0 0 2.0</pos></anchor>
```

→ Dùng sai thì plugin báo `<anchor> without <pos>; skipping` rồi rơi về
layout hard-coded (trùng vị trí, nên vẫn chạy — nhưng im lặng sai).

### R13 — LOS code **không** được encode thành quaternion roll

`Quaterniond(los, 0, 0)` tạo `orientation.x = sin(los/2)`, nên LOS=1 →
`0.479` → Python `round()` → 0 → **mất cờ NLOS**.

→ Đặt thẳng `orientation.x = los` (0/1/2) và `orientation.w = 1.0`.

### R14 — Spawn pose phải đặt trên `<include>`

`x500_base/model.sdf` có `<pose>0 0 .025</pose>` riêng, và nested include
không kế thừa pose của parent. Đặt pose ở `<model>` hoặc field `pose:` của
`EntityFactory` đều **không có tác dụng**.

```xml
<!-- ĐÚNG -->
<model name="x500">
  <include>
    <uri>model://x500_base</uri>
    <pose>5 5 0.1 0 0 0</pose>
  </include>
</model>
```

→ Đã verify: tag ở đúng `(5, 5, 0.104)`, range mỗi anchor 7.30–7.42 m
(kỳ vọng √(5²+5²+1.9²) = 7.34 m).

---

### R15 — `PosePublisher` phải attach vào **model**, không phải world

Gazebo log có dòng:

```
[gz-1] [Err] [PosePublisher.cc:192] PosePublisher plugin should be attached to
        a model entity. Failed to initialize.
```

Trong `PosePublisher::Configure()` của Gazebo Harmonic:

```cpp
this->dataPtr->model = Model(_entity);
if (!this->dataPtr->model.Valid(_ecm)) {
  gzerr << "PosePublisher plugin should be attached to a model entity. "
        << "Failed to initialize." << std::endl;
  return;   // <- thoát sớm, plugin không hoạt động
}
```

Attach ở cấp `<world>` → `Model::Valid()` false → plugin return ngay, **chưa
bao giờ khởi tạo được**. Đã bỏ hẳn khỏi `uwb_runway.sdf`.

**Lưu ý:** comment cũ trong world SDF nói plugin này *"cần thiết để có
WorldPose component"* là **sai**. `WorldPose` do **Physics system** ghi cho
dynamic entity. Thiếu PosePublisher thì UWB plugin rơi về nhánh fallback
`LocalToWorldPose()` (đi ngược parent chain) — kết quả vẫn đúng.

Đã kiểm chứng bằng cách bỏ plugin: range 7.318–7.324 m, 19.8 Hz/anchor,
tag pose `(5.000, 5.000)`, EKF beacon vẫn fuse bình thường.

---

## 7. Đánh giá mức độ khả thi

**Kết luận: khả thi, đúng kiến trúc, không cần sửa ArduPilot.**

Ước lượng: **~2.5–3.5 ngày** công sức.

| Phase | Công sức | Rủi ro | Nội dung |
|---|---|---|---|
| **Tier 2** | 0.5 ngày | Thấp | Fake Arduino → SITL, verify EKF3 beacon fusion hoạt động |
| **Tier 1** | 0.5 ngày | Thấp | `pozyx_protocol.py` + unit test byte layout |
| **Tier 3** | 1–1.5 ngày | Trung bình | Node Python đầy đủ + trilateration |
| **Tier 4** | 0.5–1 ngày | Cao | Nối Gazebo, anchor frame mapping, ENU/NED |
| **Tier 5** | 0.5 ngày | Trung bình | Verify EKF params + flight test |

**Tier 2 làm trước** vì rẻ nhất và tách rủi ro tối đa: nếu EKF3 beacon fusion
không chạy được với build này, mọi việc còn lại vô nghĩa.

---

## 8. Roadmap chi tiết

### Tier 2 — Fake Arduino → SITL (không cần Gazebo)

Mục tiêu: chứng minh EKF3 beacon fusion chạy được với build hiện tại.

1. Script Python đóng vai Arduino: gửi 4 beacon config + distance + position
   giả qua TCP → SITL serial1.
2. SITL chạy với param beacon.
3. Verify:
   - `BCN_TYPE` = 1, `AP_Beacon` healthy
   - `ekf_status` cho thấy beacon fusion
   - Log `BEA` (`LOG_BEACON_MSG`) có `dist0..3`, `posx/y/z`

### Tier 1 — Protocol module

- `pozyx_protocol.py`: encode 3 message type (pure function, không I/O).
- Unit test so byte-by-byte với layout §3.

### Tier 3 — Node Python

- Subscribe `/uwb/ranging/ros`.
- WLS trilateration (tái dùng logic từ `uwb_trilateration_node.py` cũ).
- Đóng gói + gửi qua TCP, retry/reconnect.
- Gửi distance liên tục (giữ last-known) → tránh R1.

### Tier 4 — Nối Gazebo

- Launch `uwb_runway.sdf`, spawn x500 tại tâm anchors.
- Anchor positions: lấy từ param, khớp với world.
- Beacon origin = vị trí spawn → `BCN_LATITUDE/LONGITUDE/ALT` = SITL home.

### Tier 5 — Verify & flight test

- `EK3_SRC1_POSXY=4`, `BCN_TYPE=1`.
- Check `ekf_status` flags ổn định, position hold.

---

## 9. Param ArduPilot (đúng cho version này)

```bash
# --- Beacon ---
BCN_TYPE 1              # 1 = Pozyx (KHÔNG phải 10 = SITL)
BCN_LATITUDE -35.363261
BCN_LONGITUDE 149.165230
BCN_ALT 584.9
BCN_ORIENT_YAW 0

# --- EKF3 ---
EK3_ENABLE 1
EK2_ENABLE 0
EK3_SRC1_POSXY 4       # BEACON
EK3_SRC1_POSZ 1        # BARO
EK3_SRC1_VELXY 0       # NONE
EK3_SRC1_VELZ 0        # NONE
EK3_SRC1_YAW 1         # COMPASS

# --- Không GPS ---
GPS1_TYPE 0
GPS2_TYPE 0

# --- Arming: version này dùng ARMING_CHECK bitmask ---
# ARMING_CHECK 1  (mặc định = all checks; beacon không đòi GPS nên OK)
```

---

## 10. Cấu trúc package mới

```
src/uwb_pozyx_bridge/              ← package MỚI (thay uwb_bridge cũ)
├── uwb_pozyx_bridge/
│   ├── pozyx_protocol.py          ← encode 3 message type (pure, testable)
│   ├── pozyx_sender.py            ← TCP/UDP transport + reconnect
│   ├── pozyx_node.py              ← node chính: subscribe + trilaterate + send
│   └── trilateration.py           ← WLS solver (port từ nhánh cũ)
├── config/
│   └── beacon.parm                ← param chuẩn ArduPilot
├── launch/
│   └── uwb_pozyx_sitl.launch.py
├── test/
│   └── test_pozyx_protocol.py     ← unit test byte layout
├── package.xml
├── setup.cfg
└── setup.py
```

### Tái dùng / loại bỏ từ nhánh `feat/uwb-odometry`

| File | Hành động | Lý do |
|---|---|---|
| `src/uwb_gz_plugin/**` | ✅ **Giữ nguyên toàn bộ** | Plugin mô phỏng radio, layout đã khớp Pozyx |
| `src/uwb_gz_plugin/src/ros_gz_bridge_node.cc` | ✅ Giữ | Bắt buộc — `ros_gz_bridge` factory-lookup failure |
| `uwb_trilateration_node.py` (logic WLS) | 🔄 Port | Bỏ phần publish MAVROS, giữ solver |
| `mavlink_bridge_node.py` | ❌ Bỏ | Thay bằng pozyx protocol sender |
| `uwb_to_mavros_vision_node.py` | ❌ Bỏ | Không dùng MAVROS nữa |
| `config/copter-uwb.parm` | ❌ Bỏ | Param sai version (`GPS_TYPE`, `VISO_TYPE` không tồn tại) |
| `launch/mavros_vision.launch.py` | ❌ Bỏ | Không dùng MAVROS |
| `launch/step5_sitl_end_to_end.launch.py` | 🔄 Viết lại | Cho pipeline mới |

---

## 11. Danh sách kiểm tra verify

### Tier 2
- [ ] SITL khởi động, `param show BCN_TYPE` = 1
- [ ] `param show EK3_SRC1_POSXY` = 4
- [ ] `AP_Beacon` healthy = true (không có API trực tiếp → xem log `BEA`)
- [ ] Log `BEA`: `dist0..3` khác 0, `posx/y/z` khác 0
- [ ] `ekf_status`: vertical position / horizontal position = true
- [ ] Sau ~3s: EKF không báo "stopped aiding"

### Tier 4
- [ ] `/uwb/ranging/ros` có data (4 anchor × 20 Hz)
- [ ] Vị trí trilateration khớp vị trí thật của drone trong Gazebo
- [ ] EKF position khớp trilateration trong ~0.5 m

### Tier 5
- [ ] Arm được (pre-arm check pass)
- [ ] Position hold giữ được vị trí
- [ ] Không drift khi hover

---

## 12. Tài liệu tham chiếu

- `src/ardupilot/libraries/AP_Beacon/AP_Beacon_Pozyx.cpp` — parser (đã có sẵn)
- `src/ardupilot/libraries/AP_Beacon/AP_Beacon_SITL.cpp` — tham chiếu semantics
- `src/ardupilot/libraries/AP_Beacon/AP_Beacon_Backend.cpp` — `set_*` helpers
- `src/ardupilot/libraries/AP_Beacon/AP_Beacon.cpp` — frontend, params
- `src/ardupilot/libraries/AP_NavEKF3/AP_NavEKF3_RngBcnFusion.cpp` — fusion
- `src/ardupilot/libraries/AP_NavEKF3/AP_NavEKF3_Measurements.cpp:905-1015` — read data
- `src/ardupilot/libraries/AP_NavEKF/AP_NavEKF_Source.h` — `BEACON = 4`
- `src/ardupilot/libraries/AP_SerialManager/AP_SerialManager.h:52` — `SerialProtocol_Beacon = 13`
- `src/ardupilot/libraries/AP_HAL_SITL/UARTDriver.cpp` — serial transport
- `src/ardupilot/Tools/Pozyx/IndoorLoiter/IndoorLoiter.ino` — protocol gốc
- `src/ardupilot/Tools/autotest/arducopter.py:7885` — `BeaconPosition()` test
- `src/ardupilot/Tools/autotest/default_params/copter-beacon.parm` — param mẫu
