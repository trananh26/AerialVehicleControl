# Đánh giá pipeline UWB → Pozyx → ArduPilot SITL

> ⚠️ **Tài liệu lịch sử.** File này mô tả pipeline dùng protocol Pozyx
> (`AP_Beacon`), đã bị thay thế bởi pipeline MAVLink `ODOMETRY` (id 331) +
> `AP_VisualOdom` (EXTNAV).
>
> - Kiến trúc hiện hành: **`UWB_ARCHITECTURE.md`**
> - Pipeline Pozyx cũ đã bị xóa khỏi codebase (`pozyx_node.py`,
>   `pozyx_protocol.py`, `pozyx_sender.py`, `sitl_beacon_monitor.py`).
>
> Giữ lại file này để tham khảo lịch sử phân tích và các rủi ro đã xác minh
> bằng đọc source ArduPilot.

> Tài liệu này ghi lại đánh giá kiến trúc, các rủi ro đã xác minh bằng đọc source
> ArduPilot, và roadmap triển khai.
> Ngày lập: 2026-10-26. Commit ArduPilot: `55ae8d2825` (ArduCopter-stable-2).

---

## 1. Quyết định kiến trúc

**Đã chọn:** Gazebo UWB plugin → Node Python (giả lập Arduino Uno) → Pozyx binary
protocol → SITL serial → `AP_Beacon_Pozyx` → EKF3 range-beacon fusion.

**Bỏ nhánh** `feat/uwb-odometry`**'s approach (MAVLink odometry / VISION_POSITION_ESTIMATE).**

### 1.1 Vì sao nhánh cũ có lỗi


|                  | Nhánh cũ                                        | Pipeline mới                      |
| ---------------- | ----------------------------------------------- | --------------------------------- |
| Message          | MAVLink `VISION_POSITION_ESTIMATE` (qua MAVROS) | Binary Pozyx protocol             |
| EKF3 path        | `FuseExtNav()`                                  | `FuseRngBcn()`                    |
| Cần velocity?    | **Có** → phải finite-difference → noise         | **Không**                         |
| Position source  | Position fix tuyệt đối                          | Distance từng anchor, EKF tự giải |
| Phụ thuộc MAVROS | Có (fragile)                                    | Không                             |
| Sửa ArduPilot?   | Không                                           | Không                             |


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


| Offset | Kiểu   | Ý nghĩa      |
| ------ | ------ | ------------ |
| 0      | u8     | beacon_id    |
| 1      | u8     | beacon_count |
| 2..5   | i32 LE | x (mm)       |
| 6..9   | i32 LE | y (mm)       |
| 10..13 | i32 LE | z (mm)       |
| 14     | u8     | checksum     |




### 3.3 MSGID_BEACON_DIST (0x03) — len = 6


| Offset | Kiểu   | Ý nghĩa       |
| ------ | ------ | ------------- |
| 0      | u8     | beacon_id     |
| 1..4   | u32 LE | distance (mm) |
| 5      | u8     | checksum      |




### 3.4 MSGID_POSITION (0x04) — len = 15


| Offset | Kiểu   | Ý nghĩa             |
| ------ | ------ | ------------------- |
| 0..3   | i32 LE | x (mm)              |
| 4..7   | i32 LE | y (mm)              |
| 8..11  | i32 LE | z (mm)              |
| 12..13 | i16 LE | position_error (mm) |
| 14     | u8     | checksum            |




### 3.5 Đối chiếu với parser `AP_Beacon_Pozyx.cpp`

```cpp
// CONFIG: linebuf[5]<<24 | [4]<<16 | [3]<<8 | [2]   → little-endian ✅
// DIST:   linebuf[4]<<24 | [3]<<16 | [2]<<8 | [1]   → little-endian ✅
// POS:    linebuf[3]<<24 | [2]<<16 | [1]<<8 | [0]   → little-endian ✅
```

**Protocol match hoàn hảo — không cần sửa parser.**

---



## 4. Frame convention (ENU → NED)

> ⚠️ **ĐÃ SỬA — mục này trước đây sai và là nguyên nhân drone bay sai quỹ đạo.**
> Xem §4.1 bên dưới.



### 4.1 Cơ chế thật của ArduPilot

`AP_Beacon_Pozyx.cpp` **không** tự đảo ENU→NED. Nó đọc byte thô và gán
thẳng vào hệ NED:

```cpp
// AP_Beacon_Pozyx.cpp
Vector3f beacon_pos(beacon_x * 0.001f, beacon_y * 0.001f, -beacon_z * 0.001f);
Vector3f veh_pos(vehicle_x * 0.001f, vehicle_y * 0.001f, -vehicle_z * 0.001f);
```

Nghĩa là:


| byte trên wire | ArduPilot hiểu là | Nên gửi (Gazebo ENU) |
| -------------- | ----------------- | -------------------- |
| `x`            | NED **North**     | `y` (Gazebo North)   |
| `y`            | NED **East**      | `x` (Gazebo East)    |
| `z`            | NED **Down**      | `z` (Gazebo Up)      |


`AP_Beacon_Nooploop.cpp` có `pos_m {pos_y, pos_x, -pos_z}` — tức là *nó* đảo
trục ngang. Vậy nên Pozyx và NoopLoop **không** dùng cùng một convention:
Pozyx coi byte `x` là North, NoopLoop coi byte `x` là East.

→ **Node Python phải gửi byte** `x` **= North, byte** `y` **= East.**

Hàm `enu_to_wire_mm()` trong `pozyx_protocol.py` làm đúng việc này, và là
**điểm duy nhất** trong codebase được phép đổi frame:

```python
def enu_to_wire_mm(east_m, north_m, up_m):
    """Gazebo ENU -> Pozyx wire (x = NED North, y = NED East, z = Up)."""
    return (
        int(round(north_m * 1000.0)),
        int(round(east_m * 1000.0)),
        int(round(up_m * 1000.0)),
    )
```



### 4.2 Bằng chứng thực nghiệm

Teleport thử drone trong Gazebo tới vị trí tương đối `(-1.0, +2.0)`:


|                            | x          | y          |
| -------------------------- | ---------- | ---------- |
| Gazebo truth (ENU)         | −1.000     | +2.000     |
| **Trước khi sửa** — MAVROS | **+1.957** | **−1.050** |
| **Sau khi sửa** — MAVROS   | −1.025     | +2.022     |


Sai số trước khi sửa là **4.276 m** và có dấu hiệu **hoán đổi trục**
(determinant âm — phản xạ qua đường chéo), đúng như dự đoán. Sau khi sửa
sai số là **0.034 m**.

### 4.3 Hệ quả: vì sao drone bị lật

Khi vị trí bị hoán đổi trục, vector lỗi vị trí mà autopilot phải bay về
chĩa ngược hướng với hướng yaw mà lệnh đặt — lệch nhau khoảng 90°. Bộ điều
khiển phải nghiêng hết cỡ để vừa bay về đích vừa giữ yaw, góc nghiêng vượt
`ANGLE_MAX` (30° mặc định) và động cơ bão hoà → drone lật.

Ngoài ra `setpoint_velocity` (`SubMode::VelAccel`) tắt hẳn vòng lặp
position P-term, còn EKF ở đây **không có** nguồn vận tốc ngoài
(`EK3_SRC1_VELXY = 0`, UWB chỉ fuse *khoảng cách*), nên sai số vị trí cộng dồn
không bị triệt tiêu.

→ Vì vậy `circle_mission.py` phải dùng **position control**
(`/mavros/setpoint_raw/local`, mask `0x1C0 | 0x800` → `SubMode::PosVelAccel`)
và **luôn chỉ yaw về tâm đường tròn** để góc nghiêng giữ ở mức nhỏ.

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


| Anchor | Pozyx (mm)     | Gazebo (m)    | Khớp        |
| ------ | -------------- | ------------- | ----------- |
| 0      | (0, 0)         | (0, 0, 2.0)   | ✅ xy        |
| 1      | (10000, 0)     | (10, 0, 2.0)  | ✅ xy        |
| 2      | (0, 10000)     | (10, 10, 2.0) | ⚠️ **swap** |
| 3      | (10000, 10000) | (0, 10, 2.0)  | ⚠️ **swap** |


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


| `position_error` gửi đi | EKF hiểu là | Kết quả                                             |
| ----------------------- | ----------- | --------------------------------------------------- |
| `50` (mm, theo sketch)  | 50.0 m      | ❌ không có "origin set"                             |
| `0`                     | 0.0 m       | ✅ "EKF3 IMU0 origin set" + "is using range beacons" |


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


| `EK3_SRC1_POSZ` | Kết quả                                                  |
| --------------- | -------------------------------------------------------- |
| `4` (Beacon)    | ⚠️ "origin set" nhưng **không** "is using range beacons" |
| `1` (Baro)      | ✅ "origin set" **và** "is using range beacons"           |


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

→ **Param file** `copter-uwb.parm` **của nhánh cũ không áp dụng được** (nó dùng
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



### R16 — SITL phải dùng `--model json` và model **có** ArduPilotPlugin

Đây là rủi ro nghiêm trọng nhất, vì nó tạo ra **false pass**: mọi gate đều
báo PASS trong khi drone thực tế không hề bị điều khiển.

Hai điều phải đúng cùng lúc:

1. **SITL phải nối tới FDM port 9002.** Chỉ `--model json` mới làm được
  (`SIM_JSON.h: control_port = 9002`). `--model x500` kích hoạt mô hình
   bay nội bộ của ArduPilot — SITL bay một chiếc drone ảo, hoàn toàn tách
   rời khỏi Gazebo.
2. **Model trong Gazebo phải chứa** `ArduPilotPlugin`**.** Model
  `x500_with_ardupilot` có plugin; `x500_base` thì không.

Khi đúng, log Gazebo hiện:

```
[Dbg] [SystemManager.cc:80] Loaded system [ArduPilotPlugin] for entity [27]
[Dbg] [ArduPilotPlugin.cc:1168] Computed IMU topic to be:
      world/uwb_runway/model/x500/link/base_link/sensor/imu_sensor/imu
[Wrn] [ArduPilotPlugin.cc:1599] ArduPilot controller has reset
```

Dòng *"has reset"* **không phải** lỗi — đó là báo hiệu frame counter của
SITL bị reset, tức là FDM handshake đã thành công.

**Cách kiểm chứng dứt khoát:** `tools/verify_fdm_coupling.py` arm drone và
cất cánh, rồi so sánh độ cao Gazebo với EKF. Drone chỉ bay được khi SITL
thực sự điều khiển simulation.

Kết quả đo được: Gazebo +3.037 m, EKF +2.980 m, chênh lệch 0.236 m.

### R17 — SITL phục vụ **một** client TCP cho port MAVLink

`UARTDriver::_check_connection()` trong `AP_HAL_SITL/UARTDriver.cpp`:

```cpp
if (_connected) {
    // we only want 1 connection at a time
    return;
}
```

Nên port 5760 chỉ nhận **một** kết nối TCP. MAVProxy đã giữ port này thì
mọi tool khác (monitor, compare) đều không vào được.

Hệ quả trực tiếp: **không được chạy monitor khi cần MAVProxy**. Monitor
cũng từng làm MAVProxy mất kết nối (`link 1 down`), kéo theo Mission
Planner rớt luôn.

Giải pháp: mọi tool đọc MAVLink đều đi qua **UDP output của MAVProxy**:


| Port  | Dùng cho                         |
| ----- | -------------------------------- |
| 14550 | MAVROS, và các tool verify       |
| 14551 | Mission Planner / QGroundControl |


MAVProxy tạo output với `input=False` (chỉ gửi đi, không bind), nên Mission
Planner phải tự bind UDP server trên 14551 — đúng cách Mission Planner
kết nối tới `UDP (copter)` với port 14551.

### R17b — Port 5760 còn bị chiếm bởi SITL instance cũ

SITL bind port 5760 (MAVLink) và 5761 (beacon) rồi **thoát** nếu port đã
bị chiếm:

```
bind failed on port 5760 - Address already in use
```

Dòng này dễ bị bỏ qua. Triệu chứng tiếp theo là MAVProxy retry vô hạn:

```
[mavproxy.py -6] Connection reset or closed by TCP socket
[mavproxy.py -6] Attempting reconnect
[mavproxy.py -6] [Errno 111] Connection refused sleeping
[mavproxy.py -6] link 1 down
```

Dấu hiệu phân biệt: `link 1 down` **đi kèm** `Detected vehicle 1:1 on link 0`
và `link 1 OK` là bình thường — MAVProxy khởi động trước SITL nên vài giây
đầu chưa có heartbeat. Nếu `link 1 down` đi kèm `Connection refused` liên
tục và **không** có `link 1 OK`, thì SITL đã chết.

Launch file giờ kiểm tra cả hai port trước khi khởi động và báo lỗi rõ
ràng:

```
[ERROR] [launch]: Caught exception in launch: TCP port(s) 5760 are still in
use.  A previous SITL instance is probably still running.  Stop it first:
    pkill -f arducopter
    pkill -f mavproxy.py
```

Cách xử lý:

```bash
pkill -f arducopter
pkill -f mavproxy.py
pkill -f "gz sim"
```



### R18 — MAVLink command phải dùng `target_component = 1`

Heartbeat được broadcast với component 0, nên `mav.target_component` là 0.
Lệnh gửi tới component 0 bị bỏ qua. Phải dùng `MAV_COMP_ID_AUTOPILOT1`
(= 1) cho `command_long_send`.

### R19 — SITL không có RC input: cần RC override, và throttle phải ở mức tối thiểu

SITL không có joystick, nên throttle = 0. ArduPilot **disarm** drone khi
throttle về 0, và **từ chối đổi mode** với thông báo *"throttle too high"*
nếu throttle cao.

Kịch bản hoạt động được:

1. `RC_CHANNELS_OVERRIDE` với ch1 (throttle) = **1000**, ch4 (arm) = 2000.
  Phải gửi **liên tục** trong suốt chuyến bay, nếu không drone disarm.
2. `MAV_CMD_DO_SET_MODE` **trước khi arm** (base_mode=1, custom_mode=4).
3. Arm, rồi `MAV_CMD_NAV_TAKEOFF`.



### R20 — QoS của publisher `/uwb/pose` phải khớp subscriber

`pozyx_node` publish `/uwb/pose`. Nếu publish bằng QoS mặc định (RELIABLE)
mà subscriber dùng BEST_EFFORT thì **không nhận được gì** — subscriber
đơn giản là không bao giờ thấy topic.

Bản sửa: publisher dùng cùng `QoSProfile` BEST_EFFORT với subscription
đầu vào. Đây là lý do `compare_positions.py` báo `trilateration: 0` trong
khi `ros2 topic hz /uwb/pose` vẫn ra 20 Hz.

---



### R21 — Guided takeoff cần **cả EKF origin lẫn Home position**

Đây là nguyên nhân gốc khiến drone **arm được nhưng không takeoff được**.

Triệu chứng: `MAV_CMD_NAV_TAKEOFF` bị trả về `MAV_RESULT_FAILED` (ACK 4)
liên tục, trong khi `MAV_CMD_COMPONENT_ARM_DISARM` vẫn `ACCEPTED`.

Cơ chế (`ArduCopter/mode_guided.cpp`):

```cpp
bool ModeGuided::do_user_takeoff_start(float takeoff_alt_cm)
{
    // interpret altitude as alt-above-home
    Location target_loc = copter.current_loc;
    target_loc.set_alt_cm(takeoff_alt_cm, Location::AltFrame::ABOVE_HOME);

    // provide target altitude as alt-above-ekf-origin
    if (!target_loc.get_alt_cm(Location::AltFrame::ABOVE_ORIGIN, alt_target_cm)) {
        // this should never happen but we reject the command just in case
        return false;
    }
    ...
}
```

`Location::get_alt_cm()` (`AP_Common/Location.cpp`) chuyển đổi giữa hai frame
và cần **cả hai** mốc:

| Frame cần | Nguồn | Hệ quả khi thiếu |
|---|---|---|
| `ABOVE_HOME` | `AP::ahrs().home_is_set()` | `get_alt_cm()` trả `false` |
| `ABOVE_ORIGIN` | `AP::ahrs().get_origin()` | `get_alt_cm()` trả `false` |

⇒ **Origin một mình là chưa đủ.** Phải có **Home position** mới takeoff được.

Với `GPS1_TYPE 0` (setup indoor, không GPS), ArduPilot **không tự** khởi tạo
home — home chỉ được set khi có GPS lock. Đó là lý do `origin set` xuất hiện
trong log (`EKF3 IMU0 origin set`) nhưng takeoff vẫn fail.

**Thứ tự bắt buộc** (ArduPilot từ chối `DO_SET_HOME` khi origin chưa có):

1. `SET_GPS_GLOBAL_ORIGIN` (msg id **48**) — đặt EKF origin.
   Handler: `GCS_MAVLINK::handle_set_gps_global_origin()` → `set_ekf_origin()`.
   Chỉ nhận từ GCS sysid (`SYSID_MYGCS` = 255) → phải gửi heartbeat trước.
2. `MAV_CMD_DO_SET_HOME` (cmd **179**) — đặt home position.
   Handler: `GCS_MAVLINK::handle_command_do_set_home()` → `Copter::set_home()`.
   `Copter::set_home()` kiểm tra `ahrs.get_origin()` trước, nên **bắt buộc
   origin phải có trước**.

Lưu ý thêm: EKF origin **không được persist** qua các lần restart của
autopilot, còn home thì có. Vì vậy `odometry_node` khai báo lại origin theo
chu kỳ (`ORIGIN_REFRESH_S`).

Link navigation (SERIAL1) **không trả telemetry ngược lại** cho companion
node, nên không có `COMMAND_ACK`/`HOME_POSITION` để xác nhận — khai báo được
lặp lại theo timer thay vì dựa trên phản hồi.

### R22 — ArduPilot settle thấp hơn altitude yêu cầu một chút

Sau `NAV_TAKEOFF` với `param7 = 1.5`, ArduPilot ổn định ở **~1.499m** (đo
qua `/mavros/local_position/pose`), tức thấp hơn mục tiêu ~1mm. Vì vậy
điều kiện `current_z >= target_z` **không bao giờ đúng** và mission treo ở
trạng thái `CLIMBING`.

Sửa: so sánh với dung sai (`altitude_tolerance_m = 0.1`).

### R23 — Process cũ sót lại gây quá tải CPU → MAVROS mất kết nối

Khi `ros2 launch` bị kill (`kill -9`, crash, hoặc Ctrl-C không đóng hết), các
process con của stack **vẫn chạy**: `parameter_bridge`, `robot_state_publisher`,
`uwb_gz_to_ros_bridge`, `rviz2`, `mavros_node`.

Mỗi process cũ giữ subscription vào cùng topic Gazebo và tiêu tốn **15–20%
CPU**. Sau nhiều lần chạy, load average lên tới **24+** trên máy 4 core.

Triệu chứng rất dễ nhầm — log báo lỗi liên tục **dù stack mới hoàn toàn bình
thường**:

```
[mavros.param]: PR: got an unsolicited param value idx=662, not resetting retries count 2   ← x6500 lần
[mavros.param]: PR: request param #128 timeout, retries left 2, and 1269 params still missing
[mavros.sys]:   VER: autopilot version service timeout
[mavros.mavros]: CON: Lost connection, HEARTBEAT timed out.
[mavros.cmd]:    CMD: Command 520 -- ack timeout
```

Nhưng:
* ArduPilot vẫn báo `Flight battery 100 percent` qua MAVProxy → FCU khỏe
* `odometry_node` vẫn `dropped=0` → link EKF ổn
* `/mavros/global_position` báo `lat=0, lon=0` — nhưng đó là **đúng**, vì
  `GPS1_TYPE 0` không có GPS

Nguyên nhân gốc: máy quá tải → **MAVProxy không relay heartbeat** → MAVROS
timeout. Không phải lỗi của code.

Dấu hiệu nhận biết: `load average` cao bất thường, và `ps` cho thấy nhiều
process cùng tên.

Cách xử lý:

```bash
src/uwb_pozyx_bridge/tools/cleanup_stack.sh
```

Script dừng toàn bộ stack (SIGTERM rồi SIGKILL cho process còn sót) và báo
lại load average. Sau đó launch lại sẽ **0 lỗi**.

> Lưu ý: `pkill -f mavproxy.py` / `pkill -f arducopter` **không đủ**. Phải dọn
> cả `parameter_bridge`, `robot_state_publisher`, `uvb_gz_to_ros_bridge` — đây
> mới là những process gây ngốn CPU.

### R24 — "ArduPilot controller has reset" là warning vô hại

```
[gz sim-1] [Wrn] [ArduPilotPlugin.cc:1599] ArduPilot controller has reset
```

Đây **không** phải lỗi. `ArduPilotPlugin` so sánh frame counter của SITL với
frame counter nó nhớ:

```cpp
if (pkt_frame_count < this->dataPtr->fcu_frame_count)
{
    /// \todo(anyone) implement re-initialisation
    gzwarn << "ArduPilot controller has reset\n";
}
```

Frame counter bị reset khi SITL khởi động lại (hoặc khi lock-step bị ngắt).
Message này xuất hiện 1 lần lúc boot là bình thường.

---


## 7. Đánh giá mức độ khả thi

**Kết luận: khả thi, đúng kiến trúc, không cần sửa ArduPilot.**

Ước lượng: **~2.5–3.5 ngày** công sức.


| Phase      | Công sức   | Rủi ro     | Nội dung                                                 |
| ---------- | ---------- | ---------- | -------------------------------------------------------- |
| **Tier 2** | 0.5 ngày   | Thấp       | Fake Arduino → SITL, verify EKF3 beacon fusion hoạt động |
| **Tier 1** | 0.5 ngày   | Thấp       | `pozyx_protocol.py` + unit test byte layout              |
| **Tier 3** | 1–1.5 ngày | Trung bình | Node Python đầy đủ + trilateration                       |
| **Tier 4** | 0.5–1 ngày | Cao        | Nối Gazebo, anchor frame mapping, ENU/NED                |
| **Tier 5** | 0.5 ngày   | Trung bình | Verify EKF params + flight test                          |


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
src/uwb_pozyx_bridge/              ← package chính
├── uwb_pozyx_bridge/
│   ├── pozyx_protocol.py          ← encode 3 message type (pure, testable)
│   ├── pozyx_sender.py            ← TCP/UDP transport + reconnect
│   ├── pozyx_node.py              ← node chính: subscribe + trilaterate + send
│   └── trilateration.py           ← WLS solver (port từ nhánh cũ)
├── config/
│   └── x500_uwb.parm              ← SITL params hợp nhất (x500 + beacon)
├── launch/
│   └── x500_uwb_runway.launch.py  ← launch chính, kế thừa x500_runway
├── tools/
│   ├── compare_positions.py       ← gate: 3 nguồn vị trí khớp nhau
│   ├── verify_fdm_coupling.py     ← gate: SITL thực sự điều khiển Gazebo
│   ├── sitl_beacon_monitor.py     ← set params runtime (không dùng mặc định)
│   ├── inspect_frames.py          ← gate 5.2 frame mapping
│   └── run_pipeline_verify.sh     ← chạy cả 3 gate một lần
├── test/
│   └── test_pozyx_protocol.py     ← unit test byte layout
├── package.xml
├── setup.cfg
└── setup.py
```



### Params hợp nhất

`config/x500_uwb.parm` gộp ba nguồn, theo thứ tự sau (file sau thắng file trước):

1. `ardupilot/Tools/autotest/default_params/copter.parm` — mặc định copter
2. `ardupilot_gazebo/config/gazebo-x500.parm` — tuning x500, PID, frame
3. Beacon params (BCN_*, EK3_SRC1_*, GPS off, SERIAL1_PROTOCOL=13)

SITL nhận nhiều file qua `--defaults` phân tách bằng dấu phẩy
(`AP_Param::load_defaults_file()` tách bằng `strtok_r` trên `,`).

Hai điểm khác biệt so với `beacon.parm` cũ:

- `ARMING_CHECK 0` thay vì `1048567`. Bộ x500 vốn đã tắt toàn bộ pre-arm
check, nên giá trị 0 khớp với baseline. Giá trị cũ chỉ xóa bit "GPS
lock" nhưng vẫn giữ INS/compass/RC — chặt hơn baseline và không cần.
- `FLTMODE1..6` đều trỏ tới `4` (GUIDED). Bản gốc map sang Circle, Land,
RTL, Auto, Loiter, Stabilise. **Auto không arm được từ GCS** và **Land
disarm ngay**, nên không thể cất cánh bằng MAVLink. GUIDED arm được và
là mode đúng cho GCS.



### Launch file

`launch/x500_uwb_runway.launch.py` **kế thừa** `ardupilot_gz_bringup/x500_runway.launch.py`
thay vì viết lại. Nhờ vậy có sẵn Gazebo GUI, RViz, các bridge Gazebo↔ROS,
MAVROS (tắt mặc định) và MAVProxy.

Các thay đổi trong submodule `src/ardupilot_gz`:


| File                            | Thay đổi                                                                              |
| ------------------------------- | ------------------------------------------------------------------------------------- |
| `launch/robots/robot.launch.py` | Thêm launch arg `serial0..serial9`, forward sang SITL                                 |
| `launch/robots/x500.launch.py`  | Khai báo và forward `serial0..serial9`                                                |
| `launch/x500_runway.launch.py`  | Thêm launch arg `world`, `x/y/z/R/P/Y`, `defaults`, `use_mavros`; forward xuống robot |


SITL cần `--model json` để nối FDM port 9002 với `ArduPilotPlugin` trong
`x500_with_ardupilot`. Vehicle phải spawn model **có** ArduPilotPlugin
(`x500_with_ardupilot`), không phải `x500_base` — xem R16.

### Tái dùng / loại bỏ từ nhánh `feat/uwb-odometry`


| File                                          | Hành động                | Lý do                                                     |
| --------------------------------------------- | ------------------------ | --------------------------------------------------------- |
| `src/uwb_gz_plugin/**`                        | ✅ **Giữ nguyên toàn bộ** | Plugin mô phỏng radio, layout đã khớp Pozyx               |
| `src/uwb_gz_plugin/src/ros_gz_bridge_node.cc` | ✅ Giữ                    | Bắt buộc — `ros_gz_bridge` factory-lookup failure         |
| `uwb_trilateration_node.py` (logic WLS)       | 🔄 Port                  | Bỏ phần publish MAVROS, giữ solver                        |
| `mavlink_bridge_node.py`                      | ❌ Bỏ                     | Thay bằng pozyx protocol sender                           |
| `uwb_to_mavros_vision_node.py`                | ❌ Bỏ                     | Không dùng MAVROS nữa                                     |
| `config/copter-uwb.parm`                      | ❌ Bỏ                     | Param sai version (`GPS_TYPE`, `VISO_TYPE` không tồn tại) |
| `launch/mavros_vision.launch.py`              | ❌ Bỏ                     | Không dùng MAVROS                                         |
| `launch/step5_sitl_end_to_end.launch.py`      | 🔄 Viết lại              | Cho pipeline mới                                          |


---



## 11. Danh sách kiểm tra verify



### Chạy tự động (khuyến nghị)

```bash
cd ~/ros2_project/AerialVehicleControl
bash src/uwb_pozyx_bridge/tools/run_pipeline_verify.sh
```

Script tự launch toàn bộ stack và chạy 3 gate:


| Gate | Kiểm tra                             | Tool                     |
| ---- | ------------------------------------ | ------------------------ |
| 1    | Ba nguồn vị trí khớp nhau            | `compare_positions.py`   |
| 2    | SITL thực sự điều khiển Gazebo       | `verify_fdm_coupling.py` |
| 3    | MAVProxy forward tới UDP 14550/14551 | inline                   |


Kết quả mong đợi:

```
=== summary ===
  FDM coupling : PASSED
  positions    : PASSED
  GCS link     : PASSED

PIPELINE: PASSED
```



### Chạy tay

```bash
# Dọn instance cũ trước, nếu không SITL sẽ không bind được port 5760
pkill -f "x500_uwb_runway"; pkill -f mavros_node; pkill -f odometry_node; pkill -f mavproxy.py; pkill -f arducopter; pkill -f "gz sim"; pkill -f parameter_bridge; pkill -f robot_state_publisher; pkill -f uwb_gz_to_ros_bridge; pkill -f rviz2;

# Launch toàn bộ stack (Gazebo GUI + RViz + SITL + UWB)
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch uwb_pozyx_bridge x500_uwb_runway.launch.py
```

Trong terminal khác:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash

# Ranging từ Gazebo: 4 anchor × 20 Hz
ros2 topic hz /uwb/ranging_ros

# Vị trí trilateration
ros2 topic echo /uwb/pose --once

# Vị trí thật trong Gazebo
ros2 topic echo /uwb/tag_pose_ros --once

# EKF dùng beacon
ros2 topic echo /mavros/...   # chỉ khi use_mavros:=true
```

Mission Planner: chạy `mono ~/Downloads/MissionPlanner-latest/MissionPlanner.exe`,
chọn **UDP (copter)** với port **14551**.

### Checklist thủ công

- [ ] SITL log có `JSON control interface set to 127.0.0.1:9002`
- [ ] Gazebo log có `Loaded system [ArduPilotPlugin]` và `Computed IMU topic`
- [ ] MAVProxy log có `EKF3 IMU0 is using range beacons`
- [ ] `pozyx_node` log có `tx: solutions=N` tăng đều, `dropped=0`
- [ ] `ros2 topic hz /uwb/ranging_ros` ra ~80 Hz (4 anchor × 20 Hz)
- [ ] `ros2 topic hz /uwb/pose` ra ~20 Hz
- [ ] UDP 14550 và 14551 đều có heartbeat
- [ ] Drone cất cánh được: Gazebo và EKF cùng lên (~3 m)



### Tier 2

- [ ] SITL khởi động, `param show BCN_TYPE` = 1
- [ ] `param show EK3_SRC1_POSXY` = 4
- [ ] `AP_Beacon` healthy = true (không có API trực tiếp → xem log `BEA`)
- [ ] Log `BEA`: `dist0..3` khác 0, `posx/y/z` khác 0
- [ ] `ekf_status`: vertical position / horizontal position = true
- [ ] Sau ~3s: EKF không báo "stopped aiding"



### Tier 4

- [ ] `/uwb/ranging_ros` có data (4 anchor × 20 Hz)
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

