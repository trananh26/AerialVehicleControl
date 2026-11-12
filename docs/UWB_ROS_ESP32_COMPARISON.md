# Đối chiếu MAVLink `ODOMETRY`: ROS (`uwb_pozyx_bridge`) ↔ ESP32 (`UWB_RTLS_Tag`)

Tài liệu này so sánh hai bản hiện thực độc lập của cùng một giao thức — bản tin
MAVLink `ODOMETRY` (id 331) gửi từ một nguồn vị trí UWB tới EKF3 của ArduPilot:

| | Bên A — ROS | Bên B — ESP32 |
| :-- | :-- | :-- |
| Repo | `AerialVehicleControl` (branch `x500-uwb`) | `esp32-uwb-localization` |
| Package | `uwb_pozyx_bridge` (`odometry_node.py`) | `firmware/ESP32-DWM3000-UWB/UWB_RTLS_Tag` (`mavlink_output.cpp`) |
| Encoder | `build_odometry_message()` — `pymavlink` dialect `v20.common` | `MAVLinkOdomOutput::send_odometry()` — header C `okalachev/MAVLink@2.0.35` |
| Link | TCP → SITL serial port 5761 | UART Serial2 (GPIO 17 TX / 16 RX) → Pixhawk 6C TELEM2 |
| Rate | 20 Hz (`send_rate`) | ~15–20 Hz (1 vòng DS-TWR / 4 anchor) |

> **Trạng thái: đã đồng bộ (2026-11).** Cả **7 điểm khác biệt** dưới đây đã
> được sửa ở **cả hai bên**, và đã kiểm chứng lại bằng:
>
> - 61 unit test của `uwb_pozyx_bridge` (ROS) — pass
> - host test cho `trilateration.cpp` (ESP32) — khớp số với solver ROS
> - `tools/compare_odometry_frames.py` — so khớp **từng trường** ODOMETRY hai bên
> - E2E SITL: `takeoff_land_mission` ✅, `circle_mission` ✅ (sai số bám trung bình 0.12 m)
> - E2E SITL với **layout chữ nhật** (14×7, 12×8+α=30°) ✅ — xem §11.1
>
> Tài liệu giữ lại **cả hiện trạng ban đầu lẫn trạng thái sau khi sửa**, để làm
> tài liệu đối chiếu cho các phiên bản cũ.

---

## 0. Tóm tắt nhanh

**Wire format: giống hệt byte-for-byte.** Đã kiểm chứng bằng cách compile header
MAVLink của ESP32 trên host, encode cùng một bộ giá trị với
`build_odometry_message()`, rồi so từng byte:

```
ESP32 len=245 | PY len=245
>>> IDENTICAL BYTES <<<
```

Cả hai đều là MAVLink 2 (magic `0xFD`), payload 233 byte (`MAVLINK_MSG_ID_ODOMETRY_LEN`),
CRC extra `91`, header 10 byte. Vì vậy **ArduPilot không thể phân biệt** bản tin nào
từ bản tin nào ở tầng framing.

Tuy nhiên **ngữ nghĩa các trường bên trong ban đầu không tương đương**. 8 điểm khác
biệt được liệt kê dưới đây, xếp theo mức độ nghiêm trọng.

| # | Điểm khác biệt | Mức độ | Trạng thái sau khi sửa |
| :-- | :-- | :-- | :-- |
| 1 | Hệ toạ độ anchor / trục X-Y | 🔴 **chặn** | ✅ Anchor Frame hình chữ nhật X×Y + yaw offset α — xem [`UWB_ANCHOR_LAYOUT.md`](UWB_ANCHOR_LAYOUT.md) |
| 2 | `time_usec`: boot-time vs Unix epoch | 🔴 cao | ✅ Cả 2 dùng Unix epoch (ESP32 đồng bộ qua `SYSTEM_TIME`) |
| 3 | `vx/vy/vz`: `NaN` vs `0.0` | 🟠 trung bình | ✅ Cả 2 gửi `NaN` |
| 4 | Thiếu `SET_GPS_GLOBAL_ORIGIN` / `DO_SET_HOME` | 🔴 cao | ✅ Cả 2 đều gửi; `sysid = 255`, `compid = 191` |
| 5 | Nghiệm Z (anchor đồng phẳng → 2 nghiệm đối xứng) | 🟠 trung bình | ✅ Cả 2 giải **3D chọn nhánh** theo nghiệm trước |
| 6 | `estimator_type`: VIO (2) vs NAIVE (1) | 🟢 thấp | ✅ Cả 2 dùng `MAV_ESTIMATOR_TYPE_NAIVE` (1) |
| 7 | `velocity_covariance[0]`: `NaN` vs `0.0` | 🟢 thấp | ✅ Cả 2 đặt `[0] = NaN` |
| 8 | `q[4]`: identity `(1,0,0,0)` vs identity | 🔴 **cao** | ✅ Cả 2 gửi quaternion **toàn NaN** — xem mục 11.2 |

---

## 1. So sánh trường-by-trường

Cột cuối là **trạng thái hiện tại** (sau khi đồng bộ). Cột "trước" ghi lại lệch
nhau ban đầu để đối chiếu.

| Trường MAVLink | ESP32 `send_odometry()` | ROS `build_odometry_message()` | Trước | Nay |
| :-- | :-- | :-- | :-- | :-- |
| `time_usec` (uint64) | `time_usec_unix()` — **Unix epoch** | `time.time_ns() // 1000` — **Unix epoch** | ❌ | ✅ |
| `frame_id` (uint8) | `MAV_FRAME_LOCAL_FRD` (20) | `MAV_FRAME_LOCAL_FRD` (20) | ✅ | ✅ |
| `child_frame_id` (uint8) | `MAV_FRAME_BODY_FRD` (12) | `MAV_FRAME_BODY_FRD` (12) | ✅ | ✅ |
| `x, y, z` (float) | NED sau `convertLocalToNed()` | NED sau `AnchorLayout.to_ned_3d()` | ⚠️ mục 1 | ✅ |
| `q[4]` (float) | toàn `NaN` `[NAN,NAN,NAN,NAN]` | toàn `NaN` — `unknown_attitude_quaternion()` | ❌ mục 8 | ✅ |
| `vx, vy, vz` (float) | **`NAN, NAN, NAN`** | **`NAN, NAN, NAN`** | ❌ mục 3 | ✅ |
| `rollspeed, pitchspeed, yawspeed` | `0,0,0` | `0,0,0` | ✅ | ✅ |
| `pose_covariance[21]` | `[0]=[6]=[11]=var(Q)` | `[0]=[6]=[11]=std²` | ✅ cùng layout | ✅ |
| `velocity_covariance[21]` | tất cả `0`, trừ `[0] = NaN` | tất cả `0`, trừ `[0] = NaN` | ❌ mục 7 | ✅ |
| `reset_counter` (uint8) | `0` (cứng) | tăng khi vị trí nhảy > `reset_jump_threshold_m` | ✅ ROS hơn | ✅ (giữ nguyên) |
| `estimator_type` (uint8) | `MAV_ESTIMATOR_TYPE_NAIVE` (1) | `MAV_ESTIMATOR_TYPE_NAIVE` (1) | ❌ mục 6 | ✅ |
| `quality` (int8) | `clamp(Q*100, 1, 100)` | `int` từ param `quality` (mặc định 50) | ✅ | ✅ |
| `sysid` / `compid` | `255` / `191` | `255` / `191` | ❌ mục 4 | ✅ |
| Heartbeat type | `MAV_TYPE_ONBOARD_CONTROLLER` | `MAV_TYPE_GCS` | ⚠️ mục 4 | ⚠️ (không ảnh hưởng) |
| Heartbeat rate | 1 Hz | 1 Hz | ✅ | ✅ |

### Ràng buộc frame mà ArduPilot áp đặt (cả hai đều thoả)

```3879:3888:src/ardupilot/libraries/GCS_MAVLink/GCS_Common.cpp
    if (m.frame_id != MAV_FRAME_LOCAL_FRD ||
        m.child_frame_id != MAV_FRAME_BODY_FRD) {
        // only support local FRD frame data
        return;
    }
```

Cả hai bên đều dùng đúng `20` / `12` ⇒ không bị drop.

---

## 2. Mục 1 — Hệ toạ độ (🔴 chặn, ưu tiên cao nhất)

### Quy ước chung sau khi sửa: Anchor Frame

Hai bên cùng dùng **Anchor Frame** — hệ tọa độ mà solver trilateration làm việc:

```
a0 = (0, 0, 0)        a1 = (X, 0, 0)      X = 10 m (mặc định)
a3 = (0, Y, 0)        a2 = (X, Y, 0)      Y = 10 m (mặc định)
```

* `x` hướng về `a1`, `y` hướng về `a3`, `z` hướng **xuống** (mặt phẳng anchor là `z = 0`)
* Layout **không bắt buộc là hình vuông**: `X` và `Y` là hai số độc lập.
* Khoảng cách đo **bất biến** dưới phép xoay quanh trục đứng ⇒ solver luôn dùng
  hình chữ nhật lý tưởng, chỉ **xoay kết quả** trước khi gửi đi.

Góc $\alpha$ (`config_yaw_offset_deg` bên ESP32, `alpha_deg` bên ROS) là **hướng của
`a0 → a1`, đo từ hướng Bắc, tính theo chiều kim đồng hồ nhìn từ trên xuống**.

```
N = x·cos(α) − y·sin(α)
E = x·sin(α) + y·cos(α)
D = z − anchor_plane_height_m
```

### ROS

Anchor trong world Gazebo (ENU, mét) — hình chữ nhật `X × Y` trên `z = 2.0`.
File `uwb_runway.sdf` là **template**; vị trí anchor được **sinh lại** theo layout
mỗi lần launch bởi `uwb_pozyx_bridge/uwb_world.py::render_world`:

```
      <anchor name="uwb_anchor0"><pos>0 0 2.0</pos></anchor>
      <anchor name="uwb_anchor1"><pos>0 10 2.0</pos></anchor>
      <anchor name="uwb_anchor2"><pos>10 10 2.0</pos></anchor>
      <anchor name="uwb_anchor3"><pos>10 0 2.0</pos></anchor>
```

Nguồn sự thật duy nhất cho layout là `uwb_pozyx_bridge/uwb_layout.py`:

```python
@dataclass(frozen=True)
class AnchorLayout:
    size_x_m: float = LAYOUT_SIZE_X_M        # 10.0  (cạnh a0 → a1)
    size_y_m: float = LAYOUT_SIZE_Y_M        # 10.0  (cạnh a0 → a3)
    alpha_deg: float = 0.0
    plane_height_m: float = ANCHOR_PLANE_HEIGHT_M   # 2.0
    origin_north_m: float = 0.0
    origin_east_m: float = 0.0
```

Module này cung cấp:

* `anchor_frame_positions()` — hình chữ nhật lý tưởng cho solver
* `rotate_to_ned()` — xoay ngang theo $\alpha$
* `to_ned_3d()` — xoay + dịch $z$ theo chiều cao mặt phẳng anchor
* `centre_enu()` — tâm hình chữ nhật (spawn cố định)
* `random_spawn_enu()` — điểm spawn ngẫu nhiên **bên trong** hình chữ nhật

Launch file nhận `layout_size_x_m` / `layout_size_y_m` / `layout_alpha_deg`,
sinh lại world SDF và truyền cùng bộ tham số đó cho node ⇒ plugin, model
anchor hiển thị, solver và vị trí spawn **luôn khớp nhau**. Nó cũng dùng
`random_spawn` + `spawn_seed` để đặt drone ở một điểm ngẫu nhiên trong hình
chữ nhật anchor (lệch vào trong một chút so với biên để không dính anchor).

### ESP32

Mặc định được dựng từ **hai** hằng số trong `uwb_config.h`:

```c
#define CFG_DEFAULT_LAYOUT_SIZE_X_M 10.0f   // cạnh a0 → a1
#define CFG_DEFAULT_LAYOUT_SIZE_Y_M 10.0f   // cạnh a0 → a3
#define CFG_DEFAULT_YAW_OFFSET_DEG     0.0f
#define CFG_DEFAULT_ANCHOR_PLANE_HEIGHT_M 2.0f
```

Lệnh cấu hình qua Serial:

```
cfg,x1,y1,z1,x2,y2,z2,x3,y3,z3,x4,y4,z4,yaw[,plane_height]
```

Ví dụ cho layout 10×10 m đồng phẳng, xoay 20° so với Bắc, mặt phẳng cao 2 m:

```
cfg,0,0,0,10,0,0,10,10,0,0,10,0,20,2
```

Ví dụ cho layout **hình chữ nhật** 14 m × 7 m:

```
cfg,0,0,0,14,0,0,14,7,0,0,7,0,20,2
```

### Kết quả

* Solver của **cả hai bên** chạy trên **cùng một hình chữ nhật X × Y** ⇒ nghiệm
  trilateration giống hệt nhau, chỉ khác nhau ở phép xoay cuối.
* Layout **không bắt buộc là hình vuông** — cả hai bên đều lấy `X` và `Y` từ
  cùng một cặp hằng số cấu hình.
* `alpha = 0` mặc định ở cả hai bên ⇒ layout thẳng hướng Bắc/Đông.
* `z` cùng được trừ `anchor_plane_height_m` ⇒ vị trí gửi đi luôn là **tương đối
  với EKF Origin** (mặt đất), giống nhau.

Chi tiết đầy đủ (bảng chuyển đổi layout, quy trình hiệu chỉnh yaw, checklist
trước khi bay) nằm ở [`UWB_ANCHOR_LAYOUT.md`](UWB_ANCHOR_LAYOUT.md).

---

## 3. Mục 2 — `time_usec`: boot-time vs Unix epoch (🔴 cao)

### ESP32 — trước khi sửa

```318:319:firmware/ESP32-DWM3000-UWB/UWB_RTLS_Tag/src/main.cpp
    MAVLinkOdomOutput::send_odometry(
        micros(),
```

`micros()` trên ESP32 trả `unsigned long` đếm từ lúc boot → giá trị nhỏ
(0 … 2³² µs ≈ 71 phút), **không phải** Unix epoch.

### ESP32 — sau khi sửa

Module mới `time_sync.h/.cpp`:

* đồng hồ **monotonic 64-bit** từ `esp_timer_get_time()` (không bị đổi khi NTP chỉnh
  lệch ⇒ không nhảy ngược)
* offset tới Unix epoch được lấy từ bản tin MAVLink **`SYSTEM_TIME`** (id 111) mà
  autopilot gửi xuống
* `time_usec_unix()` = `esp_timer_get_time() / 1000 + epoch_offset_us`

```215:215:firmware/ESP32-DWM3000-UWB/UWB_RTLS_Tag/src/mavlink_output.cpp
        time_usec_unix()                       // time_usec: Unix epoch
```

### ROS

```304:308:src/uwb_pozyx_bridge/uwb_pozyx_bridge/odometry_node.py
        message = build_odometry_message(
            north_m,
            east_m,
            down_m,
            time.time_ns() // 1000,
```

### Vì sao quan trọng

ArduPilot dùng `JitterCorrection` để map timestamp "offboard" về clock local:

```50:71:src/ardupilot/libraries/AP_RTC/JitterCorrection.cpp
uint64_t JitterCorrection::correct_offboard_timestamp_usec(uint64_t offboard_usec, uint64_t local_usec)
{
    int64_t diff_us = int64_t(local_usec) - int64_t(offboard_usec);

    if (!initialised ||
        diff_us < link_offset_usec) {
        // this message arrived from the remote system with a
        // timestamp that would imply the message was from the
        // future. We know that isn't possible, so we adjust down the
        // correction value
        link_offset_usec = diff_us;
        initialised = true;
    }

    int64_t estimate_us = offboard_usec + link_offset_usec;

    if (estimate_us + (max_lag_ms*1000U) < int64_t(local_usec)) {
        // this implies the message came from too far in the past. clamp the lag estimate
        // to assume the message had maximum lag
        estimate_us = local_usec - (max_lag_ms*1000U);
        link_offset_usec = estimate_us - offboard_usec;
    }
```

- `max_lag_ms = 500` (mặc định của `JitterCorrection`).
- Với **epoch** (cả 2 bên): `diff_us` = độ trễ thực (vài chục ms) ⇒ timestamp chính xác.
- Với **boot-time** (ESP32 cũ): `diff_us` = hàng trăm triệu µs ⇒ ArduPilot clamp về
  `local_usec - 500 ms` ⇒ **timestamp luôn bị lệch**, và `link_offset_usec` bị kéo
  theo → độ trễ ước lượng sai.

### Ghi chú thêm

ArduPilot còn có timeout cứng cho EXTNAV:
`AP_VISUALODOM_TIMEOUT_MS = 300 ms` ⇒ `send_rate` phải **> 3.33 Hz**, khuyến nghị
**10–20 Hz**. Cả hai bên đều đang ở 15–20 Hz ⇒ OK.

---

## 4. Mục 3 — `vx/vy/vz`: `NaN` vs `0.0` (🟠 trung bình)

### ESP32 — đúng (không đổi)

```335:336:firmware/ESP32-DWM3000-UWB/UWB_RTLS_Tag/src/main.cpp
    float vel_cov[21] = {0};
    vel_cov[0] = NAN;
```

```356:356:firmware/ESP32-DWM3000-UWB/UWB_RTLS_Tag/src/main.cpp
        NAN, NAN, NAN,
```

ArduPilot reject mẫu NaN:

```1114:1117:src/ardupilot/libraries/AP_NavEKF3/AP_NavEKF3_Measurements.cpp
    // sanity check for NaNs
    if (vel.is_nan() || isnan(err)) {
        return;
    }
```

⇒ `useExtNavVel` **không** được set ⇒ EKF3 tự tích phân vận tốc bằng IMU.

### ROS — đã sửa

```209:209:src/uwb_pozyx_bridge/uwb_pozyx_bridge/mavlink_odometry.py
    velocity_ned: Sequence[float] = (float("nan"),) * 3,
```

Gửi `NaN` ⇒ **bị** reject ⇒ `useExtNavVel = false` ⇒ EKF3 fuse vận tốc 0 m/s.

### Ảnh hưởng (nếu để sai)

Nếu ROS gửi `0.0` ⇒ **không** bị reject ⇒ `useExtNavVel = true` ⇒ EKF3 fuse vận tốc
0 m/s. Ngoài ra còn mở nhánh Z:

```336:338:src/ardupilot/libraries/AP_NavEKF3/AP_NavEKF3_PosVelFusion.cpp
    } else if (inFlight && useExtNavVel && (activeHgtSource == AP_NavEKF_Source::SourceZ::EXTNAV)) {
        stateStruct.velocity.z = extNavVelDelayed.vel.z;
```

Với `EK3_SRC1_POSZ = 1` (BARO) thì nhánh này không kích hoạt, nhưng nếu ai đó đổi
`EK3_SRC1_POSZ` sang `EXTNAV` ⇒ drone bị **đóng băng vận tốc Z**.

---

## 5. Mục 4 — `SET_GPS_GLOBAL_ORIGIN` / `DO_SET_HOME` (🔴 cao)

### ROS — có đầy đủ

```402:412:src/uwb_pozyx_bridge/uwb_pozyx_bridge/odometry_node.py
        if not self._origin_sent or now - self._origin_sent_at >= ORIGIN_REFRESH_S:
            message = build_set_gps_global_origin_message(
                float(self.get_parameter("origin_latitude_deg").value),
                float(self.get_parameter("origin_longitude_deg").value),
                float(self.get_parameter("origin_altitude_m").value),
```

kèm `build_do_set_home_message()` và `build_request_message_command()` để xác
nhận home đã được set.

### ESP32 — đã bổ sung

`mavlink_output.cpp` giờ có:

* `send_set_gps_global_origin()` — id 48, khai báo EKF origin
* `send_do_set_home()` — `MAV_CMD_DO_SET_HOME` (179) qua `COMMAND_LONG`,
  `param5/6/7` = lat/lon/alt
* `handle_incoming()` — parse `COMMAND_ACK`, `HOME_POSITION` (242),
  `GPS_GLOBAL_ORIGIN` (243) và `SYSTEM_TIME` (111)

### Vì sao `sysid` phải là 255

ArduPilot chỉ nhận `SET_GPS_GLOBAL_ORIGIN` từ đúng sysid của GCS:

```6776:6794:src/ardupilot/libraries/GCS_MAVLink/GCS_Common.cpp
    if (msg.sysid == mavlink_system.sysid) {
        // accept packets from our own components
        // (e.g. mavlink-connected companion computers)
        return true;
    }

    if (msg.sysid == sysid_my_gcs()) {
        return true;
    }
    ...
    if (!sysid_enforce()) {
        return true;
    }
```

⇒ ESP32 đổi `MAVLINK_SYSTEM_ID` từ `200` → **`255`** (`SYSID_MYGCS`).
`MAVLINK_COMPONENT_ID` = `191` (`MAV_COMP_ID_ONBOARD_COMPUTER`), giống ROS.

### Vì sao origin + home là bắt buộc

ArduPilot cần EKF origin + home để:

- `AP_AHRS::get_origin()` — chuyển đổi height giữa "above origin" và "above home".
- `ModeGuided::do_user_takeoff_start()` — `MAV_CMD_NAV_TAKEOFF` trả
  `MAV_RESULT_FAILED` nếu không có home ⇒ **drone arm được nhưng không cất cánh**.

Tag ESP32 đứng riêng sẽ **không bao giờ takeoff được** nếu thiếu origin/home.

> **Thứ tự quan trọng:** phải khai báo **origin trước**, rồi mới **home**.

> Lưu ý: đổi `sysid` ảnh hưởng heartbeat. `MAVLink_routing::learn_route()` bỏ qua
> route có `sysid+compid` trùng autopilot ⇒ `255` là lựa chọn đúng vì cần nhận
> lệnh như GCS.

---

## 6. Mục 5 — Nghiệm Z với anchor đồng phẳng (🟠 trung bình)

### Vấn đề

Anchor nằm cùng một mặt phẳng (z = 0 trong Anchor Frame) ⇒ chỉ từ khoảng cách đo
được thì **không xác định được** tag nằm ở phía nào của mặt phẳng: một tag 1 m
*dưới* mặt phẳng cho ra đúng những khoảng cách như một tag 1 m *trên* mặt phẳng.

### Trước khi sửa

| Bên | Cách làm | Vấn đề |
| :-- | :-- | :-- |
| ROS | giải 2D, giữ `z` cố định | sai số ngang tới 0.19 m khi độ cao thật lệch |
| ESP32 | giải 3D, `TRI_Z_DAMP = 1e-3`, multi-start chọn residual thấp nhất | nhánh có thể nhảy ngẫu nhiên |

### Sau khi sửa — 3D Gauss-Newton + chọn nhánh tường minh

Cả hai bên giờ dùng **cùng một thuật toán**:

1. Giải 3D Gauss-Newton từ **hai hạt giống**, một ở mỗi phía của mặt phẳng anchor
   (multi-start tại $z_{plane} \pm \text{span}$).
2. Chọn hạt giống cho **nghiệm gần nghiệm trước** (`previous_position`) nhất ⇒
   giữ đúng nhánh khi tag bay xuyên qua mặt phẳng.
3. Nếu chưa có nghiệm trước, chọn theo residual nhỏ nhất.

⇒ ước lượng **liên tục** khi cắt mặt phẳng anchor, và sai số ngang giữ ở mức nhiễu
đo bất kể độ cao thật.

### Số đo trên hình chữ nhật 10×10 m

| Chiều cao thật | Sai số ngang (2D, Z cố định) | Sai số ngang (3D, chọn nhánh) |
| :-- | :-- | :-- |
| 0.30 m | 0.074 m / p99 0.190 m | 0.045 m / p99 0.113 m |
| 0.80 m | 0.067 m / p99 0.170 m | 0.046 m / p99 0.112 m |
| 1.50 m | 0.050 m / p99 0.124 m | 0.047 m / p99 0.118 m |

Host test cho `trilateration.cpp` (ESP32) cho kết quả **khớp số** với solver ROS:
0.00 m (không nhiễu), 0.03 m (nhiễu 0.05 m), 0.10 m (bước mượt xuyên mặt phẳng).

> **Vì sao không giữ Z cố định?** Việc "giải 2D với Z cố định" *có vẻ* đơn giản hơn,
> nhưng nó **không trung lập**: khoảng cách đo được sẽ **không nhất quán** với giả định
> về độ cao, và nghiệm ngang bị lệch.

---

## 7. Mục 6 — `estimator_type` (🟢 thấp)

Trước khi sửa:

| | Giá trị |
| :-- | :-- |
| ESP32 | `MAV_ESTIMATOR_TYPE_NAIVE` (1) |
| ROS | `MAV_ESTIMATOR_TYPE_VIO` (2) |

MAVLink 2.x **không có** `MAV_ESTIMATOR_TYPE_UWB`. Giá trị `7` đã bị chiếm bởi
`MAV_ESTIMATOR_TYPE_LIDAR`.

ArduPilot **bỏ qua hoàn toàn** trường này trong `handle_odometry()`:

```3893:3902:src/ardupilot/libraries/GCS_MAVLink/GCS_Common.cpp
    Quaternion q{m.q[0],m.q[1],m.q[2],m.q[3]};

    float posErr = 0;
    float angErr = 0;
    if (!isnan(m.pose_covariance[0])) {
        posErr = cbrtf(sq(m.pose_covariance[0])+sq(m.pose_covariance[6])+sq(m.pose_covariance[11]));
        angErr = cbrtf(sq(m.pose_covariance[15])+sq(m.pose_covariance[18])+sq(m.pose_covariance[20]));
    }

    const uint32_t timestamp_ms = correct_offboard_timestamp_usec_to_ms(m.time_usec, PAYLOAD_SIZE(chan, ODOMETRY));
    visual_odom->handle_pose_estimate(m.time_usec, timestamp_ms, m.x, m.y, m.z, q, posErr, angErr, m.reset_counter, m.quality);
```

⇒ **không có hành vi khác biệt** ở ArduPilot. Nhưng để hai bên **giống nhau tuyệt
đối** (và đúng nghĩa: solver trilateration là một estimator *naive*, không phải VIO),
ROS cũng đổi sang `MAV_ESTIMATOR_TYPE_NAIVE` (1).

---

## 8. Mục 7 — `velocity_covariance[0]` (🟢 thấp)

- ESP32: `velocity_covariance[0] = NAN`, phần còn lại `0`.
- ROS (sau khi sửa): `velocity_covariance[0] = NaN`, phần còn lại `0`.

```196:200:src/uwb_pozyx_bridge/uwb_pozyx_bridge/mavlink_odometry.py
    cov = [0.0] * 21
    cov[0] = float("nan")
    return cov
```

MAVLink spec: *"If unknown, assign NaN value to first element in the array."*

ArduPilot cũng **không đọc** `velocity_covariance` (nó dùng `m.pose_covariance` cho cả
`posErr` lẫn `angErr`). Vậy nên chỉ là khác biệt về hình thức — nhưng đồng bộ cả hai
bên sang `NaN` đúng spec và đúng nghĩa "không có ước lượng vận tốc".

---

## 9. Ghi chú chung về covariance (2 bên đã khớp)

Cả hai đều đặt variance ở đúng 3 chỉ số mà ArduPilot đọc:

| Chỉ số | Ý nghĩa |
| :-- | :-- |
| `cov[0]` | phương sai X |
| `cov[6]` | phương sai Y |
| `cov[11]` | phương sai Z |
| `cov[15]`, `cov[18]`, `cov[20]` | phương sai roll / pitch / yaw |

ArduPilot **gộp 3 trục thành 1 sai số cầu** rồi dùng cho cả XY lẫn Z:

```3896:3899:src/ardupilot/libraries/GCS_MAVLink/GCS_Common.cpp
    if (!isnan(m.pose_covariance[0])) {
        posErr = cbrtf(sq(m.pose_covariance[0])+sq(m.pose_covariance[6])+sq(m.pose_covariance[11]));
        angErr = cbrtf(sq(m.pose_covariance[15])+sq(m.pose_covariance[18])+sq(m.pose_covariance[20]));
    }
```

ESP32 cũng đã ghi chú đúng điều này (tránh chiến thuật PX4 "đặt `cov[14]=999`):

```52:69:firmware/ESP32-DWM3000-UWB/UWB_RTLS_Tag/src/quality_est.cpp
    // Z variance.
    //
    // IMPORTANT: ArduPilot does NOT consume per-axis covariance here. It
    // collapses the three axis variances into ONE spherical error:
    //   posErr = cbrt(cov[0]^2 + cov[6]^2 + cov[11]^2)
    // and then uses that single value for the XY fusion
    // (R_OBS[3] = sq(constrain(posErr, 0.01, 10))) as well as for Z.
    //
    // So publishing a huge Z variance (the "999 = let the barometer handle
    // altitude" trick, which is a PX4-ism) would poison the XY measurement
    // too: posErr would become ~100 m, get clamped to 10 m, and the EKF would
    // essentially discard the UWB position.
```

⇒ **Phần covariance là mẫu mực — 2 bên khớp hoàn toàn.**

Khác biệt nhỏ (giữ nguyên, vì cả hai đều hợp lệ): ESP32 suy ra variance từ *quality*
động (`std = 0.05 / Q`), ROS dùng `position_std_m` cố định (mặc định `0.05`).

---

## 10. Kiểm chứng wire format

### 10.1 Byte-level

Đã compile header MAVLink của ESP32 trên host (`okalachev/MAVLink@2.0.35`) và
encode cùng một bộ giá trị:

| | ESP32 (C) | ROS (Python) |
| :-- | :-- | :-- |
| magic | `0xFD` (MAVLink 2) | `0xFD` (MAVLink 2) |
| payload len | 233 | 233 |
| total len | 245 | 245 |
| CRC extra | 91 | 91 |

```
ESP32 len=245 | PY len=245
>>> IDENTICAL BYTES <<<
```

Frame đầu (14 byte header MAVLink 2):

```
FD E9 00 00 00 C8 BF 4B 01 00 79 DF 0D 86 48 70
```

- `FD` — STX MAVLink 2
- `E9` — payload len 233
- `00` — incompat flags
- `00` — compat flags
- `00` — seq 0
- `C8` — sysid 200 (giai đoạn trước khi đổi; nay là `FF` = 255)
- `BF` — compid 191
- `4B 01` — msgid 331
- `00 79` — checksum low/high

### 10.2 Field-level (script tái lại được)

```bash
cd src/uwb_pozyx_bridge
python3 tools/compare_odometry_frames.py
```

Script dựng frame tham chiếu (giá trị field lấy từ `mavlink_output.cpp` của ESP32)
bằng `pymavlink`, dựng frame ROS bằng `build_odometry_message()`, rồi so **từng field**
(`NaN` được so sánh bằng `math.isnan`).

### 10.3 Script tái lại kiểm chứng byte-level

```bash
# 1. Frame từ header C của ESP32
gcc -I .pio/libdeps/release/MAVLink \
    -I .pio/libdeps/release/MAVLink/mavlink \
    -o /tmp/gen_esp /tmp/gen_esp.c -lm && /tmp/gen_esp

# 2. Frame từ Python
/usr/bin/python3 -c "
import sys; sys.path.insert(0, 'src/uwb_pozyx_bridge')
from uwb_pozyx_bridge.mavlink_odometry import build_odometry_message
from pymavlink.dialects.v20 import common as mavlink
msg = build_odometry_message(1.5, -2.25, -0.75, 123456789012345,
                              position_std_m=0.0625,
                              velocity_ned=(float('nan'),)*3,
                              quality=80)
m = mavlink.MAVLink(None, srcSystem=255, srcComponent=191); m.seq = 0
print(' '.join('%02X' % b for b in msg.pack(m)))
"
```

> Lưu ý: phải dùng **python hệ thống** (`/usr/bin/python3`) — python trong
> `~/.platformio/penv` không có `pymavlink`.

---

## 11. Kiểm chứng hành vi (E2E)

| Bài kiểm tra | Kết quả |
| :-- | :-- |
| Unit test `uwb_pozyx_bridge` (80 test) | ✅ pass |
| Host test `trilateration.cpp` (ESP32) | ✅ khớp số solver ROS |
| `compare_odometry_frames.py` | ✅ frame giống nhau từng field |
| `takeoff_land_mission` | ✅ leo 1.4 m, hạ cánh |
| `circle_mission` | ✅ vòng tròn đầy đủ, bán kính TB 1.58 m, sai số bám TB **0.12 m** |

### 11.1 Verify với layout **chữ nhật** (2026-11)

Ba layout đã chạy E2E đầy đủ (launch → SITL → mission) để xác nhận phía ROS
không còn giả định "hình vuông":

| Layout | Anchor trong world ENU | `takeoff_land` | `circle_mission` — sai số bám |
| :-- | :-- | :-- | :-- |
| vuông 10×10, α = 0° | `(0,0,2) (0,10,2) (10,10,2) (10,0,2)` | ✅ leo 1.45 m | TB **0.125 m**, p95 0.244 m |
| **chữ nhật 14×7**, α = 0° | `(0,0,2) (0,14,2) (7,14,2) (7,0,2)` | ✅ leo 1.42 m | TB **0.111 m**, p95 0.256 m |
| **chữ nhật 12×8**, α = 30° | `(0,0,2) (6,10.39,2) (12.93,6.39,2) (6.93,-4,2)` | ✅ leo 1.44 m | TB **0.103 m**, p95 0.233 m |

⇒ Layout chữ nhật **và** layout xoay đều cho sai số tương đương (hoặc tốt hơn)
layout vuông: hình học đo tốt hơn khi cạnh dài hơn ⇒ góc nhìn tới các anchor
đa dạng hơn.

Lệnh dùng để verify:

```bash
ros2 launch uwb_pozyx_bridge x500_uwb_runway.launch.py \
    layout_size_x_m:=14.0 layout_size_y_m:=7.0 layout_alpha_deg:=30.0 \
    random_spawn:=true spawn_seed:=42
```

---

## 11.2 Mục 8 — Quaternion: **NaN**, không phải identity (🔴 cao)

Cả hai bên trước đây gửi quaternion **identity** `(1, 0, 0, 0)`. Điều này
**tiêm yaw = 0 rad vào EKF3** — xem [M2] trong
`esp32-uwb-localization/docs/CODE_REVIEW_FINDINGS.md`.

### Cơ chế (đối chiếu source ArduPilot 4.6.3)

`GCS_MAVLINK::handle_odometry()` truyền nguyên vẹn `q` xuống
`AP_VisualOdom_MAV::handle_pose_estimate()`, và `NavEKF3_core::writeExtNavData()`:

```1057:1107:src/ardupilot/libraries/AP_NavEKF3/AP_NavEKF3_Measurements.cpp
void NavEKF3_core::writeExtNavData(const Vector3f &pos, const Quaternion &quat, float posErr, float angErr, uint32_t timeStamp_ms, uint16_t delay_ms, uint32_t resetTime_ms)
{
    ...
    // store position data to buffer
    storedExtNav.push(extNavDataNew);

    // protect against attitude or angle being NaN
    if (!quat.is_nan() && !isnan(angErr)) {
        ftype roll_rad, pitch_rad, yaw_rad;
        quat.to_euler(roll_rad, pitch_rad, yaw_rad);
        yaw_elements extNavYawAngDataNew;
        extNavYawAngDataNew.yawAng = yaw_rad;              // <-- yaw is taken
        ...
        storedExtNavYawAng.push(extNavYawAngDataNew);
    }
}
```

Với `EK3_SRCx_YAW = EXTNAV`, `AP_NavEKF3_MagFusion.cpp` sẽ
`alignYawAngle()` (reset yaw EKF) rồi `fuseEulerYaw(EXTNAV)` liên tục.

### Hệ quả đo được trong simulation SITL

| case | quaternion | `EK3_SRC1_YAW` | heading đo được | kết luận |
| :-- | :-- | :-- | :-- | :-- |
| A | NaN | 6 (EXTNAV) | 92°–95° | không bị kéo về 0° |
| **B** | **identity** | **6 (EXTNAV)** | **0.0° (138/138 mẫu)** | **ghim về Bắc — bug** |
| C | NaN | 1 (Compass) | 90°–91° | bình thường |
| D | identity | 1 (Compass) | 91° | bình thường |

Bay thật với **case B**: `EKF3 IMU0/IMU1 yaw aligned` → `EKF3 lane switch 1` →
vị trí nhảy `238148 m` → `647131944 m` → **ArduPilot crash** (`Floating point
exception`).

### Fix

Cả hai bên gửi quaternion **toàn NaN**:

```python
# src/uwb_pozyx_bridge/uwb_pozyx_bridge/mavlink_odometry.py
def unknown_attitude_quaternion() -> list[float]:
    # Build the quaternion for "no attitude estimate".
    return [float("nan")] * 4
```

`Quaternion::is_nan()` = true ⇒ **nhánh yaw bị bỏ qua**, còn **vị trí vẫn
được hợp nhất bình thường** (`extNavDataNew.pos` được push độc lập).

### Lưu ý vận hành

Với `EK3_SRCx_YAW = EXTNAV` **và** quaternion NaN, EKF **không khởi tạo
được** (`EKF_STATUS_REPORT.flags = 0x0`) vì không có nguồn yaw nào — ArduPilot
báo `Arm: Need Position Estimate` và từ chối arm.

⇒ **Bắt buộc `EK3_SRCx_YAW = 1` (Compass)** cho hệ thống UWB này. Quaternion
NaN không làm hỏng compass — nó chỉ ngăn EKF hấp thụ một yaw mà hệ thống
không biết.

### Kiểm chứng

| Bài kiểm tra | Kết quả |
| :-- | :-- |
| `test_ardupilot_odometry_model.py` (16 test) | ✅ pass — mô phỏng nguyên văn chuỗi ArduPilot |
| `test_mavlink_odometry.py` (80 test cả package) | ✅ pass |
| `takeoff_land_mission` (q = NaN, YAW = Compass) | ✅ leo 1.41 m, hạ cánh |
| `circle_mission` (q = NaN, YAW = Compass) | ✅ vòng tròn đầy đủ; heading 5°–359°, **0/474 mẫu ở gần Bắc** |

Công cụ dùng để verify:

```bash
# heading của EKF trong khi bay
python3 src/uwb_pozyx_bridge/tools/check_ekf_heading.py --duration 120

# A/B 4 case (đổi param runtime, không cần restart)
python3 src/uwb_pozyx_bridge/tools/ab_run_all_cases.py --cases A,B,C,D
```

---

## 12. Tham chiếu


### ArduPilot (repo này, submodule `src/ardupilot`)

| Nội dung | Đường dẫn |
| :-- | :-- |
| Handler ODOMETRY | `libraries/GCS_MAVLink/GCS_Common.cpp` (`handle_odometry`) |
| Backend EXTNAV | `libraries/AP_VisualOdom/AP_VisualOdom_MAV.cpp` |
| Ghi vào EKF | `libraries/AP_NavEKF3/AP_NavEKF3_Measurements.cpp` (`writeExtNavVelData`) |
| Fusion vị trí | `libraries/AP_NavEKF3/AP_NavEKF3_PosVelFusion.cpp` |
| Jitter timestamp | `libraries/AP_RTC/JitterCorrection.cpp` |
| Tham số VISO | `libraries/AP_VisualOdom/AP_VisualOdom.cpp` |

### ESP32 (`esp32-uwb-localization`)

| Nội dung | Đường dẫn |
| :-- | :-- |
| Encoder ODOMETRY | `firmware/ESP32-DWM3000-UWB/UWB_RTLS_Tag/src/mavlink_output.cpp` |
| Gọi encoder | `firmware/ESP32-DWM3000-UWB/UWB_RTLS_Tag/src/main.cpp` |
| Chuyển frame | `firmware/ESP32-DWM3000-UWB/UWB_RTLS_Tag/src/main.cpp` (`convertLocalToNed`) |
| Trilateration 3D + nhánh | `firmware/ESP32-DWM3000-UWB/UWB_RTLS_Tag/src/trilateration.cpp` |
| Đồng bộ thời gian | `firmware/ESP32-DWM3000-UWB/UWB_RTLS_Tag/src/time_sync.cpp` |
| Quality → covariance | `firmware/ESP32-DWM3000-UWB/UWB_RTLS_Tag/src/quality_est.cpp` |
| Layout anchor | `firmware/ESP32-DWM3000-UWB/UWB_RTLS_Tag/src/tag_config.cpp` |
| Đặc tả | `SPEC.md` (§6, §7, §10.4) |

### ROS (`AerialVehicleControl`)

| Nội dung | Đường dẫn |
| :-- | :-- |
| Encoder ODOMETRY | `src/uwb_pozyx_bridge/uwb_pozyx_bridge/mavlink_odometry.py` |
| Node chính | `src/uwb_pozyx_bridge/uwb_pozyx_bridge/odometry_node.py` |
| Transport TCP | `src/uwb_pozyx_bridge/uwb_pozyx_bridge/mavlink_sender.py` |
| Layout anchor | `src/uwb_pozyx_bridge/uwb_pozyx_bridge/uwb_layout.py` |
| Solver 3D + nhánh | `src/uwb_pozyx_bridge/uwb_pozyx_bridge/trilateration.py` |
| Thế giới Gazebo | `src/uwb_gz_plugin/worlds/uwb_runway.sdf` |
| Kiểm thử encoder | `src/uwb_pozyx_bridge/test/test_mavlink_odometry.py` |
| So khớp 2 bên | `src/uwb_pozyx_bridge/tools/compare_odometry_frames.py` |
