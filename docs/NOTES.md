# Sổ ghi chú — UWB / ArduPilot

Ghi lại các kết luận quan trọng đã tra cứu trong source ArduPilot, kèm đường dẫn file
và số dòng để tra lại nhanh. Mỗi mục là một "entry" độc lập, thêm mục mới thì append
xuống cuối (mục cũ giữ nguyên để so sánh).

**Quy ước:** mọi khẳng định về hành vi của ArduPilot đều phải kèm đường dẫn source.

---

## 2026-11-10 — So sánh chuỗi Pozyx (Beacon) vs chuỗi MAVLink ODOMETRY

**Bối cảnh:** câu hỏi "dùng package của Pozyx thì có phải tuân theo layout của Pozyx
không, và ArduPilot xử lý 2 message id khác nhau có đặt ra quy tắc layout khác nhau
không?"

**Kết luận ngắn:** **Đúng — hai chuỗi có yêu cầu layout hoàn toàn khác nhau.**
Chúng ta đang dùng chuỗi ODOMETRY ⇒ **không phải tuân theo layout convention của
Pozyx**. Layout hình chữ nhật `X × Y` là hợp lệ.

### 1.1 Hai entry point khác nhau hoàn toàn

**Chuỗi ODOMETRY (của chúng ta)** — `src/ardupilot/libraries/GCS_MAVLink/GCS_Common.cpp:3876-3908`

ArduPilot chỉ đòi hỏi 3 điều với `ODOMETRY` (msg id 331):

1. `frame_id == MAV_FRAME_LOCAL_FRD` và `child_frame_id == MAV_FRAME_BODY_FRD`
   — sai là **drop luôn**, không có ngoại lệ;
2. `x/y/z` = vị trí **NED tương đối EKF origin** (mét);
3. `pose_covariance` → sai số vị trí, `quality` phải ≥ `VISO_QUAL_MIN`.

Sau đó đi thẳng vào EXTNAV:

- `src/ardupilot/libraries/AP_VisualOdom/AP_VisualOdom_MAV.cpp:40-43`
  → `AP::ahrs().writeExtNavData(...)`
- `src/ardupilot/libraries/AP_NavEKF3/AP_NavEKF3_PosVelFusion.cpp:157-163, 578-585`
  → EKF dùng `extNavDataDelayed.pos` làm vị trí, **không có ràng buộc hình học nào**.

⇒ **ArduPilot không biết có bao nhiêu anchor, anchor ở đâu, ID thứ tự ra sao.**

**Chuỗi Pozyx / Beacon** — kênh khác hẳn, **không phải MAVLink**:
giao thức binary tự định nghĩa, serial protocol 13 (`BCN_TYPE = 1`).

- `src/ardupilot/libraries/AP_Beacon/AP_Beacon_Pozyx.cpp:113-152`
  parse 3 loại frame: `BEACON_CONFIG` (toạ độ anchor), `BEACON_DIST` (khoảng cách),
  `POSITION` (fix 3D).
- **ArduPilot KHÔNG tự làm trilateration.** Việc định vị nằm trong Arduino sketch
  (`Pozyx.doPositioning(POZYX_3D, ...)`).
- EKF tự fusion *khoảng cách + toạ độ anchor*:
  `src/ardupilot/libraries/AP_NavEKF3/AP_NavEKF3_RngBcnFusion.cpp:60-93` (`SelectRngBcnFusion`)
  và `:95+` (`FuseRngBcn`).

⇒ **Ràng buộc layout của Pozyx buộc phải báo cho ArduPilot: anchor ở đâu, cái nào là
gốc, toàn cục xoay bao nhiêu độ.** Đó chính là lý do có tham số `BCN_ORIENT_YAW`.

### 1.2 Nguồn gốc thật sự của quy tắc layout Pozyx

Reference implementation trong ArduPilot:
`src/ardupilot/Tools/Pozyx/IndoorLoiter/IndoorLoiter.ino:11-21`

```
uint16_t anchor_id[4] = { 0x601C, // (0,0)
                          0x6020, // x-axis
                          0x6057, // y-axis
                          0x605E};
int32_t anchors_x[4] = {0, 10000, 0, 10000};
int32_t anchors_y[4] = {0, 0, 10000, 10000};
```

Hình chữ nhật trục căn + mapping index→toạ độ tường minh (index 0 = gốc, 1 = trục x,
2 = trục y, 3 = đối diện). Khi cấu hình, sketch thậm chí **đo khoảng cách** để suy ra
toạ độ: `IndoorLoiter.ino:262-280`.

Ba nguyên nhân gốc của các quy tắc "N pattern / ID thấp nhất ở góc dưới-trái":

1. Thư viện Pozyx cần ≥ 3 anchor không thẳng hàng + bảng toạ độ đúng;
2. `BCN_LATITUDE/LONGITUDE/ALT` được định nghĩa là *"Beacon **origin**'s latitude"*
   ⇒ index 0 bắt buộc là gốc hình chữ nhật
   (`src/ardupilot/libraries/AP_Beacon/AP_Beacon.cpp:50-70`);
3. `BCN_ORIENT_YAW` = heading từ anchor gốc tới anchor thứ 2.

Lưu ý: backend beacon SITL của chính ArduPilot cũng dùng **hình chữ nhật**, không
phải vuông — `src/ardupilot/libraries/AP_Beacon/AP_Beacon_SITL.cpp:37-38`
(`BEACON_SPACING_NORTH 10.0`, `BEACON_SPACING_EAST 20.0`).

### 1.3 Phát hiện: quy ước α của chúng ta TRÙNG `BCN_ORIENT_YAW` của ArduPilot ✅

`src/ardupilot/libraries/AP_Beacon/AP_Beacon_Backend.cpp:86-110`

```cpp
vec_rotated.x = vector.x*orient_cos_yaw - vector.y*orient_sin_yaw;
vec_rotated.y = vector.x*orient_sin_yaw + vector.y*orient_cos_yaw;
```

> ⚠️ Comment trong source ghi `// rotate x,y by -orient_yaw` — **comment sai, code đúng**
> (đây là phép xoay chuẩn +α).

`src/uwb_pozyx_bridge/uwb_pozyx_bridge/uwb_layout.py:93-109` (`rotate_ned`)

```python
return (x * cos_a - y * sin_a, x * sin_a + y * cos_a)
```

⇒ **Cùng công thức, cùng dấu, cùng nghĩa.** Định nghĩa α của chúng ta
(`config_yaw_offset_deg` phía ESP32 / `layout_alpha_deg` phía ROS) = bearing của
hướng `a0 → a1` đo từ Bắc = đúng nghĩa `BCN_ORIENT_YAW`.

Chiều z cũng khớp: Pozyx dùng z hướng lên, ArduPilot parse thì `-beacon_z * 0.001f`
đảo sang NED (dương là xuống) — `AP_Beacon_Pozyx.cpp:119, 147` — khớp anchor frame
`z = Down` của chúng ta.

### 1.4 Đã đồng bộ: thứ tự index 2/3 theo N pattern (2026-11-17)

| index | Pozyx sketch | Layout của chúng ta (**nay đã giống**) |
|:--|:--|:--|
| 0 | (0,0) gốc | (0,0) a0 |
| 1 | (X,0) trục x | (X,0) a1 |
| 2 | (0,Y) trục y | (0,Y) a2 |
| 3 | (X,Y) đối diện | (X,Y) a3 |

Trước đây layout của chúng ta **đi vòng kim** (a2 = (X,Y), a3 = (0,Y)) —
lệch đúng ở index 2/3 so với Pozyx. Nay đã chuyển sang **N pattern**: ID
tăng dần theo đường `a0 → a1 → a2 → a3` (hình chữ N trong hình chữ nhật),
giống hệt Pozyx reference implementation.

**Đây là quy ước NỘI BỘ giữa hai bên, không phải yêu cầu của ArduPilot.**
Với chuỗi `ODOMETRY`, ArduPilot không nhìn anchor ID, nên việc đổi này
**không đổi một byte nào** của pipeline MAVLink / EKF. Mục đích là để hai bên
(thiết bị thật + mô phỏng) nói chung một ngôn ngữ layout với Pozyx, thuận tiện
khi đối chiếu với tài liệu Pozyx và với code tham chiếu của ArduPilot.

Nếu sau này chuyển sang chuỗi **beacon** (`BCN_TYPE = 1`), layout này **đã
đúng sẵn** — không cần đảo 2↔3 nữa.

Thêm: thứ tự 2/3 của Pozyx **không khớp** với mô tả "N pattern" trong wiki
(wiki nói ID tăng dần theo N là BL→BR→TR→TL, còn sketch là BL→BR→TL→TR). Đây là
điểm lệch đã biết giữa tài liệu và reference implementation của ArduPilot;
chúng ta theo **reference implementation** (sketch).

> **Cần đọc cẩn thận phần "N pattern" của wiki.** Wiki ArduPilot vẽ hình bốn
> anchor với ID tăng dần **theo vòng kim ngược** (`BL→BR→TR→TL`), trong khi
> reference implementation (`IndoorLoiter.ino`) đặt ID theo **N** thật
> (`BL→BR→TL→TR` trong hệ toạ độ của sketch). Vì vậy khi nói "N pattern" ở
> đây, ta luôn lấy **sketch làm chuẩn**, không lấy hình vẽ trong wiki.

### 1.5 Đối chiếu: nhập nhằng Z với anchor đồng phẳng (mục 5 trong danh sách đồng bộ)

Nếu đi chuỗi beacon, ArduPilot tự giải bằng **phương pháp hai giả thuyết**:

`src/ardupilot/libraries/AP_NavEKF3/AP_NavEKF3_RngBcnFusion.cpp:571-582`

```cpp
/*
Run a single state Kalman filter to estimate the vertical position offset of the range beacon constellation
Calculate using a high and low hypothesis and select the hypothesis with the lowest innovation sequence
*/
// A high and low estimate is calculated to handle the ambiguity in height
// associated with beacon positions that are co-planar
```

⇒ **Chuỗi beacon: ArduPilot đỡ nhập nhằng Z. Chuỗi ODOMETRY: ArduPilot không can
thiệp, phải tự giải ở ESP32** (multi-start + branch tracking trong `trilateration.cpp`).
Đây là khác biệt bản chất quan trọng nhất giữa hai chuỗi.

### 1.6 Bảng tổng hợp

| Tiêu chí | Chuỗi Beacon (Pozyx) | Chuỗi ODOMETRY (**của chúng ta**) |
|:--|:--|:--|
| Ai quyết định layout | ArduPilot / Pozyx sketch: hcn + index→toạ độ + gốc + N pattern | **Chính chúng ta**; ArduPilot không tham gia |
| Truyền α thế nào | `BCN_ORIENT_YAW` (ArduPilot tự xoay) | Chúng ta **tự xoay** ở ESP32 rồi gửi NED |
| Phải báo toạ độ anchor? | **Có** (frame `BEACON_CONFIG`) | **Không** |
| Ai giải nhập nhằng Z? | ArduPilot (dual-hypothesis vertical offset) | **ESP32** (Gauss-Newton multi-start + branch tracking) |
| Nguồn EKF | `EK3_SRC1_POSXY = 4 (BEACON)` | `EK3_SRC1_POSXY = 6 (EXTNAV)` ← hiện tại |
| Giao thức serial | Beacon tự định nghĩa (SerialProtocol 13) | MAVLink chuẩn |
| Cần hình chữ nhật? | Cần (không thẳng hàng + bảng toạ độ) | **Không** — hình học tuỳ ý |
| Cần hình vuông? | **Không** (SITL là 20×10) | Không |

### 1.7 Hệ quả với project

- Tiếp tục dùng `ODOMETRY + EXTNAV`. Layout hình chữ nhật `X × Y` (đã sửa trong
  `tag_config.cpp` phía ESP32) là hợp lệ; phía ROS `uwb_layout.py` đã là mô hình
  `size_x_m` / `size_y_m`.
- **Cảnh báo naming:** package ROS tên `uwb_pozyx_bridge` nhưng **không hề có protocol
  hay phần cứng Pozyx nào** — tên cũ từ lúc thiết kế theo phương án Pozyx. Người mới
  nhìn tên package rồi áp luật layout Pozyx sẽ bị sai lệch.

### 1.8 Tham chiếu

| File | Nội dung |
|:--|:--|
| `src/ardupilot/libraries/GCS_MAVLink/GCS_Common.cpp:3876-3908` | `handle_odometry` — entry của chuỗi ODOMETRY |
| `src/ardupilot/libraries/AP_VisualOdom/AP_VisualOdom_MAV.cpp` | `AP_VisualOdom_MAV` — ODOMETRY → EXTNAV |
| `src/ardupilot/libraries/AP_VisualOdom/AP_VisualOdom.cpp:29-115` | Tham số `VISO_TYPE`, `VISO_QUAL_MIN`, `VISO_SCALE`, `VISO_DELAY_MS` |
| `src/ardupilot/libraries/AP_NavEKF/AP_NavEKF_Source.h:17-37` | Enum `SourceXY` (BEACON=4, EXTNAV=6), `SourceZ` |
| `src/ardupilot/libraries/AP_Beacon/AP_Beacon.cpp` | Tham số `BCN_TYPE/LATITUDE/LONGITUDE/ALT/ORIENT_YAW` |
| `src/ardupilot/libraries/AP_Beacon/AP_Beacon_Pozyx.cpp` | Parser giao thức Pozyx |
| `src/ardupilot/libraries/AP_Beacon/AP_Beacon_Backend.cpp:86-110` | `correct_for_orient_yaw` — xoay theo `BCN_ORIENT_YAW` |
| `src/ardupilot/libraries/AP_Beacon/AP_Beacon_SITL.cpp` | Backend beacon SITL — hình chữ nhật 10×20 |
| `src/ardupilot/libraries/AP_NavEKF3/AP_NavEKF3_RngBcnFusion.cpp` | Range beacon fusion + dual-hypothesis Z |
| `src/ardupilot/Tools/Pozyx/IndoorLoiter/IndoorLoiter.ino` | Sketch Pozyx tham chiếu — nguồn gốc quy tắc layout |
| https://ardupilot.org/copter/docs/common-pozyx.html | Tài liệu Pozyx (nói về "N pattern", `BCN_ORIENT_YAW`) |

---
## 2026-11-17 — Chuyển layout anchor sang N pattern (Pozyx)

**Bối cảnh:** user muốn dùng layout **N pattern** theo tài liệu Pozyx của
ArduPilot, nhưng **giữ nguyên** pipeline MAVLink `ODOMETRY` (331) → EXTNAV
(đã loại bỏ `AP_Beacon` / `BCN_TYPE=1` từ vòng code review trước).

**Kết luận:** N pattern là **quy ước nội bộ giữa hai bên** (ROS ↔ ESP32), được
chọn để **thống nhất với Pozyx reference implementation**. Nó **không** bắt
buộc phải dùng chuỗi beacon.

### 2.1 Hình học

Pozyx reference (`IndoorLoiter.ino:11-21`):

```
uint16_t anchor_id[4] = { 0x601C, // (0,0)
                          0x6020, // x-axis
                          0x6057, // y-axis
                          0x605E};
int32_t anchors_x[4] = {0, 10000, 0,     10000};
int32_t anchors_y[4] = {0, 0,     10000, 10000};
```

| index | Pozyx | Trước đây (vòng kim) | Nay (N pattern) |
|:--|:--|:--|:--|
| 0 | (0,0) | (0,0) | (0,0) |
| 1 | (X,0) | (X,0) | (X,0) |
| 2 | (0,Y) | (X,Y) | **(0,Y)** |
| 3 | (X,Y) | (0,Y) | **(X,Y)** |

⇒ Chỉ **hoán đổi index 2 ↔ 3**.

### 2.2 Vì sao an toàn

- ArduPilot **không** biết anchor tồn tại: `GCS_Common.cpp:3876-3908` chỉ đòi
  `frame_id == MAV_FRAME_LOCAL_FRD`, `child_frame_id == MAV_FRAME_BODY_FRD` và
  vị trí NED tương đối EKF origin. `AP_VisualOdom_MAV.cpp` đưa thẳng sang
  EXTNAV. **Không có ràng buộc hình học nào.**
- Solver 2 bên đều **layout-agnostic**: chỉ đọc bảng toạ độ theo index.
  → Không sửa thuật toán, chỉ đổi bảng.
- MAVLink `ODOMETRY` (`mavlink_odometry.py`, `verify_odometry_packet.cpp`):
  quaternion `[NaN,NaN,NaN,NaN]`, velocity `NaN`, `MAV_FRAME_LOCAL_FRD` /
  `MAV_FRAME_BODY_FRD`, sysid 255 / compid 191 — **không đổi**.

### 2.3 File đã sửa

**ROS** (`AerialVehicleControl`)

| File | Thay đổi |
|:--|:--|
| `src/uwb_pozyx_bridge/uwb_pozyx_bridge/uwb_layout.py` | `anchor_frame_positions()` → N pattern; thêm `N_PATTERN_INDEX_ORDER`; `layout_from_anchor_offsets_mm()` đọc `Y` từ `a2` |
| `src/uwb_gz_plugin/worlds/uwb_runway.sdf` | `<anchor>` pos + `<include>` pose của `uwb_anchor2/3` hoán đổi |
| `src/uwb_pozyx_bridge/test/test_uwb_layout.py` | Assert layout N pattern + test "N walk" |
| `src/uwb_pozyx_bridge/test/test_uwb_world.py` | Template + expected world positions |

**ESP32** (`esp32-uwb-localization`)

| File | Thay đổi |
|:--|:--|
| `UWB_RTLS_Tag/src/uwb_config.h` | Comment layout N pattern |
| `UWB_RTLS_Tag/src/tag_config.cpp` | `CFG_DEFAULT_ANCHOR_POS[]` → N pattern; ví dụ lệnh `cfg` |
| `UWB_RTLS_Tag/src/tag_config.h` | Comment header |
| `UWB_RTLS_Tag/src/main.cpp` | Comment anchor frame |
| `tools/check_rect_layout.cpp` | `build_layout()` → N pattern |

### 2.4 Kiểm chứng

- ROS: `pytest src/uwb_pozyx_bridge/test/` → 83 passed.
- ESP32: `tools/run_check_rect_layout.sh` → `h_err = 0.0000 m` trên layout
  chữ nhật 14 m × 7 m (N pattern).
- ESP32: `tools/run_verify_odometry.sh` → ALL CHECKS PASSED (MAVLink frame /
  msgid / covariance / NaN velocity).
- ESP32: `pio run -e release` → SUCCESS.

### 2.5 Hệ quả về mặt thực địa

Bảng dán nhãn board (`anchor1..anchor4`) giờ map như sau (id ESP32 = id ROS + 1):

| Board | Anchor | Anchor Frame | ROS id |
|:--|:--|:--|:--|
| `anchor1` | a0 | (0,0) | 0 |
| `anchor2` | a1 | (X,0) | 1 |
| `anchor3` | a2 | (0,Y) | 2 |
| `anchor4` | a3 | (X,Y) | 3 |

Lệnh serial cho layout 10×10, α = 20°, `h_plane` = 2 m:

```
cfg,0,0,0,10,0,0,0,10,0,10,10,0,20,2
```
