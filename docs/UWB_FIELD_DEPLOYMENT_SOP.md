# SOP — Triển khai UWB RTLS thực tế (từ clone code đến bay thử)

**Áp dụng cho:** hệ UWB DS-TWR (4 Anchor + 1 Tag) định vị trong nhà, kết hợp ArduPilot qua MAVLink `ODOMETRY`.

**Hai repo liên quan:**

| Repo | Vai trò | Remote |
|---|---|---|
| `esp32-uwb-localization` | Firmware ESP32 (Tag + Anchor), MAVLink encoder | `git@github.com:NguyenAn20279/esp32-uwb-localization.git` |
| `AerialVehicleControl` | Simulation ROS 2 Humble + Gazebo Harmonic + ArduPilot SITL (để đối chiếu layout & thuật toán) | `git@github.com:trananh26/AerialVehicleControl.git` |

> **Nguyên tắc bất di bất dịch:** cùng một bộ số đo được phải dùng cho **cả hai** repo.
> Layout sai ở simulation thì thật cũng sai — vì vậy luôn sửa song song.

---

## 0. Tóm tắt các bước

```
0. Chuẩn bị phần cứng + dụng cụ
1. Clone code (2 repo) + cài đặt môi trường
2. Khảo sát vị trí anchor thực tế trong không gian  ← BƯỚC QUAN TRỌNG NHẤT
3. Sửa config theo vị trí đã khảo sát (ESP + ROS)
4. Build & flash 4 Anchor + 1 Tag
5. Test tầng 1: Serial Tag (ranging + trilateration)
6. Test tầng 2: MAVLink ODOMETRY → Pixhawk (EKF origin/home)
7. Test tầng 3: Static → Walk → Flight
```

---

## 1. Chuẩn bị phần cứng

| Thiết bị | Số lượng | Ghi chú |
|---|---|---|
| ESP32 + DWM3000 (board) | 5 | 4 Anchor + 1 Tag |
| Pixhawk 6C + ArduPilot 4.6.3/4.7 | 1 | |
| Đầm cầu serial USB-C | 5 | 1 cổng / board, dán nhãn A1..A4, TAG |
| Máy đo khoảng cách laser (Disto/laser rangefinder) | 1 | Đo cạnh & độ cao |
| La bàn / điện thoại có GPS + la bàn | 1 | Đo hướng Bắc (bearing) |
| Thước dây / sợi thước laser | 1 | |
| Dánh nhãn dán, bút đánh dấu sàn | 1 | Đánh dấu vị trí Anchor trên sàn |

### 1.1 Lắp đặt Antenna Delay

`CFG_ANTENNA_DELAY` (đơn vị DWM3000, 1 đơn vị ≈ 8 ps) là **giá trị hiệu chuẩn riêng cho từng board**, phải flash riêng kèm `ANCHOR_ID`.

- Giá trị mặc định (board mẫu): `16350` — dùng tạm cho bước 5 (Static test).
- Hiệu chuẩn đúng: dùng `firmware/ESP32-DWM3000-UWB/calib/` đo khoảng cách ở nhiều điểm, fit sai số theo khoảng cách.
- **Mỗi board có một giá trị riêng** → ghi vào bảng bên dưới và nhét vào `platformio.ini` khi flash.

| Board | `ANCHOR_ID` | `CFG_ANTENNA_DELAY` | Vị trí thực tế |
|---|---|---|---|
| A1 | 1 | 16350 | |
| A2 | 2 | 16350 | |
| A3 | 3 | 16350 | |
| A4 | 4 | 16350 | |
| Tag | 10 | 16350 | Trên drone |

### 1.2 Nối Tag ↔ Pixhawk (UART TELEM2)

| ESP32 Pin | Pixhawk 6C TELEM2 | Ghi chú |
|---|---|---|
| TX (GPIO 17) | RX2 | MAVLink `ODOMETRY` → Pixhawk |
| RX (GPIO 16) | TX2 | Phản hồi / heartbeat |
| GND | GND | Chung mass — **bắt buộc** |
| 5V | 5V | |

---

## 2. Clone code & cài môi trường

### 2.1 Clone firmware ESP32

```bash
cd ~/
git clone --recurse-submodules git@github.com:NguyenAn20279/esp32-uwb-localization.git
cd esp32-uwb-localization
git submodule update --init --recursive   # nếu clone không kèm submodule
```

### 2.2 Clone repo simulation ROS 2

```bash
cd ~/
git clone --recurse-submodules -b x500_model \
  https://github.com/trananh26/AerialVehicleControl.git
cd AerialVehicleControl
git submodule update --init --recursive
```

> Nhánh `x500_model` là nhánh chứa pipeline UWB + ArduPilot SITL.

### 2.3 Cài PlatformIO (Ubuntu)

```bash
# Cách A: extension PlatformIO IDE trong Cursor
# Cách B: PlatformIO Core CLI
curl -fsSL -o get-platformio.py https://raw.githubusercontent.com/platformio/platformio-core/develop/get-platformio.py
python3 get-platformio.py

export PATH="$HOME/.platformio/penv/bin:$PATH"
echo 'export PATH="$HOME/.platformio/penv/bin:$PATH"' >> ~/.bashrc
pio --version
```

Quyền cổng serial:

```bash
sudo usermod -aG dialout $USER   # logout/login lại
pio device list                   # liệt kê /dev/ttyUSB* /dev/ttyACM*
```

### 2.4 Cài ROS 2 Humble + Gazebo Harmonic + MAVROS (chỉ khi cần simulation)

Theo hướng dẫn đầy đủ trong `AerialVehicleControl/ROS2_Gazebo_ArduPilot_Setup.md`:

```bash
sudo apt install -y ros-humble-desktop ros-dev-tools python3-vcstool
sudo apt install -y ros-humble-mavros ros-humble-mavros-extras
sudo apt install -y libgz-sim8-dev rapidjson-dev

cd ~/AerialVehicleControl
source /opt/ros/humble/setup.bash
rosdep update
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-up-to ardupilot_gz_bringup drone_control uwb_pozyx_bridge
source install/setup.bash
```

Cài thêm ArduPilot SITL:

```bash
cd ~/AerialVehicleControl/src/ardupilot
./Tools/environment_install/install-prereqs-ubuntu.sh -y
rm -f .lock-waf_linux_build
```

### 2.5 Kiểm tra firmware build được (chưa cần phần cứng)

```bash
cd ~/esp32-uwb-localization/firmware/ESP32-DWM3000-UWB/UWB_RTLS_Tag
pio run -e release
```

Kết quả mong đợi: `SUCCESS`, không warning mới.

---

## 3. Khảo sát vị trí Anchor trong không gian (BƯỚC QUAN TRỌNG NHẤT)

### 3.1 Định nghĩa Anchor Frame

Solver làm việc trên **hình chữ nhật lý tưởng** trong Anchor Frame (trục tay phải, **Z hướng xuống**):

```
a0 = (0, 0, 0)      ← GỐC
a1 = (X, 0, 0)      ← X = chiều dài cạnh a0 → a1
a3 = (0, Y, 0)      ← Y = chiều dài cạnh a0 → a3
a2 = (X, Y, 0)      ← góc đối diện
```

- Mặt phẳng anchor: `z = 0`, cao `h_plane` so với sàn.
- Tag bay **dưới** mặt phẳng anchor ⇒ `z > 0` (chiều sâu).

### 3.2 Quy tắc chọn GỐC (quan trọng — quyết định trục X/Y)

Đứng tại **a0**, nhìn về **a3**:
- **Anchor bên TRÁI** = a1 → cạnh `a0 → a1` là hướng **+X**
- **Anchor bên PHẢI** = ... (a1 nằm bên trái ⇒ a3 nằm bên phải ⇒ cạnh `a0 → a3` là +Y)

Hệ quả: hệ X–Y của Anchor Frame là **cùng chiều với** hệ Bắc–Đông khi nhìn từ trên xuống và `α = 0`.

> ⚠️ **Quy tắc này phải giữ nhất quán giữa ESP32 và ROS.** Trong code ESP32, `ANCHOR_POS_LOCAL[0]` là **a0**, `[1]`=a1, `[2]`=a2, `[3]`=a3.

### 3.3 Quy trình khảo sát (checklist hiện trường)

Bước 1 — **Chọn gốc a0**: đặt tại góc nào? Ghi lại tọa độ GPS (nếu có) để dùng làm `CFG_WORLD_LATITUDE_DEG` / `CFG_WORLD_LONGITUDE_DEG`.

Bước 2 — **Đo cạnh X** (a0 → a1): khoảng cách **thẳng** giữa hai vị trí neo, tính cả chiều cao neo nếu lệch nhau. Ghi `X`.

Bước 3 — **Đo cạnh Y** (a0 → a3): tương tự. Ghi `Y`.

Bước 4 — **Đo độ cao mặt phẳng anchor** `h_plane`: khoảng cách từ mặt sàn (hoặc từ mặt phẳng ngang tham chiếu — EKF origin) lên tâm antenna của anchor. **Các 4 anchor phải cùng cao**, sai số ≤ 5 cm, nếu không Z của Tag sẽ lệch.

Bước 5 — **Đo góc lệch α**: góc bearing của hướng `a0 → a1`, tính **từ hướng Bắc, chiều kim đồng hồ nhìn từ trên xuống**.
- Dùng la bàn điện thoại: mục tiêu đặt tại a0, hướng nhìn về a1, đọc bearing.
- Nếu `a0 → a1` chạy theo hướng Đông thì α ≈ 90°; theo Bắc thì α ≈ 0°.

Bước 6 — **Đánh dấu sàn** tại vị trí 4 anchor, dán nhãn A1..A4 tương ứng với `ANCHOR_ID` đã flash.

Bước 7 — **Điền bảng** (mẫu):

| Anchor | ID | `x` | `y` | `z` (rel. mặt phẳng) | Ghi chú |
|---|---|---|---|---|---|
| a0 | 1 | 0.00 | 0.00 | 0.00 | Gốc |
| a1 | 2 | `X` | 0.00 | 0.00 | |
| a2 | 3 | `X` | `Y` | 0.00 | |
| a3 | 4 | 0.00 | `Y` | 0.00 | |

Trong đó `X`, `Y` là kích thước **lý tưởng** của hình chữ nhật solver dùng, KHÔNG phải tọa độ thô đo được nếu 4 neo không hoàn hảo vuông góc. Tọa độ thô dùng để **sửa sai lệch** (xem §3.5).

### 3.4 Vì sao solver dùng hình chữ nhật lý tưởng

Khoảng cách đo **bất biến** dưới phép xoay quanh trục đứng, nên solver chỉ cần hình chữ nhật lý tưởng `X × Y` rồi **xoay kết quả** theo `α` trước khi gửi đi:

$$
\begin{aligned}
N &= x\cos\alpha - y\sin\alpha \\
E &= x\sin\alpha + y\cos\alpha \\
D &= z - h_{\text{plane}}
\end{aligned}
$$

### 3.5 Xử lý neo lệch vị trí thực tế

Nếu 4 neo không tạo đúng hình chữ nhật (đây là trường hợp phổ biến):

- **Cách chuẩn (khuyến nghị):** đặt lại neo cho vuông góc bằng dụng cụ, hoặc chấp nhận sai số và ghi vào bảng tọa độ thô.
- **Cách mềm:** dùng bộ 4 tọa độ thô (không vuông) qua lệnh serial `cfg` (xem §4.2) — Tag sẽ dùng đúng 4 vị trí đó. Khi đó `cfg` nhận **13 token bắt buộc + 1 tuỳ chọn** (x1,y1,z1,...,x4,y4,z4,yaw[,plane_height]).

### 3.6 Kiểm tra chéo bằng simulation (khuyến nghị)

Trước khi đi hiện trường, chạy simulation với **đúng** số đo đã khảo sát để xác nhận thuật toán và layout là đúng:

```bash
cd ~/AerialVehicleControl
source /opt/ros/humble/setup.bash && source install/setup.bash
ros2 launch uwb_pozyx_bridge x500_uwb_runway.launch.py \
    layout_size_x_m:=12.0 \
    layout_size_y_m:=8.0 \
    layout_alpha_deg:=30.0
```

Trong terminal khác:

```bash
ros2 run drone_control circle_mission
```

**Tiêu chí đạt** (tham khảo từ các lần chạy đã kiểm chứng):

| Layout | Mean error | P95 error |
|---|---|---|
| 10 × 10 m, α=0° | 0.125 m | 0.244 m |
| 14 × 7 m, α=0° | 0.111 m | 0.256 m |
| 12 × 8 m, α=30° | 0.103 m | 0.233 m |

Nếu chạy được với sai số < 0.3 m ở layout đã khảo sát ⇒ thuật toán OK, chỉ còn việc nhập số vào firmware.

---

## 4. Sửa config theo vị trí đã khảo sát

### 4.1 ESP32 — file config trung tâm

File: `firmware/ESP32-DWM3000-UWB/UWB_RTLS_Tag/src/uwb_config.h`

```c
// §2. Layout
#ifndef CFG_DEFAULT_LAYOUT_SIZE_X_M
#define CFG_DEFAULT_LAYOUT_SIZE_X_M 10.0f     // ← X (a0→a1), thay bằng giá trị đo
#endif

#ifndef CFG_DEFAULT_LAYOUT_SIZE_Y_M
#define CFG_DEFAULT_LAYOUT_SIZE_Y_M 10.0f     // ← Y (a0→a3), thay bằng giá trị đo
#endif

#ifndef CFG_DEFAULT_YAW_OFFSET_DEG
#define CFG_DEFAULT_YAW_OFFSET_DEG 0.0f      // ← α, thay bằng bearing đo
#endif

#ifndef CFG_DEFAULT_ANCHOR_PLANE_HEIGHT_M
#define CFG_DEFAULT_ANCHOR_PLANE_HEIGHT_M 2.0f   // ← h_plane, thay bằng chiều cao đo
#endif

// §3. World ref (origin EKF toàn cầu)
#ifndef CFG_WORLD_LATITUDE_DEG
#define CFG_WORLD_LATITUDE_DEG   (-35.363262)   // ← GPS của anchor a0
#endif
#ifndef CFG_WORLD_LONGITUDE_DEG
#define CFG_WORLD_LONGITUDE_DEG  (149.165237)
#endif
#ifndef CFG_WORLD_ALTITUDE_M
#define CFG_WORLD_ALTITUDE_M     (584.0)        // ← độ cao a0 (m) so với mực nước biển
#endif
```

`tag_config.cpp` **tự sinh** 4 tọa độ neo từ 3 hằng số trên (`a0, a1, a2, a3` theo thứ tự rectangle) — không cần sửa file này nếu neo vuông góc.

### 4.2 ESP32 — ghi đè lúc chạy qua Serial (không cần build lại)

Tag nhận lệnh `cfg` trên **UART0 (USB)**:

```
cfg,x1,y1,z1,x2,y2,z2,x3,y3,z3,x4,y4,z4,yaw[,plane_height]
```

Ví dụ — rectangle 12 × 8 m, α = 30°, mặt phẳng cao 2.2 m:

```
cfg,0,0,0,12,0,0,12,8,0,0,8,0,30,2.2
```

Gửi được:
- **Trong 5 s đầu sau khi bật nguồn** (cửa sổ config, `CFG_CONFIG_WINDOW_MS`) → ESP32 in lại config rồi bắt đầu ranging.
- **Bất kỳ lúc nào** khi đang chạy → cập nhật tức thì.

Giá trị mặc định lấy từ `uwb_config.h` sẽ bị ghi đè khi có lệnh `cfg`. **Cả hai phải khớp nhau**, nếu không sẽ gây nhầm lẫn khi debug.

### 4.3 ESP32 — Antenna delay từng board

File `firmware/ESP32-DWM3000-UWB/UWB_RTLS_Anchor/src/uwb_config.h`:

```c
#ifndef CFG_ANTENNA_DELAY
#define CFG_ANTENNA_DELAY       16350   // ← giá trị riêng của board này
#endif
```

Để flash giá trị riêng mà không sửa file, dùng build flag:

```ini
; platformio.ini của Anchor
[env:anchor1]
build_flags = -DANCHOR_ID=1 -DUWB_DEBUG=0 -DCFG_ANTENNA_DELAY=16355
```

### 4.4 ROS 2 simulation — cùng layout

Launch file: `src/uwb_pozyx_bridge/launch/x500_uwb_runway.launch.py`
(world SDF được **sinh động** theo đúng số đo — không sửa file `.sdf` thủ công):

```bash
ros2 launch uwb_pozyx_bridge x500_uwb_runway.launch.py \
    layout_size_x_m:=12.0 \
    layout_size_y_m:=8.0 \
    layout_alpha_deg:=30.0
```

Tham số node `odometry_node` (đặt trong launch hoặc override khi chạy):

| Tham số | Ý nghĩa | Đối chiếu ESP32 |
|---|---|---|
| `layout_size_x_m` | cạnh a0→a1 | `CFG_DEFAULT_LAYOUT_SIZE_X_M` |
| `layout_size_y_m` | cạnh a0→a3 | `CFG_DEFAULT_LAYOUT_SIZE_Y_M` |
| `layout_alpha_deg` | bearing a0→a1 từ Bắc | `CFG_DEFAULT_YAW_OFFSET_DEG` |
| `anchor_plane_height_m` | cao mặt phẳng anchor | `CFG_DEFAULT_ANCHOR_PLANE_HEIGHT_M` |

### 4.5 Bảng đối chiếu nhanh (dán vào tường)

| Thông số | ESP32 (`uwb_config.h`) | ROS 2 (launch) |
|---|---|---|
| Cạnh X | `CFG_DEFAULT_LAYOUT_SIZE_X_M` | `layout_size_x_m` |
| Cạnh Y | `CFG_DEFAULT_LAYOUT_SIZE_Y_M` | `layout_size_y_m` |
| Góc α | `CFG_DEFAULT_YAW_OFFSET_DEG` | `layout_alpha_deg` |
| Cao mặt phẳng | `CFG_DEFAULT_ANCHOR_PLANE_HEIGHT_M` | `anchor_plane_height_m` |
| Origin | `CFG_WORLD_LATITUDE/LONGITUDE/ALTITUDE` | (SITL tự sinh) |

---

## 5. Build & Flash

### 5.1 Build trước khi đi hiện trường

```bash
cd ~/esp32-uwb-localization/firmware/ESP32-DWM3000-UWB/UWB_RTLS_Anchor
for e in anchor1 anchor2 anchor3 anchor4; do pio run -e $e; done

cd ../UWB_RTLS_Tag
pio run -e release
```

### 5.2 Flash 4 Anchor

Mỗi board **phải flash một lần** với `ANCHOR_ID` tương ứng. Chỉ cắm **một** board vào USB tại một thời điểm.

```bash
cd ~/esp32-uwb-localization/firmware/ESP32-DWM3000-UWB/UWB_RTLS_Anchor

pio run -e anchor1 -t upload --upload-port /dev/ttyUSB0   # board gắn nhãn A1
pio run -e anchor2 -t upload --upload-port /dev/ttyUSB0   # board gắn nhãn A2
pio run -e anchor3 -t upload --upload-port /dev/ttyUSB0   # board gắn nhãn A3
pio run -e anchor4 -t upload --upload-port /dev/ttyUSB0   # board gắn nhãn A4
```

Banner mong đợi trên Serial Monitor (115200):

```text
> ANCHOR 1 - Ready for DS-TWR ranging [IRQ MODE + WDT] <
```

> ⚠️ **Số trên banner phải khớp với vị trí thực tế.** Anchor `ID=1` ứng với `a0`, `ID=2` → `a1`, `ID=3` → `a2`, `ID=4` → `a3`.

### 5.3 Flash Tag

```bash
cd ~/esp32-uwb-localization/firmware/ESP32-DWM3000-UWB/UWB_RTLS_Tag
pio run -e release -t upload --upload-port /dev/ttyUSB0
```

Banner mong đợi:

```text
> TAG RTLS ready <
```

Ngay sau đó Tag in ra config hiện tại — **đối chiếu với bảng khảo sát**:

```text
=== UWB Config ===
Current Anchor Positions (m, anchor frame):
  A1: (0.000, 0.000, 0.000)
  A2: (12.000, 0.000, 0.000)
  A3: (12.000, 8.000, 0.000)
  A4: (0.000, 8.000, 0.000)
Yaw Offset (a0->a1 bearing from North): 30.0 deg
Anchor Plane Height: 2.20 m
==================
```

Nếu sai → gửi lệnh `cfg` (§4.2) hoặc sửa `uwb_config.h` rồi flash lại.

### 5.4 Cài Tag lên drone

1. Gắn DWM3000 trên drone, ** antenna hướng lên, cách xa vật kim loại ≥ 20 cm**.
2. Gắn ESP32, cắm UART2 → Pixhawk TELEM2.
3. Cấp nguồn Pixhawk trước, rồi bật nguồn Tag (thứ tự ngược lại thì Tag không nhận được `SYSTEM_TIME`).

### 5.5 Cấu hình Pixhawk / ArduPilot

```bash
# Qua Mission Planner: Full Parameter List
VISO_TYPE      = 1     # ExternalNav (UWB)
EK3_SRC1_POSXY = 6     # ExternalNav
EK3_SRC1_VELXY = 0     # None — EKF tự tích phân từ IMU
EK3_SRC1_POSZ  = 1     # Barometer
EK3_SRC1_YAW   = 1     # Compass
AHRS_OPTIONS  = 16     # cho phép EKF dùng ExternalNav
EK3_SRC1_POSXY  = 6
```

> ⚠️ `VISO_TYPE = 1` (ExternalNav) **bắt buộc**. Với `VISO_TYPE = 0` (None), gói `ODOMETRY` bị bỏ qua.

Đặt EKF Origin (nếu chưa có): Mission Planner → chuột phải trên bản đồ → *Set EKF Origin Here*, hoặc:

```bash
mavproxy.py --device /dev/ttyUSB0 --baudrate 115200
> wp setorigin <lat> <lon> <alt>
```

> Tag cũng tự gửi `SET_GPS_GLOBAL_ORIGIN` (#48) + `MAV_CMD_DO_SET_HOME` (#179) mỗi 15 s, nên origin/home sẽ tự được set khi link MAVLink lên. **Origin phải đi trước Home** — Tag đã xử lý đúng thứ tự này.

---

## 6. Test tầng 1 — Tag Serial (không cần drone)

Đặt Tag cạnh anchor a0, mở Serial Monitor 115200:

```bash
cd ~/esp32-uwb-localization/firmware/ESP32-DWM3000-UWB/UWB_RTLS_Tag
pio device monitor --port /dev/ttyUSB0 -b 115200
```

### 6.1 Kiểm tra ranging

Trong `debug` env (`pio run -e debug -t upload`) sẽ thấy log khoảng cách từng anchor. Ở `release` env, output là CSV:

```text
NED,1.230,2.340,0.92,4,0.05
```

Định dạng: `NED, x, y, quality, n_anchors, residual_rms`

| Trường | Giá trị mong đợi |
|---|---|
| `x`, `y` | vị trí trong Anchor Frame (m) |
| `quality` | 0..1, càng cao càng tốt |
| `n_anchors` | 4 (đứng ở giữa, không bị che chắn) |
| `residual_rms` | < 0.1 m khi đứng yên |

### 6.2 Kiểm tra vị trí

Đặt Tag **tại a0** → kết quả phải là `x≈0, y≈0`.
Đặt Tag **tại a1** → `x≈X, y≈0`.
Đặt Tag **tại a3** → `x≈0, y≈Y`.

Sai số cho phép: **< 0.3 m** (đo thẳng, không bị che chắn bởi vật kim loại).

Nếu sai → kiểm tra lại: đúng `ANCHOR_ID` gắn với đúng vị trí? đúng `α`? đúng chiều cao mặt phẳng?

### 6.3 Ghi log để phân tích

```bash
cd ~/esp32-uwb-localization
python3 tools/log_position.py --port /dev/ttyUSB0 --output logs/field_test.csv
python3 tools/plot_position_3d.py --port /dev/ttyUSB0 --log
```

---

## 7. Test tầng 2 — MAVLink → Pixhawk

### 7.1 Kiểm tra gói ODOMETRY

Mission Planner → MAVLink Inspector: tìm message `ODOMETRY` (id 331) từ `sysid=255`, `compid=191`.

| Trường | Giá trị đúng |
|---|---|
| `time_usec` | Unix epoch (µs), không phải boot time |
| `frame_id` | 20 (`MAV_FRAME_LOCAL_FRD`) |
| `child_frame_id` | 12 (`MAV_FRAME_BODY_FRD`) |
| `x`, `y` | vị trí NED (m) |
| `z` | chiều sâu dưới EKF origin |
| `vx/vy/vz` | `NaN` (EKF tự tích phân từ IMU) |
| `velocity_covariance[0]` | `NaN` |
| `estimator_type` | 1 (`MAV_ESTIMATOR_TYPE_NAIVE`) |
| `quality` | > 0 |

### 7.2 Kiểm tra EKF khóa vị trí

Mission Planner → **EKF Status**:
- `Horiz Pos` (Position Horizontal) — phải **xanh** (< 0.3 m)
- `Vert Pos` — xanh
- `Compass` — xanh

Hoặc qua MAVProxy:

```
message EKF_STATUS_REPORT
```

| Chỉ số | Ngưỡng |
|---|---|
| `pos_horiz_ratio` | < 0.5 |
| `flags` | có `GPS_GLITCH_OFF` + `POS_HORIZ_ABS` |

Kiểm tra innovation:

```
message NKF6.SV
```
`Pos X/Y Innovation` ≤ 0.3.

### 7.3 Kiểm tra Origin & Home

```
message HOME_POSITION
message GPS_GLOBAL_ORIGIN
```

Phải có giá trị khác 0. Nếu Home = 0 ⇒ `DO_SET_HOME` bị ArduPilot từ chối (origin chưa set) → **drone arm được nhưng không bay lên**.

---

## 8. Test tầng 3 — Bay thực tế

### 8.1 Static test

1. Đặt drone trên sàn (hoặc trên bàn) **tại a0**, đội cao thu được ~1 m.
2. Mở Mission Planner, đợi EKF xanh.
3. Kiểm tra vị trí trên bản đồ không nhảy lung tung, không trôi.

### 8.2 Walk test

1. Cầm drone đi quanh bên trong rectangle anchor.
2. Quan sát icon drone trên bản đồ Mission Planner: phải **di chuyển mượt**, không giật lùi, không nhảy điểm.
3. Quay video để đối chiếu với vị trí thực tế.

### 8.3 Flight test

1. `Arm` → `Auto` (AltHold) → cất cánh lên ~1 m.
2. Đợi ổn định → chuyển `Loiter`/`PosHold`.
3. Drone phải **giữ vị trí phẳng đẳm**, không xoắn ốc (toilet-bowling).
4. Bay qua lại giữa các vị trí đã đánh dấu (a0 → a1 → a2 → a3 → a0), kiểm tra độ trùng khớp.
5. Bay tới gần rìa rectangle (sát cạnh ngoài) → kiểm tra hành vi khi mất anchor.

> ⚠️ **Safety:** kiểm tra độ trùng khớp EKF **trước khi** bay tự động; luôn có người cầm cần sao lưu; test trong khu vực trống.

### 8.4 Xử lý sự cố

| Triệu chứng | Nguyên nhân thường gặp | Xử lý |
|---|---|---|
| `ODOMETRY` không xuất hiện | `VISO_TYPE ≠ 1`; sai UART; sai baud | Kiểm tra `VISO_TYPE=1`, TX/RX đúng chân |
| EKF không khóa vị trí | Chưa có origin | Kiểm tra `GPS_GLOBAL_ORIGIN` ≠ 0 |
| Arm được nhưng không cất cánh | Thiếu Home | Kiểm tra `HOME_POSITION` ≠ 0 |
| Vị trí lệch hẳn một góc | Sai `α` hoặc sai quy tắc tay trái/phải | Kiểm tra lại bearing và quy tắc §3.2 |
| Vị trí lệch theo một trục | `X`/`Y` đo sai | Đo lại cạnh |
| Độ cao lệch theo chiều sâu | `h_plane` sai | Đo lại chiều cao neo |
| Nhiễu lớn, quality thấp | Antenna bị che, antenna delay sai | Dời antenna, hiệu chuẩn delay |
| Anchor không trả lời | Flash sai `ANCHOR_ID`; board treo | Kiểm tra banner, `lightRecovery`/`heavyRecovery` log |

---

## 9. Bảng ghi chép hiện trường (in ra mang theo)

```
Ngày: __________      Site: ______________________
Người thực hiện: ____________________

KÍCH THƯỚC ĐO
X (a0→a1) = ______ m       Y (a0→a3) = ______ m
α (bearing a0→a1 từ Bắc) = ______ °
h_plane (cao mặt phẳng anchor) = ______ m
GPS a0: lat ______  lon ______  alt ______

TỌA ĐỘ NEO (m)
A1(id1) a0: ( ____, ____, ____ )
A2(id2) a1: ( ____, ____, ____ )
A3(id3) a2: ( ____, ____, ____ )
A4(id4) a3: ( ____, ____, ____ )

ANTENNA DELAY
A1 ______  A2 ______  A3 ______  A4 ______  TAG ______

KIỂM TRA TẦNG 1 (Serial Tag)
Tại a0  → x=____ y=____  OK? ____
Tại a1  → x=____ y=____  OK? ____
Tại a3  → x=____ y=____  OK? ____

KIỂM TRA TẦNG 2 (MAVLink/EKF)
ODOMETRY frame_id=____ child=____ time_usec ok? ____
EKF Horiz Pos = ____   Vert Pos = ____   Compass = ____
HOME_POSITION = ____   GPS_GLOBAL_ORIGIN = ____

KIỂM TRA TẦNG 3 (Bay)
Static ok? ____   Walk ok? ____   Flight ok? ____
Ghi chú: ________________________________
```

---

## 10. Tham khảo

| Tài liệu | Nội dung |
|---|---|
| `firmware/ESP32-DWM3000-UWB/BUILD_FLASH_ubuntu.md` | Build & flash chi tiết (Ubuntu) |
| `firmware/ESP32-DWM3000-UWB/BUILD_FLASH.md` | Build & flash (Windows) |
| `SPEC.md` | Đặc tả MAVLink, mapping ODOMETRY, tham số ArduPilot |
| `docs/ardupilot_non_gps_operation.md` | Vận hành Non-GPS (EKF origin, ARM, GUIDED) |
| `tools/check_rect_layout.cpp` | Test host solver với rectangle (chạy khi đổi thuật toán) |
| ROS: `docs/UWB_ANCHOR_LAYOUT.md` | Quy ước layout & công thức xoay |
| ROS: `docs/UWB_ROS_ESP32_COMPARISON.md` | Bảng đối chiếu từng trường ESP32 ↔ ROS |
| ROS: `docs/NOTES.md` | Sổ ghi chú các kết luận tra cứu source ArduPilot (vd: Pozyx vs ODOMETRY) |
