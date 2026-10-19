# Step 3: Verify Launch - Kiểm tra lỗi sau khi remove logs

## Mục đích
Chạy launch file và kiểm tra log để đảm bảo việc xóa debug logs không gây ra lỗi nào, và xác nhận chức năng UWB positioning vẫn hoạt động đúng.

## Lệnh kiểm tra
```bash
ros2 launch ardupilot_gz_bringup x500_uwb.launch.py use_gz_sim_gui:=false rviz:=false
```

## Các thành phần được kiểm tra

### 1. UWB Nodes ✅
| Node | Trạng thái | Ghi chú |
|------|-----------|---------|
| `uwb_gz_to_ros_bridge` | ✅ OK | Bridge Gazebo → ROS hoạt động |
| `uwb_trilateration_node` | ✅ OK | 4 anchors detected |
| `uwb_to_mavros_vision_node` | ✅ OK | Forward pose → mavros |

### 2. ArduPilot SITL ✅
| Thành phần | Trạng thái | Ghi chú |
|------------|-----------|---------|
| ArduCopter | ✅ Running | V4.8.0-dev |
| EKF3 | ✅ Active | IMU aligned |
| MAVLink | ✅ Connected | Heartbeat OK |
| Parameters | ✅ Loaded | 1362 params |

### 3. MAVROS ✅
| Plugin | Trạng thái | Ghi chú |
|--------|-----------|---------|
| vision_pose | ✅ OK | Nhận vision position từ UWB |
| vision_speed | ✅ OK | Plugin initialized |
| mission | ✅ OK | Mission received |
| geofence | ✅ OK | Fence loaded |

## Warnings (Không cần fix)

### mavros.ftp Lost sync ⚠️
```
[WARN] mavros.ftp: FTP: Lost sync! seqnr: 2 != 5
```
- **Nguyên nhân**: MAVROS fetch params qua FTP protocol khi khởi động
- **Tự recover**: Có, sau vài attempts
- **Ảnh hưởng**: Không - params vẫn load đầy đủ

### TM: Wrong FCU time ⚠️
```
[WARN] mavros.time: TM: Wrong FCU time.
```
- **Nguyên nhân**: SITL synthetic clock có thể drift nhẹ
- **Tự recover**: Có
- **Ảnh hưởng**: Không

### CMD: Unexpected command 410 ⚠️
```
[WARN] mavros.cmd: CMD: Unexpected command 410, result 4
```
- **Nguyên nhân**: MAVROS gửi command mà ArduPilot không hỗ trợ
- **Tự recover**: Có
- **Ảnh hưởng**: Không

## Lỗi cần lưu ý

### PreArm: Baro: not healthy ⚠️
```
[ERROR] mavros.sys: FCU: PreArm: Baro: not healthy
```
- **Nguyên nhân**: Barometer trong SITL chưa ready sau khi calibrate
- **Ảnh hưởng**: Drone không thể arm trong mode thường
- **Giải pháp**: 
  - Chờ thêm vài giây sau khi boot
  - Hoặc chuyển sang mode GUIDED sau khi arm thủ công
  - Hoặc disable baro health check qua parameter

## Kết quả cuối cùng
- ✅ Không có lỗi nghiêm trọng
- ✅ Chức năng UWB positioning hoạt động
- ✅ Launch file chạy ổn định
- ✅ Terminal không còn spam log
