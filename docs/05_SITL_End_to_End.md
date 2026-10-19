# Step 5: SITL End-to-End

## Mô tả
Launch file end-to-end để test toàn bộ hệ thống UWB positioning từ Gazebo simulation đến ArduPilot EKF.

## Chức năng
Launch file `step5_sitl_end_to_end.launch.py` khởi động đồng thời:
1. ArduPilot SITL (`arducopter`)
2. UWB pipeline (gz → ROS → trilateration → relay)
3. MAVROS với `vision_pose` plugin

## Architecture
```
┌─────────────────────────────────────────────────────────────────┐
│              step5_sitl_end_to_end.launch.py                    │
├─────────────────────────────────────────────────────────────────┤
│                                                                  │
│  ┌──────────────┐    ┌──────────────┐    ┌──────────────────┐  │
│  │ ArduPilot   │    │  UWB        │    │    MAVROS       │  │
│  │ SITL        │◄───│  Pipeline   │    │  (vision_pose)  │  │
│  │ (TCP 5760)  │    │             │    │                 │  │
│  └──────────────┘    └──────────────┘    └─────────────────┘  │
│         │                   │                    │             │
│         │    MAVLink        │    MAVLink         │             │
│         └───────────────────┴────────────────────┘             │
│                                                                  │
└─────────────────────────────────────────────────────────────────┘
```

## End-to-End Data Flow
```
1. Gazebo World (UWB Plugin)
   └── /uwb/ranging/tag (20 Hz)
          │
          ▼
2. ros_gz_bridge
   └── /uwb/ranging/ros
          │
          ▼
3. uwb_trilateration_node (WLS Solver)
   └── /x500/uwb/pose_cov
          │
          ▼
4. uwb_to_mavros_vision_node
   └── /mavros/vision_pose/pose
          │
          ▼
5. MAVROS (VISION_POSITION_ESTIMATE MAVLink message)
   └── TCP → ArduPilot SITL
          │
          ▼
6. ArduPilot EKF3 (Vision Position Estimator)
   └── Attitude, Position Estimate
```

## Launch Arguments
| Argument | Default | Mô tả |
|----------|---------|--------|
| `sitl_bin` | `.../arducopter` | Path to SITL binary |
| `parm_file` | `copter-uwb.parm` | UWB ArduPilot params |
| `sitl_tcp_port` | `5760` | SITL TCP listen port |
| `fcu_url` | `tcp://127.0.0.1:5760` | MAVROS → SITL URL |

## Parameters (copter-uwb.parm)
```bash
# Disable GPS, enable vision EKF
GPS_TYPE = 0          # No GPS
EK3_SRC1_POSXY = 6   # Vision position XY
EK3_SRC1_VELXY = 0   # No velocity source
EK3_SRC1_POSZ = 1    # Barometer for Z
EK3_SRC1_YAW = 6     # Vision yaw
VISO_TYPE = 1         # Enable visual odometry
```

## Test Workflow
1. Launch: `ros2 launch uwb_bridge step5_sitl_end_to_end.launch.py`
2. Kiểm tra EKF status: `ros2 topic echo /mavros/ekf_status`
3. Kiểm tra vision pose: `ros2 topic echo /mavros/vision_pose/pose`
4. Arm drone và test position hold

## Verification
- [ ] EKF3 reports vision as primary position source
- [ ] Position estimates match UWB trilateration output
- [ ] Drone holds position using UWB positioning
