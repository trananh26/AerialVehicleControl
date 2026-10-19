# Step 3: ROS 2 Bridge + Trilateration + MAVLink

## Mô tả
Package `uwb_bridge` chứa các ROS 2 nodes để bridge dữ liệu UWB từ Gazebo sang ROS 2, thực hiện trilateration, và forward kết quả đến MAVLink/MAVROS.

## Chức năng

### 1. ros_gz_bridge (C++ - đã build sẵn)
- **File**: `config/ros_gz_bridge.yaml`
- **Chức năng**: Bridge Gazebo → ROS 2 topics

| Gazebo Topic | ROS Topic | Type |
|--------------|-----------|------|
| `/uwb/ranging/tag` | `/uwb/ranging/ros` | Pose |
| `/uwb/tag_pose/tag` | `/uwb/tag_pose/ros` | Pose |
| `/uwb/anchors/tag` | `/uwb/anchors/0` | Marker |

### 2. uwb_trilateration_node.py
- **Chức năng**: Weighted Least-Squares Trilateration
- **Input**: Ranging data từ 4 anchors
- **Output**: `PoseWithCovarianceStamped` (position estimation)

**Thuật toán:**
1. **Linear seed**: Closed-form linearisation để estimate vị trí ban đầu
2. **Gauss-Newton WLS**: Iterative weighted least squares
3. **Covariance estimation**: Tính toán covariance matrix từ Jacobian

**Parameters:**
| Param | Default | Mô tả |
|-------|---------|--------|
| `input_topic` | `/uwb/ranging/ros` | Input ranging |
| `anchor_positions` | 4 corners 10x10m | Vị trí anchors |
| `los_std` | 0.10m | LOS noise std |
| `nlos_soft_std` | 0.30m | NLOS soft noise |
| `nlos_hard_std` | 0.60m | NLOS hard noise |
| `min_anchors` | 3 | Số anchors tối thiểu |

### 3. uwb_to_mavros_vision_node.py
- **Chức năng**: Bridge UWB pose → MAVROS vision_pose
- **Input**: `/x500/uwb/pose_cov`
- **Output**: `/mavros/vision_pose/pose`

### 4. mavlink_bridge_node.py
- **Chức năng**: MAVLink bridge cho custom messages

## Data Flow
```
Gazebo
  └── /uwb/ranging/tag ──────────────────────┐
                                            ▼
                                    ros_gz_bridge
                                            │
                                            ▼
                                  /uwb/ranging/ros
                                            │
                                            ▼
                              uwb_trilateration_node
                              (WLS Trilateration)
                                            │
                                            ▼
                                  /x500/uwb/pose_cov
                                            │
                                            ▼
                              uwb_to_mavros_vision_node
                                            │
                                            ▼
                                  /mavros/vision_pose/pose
                                            │
                                            ▼
                                    ArduPilot EKF3
```

## Anchors Layout (10x10m square)
```
    y=10
  A2(10,10) ───── A3(0,10)
    │              │
    │   Drone      │
    │   (5,5)      │
    │              │
  A0(0,0)  ───── A1(10,0)
    x=0          x=10
```

## Link với các bước khác
- **依赖**: UWB Plugin + SDF
- **Sử dụng bởi**: MAVROS Integration (Step 4)
