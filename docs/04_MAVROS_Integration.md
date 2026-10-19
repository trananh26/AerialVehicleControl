# Step 4: MAVROS Integration

## Mô tả
MAVROS integration kết nối hệ thống UWB positioning với ArduPilot thông qua ArduPilot SITL và MAVProxy.

## Chức năng

### Launch Files
| File | Mô tả |
|------|--------|
| `x500_uwb.launch.py` | Main launch file cho x500 với UWB |

### Launch Arguments
| Argument | Default | Mô tả |
|----------|---------|--------|
| `world_name` | `uwb_runway` | Gazebo world với 4 UWB anchors |
| `robot_name` | `x500` | Namespace cho robot |
| `instance` | `0` | SITL instance (ảnh hưởng port) |
| `x` | `5.0` | Initial x position (m) |
| `y` | `5.0` | Initial y position (m) |
| `z` | `0.2` | Initial z position (m) |

## Architecture

### Node Graph
```
┌─────────────────────────────────────────────────────────────────┐
│                        x500_uwb.launch.py                        │
├─────────────────────────────────────────────────────────────────┤
│                                                                  │
│  ┌──────────────┐    ┌──────────────┐    ┌──────────────────┐  │
│  │  gz server   │    │   gz GUI     │    │      RViz        │  │
│  │  (world+sitl)│    │  (optional)  │    │   (optional)     │  │
│  └──────────────┘    └──────────────┘    └──────────────────┘  │
│                                                                  │
│  ┌──────────────────────────────────────────────────────────┐   │
│  │                      Robot Nodes                          │   │
│  │                                                           │   │
│  │  ┌─────────────┐  ┌─────────────┐  ┌─────────────────┐  │   │
│  │  │ robot_state │  │ros_gz_bridge│  │  ros_gz_sim     │  │   │
│  │  │ publisher   │  │  (x500)     │  │  create (spawn) │  │   │
│  │  └─────────────┘  └─────────────┘  └─────────────────┘  │   │
│  │                                                           │   │
│  │  ┌─────────────┐  ┌─────────────┐  ┌─────────────────┐  │   │
│  │  │ ardupilot   │  │  MAVProxy   │  │    MAVROS       │  │   │
│  │  │ SITL        │  │             │  │  (vision_pose)  │  │   │
│  │  └─────────────┘  └─────────────┘  └─────────────────┘  │   │
│  │                                                           │   │
│  │  ┌─────────────┐  ┌─────────────┐  ┌─────────────────┐  │   │
│  │  │uwb_gz_bridge│  │trilateration│  │  relay          │  │   │
│  │  │ (C++)       │  │   (WLS)     │  │(→mavros vision) │  │   │
│  │  └─────────────┘  └─────────────┘  └─────────────────┘  │   │
│  └──────────────────────────────────────────────────────────┘   │
└─────────────────────────────────────────────────────────────────┘
```

### Port Configuration
| Port | Mô tả |
|------|-------|
| `5760 + 10*N` | SITL master (MAVLink) |
| `5501 + 10*N` | SITL TCP |
| `14550 + 10*N` | MAVROS output |

### MAVROS Plugins
| Plugin | Mô tả |
|--------|--------|
| `vision_pose` | Nhận vision position từ UWB trilateration |
| `mission` | Mission control |
| `param` | Parameter management |
| `geofence` | Geofence support |

## Link với các bước khác
- **依赖**: ROS 2 Bridge (Step 3)
- **Sử dụng bởi**: SITL End-to-End (Step 5)
