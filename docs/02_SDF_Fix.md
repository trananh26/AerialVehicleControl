# Step 2: SDF Fix

## Mô tả
Các file SDF (Simulation Description Format) định nghĩa models và world cho Gazebo simulation.

## Chức năng

### Models
| File | Mô tả |
|------|--------|
| `models/uwb_anchor/model.sdf` | Model template cho UWB anchor (cube 20x20x20cm) |
| `models/uwb_anchor/model.config` | Metadata cho anchor model |
| `models/x500_with_uwb/model.sdf` | X500 drone với UWB tag attached |
| `models/x500_with_uwb/model.config` | Metadata cho x500 model |

### Worlds
| File | Mô tả |
|------|--------|
| `worlds/uwb_runway.sdf` | World file với runway, 4 UWB anchors |
| `worlds/uwb_plugin_test.sdf` | Test world đơn giản |

## World Configuration
- **4 UWB Anchors**: đặt tại corners của 10x10m square (z=2m)
- **Update rate**: 20 Hz
- **Max range**: 50m
- **LOS std**: 0.05m (Line-of-Sight noise)

## Anchors Layout
```
    y=10
  A2(10,10) ───── A3(0,10)
    │              │
    │   10x10m     │
    │              │
  A0(0,0)  ───── A1(10,0)
    x=0          x=10
```

## Link với các bước khác
- **依赖**: UWB Plugin (`uwb_gz_plugin`)
- **Sử dụng bởi**: ROS 2 Bridge (gazebo_ros)
