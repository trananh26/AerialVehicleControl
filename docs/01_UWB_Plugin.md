# Step 1: UWB Gazebo Plugin

## Mô tả
Gazebo plugin mô phỏng hệ thống UWB (Ultra-Wideband) positioning trong simulation. Plugin tạo các topic `/uwb/ranging/tag` và `/uwb/tag_pose/tag` để gửi ranging data và pose information.

## Chức năng
- **`UwbRanging.cc`**: Xử lý tính toán khoảng cách giữa tag và các anchor
- **`UwbSystem.cc`**: Quản lý system state, pose estimation, simulation step
- **`ros_gz_bridge_node.cc`**: Bridge dữ liệu từ Gazebo sang ROS 2

## Topics
| Topic | Direction | Type |
|-------|-----------|------|
| `/uwb/ranging/tag` | Gazebo → ROS | `gz::msgs::Pose` |
| `/uwb/tag_pose/tag` | Gazebo → ROS | `gz::msgs::Pose` |

## Files
```
src/uwb_gz_plugin/
├── CMakeLists.txt
├── package.xml
├── include/uwb_gz_plugin/
│   ├── UwbRanging.hh
│   └── UwbSystem.hh
├── src/
│   ├── UwbRanging.cc
│   ├── UwbSystem.cc
│   └── ros_gz_bridge_node.cc
├── models/           # SDF models cho anchor và tag
├── worlds/           # SDF world files
└── hooks/            # Gazebo hooks
```

## Debug logs đã xóa
- `OnPose` log spam 20Hz trong `ros_gz_bridge_node.cc`
- `UwbSystem Dbg` log mỗi 50 cycles trong `UwbSystem.cc`
