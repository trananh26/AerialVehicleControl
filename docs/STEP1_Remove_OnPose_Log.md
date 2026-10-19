# Step 1: Remove OnPose Debug Log Spam

## Mục đích
Xóa dòng log `OnPose called #N, id_z=X range=Y.X` trong `ros_gz_bridge_node.cc` đang spam 20 lần/giây gây lag terminal và ảnh hưởng hiệu năng chương trình.

## Chức năng bị ảnh hưởng
- **File**: `src/uwb_gz_plugin/src/ros_gz_bridge_node.cc`
- **Hàm**: `OnPose()` - callback nhận pose từ Gazebo UWB plugin
- **Tần suất log**: 20 Hz (mỗi 50ms)
- **Nội dung log**:
  ```
  [INFO] [x500.uwb_gz_to_ros_bridge]: OnPose called #1865, id_z=3 range=7.328
  ```

## Nguyên nhân gây lag
- Mỗi callback gọi `RCLCPP_INFO()` tạo output ra terminal
- 20 Hz log = 1200 dòng/phút = quá nhiều cho terminal buffer
- Gây I/O overhead và làm chậm các process khác trong launch file

## Thay đổi
```diff
-  static int count = 0;
-  if (++count % 4 == 1) {
-    RCLCPP_INFO(get_logger(), "OnPose called #%d, id_z=%.0f range=%.3f",
-      count, msg.position().z(), msg.position().x());
-  }
```
→ Xóa hoàn toàn block log, giữ nguyên logic xử lý pose

## Kết quả
- ✅ Log spam đã xóa hoàn toàn
- ✅ Chức năng bridge pose từ Gazebo → ROS vẫn hoạt động bình thường
- ✅ Giảm đáng kể terminal output và I/O overhead

## Lệnh kiểm tra
```bash
grep "OnPose" /tmp/x500_uwb_nolog.log | wc -l
# Output: 0
```
