# Step 2: Remove UwbSystem Debug Log

## Mục đích
Xóa block debug log `Dbg tag_pose=(...)` trong `UwbSystem.cc` - log này xuất hiện mỗi 50 simulation cycles (khoảng 2.5 giây) và gây spam terminal thêm.

## Chức năng bị ảnh hưởng
- **File**: `src/uwb_gz_plugin/src/UwbSystem.cc`
- **Hàm**: `PostUpdate()` - được gọi mỗi simulation step
- **Tần suất log**: 0.4 Hz (mỗi 50 cycles ≈ 2.5s)
- **Nội dung log**:
  ```
  [UwbSystem] Dbg WorldPose tag_pose=(0.0, 0.0, 0.0)
  ```
- **Biến theo dõi**: `static int dbg = 0;`

## Nguyên nhân gây lag
- Dù tần suất thấp hơn OnPose, nhưng vẫn gây spam không cần thiết
- Thông tin `tag_pose` chỉ hữu ích khi debug, không cần thiết trong production

## Thay đổi
```diff
-  static int dbg = 0;
-  if (++dbg % 50 == 0) {
-    gzmsg << "[UwbSystem] Dbg " << pose_src
-          << " tag_pose=(" << tag_pose.X() << ", " << tag_pose.Y() << ", "
-          << tag_pose.Z() << ")" << std::endl;
-  }
```
→ Xóa hoàn toàn block debug log

## Kết quả
- ✅ Debug log đã xóa hoàn toàn
- ✅ Chức năng UWB ranging trong Gazebo vẫn hoạt động bình thường
- ✅ Giảm thêm terminal output

## Lệnh kiểm tra
```bash
grep "UwbSystem.*Dbg" /tmp/x500_uwb_nolog.log | wc -l
# Output: 0
```
