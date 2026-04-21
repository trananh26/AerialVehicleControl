# 0. Kết nối phần cứng
Thực hiện cấp nguồn cho pi, cắm cổng usb của pi kết nối với cổng mini usb của pixhawk

# 1. Kết nối mạng & SSH vào Raspberry Pi
## Bước 1: Tạo mạng WiFi
Dùng điện thoại bật hotspot:
SSID: POCO F3
Password: 11111111

## Bước 2: Kết nối thiết bị
Kết nối:
Raspberry Pi vào WiFi
Laptop (Ubuntu) vào cùng WiFi
👉 Đảm bảo cả 2 cùng mạng (có thể kiểm tra số thiết bị kết nối trên hotspot)

## Bước 3: Xác định dải mạng
Trên Ubuntu:
ifconfig
Ví dụ: inet 192.168.1.23  netmask 255.255.255.0
👉 Suy ra:IP máy: 192.168.1.23 Dải mạng: 192.168.1.0/24

## Bước 4: Quét IP trong mạng bằng nmap
Cài nếu chưa có: sudo apt install nmap

Quét mạng: nmap -sn 192.168.1.0/24

👉 Kết quả:
Danh sách tất cả thiết bị đang online
Tìm IP của Raspberry Pi (thường tên raspberrypi hoặc dựa vào MAC)

## Bước 5: SSH vào Raspberry Pi
ssh pi@<ip_cua_pi>
Ví dụ: ssh pi@192.168.1.10


# 2. Chạy MAVROS và điều khiển drone
Sau khi SSH vào Pi, mở 2 terminal SSH riêng biệt
Terminal 1: Chạy MAVROS
ros2 launch mavros apm.launch fcu_url:=/dev/ttyACM0:57600

Terminal 2: Chạy chương trình điều khiển
Di chuyển vào workspace:cd workspace/drone_control (check lại folder trên folder src)

Source môi trường: source install/setup.bash

Cất cánh & hạ cánh: ros2 run drone_control takeoff_land_mission

Bay theo quỹ đạo tròn: ros2 run drone_control circle_mission