# Hệ thống Cảnh báo Nhiệt độ sử dụng RTOS (RTOS-Based Over-Temperature Alarm System)

Đây là một dự án môn học "Hệ thống Nhúng", được phát triển để xây dựng một thiết bị có khả năng giám sát nhiệt độ môi trường và phát ra cảnh báo bằng còi khi nhiệt độ vượt qua một ngưỡng được thiết lập sẵn. 
Toàn bộ hệ thống được quản lý bởi Hệ điều hành thời gian thực FreeRTOS.

## Tính năng chính

- **Giám sát nhiệt độ thời gian thực:** Liên tục đọc dữ liệu nhiệt độ từ cảm biến DHT11.
- **Cảnh báo quá ngưỡng:** Tự động kích hoạt còi báo động khi nhiệt độ đo được vượt quá ngưỡng cài đặt.
- **Thiết lập ngưỡng động:** Người dùng có thể thay đổi ngưỡng nhiệt độ cảnh báo thông qua điều khiển hồng ngoại (IR remote).
- **Hiển thị thông tin:** Hiển thị nhiệt độ hiện tại và các trạng thái lên màn hình OLED.
- **Kiến trúc đa nhiệm:** Sử dụng FreeRTOS để quản lý các tác vụ (đọc cảm biến, điều khiển còi, giao tiếp UART) một cách độc lập và hiệu quả.

## Phần cứng sử dụng

- Vi điều khiển: **STM32F103C8T6 (Blue Pill)**
- Cảm biến: **DHT11** (Nhiệt độ & Độ ẩm)
- Cơ cấu chấp hành: **Còi báo (Buzzer)**
- Hiển thị: **Màn hình OLED 0.96 inch (I2C)**
- Giao tiếp: **Mắt thu hồng ngoại (IR Receiver)**

## Công nghệ & Phần mềm

- **Ngôn ngữ:** C/C++
- **Hệ điều hành:** FreeRTOS
- **IDE:** STM32CubeIDE
- **Giao thức:** UART (để gửi dữ liệu lên máy tính giám sát)
