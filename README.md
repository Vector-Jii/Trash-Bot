# 🧹 Trash Collection Rower

A smart, ESP32-powered robotic vehicle designed to automate roadside trash collection. With real-time remote control capabilities and a precision servo-controlled arm, this system helps reduce manual labor and contributes to a cleaner environment.

## 🚀 Features
- Mobile/Wi-Fi-based remote control
- Servo-powered robotic arm and claw for trash pickup
- Precision maneuvering using dual motor drives
- Efficient obstacle detection via ultrasonic sensor
- Custom acrylic chassis for durable construction

## 🧰 Hardware Components
- ESP32 microcontroller
- PCA9685 (16-channel PWM servo driver)
- Servo Motors ×4
- Ultrasonic Sensor
- L298N Motor Driver
- 8×2 Wheels ×4
- Acrylic Sheet (Body Construction)

## 🛠️ Software Used
- Arduino IDE

## 📡 Remote Access
To control the robot remotely:
- Connect your mobile device to the Wi-Fi SSID specified in the code
- Use the provided password to access the control interface

## ⚙️ How It Works
- The ESP32 manages all communication and control logic.
- The robot can be directed in all directions for flexible navigation.
- The servo arm and claw are manually controlled via the web/mobile interface.
- Ultrasonic sensor provides obstacle awareness for smoother operation.

## 🧾 Repository Structure
```bash
├── /src
│   └── trash_collection.ino
├── /docs
│   └── schematic.png
├── README.md
