# 🌱 Smart Autonomous Agricultural Robot (ESP32)

An ESP32-powered autonomous agricultural rover built and exhibited in 6 days at **Sri Sri University's Faculty of Engineering & Technology** exhibition. It navigates rows while dodging obstacles, probes soil moisture, digs, drops seeds, waters the soil, and hosts its own local Wi-Fi control dashboard.

![Smart Autonomous Agricultural Robot](SmartAgri%20Robot.jpeg)

---

## ⚡ Features
- **Autonomous Obstacle Avoidance:** Front-mounted HC-SR04 ultrasonic sensor on a scanning SG90 servo compares left vs. right clearance using 3-sample median filtering to steer around obstacles.
- **Automated Planting Cycle:** At programmed intervals, the rover halts and runs a non-blocking multi-stage sequence:
  1. Lowers a servo-mounted soil moisture probe (12-bit ADC readings calibrated between dry `3834` and wet `1278`).
  2. Executes 5 digging cycles using a rear SG90 actuator.
  3. Dispenses seeds via a servo-controlled hopper gate.
  4. Triggers an active-LOW relay water pump for targeted irrigation.
- **Local Wi-Fi Web Dashboard:** Spins up a standalone ESP32 Access Point (`SSID: AgriRobot`) for switching between **Auto** and **Manual** modes and viewing live telemetry from a mobile browser.

---

## 🛠️ Hardware & Bill of Materials
- **Microcontroller:** ESP32 DevKitC (38-pin)
- **Drive System:** L298N Dual H-Bridge Motor Driver + 4× BO Gear Motors
- **Sensors:** HC-SR04 Ultrasonic Distance Sensor, Analog Soil Moisture Sensor
- **Actuators:** 4× SG90 Micro Servos (Ultrasonic Scanner, Soil Probe, Rear Digger, Seed Dispenser)
- **Irrigation:** 5V Single-Channel Relay Module (`SRD-05VDC-SL-C`, Active-LOW) + Mini Submersible Water Pump
- **Power:** 7.4V Li-ion Battery Pack + LM2596 DC-DC Buck Converter (regulated 5V rail for servos/logic)

---

## 🔌 ESP32 GPIO Pinout

| Component | Pin / Signal | ESP32 GPIO | Notes |
| :--- | :--- | :--- | :--- |
| **L298N Motor Driver** | `IN1` / `IN2` | `GPIO 25` / `GPIO 26` | Left motors direction |
| **L298N Motor Driver** | `IN3` / `IN4` | `GPIO 27` / `GPIO 14` | Right motors direction |
| **L298N Motor Driver** | `ENA` / `ENB` | `GPIO 32` / `GPIO 33` | PWM speed control |
| **HC-SR04 Ultrasonic** | `TRIG` / `ECHO` | `GPIO 23` / `GPIO 22` | Distance sensing |
| **Scanner Servo** | `Signal` | `GPIO 18` | Center `90°`, Right `10°`, Left `170°` |
| **Digging Servo** | `Signal` | `GPIO 21` | Rest `180°`, Dig `0°` |
| **Seed Dispenser Servo** | `Signal` | `GPIO 13` | Closed `90°`, Open `160°` |
| **Soil Probe Servo** | `Signal` | `GPIO 16` | Up `0°`, Probe down `90°` |
| **Soil Moisture Sensor** | `AOUT` | `GPIO 34` | 12-bit analog input (`ADC1`) |
| **Water Pump Relay** | `IN` | `GPIO 19` | Active-LOW trigger |

---

## 📐 Circuit Diagram
![Smart Autonomous Agricultural Robot Circuit](smart-agri-robot-car.png)

*Editable Cirkit Designer project file: [`Smart Agri Robot circuit diagram.ckt`](your_circuit_file.ckt)*


## 🚀 How to Run
1. Open `Smart AgriRobot.ino` in **Arduino IDE** with the **ESP32 Board Package (Core 3.x)** installed.
2. Install the `ESP32Servo` library via Library Manager.
3. Select **ESP32 Dev Module** and upload the sketch.
4. Connect your phone or tablet to the **`AgriRobot`** Wi-Fi network and open the web interface in your browser.
