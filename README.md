# Synapto: Multi-Modal Edge AI Orchestration Framework

Bridging Cognitive AI and Industrial Control with NVIDIA Jetson & ROS 2

**Synapto** is an open-source Industrial IoT (IIoT) framework designed for autonomous process management. It leverages the **NVIDIA Jetson Orin Nano Super** for high-level perception and the **M5Stack-PLC** for deterministic execution, creating a "Neural-to-Mechanical" bridge.

---

## 🚀 Key Features

- **Cognitive Edge (Synapto-Core):** Powered by the NVIDIA Jetson Orin Nano Super (67 TOPS), utilizing NVIDIA Isaac ROS and DeepStream for real-time sensor fusion and AI inference.
- **Real-Time Execution (Synapto-Reflex):** micro-ROS implementation on M5Stack-PLC (ESP32) for deterministic, low-latency I/O and motor control.
- **3D Digital Twin:** A high-fidelity spatial replica built in Godot Engine, pixel-streamed via WebRTC for zero-latency remote monitoring.
- **Industrial Reliability:** Full PackML (ISA-TR88.00.02) state machine compliance for predictable process lifecycles.
- **Edge Historian:** High-speed data logging and AI model caching on NVMe SSD storage.

[📖 View Full Documentation on Wiki](https://github.com/engdaniyalh/synapto/wiki)

---

## 🏗 System Architecture

### Hardware Stack
- **Compute:** NVIDIA Jetson Orin Nano Super Developer Kit (8GB)
- **PLC/Control:** M5Stack-PLC (ESP32 with Opto-isolated I/O)
- **Storage:** 500GB+ NVMe SSD (High-IOPS for Edge Historian)
- **Vision:** MLX90640 Thermal / USB Optical

### Software Stack
- **Middleware:** ROS 2 Humble (DDS/micro-ROS)
- **AI:** TensorRT, DeepStream SDK
- **Visualization:** Godot Engine 4.x (Vulkan) + Flutter
- **Containerization:** Docker + NVIDIA Container Toolkit

[🔧 Hardware Setup Guide](https://github.com/engdaniyalh/synapto/wiki/Hardware-Setup) | [📦 Software Dependencies](https://github.com/engdaniyalh/synapto/wiki/Dependencies)

---

## 📂 Project Structure

```
├── synapto_core/          # ROS 2 Humble nodes (C++/Python)
├── synapto_reflex/        # M5Stack micro-ROS firmware (Arduino/C++)
├── synapto_twin/          # Godot 3D Digital Twin project
├── synapto_portal/        # Flutter Orchestrator Mobile App
├── docker/                # Container configurations (JetPack optimized)
└── manifests/             # Universal Process JSON/Protobuf definitions
```

[📁 Repository Structure Explained](https://github.com/engdaniyalh/synapto/wiki/Repository-Structure)

---

## 🛠 Installation & Quickstart

### 1. Flash the Reflex (M5Stack)
Ensure you have the `micro_ros_arduino` library installed. Upload the provided `.ino` file to your M5Stack-PLC.

[📘 M5Stack Flashing Guide](https://github.com/engdaniyalh/synapto/wiki/Flashing-M5Stack)

### 2. Launch the Core (Jetson)
Use Docker Compose to launch the AI perception and ROS 2 bridge:

```bash
docker-compose up -d
```

[🚀 Jetson Deployment Guide](https://github.com/engdaniyalh/synapto/wiki/Jetson-Deployment)

### 3. Connect the Portal
Open the Synapto Portal (Flutter) on your mobile device. The system will automatically discover the node via mDNS.

[📱 Mobile App Setup](https://github.com/engdaniyalh/synapto/wiki/Mobile-App)

---

## 📊 Industrial Standards Implementation

Synapto is built for the factory floor. It implements:

- **PackML State Machine:** Standardized transitions (Idle, Starting, Execute, Aborted)
- **OPC UA Pub/Sub:** Industry-standard "Northbound" data plumbing for SCADA integration (Ignition/Siemens)
- **Fail-Safe Interlocks:** Hardware-level watchdogs to ensure the Reflex enters a safe state if the Core loses heartbeat

[🏭 Industrial Compliance Details](https://github.com/engdaniyalh/synapto/wiki/Industrial-Standards)

---

## 📽 Demo & Visualization

The Digital Twin in Godot provides a real-time spatial representation of the process. Below is a conceptual view of the Pixel Streaming interface:

```
┌─────────────────────────────────────────────────────────────┐
│                    Synapto Digital Twin                      │
│  ┌───────────────────────────────────────────────────────┐  │
│  │                                                       │  │
│  │    [AI Perception Overlay]    [Real-time Metrics]    │  │
│  │                                                       │  │
│  │         ╱╲          ╱╲         ╱╲                    │  │
│  │        ╱  ╲        ╱  ╲       ╱  ╲                   │  │
│  │       ╱    ╲______╱    ╲_____╱    ╲                  │  │
│  │      ╱      ╲    ╱      ╲    ╱      ╲                 │  │
│  │     ╱________╲__╱________╲__╱________╲                │  │
│  │                                                       │  │
│  │    State: EXECUTE   Temp: 42°C   Cycle: 98%          │  │
│  └───────────────────────────────────────────────────────┘  │
│  [◀] [▶] [↺] [🔍]                     FPS: 60 | Latency: 8ms │
└─────────────────────────────────────────────────────────────┘
```

[🎮 Digital Twin Configuration](https://github.com/engdaniyalh/synapto/wiki/Digital-Twin)

---

## 📚 Additional Resources

- [Troubleshooting Guide](https://github.com/engdaniyalh/synapto/wiki/Troubleshooting)
- [API Reference](https://github.com/engdaniyalh/synapto/wiki/API)
- [Contributing Guidelines](https://github.com/engdaniyalh/synapto/wiki/Contributing)
- [FAQ](https://github.com/engdaniyalh/synapto/wiki/FAQ)
- [Release Notes](https://github.com/engdaniyalh/synapto/wiki/Releases)

---

## 👤 Author

**Daniyal Hasan**

[![LinkedIn](https://img.shields.io/badge/LinkedIn-0077B5?style=for-the-badge&logo=linkedin&logoColor=white)](https://linkedin.com/in/daniyalhasan)

---

## 📄 License

MIT © Daniyal Hasan

---

*Built for the next generation of industrial automation.* ⚙️🤖

---

**Quick Links:**
[🏠 Home](https://github.com/engdaniyalh/synapto) | [📖 Wiki](https://github.com/engdaniyalh/synapto/wiki) | [🐛 Issues](https://github.com/engdaniyalh/synapto/issues) | [✨ Discussions](https://github.com/engdaniyalh/synapto/discussions)
