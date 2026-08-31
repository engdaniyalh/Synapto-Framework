# Synapto Framework

[![CI/CD Pipeline](https://github.com/engdaniyalh/Synapto-Framework/actions/workflows/ci.yml/badge.svg)](https://github.com/engdaniyalh/Synapto-Framework/actions)
[![C++ Standard](https://img.shields.io/badge/C%2B%2B-20-blue.svg)](https://isocpp.org/)
[![Python Version](https://img.shields.io/badge/Python-3.10-blue.svg)](https://python.org)
[![OPC UA](https://img.shields.io/badge/OPC%20UA-IEC%2062541-green.svg)](https://opcfoundation.org/)
[![PackML](https://img.shields.io/badge/Standard-ISA--TR88.00.02-orange.svg)](https://www.omac.org/packml)
[![Physics](https://img.shields.io/badge/Digital%20Twin-MuJoCo-red.svg)](https://mujoco.org/)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)

> **Industrial Edge AI Framework bridging 100Hz deterministic real-time control (C++20 PREEMPT_RT) and predictive anomaly detection (ONNX Runtime) via OPC UA (IEC 62541) and high-fidelity physics digital twins (MuJoCo).**

---

## 🏗 System Architecture

Synapto enforces a **strict two-thread dual-domain architecture** at the edge, guaranteeing deterministic real-time execution while facilitating asynchronous cloud/SCADA connectivity and machine learning inference:

```text
 INDUSTRIAL HARDWARE / REAL-TIME EDGE (Synapto-Reflex)
┌────────────────────────────────────────────────────────────────────────────────────────┐
│                                                                                        │
│   [ RT Thread: SCHED_FIFO (98) ]                      [ Non-RT Network Worker ]        │
│   ┌────────────────────────────┐    SPSC Ring Buffer  ┌────────────────────────────┐   │
│   │ • 100Hz clock_nanosleep    │ ───────────────────> │ • Drains Lock-Free Queue   │   │
│   │ • ISA PackML State Machine │  (64B Cache Line     │ • open62541 OPC UA Server  │   │
│   │ • Stochastic Motor Physics │   Acquire/Release)   │ • Non-blocking UDP Egress  │   │
│   │ • Zero Alloc / Zero Mutex  │                      └─────────────┬──────────────┘   │
│   └────────────────────────────┘                                    │                  │
└─────────────────────────────────────────────────────────────────────┼──────────────────┘
                                                                      │
                                                   OPC UA TCP (4840)  │  UDP Egress (4242)
                                                                      │  {"seq", "state", "curr"}
                                        ┌─────────────────────────────┴──────────┐
                                        ▼                                        ▼
┌────────────────────────────────────────────────────────┐   ┌───────────────────────────┐
│               AI CORE & PREDICTIVE TWIN                │   │    PHYSICS DIGITAL TWIN   │
│   ┌────────────────────────────────────────────────┐   │   │   ┌───────────────────┐   │
│   │ Async OPC UA Client (asyncua)                  │   │   │   │ MuJoCo Simulation │   │
│   │  ├─ State-Aware Gating (Bypasses Inrush)       │   │   │   │  ├─ Conveyor Mesh │   │
│   │  ├─ ONNX Runtime Inference (LSTM Anomaly)      │   │   │   │  ├─ Motor Actuator│   │
│   │  └─ Closed-Loop Safety (PackML ABORT Trigger)  │   │   │   │  └─ Andon Tower   │   │
│   └────────────────────────────────────────────────┘   │   │   └───────────────────┘   │
└────────────────────────────────────────────────────────┘   └───────────────────────────┘
```

---

## 🚀 Key Innovations & Highlights

### 1. Dual-Domain Real-Time Decoupling (C++20 PREEMPT_RT)
- **Hard RT Thread (`SCHED_FIFO`, priority 98):** Employs `mlockall` and `clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME)` for jitter-free 100Hz (10ms) control cycles. Completely eliminates priority inversion by avoiding mutexes, dynamic allocations, and blocking syscalls.
- **Lock-Free SPSC Ring Buffer:** 64-byte cache-line aligned atomic indices (`head_`, `tail_`) with explicit acquire-release semantics for zero contention between producer and consumer.

### 2. State-Aware Anomaly Gating
- Industrial motors produce transient inrush current during state changes (`STARTING`, `RESETTING`).
- Synapto's AI Core gates anomaly evaluation to **strictly active `EXECUTE (6)` states**, preventing false-positive emergency shutdowns while ensuring immediate response ($\le 50\text{ms}$) to mechanical stalls and bearing wear.

### 3. ISA-TR88.00.02 PackML Conformance
- Full separation of **Waiting States** (`STOPPED`, `IDLE`, `EXECUTE`, `HELD`, `ABORTED`) and **Acting States** (`STARTING`, `STOPPING`, `HOLDING`, `ABORTING`, `RESETTING`, `CLEARING`).
- Emergency `ABORT` command immediately transitions the system into `ABORTING` (cutting actuator velocity) before settling in `ABORTED`.

### 4. High-Fidelity MuJoCo Digital Twin
- Real-time physics replica receiving monotonic, timestamped UDP telemetry.
- Rejects stale/out-of-order packets and dynamically synchronizes actuator velocity and Andon tower RGBA illumination (Green = `EXECUTE`, Red = `ABORTED`, Amber = `HELD`, Blue = `TRANSITIONING`, White = `IDLE`).
- Built-in `--headless` support for continuous integration (CI) environments.

---

## 📊 Protocol Specifications & Interface Versioning

### OPC UA Information Model (Port 4840)
- **Namespace URI:** `urn:synapto:control:v1` (dynamically discoverable via client namespace table)

| Node ID | Browse Name | Data Type | Access | Description |
| :--- | :--- | :--- | :--- | :--- |
| `ns=1;s=PackML.StateCurrent` | `StateCurrent` | `Int32` | Read | Current PackML state integer |
| `ns=1;s=Telemetry.MotorCurrent` | `MotorCurrent` | `Float` | Read | Real-time motor current draw (Amperes) |
| `ns=1;s=PackML.Command` | `Command` | `Int32` | Read/Write | External state command trigger (e.g., 8=ABORT, 2=START) |

### UDP Telemetry Stream Protocol (Port 4242)
The Reflex engine broadcasts versioned, monotonically sequenced telemetry JSON frames at 100Hz:
```json
{
  "v": 1,
  "seq": 10492,
  "ts_us": 1725055200100234,
  "state": 6,
  "current": 12.48,
  "target_vel": 1.5
}
```

### PackML State Matrix

| State ID | State Name | Type | Allowed Transition Commands |
| :---: | :--- | :---: | :--- |
| **1** | `CLEARING` | Acting | *Auto-advances to STOPPED* |
| **2** | `STOPPED` | Waiting | `RESET (1)` $\to$ `RESETTING` |
| **3** | `STARTING` | Acting | *Auto-advances to EXECUTE* |
| **4** | `IDLE` | Waiting | `START (2)` $\to$ `STARTING`, `STOP (3)` $\to$ `STOPPING` |
| **6** | `EXECUTE` | Waiting | `STOP (3)` $\to$ `STOPPING`, `HOLD (4)` $\to$ `HOLDING`, `ABORT (8)` $\to$ `ABORTING` |
| **7** | `STOPPING` | Acting | *Auto-advances to STOPPED* |
| **8** | `ABORTING` | Acting | *Auto-advances to ABORTED* |
| **9** | `ABORTED` | Waiting | `CLEAR (9)` $\to$ `CLEARING` |
| **10** | `HOLDING` | Acting | *Auto-advances to HELD* |
| **11** | `HELD` | Waiting | `UNHOLD (5)` $\to$ `UNHOLDING`, `STOP (3)` $\to$ `STOPPING` |
| **15** | `RESETTING` | Acting | *Auto-advances to IDLE* |

---

## 📂 Repository Structure

```
Synapto-Framework/
├── .github/
│   └── workflows/
│       └── ci.yml                     # GitHub Actions CI (C++20 GCC, PyTest, Docker build)
├── docker/
│   ├── Dockerfile.reflex              # C++20 open62541 PREEMPT_RT container
│   ├── Dockerfile.core                # Python AI Core (ONNX Runtime, asyncua)
│   └── Dockerfile.twin                # MuJoCo 3D physics simulator
├── docker-compose.yml                 # Multi-container edge orchestration
├── src/
│   ├── reflex_engine/                 # Hard Real-Time Deterministic & OPC UA Layer
│   │   ├── include/
│   │   │   ├── packml_sm.hpp          # ISA-TR88.00.02 State Machine
│   │   │   └── lockfree_queue.hpp     # 64-byte cache-aligned SPSC ring buffer
│   │   ├── src/
│   │   │   └── main.cpp               # Dual-thread RT engine + open62541 Server
│   │   └── CMakeLists.txt             # Reflex engine target build
│   ├── ai_core/                       # Predictive Edge AI Inference & Control Daemon
│   │   ├── config.py                  # Pydantic Settings environment configuration
│   │   ├── export_model.py            # PyTorch MotorLSTM & ONNX exporter
│   │   ├── onnx_inference.py          # State-gated ONNX Runtime anomaly scoring
│   │   └── main.py                    # Async OPC UA closed-loop control client
│   └── twin/                          # MuJoCo Physics Digital Twin
│       ├── model/
│       │   └── machine.xml            # Industrial conveyor & Andon tower XML
│       └── scripts/
│           └── twin_viewer.py         # Sequenced UDP receiver & MuJoCo viewer
├── tests/
│   ├── test_reflex_engine/
│   │   ├── CMakeLists.txt
│   │   └── test_packml.cpp            # GTest: SPSC 10^6 msgs throughput & PackML SM
│   └── test_ai_core/
│       └── test_inference.py          # PyTest: PyTorch-ONNX parity & state-gated anomaly
├── CMakeLists.txt                     # Top-level CMake configuration (FetchContent)
├── requirements.txt                   # Python dependencies
├── pyproject.toml                     # Python packaging & test configuration
├── LICENSE                            # MIT License
└── README.md                          # Documentation
```

---

## 🛠 Quickstart & Build Instructions

### Prerequisites
- **C++ Compiler:** GCC 11+ / Clang 14+ supporting C++20
- **CMake:** Version 3.20+
- **Python:** 3.10+
- **Docker & Docker Compose** (Optional for container deployment)

---

### 1. Build and Run Native C++ Reflex Engine & Tests

```bash
# Configure CMake (automatically fetches open62541 & GoogleTest)
cmake -B build -DCMAKE_BUILD_TYPE=Release

# Compile all targets
cmake --build build -j$(nproc)

# Run C++ Google Test suite
ctest --test-dir build --output-on-failure

# Launch the Reflex Engine
./build/src/reflex_engine/synapto_reflex
```

---

### 2. Run Python AI Core & PyTest Suite

```bash
# Create and activate virtual environment
python3 -m venv .venv
source .venv/bin/activate

# Install dependencies
pip install -r requirements.txt

# Run Python verification tests (Parity, State-gating, Anomaly triggers)
pytest -v tests/test_ai_core

# Export ONNX model binary
python src/ai_core/export_model.py

# Launch the AI Core Closed-Loop Control Daemon
python src/ai_core/main.py
```

---

### 3. Launch MuJoCo Digital Twin

```bash
# Interactive 3D Viewer Mode
python src/twin/scripts/twin_viewer.py

# Headless Validation Mode (for CI / Servers)
python src/twin/scripts/twin_viewer.py --headless --duration 5.0
```

---

### 4. Full-Stack Edge Deployment with Docker Compose

Deploy the complete multi-container network with container health checking and real-time scheduling permissions:

```bash
docker compose up --build
```

---

## 🧪 Verification & Acceptance Suite

| Component | Test File | Scope & Acceptance Criteria |
| :--- | :--- | :--- |
| **SPSC Queue** | `tests/test_reflex_engine/test_packml.cpp` | Multi-threaded producer-consumer throughput ($10^6$ messages, 0 drops, 0 corruption). |
| **PackML State Machine** | `tests/test_reflex_engine/test_packml.cpp` | Validates standard ISA-TR88.00.02 state chains and rejects direct illegal transitions. |
| **PyTorch/ONNX Parity** | `tests/test_ai_core/test_inference.py` | Maximum numerical error between eager PyTorch and ONNX Runtime: $\max \|y_{\text{torch}} - y_{\text{onnx}}\| < 10^{-5}$. |
| **State-Gated Safety** | `tests/test_ai_core/test_inference.py` | Validates that inrush spikes in `STARTING` are ignored, while spikes in `EXECUTE` trigger `ABORT`. |
| **CI Automation** | `.github/workflows/ci.yml` | Automated GitHub Actions workflow compiling C++20, executing PyTest, and building Docker images. |

---

## 👤 Author

**Daniyal Hasan**
- **GitHub:** [@engdaniyalh](https://github.com/engdaniyalh)
- **LinkedIn:** [daniyalhasan](https://linkedin.com/in/daniyalhasan)

---

## 📄 License

This project is licensed under the [MIT License](LICENSE).
