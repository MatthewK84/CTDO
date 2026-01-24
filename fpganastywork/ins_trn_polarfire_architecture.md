# INS + TRN Navigation Computer Architecture

## GPS-Denied Navigation System for PolarFire SoC

| Parameter | Value |
|-----------|-------|
| **Target Platform** | Microchip PolarFire SoC (MPFS250T/MPFS095T) |
| **Flight Controller** | CubeOrange+ (PX4/ArduPilot) |
| **Companion Computer** | Raspberry Pi 5 (TRN correlation) |
| **Design Language** | SystemVerilog / Verilog (FPGA), C (E51), C++ (U54) |
| **FPGA Clock** | 100 MHz system, 150 MHz DSP |
| **Interfaces** | MAVLink (to FC), SPI/I2C (sensors), USB/SPI (to Pi 5) |

---

## Table of Contents

1. [System Overview](#system-overview)
2. [Platform Architecture](#platform-architecture)
3. [Processing Partitioning](#processing-partitioning)
4. [Top-Level Block Diagram](#top-level-block-diagram)
5. [Module Hierarchy](#module-hierarchy)
6. [Clock and Reset Architecture](#clock-and-reset-architecture)
7. [RISC-V Core Assignments](#risc-v-core-assignments)
8. [FPGA Fabric Modules](#fpga-fabric-modules)
9. [MAVLink Integration](#mavlink-integration)
10. [Memory Architecture](#memory-architecture)
11. [Data Flow](#data-flow)
12. [Register Maps](#register-maps)
13. [Timing Budgets](#timing-budgets)
14. [Resource Estimates](#resource-estimates)
15. [Power Analysis](#power-analysis)
16. [Build and Simulation](#build-and-simulation)

---

## System Overview

This architecture implements a GPS-denied navigation system using terrain-relative navigation (TRN) fused with inertial navigation (INS). The system is partitioned across three compute elements:

| Component | Role | Why |
|-----------|------|-----|
| **CubeOrange+** | Flight control, attitude estimation, motor mixing | Proven, certifiable, handles safety-critical control loops |
| **PolarFire SoC** | INS propagation, covariance, sensor fusion, Kalman updates | Deterministic real-time processing, low power, SEU-immune |
| **Raspberry Pi 5** | TRN correlation, DTED management, path planning | High-level compute for terrain matching algorithms |

### Design Philosophy

- **Separation of concerns**: Flight-critical control on certified hardware (CubeOrange).
- **Deterministic navigation**: Hard real-time INS on PolarFire E51 core.
- **Flash-based reliability**: No FPGA configuration upsets at altitude.
- **Low power**: ~3W total for navigation computer (vs 8-12W for Zynq UltraScale+).
- **MAVLink native**: Standard interface to flight controller ecosystem.

### Why PolarFire SoC?

| Advantage | Benefit for GPS-Denied Nav |
|-----------|---------------------------|
| 50% lower static power | Extended flight time on Group 1 UAS |
| Flash-based FPGA | No SEU-induced bitstream corruption |
| RISC-V determinism | Predictable INS timing without ARM complexity |
| Asymmetric multiprocessing | E51 bare-metal + U54 Linux simultaneously |
| Defense-grade security | Secure boot, tamper detection for sensitive missions |

---

## Platform Architecture

### PolarFire SoC Overview

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                        PolarFire SoC MPFS250T                               │
│                                                                             │
│  ┌─────────────────────────────────────────────────────────────────────┐   │
│  │                  Microprocessor Subsystem (MSS)                      │   │
│  │                                                                      │   │
│  │   ┌─────────┐  ┌─────────┐  ┌─────────┐  ┌─────────┐  ┌─────────┐  │   │
│  │   │   E51   │  │   U54   │  │   U54   │  │   U54   │  │   U54   │  │   │
│  │   │ Monitor │  │  Core 1 │  │  Core 2 │  │  Core 3 │  │  Core 4 │  │   │
│  │   │ (RT)    │  │ (Linux) │  │ (Linux) │  │ (Linux) │  │ (Linux) │  │   │
│  │   │ 600MHz  │  │ 600MHz  │  │ 600MHz  │  │ 600MHz  │  │ 600MHz  │  │   │
│  │   └────┬────┘  └────┬────┘  └────┬────┘  └────┬────┘  └────┬────┘  │   │
│  │        │            │            │            │            │       │   │
│  │        └────────────┴─────┬──────┴────────────┴────────────┘       │   │
│  │                           │                                         │   │
│  │                    ┌──────┴──────┐                                  │   │
│  │                    │  L2 Cache   │  2MB (configurable cache/SRAM)   │   │
│  │                    │  /Scratchpad│                                  │   │
│  │                    └──────┬──────┘                                  │   │
│  │                           │                                         │   │
│  │   ┌───────────────────────┴───────────────────────────────────┐    │   │
│  │   │              AXI Interconnect Matrix                       │    │   │
│  │   └─────┬─────────┬─────────┬─────────┬─────────┬─────────────┘    │   │
│  │         │         │         │         │         │                   │   │
│  │    ┌────┴────┐ ┌──┴──┐  ┌───┴───┐ ┌───┴───┐ ┌───┴───┐             │   │
│  │    │  DDR4   │ │QSPI │  │ USB   │ │ GigE  │ │ CAN   │             │   │
│  │    │ Ctrl    │ │Flash│  │ 2.0   │ │ MAC   │ │ 2.0   │             │   │
│  │    └─────────┘ └─────┘  └───────┘ └───────┘ └───────┘             │   │
│  │                                                                     │   │
│  └─────────────────────────────────┬───────────────────────────────────┘   │
│                                    │                                        │
│                              AHB/APB Bridge                                 │
│                                    │                                        │
│  ┌─────────────────────────────────┴───────────────────────────────────┐   │
│  │                      FPGA Fabric (254K LEs)                          │   │
│  │                                                                      │   │
│  │   ┌─────────────┐  ┌─────────────┐  ┌─────────────┐                 │   │
│  │   │   Sensor    │  │    INS      │  │  Kalman     │                 │   │
│  │   │  Interface  │  │ Accel/Cov   │  │  Update     │                 │   │
│  │   │   Block     │  │  Engine     │  │  Engine     │                 │   │
│  │   └─────────────┘  └─────────────┘  └─────────────┘                 │   │
│  │                                                                      │   │
│  │   ┌─────────────┐  ┌─────────────┐  ┌─────────────┐                 │   │
│  │   │  Altitude   │  │   State     │  │   Math      │                 │   │
│  │   │  Profile    │  │   Memory    │  │   Units     │                 │   │
│  │   │   Buffer    │  │   (LSRAM)   │  │  (DSP/FPU)  │                 │   │
│  │   └─────────────┘  └─────────────┘  └─────────────┘                 │   │
│  │                                                                      │   │
│  └──────────────────────────────────────────────────────────────────────┘   │
│                                                                             │
└─────────────────────────────────────────────────────────────────────────────┘
```

### Device Selection

| Variant | Logic Elements | LSRAM | Math Blocks | Use Case |
|---------|---------------|-------|-------------|----------|
| MPFS095T | 95K | 480 KB | 336 | Discovery Kit, prototyping |
| MPFS250T | 254K | 1.5 MB | 784 | Icicle Kit, production |
| MPFS460T | 461K | 2.5 MB | 1520 | Maximum headroom |

**Recommended**: MPFS250T (Icicle Kit) for development, MPFS095T for weight-constrained deployment.

---

## Processing Partitioning

### Core Assignment Strategy

```
┌─────────────────────────────────────────────────────────────────────────┐
│                    Processing Partitioning                               │
├─────────────────────────────────────────────────────────────────────────┤
│                                                                         │
│   E51 Monitor Core (Bare-Metal, Real-Time)                              │
│   ┌─────────────────────────────────────────────────────────────────┐  │
│   │  • INS state propagation (400 Hz)                                │  │
│   │  • Quaternion integration                                        │  │
│   │  • Covariance propagation trigger                                │  │
│   │  • Kalman update coordination                                    │  │
│   │  • Watchdog management                                           │  │
│   │  • Direct FPGA fabric interface                                  │  │
│   │                                                                  │  │
│   │  Timing: Deterministic, < 100 µs per IMU sample                 │  │
│   │  Memory: L2 scratchpad (no cache), TCM                          │  │
│   └─────────────────────────────────────────────────────────────────┘  │
│                                                                         │
│   U54 Core 1 (Linux, MAVLink Handler)                                   │
│   ┌─────────────────────────────────────────────────────────────────┐  │
│   │  • MAVLink parsing/generation                                    │  │
│   │  • VISION_POSITION_ESTIMATE output to CubeOrange                │  │
│   │  • ATTITUDE subscription from CubeOrange                        │  │
│   │  • UART/USB driver for FC communication                         │  │
│   │  • System logging                                                │  │
│   └─────────────────────────────────────────────────────────────────┘  │
│                                                                         │
│   U54 Core 2 (Linux, Pi 5 Interface)                                    │
│   ┌─────────────────────────────────────────────────────────────────┐  │
│   │  • SPI/USB communication with Raspberry Pi 5                    │  │
│   │  • Altitude profile packaging                                    │  │
│   │  • TRN fix reception and validation                             │  │
│   │  • DTED cache management                                        │  │
│   └─────────────────────────────────────────────────────────────────┘  │
│                                                                         │
│   U54 Cores 3-4 (Linux, Available)                                      │
│   ┌─────────────────────────────────────────────────────────────────┐  │
│   │  • Future: On-board SLAM                                        │  │
│   │  • Future: Visual odometry preprocessing                        │  │
│   │  • Future: AI/ML inference (VectorBlox acceleration)            │  │
│   │  • Development: Debug, shell, diagnostics                       │  │
│   └─────────────────────────────────────────────────────────────────┘  │
│                                                                         │
│   FPGA Fabric (Hardware Acceleration)                                   │
│   ┌─────────────────────────────────────────────────────────────────┐  │
│   │  • SPI master for IMU (ADIS16500)                               │  │
│   │  • SPI/I2C master for barometer (BMP390)                        │  │
│   │  • UART for LiDAR (TF-Luna)                                     │  │
│   │  • Matrix multiply engine (covariance)                          │  │
│   │  • 2×2 matrix inverse (Kalman gain)                             │  │
│   │  • Floating-point pipelines (FPU acceleration)                  │  │
│   │  • State memory (LSRAM)                                         │  │
│   │  • Altitude profile FIFO                                        │  │
│   └─────────────────────────────────────────────────────────────────┘  │
│                                                                         │
└─────────────────────────────────────────────────────────────────────────┘
```

### AMP (Asymmetric Multiprocessing) Configuration

```c
// Memory map for AMP configuration
// E51 uses L2 as scratchpad, U54s use as cache

#define E51_SCRATCHPAD_BASE    0x08000000  // 512KB L2 scratchpad for E51
#define E51_SCRATCHPAD_SIZE    0x00080000

#define SHARED_MEMORY_BASE     0x08080000  // 256KB shared between E51 and U54
#define SHARED_MEMORY_SIZE     0x00040000

#define U54_CACHE_BASE         0x080C0000  // Remaining L2 as cache for U54s
```

---

## Top-Level Block Diagram

```
┌─────────────────────────────────────────────────────────────────────────────────────────────┐
│                                    SYSTEM ARCHITECTURE                                      │
│                                                                                             │
│  ┌─────────────────────────────────────────────────────────────────────────────────────┐    │
│  │                              CubeOrange+ Flight Controller                           │   │
│  │                                                                                      │   │
│  │   ┌─────────────┐    ┌─────────────┐    ┌─────────────┐    ┌─────────────┐           │   │
│  │   │  PX4/AP     │    │   EKF2/3    │    │   Mixer     │    │   Motors    │           │   │
│  │   │  Firmware   │◄──►│   Filter    │───►│   Outputs   │───►│   ESCs      │           │   │
│  │   └──────┬──────┘    └──────┬──────┘    └─────────────┘    └─────────────┘           │   │
│  │          │                  │                                                        │   │
│  │          │ MAVLink          │ VISION_POSITION_ESTIMATE                               │   │
│  │          │ UART             │ (from PolarFire)                                       │   │
│  └──────────┼──────────────────┼───────────────────────────────────────────────────────┘    │
│             │                  │                                                            │
│             │ 921600 baud      │                                                            │
│             │ TELEM2 port      │                                                            │
│             ▼                  ▼                                                            │
│  ┌─────────────────────────────────────────────────────────────────────────────────────┐    │
│  │                              PolarFire SoC Navigation Computer                     │   │
│  │                                                                                    │   │
│  │   ┌───────────────────────────────────────────────────────────────────────────┐    │   │
│  │   │                        MSS (RISC-V Cores)                                 │    │   │
│  │   │                                                                           │    │   │
│  │   │   ┌─────────────┐    ┌─────────────┐    ┌─────────────┐                  │     │   │
│  │   │   │    E51      │    │  U54 Core 1 │    │  U54 Core 2 │                  │     │   │
│  │   │   │ (Bare-Metal)│    │  (Linux)    │    │  (Linux)    │                  │     │   │
│  │   │   │             │    │             │    │             │                  │     │   │
│  │   │   │ • INS prop  │    │ • MAVLink   │    │ • Pi 5 comm │                  │     │   │
│  │   │   │ • Quat int  │    │ • FC UART   │    │ • TRN fixes │                  │     │   │
│  │   │   │ • Cov trig  │    │ • Logging   │    │ • Profiles  │                  │     │   │
│  │   │   │             │    │             │    │             │                  │     │   │
│  │   │   └──────┬──────┘    └──────┬──────┘    └──────┬──────┘                  │     │   │
│  │   │          │                  │                  │                         │     │   │
│  │   │          │                  │                  │                         │     │   │
│  │   │          ▼                  ▼                  ▼                         │     │   │
│  │   │   ┌─────────────────────────────────────────────────────────────────┐   │     │   │
│  │   │   │              Shared Memory (L2 Scratchpad)                      │   │     │   │
│  │   │   │                                                                 │   │     │   │
│  │   │   │   ┌─────────────┐  ┌─────────────┐  ┌─────────────┐             │   │     │   │
│  │   │   │   │  INS State  │  │  TRN Fix    │  │  Altitude   │             │   │     │   │
│  │   │   │   │  (current)  │  │  (latest)   │  │  Profile    │             │   │     │   │
│  │   │   │   └─────────────┘  └─────────────┘  └─────────────┘             │   │     │   │
│  │   │   │                                                                 │   │     │   │
│  │   │   └─────────────────────────────────────────────────────────────────┘   │     │   │
│  │   │                                                                           │     │   │
│  │   └───────────────────────────────────────────────────────────────────────────┘     │   │
│  │                                          │                                          │   │
│  │                                    APB/AHB Bus                                      │   │
│  │                                          │                                          │   │
│  │   ┌───────────────────────────────────────────────────────────────────────────┐     │   │
│  │   │                         FPGA Fabric                                       │     │   │
│  │   │                                                                           │     │   │
│  │   │   ┌─────────────┐    ┌─────────────┐    ┌─────────────┐                   │     │   │
│  │   │   │    SPI      │    │    IMU      │    │    INS      │                   │     │   │
│  │   │   │   Master    │───►│  Interface  │───►│   Accel     │                   │     │   │
│  │   │   │  (2 MHz)    │    │ (calibrate) │    │  Engine     │                   │     │   │
│  │   │   └─────────────┘    └─────────────┘    └──────┬──────┘                   │     │   │
│  │   │                                                │                          │     │   │
│  │   │   ┌─────────────┐    ┌─────────────┐          ▼                           │     │   │
│  │   │   │    SPI      │    │    Baro     │    ┌─────────────┐                   │     │   │
│  │   │   │   Master    │───►│  Interface  │    │ Covariance  │                   │     │   │
│  │   │   │  (Baro)     │    │             │    │ Propagator  │                   │     │   │
│  │   │   └─────────────┘    └─────────────┘    └──────┬──────┘                   │     │   │
│  │   │                                                │                          │     │   │
│  │   │   ┌─────────────┐    ┌─────────────┐          ▼                           │     │   │
│  │   │   │   UART      │    │   LiDAR     │    ┌─────────────┐                   │     │   │
│  │   │   │    RX       │───►│  Interface  │───►│   Kalman    │                   │     │   │
│  │   │   │ (115200)    │    │             │    │   Update    │                   │     │   │
│  │   │   └─────────────┘    └─────────────┘    └──────┬──────┘                   │     │   │
│  │   │                                                │                          │     │   │
│  │   │                                                ▼                          │     │   │
│  │   │                                         ┌─────────────┐                   │     │   │
│  │   │                                         │    State    │                   │     │   │
│  │   │                                         │   Memory    │                   │     │   │
│  │   │                                         │   (LSRAM)   │                   │     │   │
│  │   │                                         └─────────────┘                   │     │   │
│  │   │                                                                           │     │   │
│  │   └───────────────────────────────────────────────────────────────────────────┘     │   │
│  │                                                                                      │   │
│  └──────────────────────────────────────────────────────────────────────────────────────┘   │
│                  │                                              │                           │
│                  │ SPI (40 MHz)                                 │                           │
│                  │                                              │                           │
│                  ▼                                              ▼                           │
│  ┌─────────────────────────────────┐           ┌─────────────────────────────────┐         │
│  │        Raspberry Pi 5           │           │           Sensors               │         │
│  │                                 │           │                                 │         │
│  │  ┌─────────────────────────┐   │           │  ┌─────────┐  ┌─────────┐       │         │
│  │  │     TRN Correlator      │   │           │  │ADIS16500│  │ BMP390  │       │         │
│  │  │                         │   │           │  │  IMU    │  │  Baro   │       │         │
│  │  │  • Profile matching     │   │           │  └─────────┘  └─────────┘       │         │
│  │  │  • DTED lookup          │   │           │                                 │         │
│  │  │  • Position fix calc    │   │           │  ┌─────────┐                    │         │
│  │  │  • Covariance estimate  │   │           │  │ TF-Luna │                    │         │
│  │  │                         │   │           │  │  LiDAR  │                    │         │
│  │  └─────────────────────────┘   │           │  └─────────┘                    │         │
│  │                                 │           │                                 │         │
│  │  ┌─────────────────────────┐   │           └─────────────────────────────────┘         │
│  │  │     DTED Storage        │   │                                                       │
│  │  │     (SD Card)           │   │                                                       │
│  │  └─────────────────────────┘   │                                                       │
│  │                                 │                                                       │
│  └─────────────────────────────────┘                                                       │
│                                                                                             │
└─────────────────────────────────────────────────────────────────────────────────────────────┘
```

### External Interfaces

```
                    Physical Connections
    
    ┌─────────────┐
    │ CubeOrange+ │
    │   TELEM2    │◄────────── UART (921600, 3.3V) ──────────┐
    └─────────────┘                                          │
                                                             │
    ┌─────────────┐                                          │
    │ ADIS16500   │◄────────── SPI (2 MHz, Mode 3) ──────────┤
    │    IMU      │            PMOD JA                       │
    └─────────────┘                                          │
                                                             │
    ┌─────────────┐                                          │
    │  BMP390     │◄────────── SPI (1 MHz, Mode 0) ──────────┤
    │   Baro      │            PMOD JB                       │
    └─────────────┘                                          │
                                                             │
    ┌─────────────┐                                   ┌──────┴──────┐
    │  TF-Luna    │◄────────── UART (115200) ────────│  PolarFire  │
    │   LiDAR     │            PMOD JC               │    SoC      │
    └─────────────┘                                  │   Icicle    │
                                                     │    Kit      │
    ┌─────────────┐                                  └──────┬──────┘
    │    Pi 5     │◄────────── SPI (40 MHz) ─────────────────┤
    │             │            or USB                        │
    └─────────────┘                                          │
                                                             │
    ┌─────────────┐                                          │
    │   Debug     │◄────────── USB-UART ─────────────────────┘
    │  Console    │            (HSS/Linux)
    └─────────────┘
```

---

## Module Hierarchy

```
ins_trn_polarfire/
│
├── mss_config/                          # PolarFire MSS configuration
│   ├── mss_sw_config.xml               # MSS Configurator export
│   └── soc_config/                     # Generated MSS configuration
│
├── e51_firmware/                        # E51 bare-metal firmware
│   ├── main.c                          # Entry point, initialization
│   ├── ins_propagation.c              # INS state propagation
│   ├── ins_propagation.h
│   ├── quaternion.c                   # Quaternion math
│   ├── quaternion.h
│   ├── kalman_interface.c             # Interface to FPGA Kalman engine
│   ├── kalman_interface.h
│   ├── shared_memory.c                # E51-U54 shared memory interface
│   ├── shared_memory.h
│   ├── fpga_regs.h                    # FPGA register definitions
│   └── startup.S                      # E51 startup code
│
├── u54_linux/                           # U54 Linux applications
│   ├── mavlink_handler/
│   │   ├── mavlink_handler.cpp        # MAVLink processing
│   │   ├── fc_interface.cpp           # CubeOrange communication
│   │   └── CMakeLists.txt
│   │
│   ├── pi5_interface/
│   │   ├── pi5_comm.cpp               # Pi 5 SPI/USB interface
│   │   ├── trn_manager.cpp            # TRN fix handling
│   │   └── CMakeLists.txt
│   │
│   └── system_monitor/
│       ├── monitor.cpp                # Health monitoring
│       └── CMakeLists.txt
│
├── fpga_fabric/                         # FPGA RTL
│   │
│   ├── top/
│   │   └── ins_trn_fabric_top.sv      # Top-level fabric module
│   │
│   ├── sensor_subsystem/
│   │   ├── spi_master_imu.sv          # ADIS16500 interface
│   │   ├── spi_phy.sv                 # SPI physical layer
│   │   ├── burst_controller.sv        # IMU burst read FSM
│   │   ├── spi_master_baro.sv         # BMP390 interface
│   │   ├── baro_compensation.sv       # Temperature compensation
│   │   ├── uart_rx_lidar.sv           # TF-Luna UART receiver
│   │   ├── frame_parser.sv            # LiDAR frame extraction
│   │   └── imu_calibration.sv         # Bias/scale factor application
│   │
│   ├── ins_engine/
│   │   ├── ins_acceleration.sv        # Specific force to nav frame
│   │   ├── dcm_from_quat.sv           # Quaternion to DCM conversion
│   │   ├── gravity_model.sv           # WGS84 gravity computation
│   │   └── matrix_vector_mult_3x3.sv  # 3×3 matrix-vector multiply
│   │
│   ├── covariance/
│   │   ├── phi_matrix_gen.sv          # State transition matrix
│   │   ├── q_matrix_gen.sv            # Process noise matrix
│   │   ├── covariance_propagator.sv   # P = ΦPΦ' + Q
│   │   ├── matrix_multiply_15x15.sv   # Systolic array multiplier
│   │   └── matrix_transpose.sv        # In-place transpose
│   │
│   ├── kalman/
│   │   ├── kalman_update_engine.sv    # Complete Kalman update
│   │   ├── innovation_calc.sv         # z - Hx computation
│   │   ├── kalman_gain_calc.sv        # K = PH'(HPH'+R)^-1
│   │   ├── matrix_inverse_2x2.sv      # Direct 2×2 inverse
│   │   ├── state_update.sv            # x = x + K(z - Hx)
│   │   └── covariance_update.sv       # P = (I - KH)P
│   │
│   ├── altitude_profile/
│   │   ├── altitude_profile_mgr.sv    # Profile collection manager
│   │   ├── profile_fifo.sv            # Circular sample buffer
│   │   ├── timestamp_correlator.sv    # Sample-position matching
│   │   └── dma_engine.sv              # AXI-Stream DMA
│   │
│   ├── memory/
│   │   ├── ins_state_ram.sv           # Position/velocity/attitude
│   │   ├── error_state_ram.sv         # 15-element error state
│   │   ├── p_matrix_lsram.sv          # Covariance matrix (symmetric)
│   │   └── calibration_rom.sv         # IMU calibration parameters
│   │
│   ├── math/
│   │   ├── fp_multiply.sv             # IEEE 754 multiply
│   │   ├── fp_add.sv                  # IEEE 754 add/subtract
│   │   ├── fp_sqrt.sv                 # Square root (Newton-Raphson)
│   │   ├── fp_divide.sv               # Division
│   │   ├── fp_sincos.sv               # CORDIC sin/cos
│   │   └── trig_lut.sv                # Lookup table for trig
│   │
│   ├── interface/
│   │   ├── apb_reg_bank.sv            # APB slave register interface
│   │   ├── ahb_fabric_if.sv           # AHB-Lite fabric interface
│   │   └── interrupt_gen.sv           # Interrupt controller
│   │
│   └── common/
│       ├── sync_fifo.sv               # Synchronous FIFO
│       ├── cdc_sync.sv                # Clock domain crossing
│       └── edge_detect.sv             # Edge detection
│
├── constraints/
│   ├── icicle_kit_pinout.pdc          # Pin assignments
│   ├── timing.sdc                     # Timing constraints
│   └── io_constraints.sdc             # I/O timing
│
├── tb/
│   ├── tb_ins_trn_fabric_top.sv       # Top-level testbench
│   ├── tb_ins_engine.sv               # INS engine tests
│   ├── tb_kalman_update.sv            # Kalman update tests
│   ├── imu_bfm.sv                     # IMU bus functional model
│   └── reference_model/
│       └── ins_reference.py           # Python golden model
│
├── scripts/
│   ├── build_fabric.tcl               # Libero build script
│   ├── build_mss.tcl                  # MSS configuration script
│   ├── program.tcl                    # Programming script
│   └── sim.tcl                        # Simulation script
│
└── docs/
    ├── architecture.md                # This document
    ├── register_map.md                # Detailed register documentation
    └── integration_guide.md           # System integration guide
```

---

## Clock and Reset Architecture

### Clock Generation

```
                    ┌────────────────────────────────────────────────────┐
                    │              Clock Architecture                    │
                    │                                                    │
    125 MHz ───────►│   ┌─────────────────────────────────────────────┐ │
    (oscillator)    │   │            MSS PLL                          │ │
                    │   │                                             │ │
                    │   │   ┌─────────────────────────────────────┐   │ │
                    │   │   │  MSS Reference: 125 MHz             │   │ │
                    │   │   │                                     │   │ │
                    │   │   │  CPU Clock:     600 MHz             │   │ │
                    │   │   │  AXI Clock:     300 MHz             │   │ │
                    │   │   │  AHB Clock:     150 MHz             │   │ │
                    │   │   │  APB Clock:      75 MHz             │   │ │
                    │   │   └─────────────────────────────────────┘   │ │
                    │   │                                             │ │
                    │   └─────────────────────────────────────────────┘ │
                    │                                                    │
                    │   ┌─────────────────────────────────────────────┐ │
                    │   │           Fabric CCC (Clock Conditioning)   │ │
                    │   │                                             │ │
                    │   │   Input:   125 MHz reference                │ │
                    │   │                                             │ │
                    │   │   ┌─────────────────────────────────────┐   │ │
                    │   │   │  clk_sys:     100 MHz (system)      │   │ │
                    │   │   │  clk_dsp:     150 MHz (math units)  │   │ │
                    │   │   │  clk_spi:      50 MHz (SPI master)  │   │ │
                    │   │   └─────────────────────────────────────┘   │ │
                    │   │                                             │ │
                    │   └─────────────────────────────────────────────┘ │
                    │                                                    │
                    └────────────────────────────────────────────────────┘

    Clock Domain Assignment:
    
    ┌────────────────────────┬──────────────────────────────────────────┐
    │  Clock Domain          │  Modules                                 │
    ├────────────────────────┼──────────────────────────────────────────┤
    │  clk_sys (100 MHz)     │  APB registers, state machines,         │
    │                        │  sensor interfaces, control logic        │
    ├────────────────────────┼──────────────────────────────────────────┤
    │  clk_dsp (150 MHz)     │  Matrix multiply, covariance prop,      │
    │                        │  Kalman update, FPU pipelines            │
    ├────────────────────────┼──────────────────────────────────────────┤
    │  clk_spi (50 MHz)      │  SPI PHY (divided for 2 MHz SCLK)       │
    └────────────────────────┴──────────────────────────────────────────┘
```

### Reset Sequencing

```
    Power-On Reset Sequence
    
    VDD Stable
        │
        ▼
    ┌─────────────────┐
    │  MSS Reset      │  (hardware reset, POR)
    │  Release        │
    └────────┬────────┘
             │
             ▼
    ┌─────────────────┐
    │  HSS Boot       │  (Hart Software Services)
    │  (E51 only)     │  - Initialize DDR4
    │                 │  - Configure MSS
    │                 │  - Load U-Boot or Linux
    └────────┬────────┘
             │
             ▼
    ┌─────────────────┐
    │  Fabric CCC     │  (wait for lock)
    │  Lock           │
    └────────┬────────┘
             │
             ▼
    ┌─────────────────┐
    │  Fabric Reset   │  (synchronized release)
    │  Release        │
    └────────┬────────┘
             │
             ├─────────────────────────────┐
             │                             │
             ▼                             ▼
    ┌─────────────────┐          ┌─────────────────┐
    │  FPGA Init      │          │  E51 App Start  │
    │  - Clear LSRAM  │          │  - INS init     │
    │  - Config regs  │          │  - Sensor cfg   │
    │  - Sensor init  │          │  - Wait sensors │
    └────────┬────────┘          └────────┬────────┘
             │                             │
             └──────────┬──────────────────┘
                        │
                        ▼
               ┌─────────────────┐
               │  Operational    │
               │  (400 Hz INS)   │
               └─────────────────┘
```

### Reset Distribution

```systemverilog
// Fabric reset synchronization
module fabric_reset_sync (
    input  logic clk,
    input  logic mss_reset_n,      // From MSS
    input  logic ccc_lock,         // CCC lock status
    output logic fabric_reset_n    // Synchronized fabric reset
);

    logic [3:0] reset_pipe;
    
    always_ff @(posedge clk or negedge mss_reset_n) begin
        if (!mss_reset_n) begin
            reset_pipe <= 4'b0000;
        end else if (ccc_lock) begin
            reset_pipe <= {reset_pipe[2:0], 1'b1};
        end else begin
            reset_pipe <= 4'b0000;
        end
    end
    
    assign fabric_reset_n = reset_pipe[3];

endmodule
```

---

## RISC-V Core Assignments

### E51 Monitor Core (Bare-Metal INS Engine)

The E51 core runs bare-metal code for deterministic INS propagation.

```c
// e51_firmware/main.c

#include "ins_propagation.h"
#include "kalman_interface.h"
#include "shared_memory.h"
#include "fpga_regs.h"

#define IMU_SAMPLE_PERIOD_US    2500    // 400 Hz
#define COVARIANCE_DECIMATION   10      // 40 Hz covariance prop

volatile shared_mem_t *shared = (shared_mem_t *)SHARED_MEMORY_BASE;

// INS state (in E51 scratchpad for fast access)
static ins_state_t ins_state __attribute__((section(".scratchpad")));
static error_state_t error_state __attribute__((section(".scratchpad")));
static uint32_t sample_count = 0;

void e51_main(void) {
    // Initialize INS state
    ins_init(&ins_state);
    error_state_init(&error_state);
    
    // Configure IMU via FPGA registers
    FPGA_REG(IMU_CFG) = IMU_CFG_ENABLE | IMU_CFG_400HZ;
    FPGA_REG(BARO_CFG) = BARO_CFG_ENABLE;
    FPGA_REG(LIDAR_CFG) = LIDAR_CFG_ENABLE;
    
    // Enable IMU data ready interrupt
    FPGA_REG(IRQ_ENABLE) = IRQ_IMU_READY;
    
    // Main loop (interrupt-driven)
    while (1) {
        // Wait for IMU interrupt
        __asm__ volatile ("wfi");
        
        // Check if IMU data ready
        if (FPGA_REG(IRQ_STATUS) & IRQ_IMU_READY) {
            process_imu_sample();
            FPGA_REG(IRQ_STATUS) = IRQ_IMU_READY;  // Clear interrupt
        }
        
        // Check for TRN fix from U54
        if (shared->trn_fix_ready) {
            apply_trn_fix();
            shared->trn_fix_ready = 0;
        }
    }
}

void process_imu_sample(void) {
    imu_data_t imu;
    
    // Read calibrated IMU data from FPGA
    imu.gyro_x = FPGA_REG_F(GYRO_X_CAL);
    imu.gyro_y = FPGA_REG_F(GYRO_Y_CAL);
    imu.gyro_z = FPGA_REG_F(GYRO_Z_CAL);
    imu.accel_x = FPGA_REG_F(ACCEL_X_CAL);
    imu.accel_y = FPGA_REG_F(ACCEL_Y_CAL);
    imu.accel_z = FPGA_REG_F(ACCEL_Z_CAL);
    
    // INS mechanization (attitude, velocity, position)
    ins_propagate(&ins_state, &imu, IMU_SAMPLE_PERIOD_US * 1e-6f);
    
    // Update error state propagation (Φ matrix applied)
    error_state_propagate(&error_state, &ins_state);
    
    // Trigger covariance propagation in FPGA (decimated)
    sample_count++;
    if ((sample_count % COVARIANCE_DECIMATION) == 0) {
        // Write current state to FPGA for covariance update
        fpga_write_phi_matrix(&ins_state);
        FPGA_REG(COV_CTRL) = COV_CTRL_START;
    }
    
    // Update shared memory for U54 access
    shared->ins_state = ins_state;
    shared->ins_timestamp = get_timestamp_us();
    __asm__ volatile ("fence");  // Memory barrier
}

void apply_trn_fix(void) {
    // Read TRN fix from shared memory
    trn_fix_t fix = shared->trn_fix;
    
    // Write TRN fix to FPGA Kalman engine
    FPGA_REG_F(TRN_POS_N) = fix.pos_n;
    FPGA_REG_F(TRN_POS_E) = fix.pos_e;
    FPGA_REG_F(TRN_COV_NN) = fix.cov_nn;
    FPGA_REG_F(TRN_COV_EE) = fix.cov_ee;
    FPGA_REG_F(TRN_COV_NE) = fix.cov_ne;
    
    // Trigger Kalman update in FPGA
    FPGA_REG(KALMAN_CTRL) = KALMAN_CTRL_START;
    
    // Wait for completion
    while (!(FPGA_REG(KALMAN_STATUS) & KALMAN_STATUS_DONE));
    
    // Read corrected state from FPGA
    ins_state.pos_n -= FPGA_REG_F(CORRECTION_POS_N);
    ins_state.pos_e -= FPGA_REG_F(CORRECTION_POS_E);
    ins_state.pos_d -= FPGA_REG_F(CORRECTION_POS_D);
    ins_state.vel_n -= FPGA_REG_F(CORRECTION_VEL_N);
    ins_state.vel_e -= FPGA_REG_F(CORRECTION_VEL_E);
    ins_state.vel_d -= FPGA_REG_F(CORRECTION_VEL_D);
    
    // Apply attitude correction via quaternion
    apply_attitude_correction(&ins_state,
        FPGA_REG_F(CORRECTION_ATT_X),
        FPGA_REG_F(CORRECTION_ATT_Y),
        FPGA_REG_F(CORRECTION_ATT_Z));
    
    // Update bias estimates
    ins_state.bias_ax -= FPGA_REG_F(CORRECTION_BIAS_AX);
    ins_state.bias_ay -= FPGA_REG_F(CORRECTION_BIAS_AY);
    ins_state.bias_az -= FPGA_REG_F(CORRECTION_BIAS_AZ);
    ins_state.bias_gx -= FPGA_REG_F(CORRECTION_BIAS_GX);
    ins_state.bias_gy -= FPGA_REG_F(CORRECTION_BIAS_GY);
    ins_state.bias_gz -= FPGA_REG_F(CORRECTION_BIAS_GZ);
}
```

### U54 Core 1 (Linux MAVLink Handler)

```cpp
// u54_linux/mavlink_handler/mavlink_handler.cpp

#include <mavlink/v2.0/common/mavlink.h>
#include "fc_interface.h"
#include "shared_memory.h"

class MAVLinkHandler {
public:
    MAVLinkHandler(const std::string& uart_device)
        : uart_(uart_device, 921600)
        , system_id_(1)
        , component_id_(MAV_COMP_ID_ONBOARD_COMPUTER)
    {
        shared_mem_ = mmap_shared_memory();
    }
    
    void run() {
        while (running_) {
            // Read and parse incoming MAVLink from CubeOrange
            process_incoming();
            
            // Send vision position estimate at 30 Hz
            if (should_send_vision_position()) {
                send_vision_position_estimate();
            }
            
            // Send heartbeat at 1 Hz
            if (should_send_heartbeat()) {
                send_heartbeat();
            }
            
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
    }
    
private:
    void send_vision_position_estimate() {
        // Read latest INS state from shared memory
        __sync_synchronize();  // Memory barrier
        ins_state_t state = shared_mem_->ins_state;
        uint64_t timestamp = shared_mem_->ins_timestamp;
        
        mavlink_message_t msg;
        mavlink_msg_vision_position_estimate_pack(
            system_id_,
            component_id_,
            &msg,
            timestamp,              // usec
            state.pos_n,            // x (North)
            state.pos_e,            // y (East)
            -state.pos_d,           // z (Up, NED to ENU)
            state.roll,             // roll
            state.pitch,            // pitch
            state.yaw,              // yaw
            nullptr,                // covariance (optional)
            0                       // reset_counter
        );
        
        send_message(msg);
    }
    
    void process_incoming() {
        uint8_t buf[256];
        int len = uart_.read(buf, sizeof(buf));
        
        for (int i = 0; i < len; i++) {
            mavlink_message_t msg;
            mavlink_status_t status;
            
            if (mavlink_parse_char(MAVLINK_COMM_0, buf[i], &msg, &status)) {
                handle_message(msg);
            }
        }
    }
    
    void handle_message(const mavlink_message_t& msg) {
        switch (msg.msgid) {
            case MAVLINK_MSG_ID_HEARTBEAT:
                // FC is alive
                fc_connected_ = true;
                last_heartbeat_ = std::chrono::steady_clock::now();
                break;
                
            case MAVLINK_MSG_ID_ATTITUDE:
                // Could use for comparison/validation
                break;
                
            case MAVLINK_MSG_ID_LOCAL_POSITION_NED:
                // FC's position estimate (for logging/comparison)
                break;
                
            default:
                break;
        }
    }
    
    void send_heartbeat() {
        mavlink_message_t msg;
        mavlink_msg_heartbeat_pack(
            system_id_,
            component_id_,
            &msg,
            MAV_TYPE_ONBOARD_CONTROLLER,
            MAV_AUTOPILOT_INVALID,
            0, 0, MAV_STATE_ACTIVE
        );
        send_message(msg);
    }
    
    void send_message(const mavlink_message_t& msg) {
        uint8_t buf[MAVLINK_MAX_PACKET_LEN];
        int len = mavlink_msg_to_send_buffer(buf, &msg);
        uart_.write(buf, len);
    }
    
    UARTDevice uart_;
    shared_mem_t* shared_mem_;
    uint8_t system_id_;
    uint8_t component_id_;
    bool fc_connected_ = false;
    bool running_ = true;
    std::chrono::steady_clock::time_point last_heartbeat_;
};

int main() {
    MAVLinkHandler handler("/dev/ttyUSB0");  // Adjust for your setup
    handler.run();
    return 0;
}
```

### U54 Core 2 (Pi 5 Interface)

```cpp
// u54_linux/pi5_interface/pi5_comm.cpp

#include "spi_device.h"
#include "shared_memory.h"
#include "trn_protocol.h"

class Pi5Interface {
public:
    Pi5Interface(const std::string& spi_device)
        : spi_(spi_device, 40000000)  // 40 MHz SPI
    {
        shared_mem_ = mmap_shared_memory();
    }
    
    void run() {
        while (running_) {
            // Send altitude profile to Pi 5 when requested
            if (pi5_requests_profile()) {
                send_altitude_profile();
            }
            
            // Check for TRN fix from Pi 5
            if (pi5_has_fix()) {
                receive_trn_fix();
            }
            
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
    }
    
private:
    void send_altitude_profile() {
        // Read profile from FPGA via shared memory or direct access
        altitude_profile_t profile;
        
        // Get profile from FPGA DMA buffer
        read_altitude_profile(&profile);
        
        // Pack and send to Pi 5
        trn_profile_packet_t packet;
        packet.header.type = TRN_MSG_PROFILE;
        packet.header.length = sizeof(profile);
        packet.timestamp = get_timestamp_us();
        packet.num_samples = profile.count;
        
        for (int i = 0; i < profile.count; i++) {
            packet.samples[i].pos_n = profile.samples[i].pos_n;
            packet.samples[i].pos_e = profile.samples[i].pos_e;
            packet.samples[i].terrain_elev = profile.samples[i].terrain_elev;
            packet.samples[i].timestamp = profile.samples[i].timestamp;
        }
        
        spi_.transfer(&packet, nullptr, sizeof(packet));
    }
    
    void receive_trn_fix() {
        trn_fix_packet_t packet;
        
        // Read fix from Pi 5
        spi_.transfer(nullptr, &packet, sizeof(packet));
        
        if (packet.header.type == TRN_MSG_FIX && validate_checksum(&packet)) {
            // Write to shared memory for E51 consumption
            __sync_synchronize();
            shared_mem_->trn_fix.pos_n = packet.pos_n;
            shared_mem_->trn_fix.pos_e = packet.pos_e;
            shared_mem_->trn_fix.cov_nn = packet.cov_nn;
            shared_mem_->trn_fix.cov_ee = packet.cov_ee;
            shared_mem_->trn_fix.cov_ne = packet.cov_ne;
            shared_mem_->trn_fix.timestamp = packet.timestamp;
            shared_mem_->trn_fix_ready = 1;
            __sync_synchronize();
            
            log_trn_fix(packet);
        }
    }
    
    SPIDevice spi_;
    shared_mem_t* shared_mem_;
    bool running_ = true;
};
```

---

## FPGA Fabric Modules

### Top-Level Fabric Module

```systemverilog
// fpga_fabric/top/ins_trn_fabric_top.sv

module ins_trn_fabric_top (
    // Clock and reset from MSS
    input  logic        mss_clk,           // 100 MHz from MSS
    input  logic        mss_reset_n,
    
    // CCC outputs
    output logic        ccc_lock,
    
    // APB interface (from MSS)
    input  logic        pclk,
    input  logic        presetn,
    input  logic [31:0] paddr,
    input  logic        psel,
    input  logic        penable,
    input  logic        pwrite,
    input  logic [31:0] pwdata,
    output logic [31:0] prdata,
    output logic        pready,
    output logic        pslverr,
    
    // AHB interface (for DMA, optional)
    // ... (AHB signals)
    
    // External sensor interfaces
    output logic        imu_spi_sclk,
    output logic        imu_spi_mosi,
    input  logic        imu_spi_miso,
    output logic        imu_spi_cs_n,
    
    output logic        baro_spi_sclk,
    output logic        baro_spi_mosi,
    input  logic        baro_spi_miso,
    output logic        baro_spi_cs_n,
    
    input  logic        lidar_uart_rx,
    
    // Interrupt to MSS
    output logic        fabric_irq
);

    // =========================================================================
    // Clock Generation (CCC)
    // =========================================================================
    
    logic clk_sys;      // 100 MHz
    logic clk_dsp;      // 150 MHz
    logic clk_spi;      // 50 MHz
    logic pll_lock;
    
    fabric_ccc u_ccc (
        .ref_clk    (mss_clk),
        .clk_sys    (clk_sys),
        .clk_dsp    (clk_dsp),
        .clk_spi    (clk_spi),
        .pll_lock   (pll_lock)
    );
    
    assign ccc_lock = pll_lock;
    
    // =========================================================================
    // Reset Synchronization
    // =========================================================================
    
    logic rst_sys_n;
    logic rst_dsp_n;
    
    fabric_reset_sync u_rst_sys (
        .clk            (clk_sys),
        .mss_reset_n    (mss_reset_n),
        .ccc_lock       (pll_lock),
        .fabric_reset_n (rst_sys_n)
    );
    
    fabric_reset_sync u_rst_dsp (
        .clk            (clk_dsp),
        .mss_reset_n    (mss_reset_n),
        .ccc_lock       (pll_lock),
        .fabric_reset_n (rst_dsp_n)
    );
    
    // =========================================================================
    // APB Register Bank
    // =========================================================================
    
    // Register interface signals
    logic [31:0] reg_imu_cfg;
    logic [31:0] reg_baro_cfg;
    logic [31:0] reg_lidar_cfg;
    logic [31:0] reg_ctrl;
    logic [31:0] reg_status;
    logic [31:0] reg_irq_enable;
    logic [31:0] reg_irq_status;
    
    // INS state outputs (read by E51)
    logic [31:0] reg_pos_n, reg_pos_e, reg_pos_d;
    logic [31:0] reg_vel_n, reg_vel_e, reg_vel_d;
    logic [31:0] reg_quat_w, reg_quat_x, reg_quat_y, reg_quat_z;
    
    // Calibrated sensor data
    logic [31:0] reg_gyro_x_cal, reg_gyro_y_cal, reg_gyro_z_cal;
    logic [31:0] reg_accel_x_cal, reg_accel_y_cal, reg_accel_z_cal;
    
    // TRN interface registers
    logic [31:0] reg_trn_pos_n, reg_trn_pos_e;
    logic [31:0] reg_trn_cov_nn, reg_trn_cov_ee, reg_trn_cov_ne;
    logic [31:0] reg_trn_ctrl, reg_trn_status;
    
    // Correction outputs
    logic [31:0] reg_corr_pos_n, reg_corr_pos_e, reg_corr_pos_d;
    logic [31:0] reg_corr_vel_n, reg_corr_vel_e, reg_corr_vel_d;
    logic [31:0] reg_corr_att_x, reg_corr_att_y, reg_corr_att_z;
    logic [31:0] reg_corr_bias_ax, reg_corr_bias_ay, reg_corr_bias_az;
    logic [31:0] reg_corr_bias_gx, reg_corr_bias_gy, reg_corr_bias_gz;
    
    // Covariance control
    logic [31:0] reg_cov_ctrl, reg_cov_status;
    
    apb_reg_bank u_apb_regs (
        .pclk           (pclk),
        .presetn        (presetn),
        .paddr          (paddr),
        .psel           (psel),
        .penable        (penable),
        .pwrite         (pwrite),
        .pwdata         (pwdata),
        .prdata         (prdata),
        .pready         (pready),
        .pslverr        (pslverr),
        
        // Configuration registers
        .reg_imu_cfg    (reg_imu_cfg),
        .reg_baro_cfg   (reg_baro_cfg),
        .reg_lidar_cfg  (reg_lidar_cfg),
        .reg_ctrl       (reg_ctrl),
        .reg_irq_enable (reg_irq_enable),
        
        // Status registers
        .reg_status     (reg_status),
        .reg_irq_status (reg_irq_status),
        
        // Sensor data (read-only)
        .reg_gyro_x_cal (reg_gyro_x_cal),
        .reg_gyro_y_cal (reg_gyro_y_cal),
        .reg_gyro_z_cal (reg_gyro_z_cal),
        .reg_accel_x_cal(reg_accel_x_cal),
        .reg_accel_y_cal(reg_accel_y_cal),
        .reg_accel_z_cal(reg_accel_z_cal),
        
        // TRN interface
        .reg_trn_pos_n  (reg_trn_pos_n),
        .reg_trn_pos_e  (reg_trn_pos_e),
        .reg_trn_cov_nn (reg_trn_cov_nn),
        .reg_trn_cov_ee (reg_trn_cov_ee),
        .reg_trn_cov_ne (reg_trn_cov_ne),
        .reg_trn_ctrl   (reg_trn_ctrl),
        .reg_trn_status (reg_trn_status),
        
        // Corrections
        .reg_corr_pos_n (reg_corr_pos_n),
        .reg_corr_pos_e (reg_corr_pos_e),
        .reg_corr_pos_d (reg_corr_pos_d),
        .reg_corr_vel_n (reg_corr_vel_n),
        .reg_corr_vel_e (reg_corr_vel_e),
        .reg_corr_vel_d (reg_corr_vel_d),
        .reg_corr_att_x (reg_corr_att_x),
        .reg_corr_att_y (reg_corr_att_y),
        .reg_corr_att_z (reg_corr_att_z),
        .reg_corr_bias_ax(reg_corr_bias_ax),
        .reg_corr_bias_ay(reg_corr_bias_ay),
        .reg_corr_bias_az(reg_corr_bias_az),
        .reg_corr_bias_gx(reg_corr_bias_gx),
        .reg_corr_bias_gy(reg_corr_bias_gy),
        .reg_corr_bias_gz(reg_corr_bias_gz),
        
        // Covariance control
        .reg_cov_ctrl   (reg_cov_ctrl),
        .reg_cov_status (reg_cov_status)
    );
    
    // =========================================================================
    // Sensor Subsystem
    // =========================================================================
    
    // IMU raw data
    logic        imu_data_valid;
    logic [15:0] imu_gyro_x_raw, imu_gyro_y_raw, imu_gyro_z_raw;
    logic [15:0] imu_accel_x_raw, imu_accel_y_raw, imu_accel_z_raw;
    logic [15:0] imu_temp_raw;
    
    spi_master_imu u_imu (
        .clk            (clk_sys),
        .rst_n          (rst_sys_n),
        
        .spi_sclk       (imu_spi_sclk),
        .spi_mosi       (imu_spi_mosi),
        .spi_miso       (imu_spi_miso),
        .spi_cs_n       (imu_spi_cs_n),
        
        .cfg_enable     (reg_imu_cfg[0]),
        .cfg_spi_div    (reg_imu_cfg[15:8]),
        
        .data_valid     (imu_data_valid),
        .gyro_x         (imu_gyro_x_raw),
        .gyro_y         (imu_gyro_y_raw),
        .gyro_z         (imu_gyro_z_raw),
        .accel_x        (imu_accel_x_raw),
        .accel_y        (imu_accel_y_raw),
        .accel_z        (imu_accel_z_raw),
        .temp           (imu_temp_raw)
    );
    
    // IMU calibration
    logic [31:0] gyro_x_cal, gyro_y_cal, gyro_z_cal;
    logic [31:0] accel_x_cal, accel_y_cal, accel_z_cal;
    
    imu_calibration u_imu_cal (
        .clk            (clk_sys),
        .rst_n          (rst_sys_n),
        
        .data_valid_in  (imu_data_valid),
        .gyro_x_raw     (imu_gyro_x_raw),
        .gyro_y_raw     (imu_gyro_y_raw),
        .gyro_z_raw     (imu_gyro_z_raw),
        .accel_x_raw    (imu_accel_x_raw),
        .accel_y_raw    (imu_accel_y_raw),
        .accel_z_raw    (imu_accel_z_raw),
        
        // Calibration parameters (from registers)
        .gyro_bias_x    (reg_cal_gyro_bias_x),
        .gyro_bias_y    (reg_cal_gyro_bias_y),
        .gyro_bias_z    (reg_cal_gyro_bias_z),
        .accel_bias_x   (reg_cal_accel_bias_x),
        .accel_bias_y   (reg_cal_accel_bias_y),
        .accel_bias_z   (reg_cal_accel_bias_z),
        
        // Calibrated outputs (IEEE 754 float)
        .data_valid_out (cal_data_valid),
        .gyro_x_cal     (gyro_x_cal),
        .gyro_y_cal     (gyro_y_cal),
        .gyro_z_cal     (gyro_z_cal),
        .accel_x_cal    (accel_x_cal),
        .accel_y_cal    (accel_y_cal),
        .accel_z_cal    (accel_z_cal)
    );
    
    // Register calibrated data for E51 access
    always_ff @(posedge clk_sys) begin
        if (cal_data_valid) begin
            reg_gyro_x_cal  <= gyro_x_cal;
            reg_gyro_y_cal  <= gyro_y_cal;
            reg_gyro_z_cal  <= gyro_z_cal;
            reg_accel_x_cal <= accel_x_cal;
            reg_accel_y_cal <= accel_y_cal;
            reg_accel_z_cal <= accel_z_cal;
        end
    end
    
    // Barometer interface
    logic        baro_data_valid;
    logic [31:0] baro_pressure;
    logic [31:0] baro_temperature;
    
    spi_master_baro u_baro (
        .clk            (clk_sys),
        .rst_n          (rst_sys_n),
        
        .spi_sclk       (baro_spi_sclk),
        .spi_mosi       (baro_spi_mosi),
        .spi_miso       (baro_spi_miso),
        .spi_cs_n       (baro_spi_cs_n),
        
        .cfg_enable     (reg_baro_cfg[0]),
        
        .data_valid     (baro_data_valid),
        .pressure       (baro_pressure),
        .temperature    (baro_temperature)
    );
    
    // LiDAR interface
    logic        lidar_data_valid;
    logic [15:0] lidar_distance;
    logic [15:0] lidar_strength;
    
    uart_rx_lidar u_lidar (
        .clk            (clk_sys),
        .rst_n          (rst_sys_n),
        
        .uart_rx        (lidar_uart_rx),
        
        .cfg_enable     (reg_lidar_cfg[0]),
        
        .data_valid     (lidar_data_valid),
        .distance       (lidar_distance),
        .strength       (lidar_strength)
    );
    
    // =========================================================================
    // Covariance Propagation Engine
    // =========================================================================
    
    logic cov_busy;
    logic cov_done;
    
    covariance_propagator u_cov_prop (
        .clk            (clk_dsp),
        .rst_n          (rst_dsp_n),
        
        .start          (reg_cov_ctrl[0]),
        .busy           (cov_busy),
        .done           (cov_done),
        
        // Φ matrix input (from E51 via registers)
        .phi_write_en   (phi_write_en),
        .phi_write_addr (phi_write_addr),
        .phi_write_data (phi_write_data),
        
        // P matrix LSRAM interface
        .p_addr         (p_addr),
        .p_rdata        (p_rdata),
        .p_wdata        (p_wdata),
        .p_we           (p_we),
        
        // Q matrix (from ROM)
        .q_diag         (q_diag)
    );
    
    assign reg_cov_status = {30'b0, cov_done, cov_busy};
    
    // =========================================================================
    // Kalman Update Engine
    // =========================================================================
    
    logic kalman_start;
    logic kalman_busy;
    logic kalman_done;
    
    assign kalman_start = reg_trn_ctrl[0];
    
    kalman_update_engine u_kalman (
        .clk            (clk_dsp),
        .rst_n          (rst_dsp_n),
        
        .start          (kalman_start),
        .busy           (kalman_busy),
        .done           (kalman_done),
        
        // TRN measurement input
        .trn_pos_n      (reg_trn_pos_n),
        .trn_pos_e      (reg_trn_pos_e),
        .trn_cov_nn     (reg_trn_cov_nn),
        .trn_cov_ee     (reg_trn_cov_ee),
        .trn_cov_ne     (reg_trn_cov_ne),
        
        // Current INS state (for innovation)
        .ins_pos_n      (ins_state_pos_n),
        .ins_pos_e      (ins_state_pos_e),
        
        // P matrix interface
        .p_addr         (kalman_p_addr),
        .p_rdata        (kalman_p_rdata),
        .p_wdata        (kalman_p_wdata),
        .p_we           (kalman_p_we),
        
        // Correction outputs
        .corr_pos_n     (reg_corr_pos_n),
        .corr_pos_e     (reg_corr_pos_e),
        .corr_pos_d     (reg_corr_pos_d),
        .corr_vel_n     (reg_corr_vel_n),
        .corr_vel_e     (reg_corr_vel_e),
        .corr_vel_d     (reg_corr_vel_d),
        .corr_att_x     (reg_corr_att_x),
        .corr_att_y     (reg_corr_att_y),
        .corr_att_z     (reg_corr_att_z),
        .corr_bias_ax   (reg_corr_bias_ax),
        .corr_bias_ay   (reg_corr_bias_ay),
        .corr_bias_az   (reg_corr_bias_az),
        .corr_bias_gx   (reg_corr_bias_gx),
        .corr_bias_gy   (reg_corr_bias_gy),
        .corr_bias_gz   (reg_corr_bias_gz)
    );
    
    assign reg_trn_status = {30'b0, kalman_done, kalman_busy};
    
    // =========================================================================
    // Altitude Profile Buffer
    // =========================================================================
    
    altitude_profile_mgr u_profile (
        .clk            (clk_sys),
        .rst_n          (rst_sys_n),
        
        .baro_valid     (baro_data_valid),
        .baro_altitude  (baro_altitude),
        
        .lidar_valid    (lidar_data_valid),
        .lidar_agl      (lidar_distance),
        
        .ins_pos_n      (ins_state_pos_n),
        .ins_pos_e      (ins_state_pos_e),
        
        // DMA interface to DDR
        .dma_start      (reg_profile_ctrl[0]),
        .dma_busy       (profile_dma_busy),
        .dma_done       (profile_dma_done),
        .dma_addr       (reg_profile_addr),
        .dma_count      (reg_profile_count)
    );
    
    // =========================================================================
    // Interrupt Generation
    // =========================================================================
    
    logic irq_imu_ready;
    logic irq_kalman_done;
    logic irq_cov_done;
    
    assign irq_imu_ready = cal_data_valid;
    assign irq_kalman_done = kalman_done;
    assign irq_cov_done = cov_done;
    
    assign fabric_irq = (reg_irq_enable[0] & irq_imu_ready) |
                        (reg_irq_enable[1] & irq_kalman_done) |
                        (reg_irq_enable[2] & irq_cov_done);
    
    // Sticky interrupt status
    always_ff @(posedge clk_sys or negedge rst_sys_n) begin
        if (!rst_sys_n) begin
            reg_irq_status <= 32'b0;
        end else begin
            // Set on event
            if (irq_imu_ready)   reg_irq_status[0] <= 1'b1;
            if (irq_kalman_done) reg_irq_status[1] <= 1'b1;
            if (irq_cov_done)    reg_irq_status[2] <= 1'b1;
            
            // Clear on write-1-to-clear
            if (irq_status_clear[0]) reg_irq_status[0] <= 1'b0;
            if (irq_status_clear[1]) reg_irq_status[1] <= 1'b0;
            if (irq_status_clear[2]) reg_irq_status[2] <= 1'b0;
        end
    end

endmodule
```

### Matrix Multiply Engine (Covariance Propagation)

```systemverilog
// fpga_fabric/covariance/matrix_multiply_15x15.sv

module matrix_multiply_15x15 (
    input  logic        clk,
    input  logic        rst_n,
    
    input  logic        start,
    output logic        busy,
    output logic        done,
    
    // Matrix A input (15x15, row-major)
    input  logic [31:0] a_data,
    output logic [7:0]  a_addr,
    
    // Matrix B input (15x15, row-major)
    input  logic [31:0] b_data,
    output logic [7:0]  b_addr,
    
    // Matrix C output (15x15, row-major)
    output logic [31:0] c_data,
    output logic [7:0]  c_addr,
    output logic        c_we
);

    // =========================================================================
    // Architecture: Output-stationary systolic array (4x4 PE array)
    // Computes 15x15 multiply in tiles
    // =========================================================================
    
    localparam MATRIX_SIZE = 15;
    localparam TILE_SIZE = 4;
    localparam NUM_TILES = 4;  // Ceiling(15/4)
    
    typedef enum logic [2:0] {
        IDLE,
        LOAD_A,
        LOAD_B,
        COMPUTE,
        STORE_C,
        NEXT_TILE,
        FINISH
    } state_t;
    
    state_t state, next_state;
    
    // Tile indices
    logic [2:0] tile_row, tile_col, tile_k;
    
    // Local element indices within tile
    logic [2:0] local_row, local_col, local_k;
    
    // Processing element array (4x4)
    logic [31:0] pe_a [0:3][0:3];      // A matrix values
    logic [31:0] pe_b [0:3][0:3];      // B matrix values
    logic [31:0] pe_c [0:3][0:3];      // Accumulated C values
    
    // FP multiply-add units (shared, pipelined)
    logic [31:0] fma_a [0:15];
    logic [31:0] fma_b [0:15];
    logic [31:0] fma_c [0:15];
    logic [31:0] fma_result [0:15];
    logic        fma_valid_in;
    logic        fma_valid_out;
    
    // Instantiate 16 FP MAC units
    genvar i;
    generate
        for (i = 0; i < 16; i++) begin : gen_fma
            fp_mac u_fma (
                .clk        (clk),
                .rst_n      (rst_n),
                .a          (fma_a[i]),
                .b          (fma_b[i]),
                .c          (fma_c[i]),
                .valid_in   (fma_valid_in),
                .result     (fma_result[i]),
                .valid_out  (fma_valid_out)
            );
        end
    endgenerate
    
    // State machine
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= IDLE;
        end else begin
            state <= next_state;
        end
    end
    
    always_comb begin
        next_state = state;
        
        case (state)
            IDLE: begin
                if (start) next_state = LOAD_A;
            end
            
            LOAD_A: begin
                if (load_a_done) next_state = LOAD_B;
            end
            
            LOAD_B: begin
                if (load_b_done) next_state = COMPUTE;
            end
            
            COMPUTE: begin
                if (compute_done) next_state = STORE_C;
            end
            
            STORE_C: begin
                if (store_done) next_state = NEXT_TILE;
            end
            
            NEXT_TILE: begin
                if (all_tiles_done)
                    next_state = FINISH;
                else if (k_tiles_done)
                    next_state = STORE_C;
                else
                    next_state = LOAD_A;
            end
            
            FINISH: begin
                next_state = IDLE;
            end
        endcase
    end
    
    // Address generation for A matrix
    // A[tile_row*4 + local_row][tile_k*4 + local_k]
    always_comb begin
        logic [7:0] row_idx, col_idx;
        row_idx = tile_row * TILE_SIZE + local_row;
        col_idx = tile_k * TILE_SIZE + local_k;
        
        // Bounds check for 15x15
        if (row_idx < MATRIX_SIZE && col_idx < MATRIX_SIZE)
            a_addr = row_idx * MATRIX_SIZE + col_idx;
        else
            a_addr = 8'd0;  // Out of bounds, will load zero
    end
    
    // ... (similar for B and C addressing)
    
    assign busy = (state != IDLE);
    assign done = (state == FINISH);

endmodule
```

---

## MAVLink Integration

### Message Flow

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                        MAVLink Message Flow                                  │
├─────────────────────────────────────────────────────────────────────────────┤
│                                                                             │
│   PolarFire → CubeOrange (30 Hz):                                          │
│   ┌─────────────────────────────────────────────────────────────────────┐  │
│   │  VISION_POSITION_ESTIMATE (#102)                                    │  │
│   │  ├── usec:        Timestamp (microseconds)                         │  │
│   │  ├── x:           Position North (m)                               │  │
│   │  ├── y:           Position East (m)                                │  │
│   │  ├── z:           Position Up (m, ENU convention)                  │  │
│   │  ├── roll:        Roll angle (rad)                                 │  │
│   │  ├── pitch:       Pitch angle (rad)                                │  │
│   │  ├── yaw:         Yaw angle (rad)                                  │  │
│   │  └── covariance:  [21] (optional, upper triangle)                  │  │
│   └─────────────────────────────────────────────────────────────────────┘  │
│                                                                             │
│   PolarFire → CubeOrange (1 Hz):                                           │
│   ┌─────────────────────────────────────────────────────────────────────┐  │
│   │  HEARTBEAT (#0)                                                     │  │
│   │  ├── type:        MAV_TYPE_ONBOARD_CONTROLLER                      │  │
│   │  ├── autopilot:   MAV_AUTOPILOT_INVALID                            │  │
│   │  └── system_status: MAV_STATE_ACTIVE                               │  │
│   └─────────────────────────────────────────────────────────────────────┘  │
│                                                                             │
│   CubeOrange → PolarFire (subscribed):                                     │
│   ┌─────────────────────────────────────────────────────────────────────┐  │
│   │  HEARTBEAT (#0)         - FC health monitoring                      │  │
│   │  ATTITUDE (#30)         - FC attitude (comparison/fallback)         │  │
│   │  LOCAL_POSITION_NED(#32)- FC position (comparison/logging)          │  │
│   │  SYSTEM_TIME (#2)       - Time synchronization                      │  │
│   └─────────────────────────────────────────────────────────────────────┘  │
│                                                                             │
└─────────────────────────────────────────────────────────────────────────────┘
```

### PX4 Configuration for External Vision

```yaml
# CubeOrange PX4 Parameters for Vision Position Fusion

# Enable vision position fusion
EKF2_EV_CTRL: 15          # Enable all: pos, vel, yaw, height
EKF2_HGT_REF: 3           # Use vision as height reference

# Vision measurement delay (calibrate for your system)
EKF2_EV_DELAY: 50         # 50 ms delay compensation

# Vision noise parameters (tune based on TRN accuracy)
EKF2_EVP_NOISE: 0.1       # Vision position noise (m)
EKF2_EVV_NOISE: 0.1       # Vision velocity noise (m/s)

# Disable GPS (GPS-denied operation)
EKF2_GPS_CTRL: 0          # Disable GPS fusion

# MAVLink configuration
MAV_0_CONFIG: 102         # TELEM2 port
MAV_0_RATE: 50000         # 50 kB/s bandwidth
MAV_USEHILGPS: 0          # Don't use HIL GPS
```

### ArduPilot Configuration

```
# ArduPilot Parameters for Visual Odometry

# Enable visual odometry
VISO_TYPE = 1             # MAVLink visual odometry

# EKF source configuration
EK3_SRC1_POSXY = 6        # ExternalNav for horizontal position
EK3_SRC1_VELXY = 6        # ExternalNav for horizontal velocity
EK3_SRC1_POSZ = 6         # ExternalNav for vertical position

# Vision delay
EK3_VISO_DELAY = 50       # 50 ms delay compensation

# Disable GPS
GPS_TYPE = 0              # No GPS

# Serial port configuration
SERIAL2_PROTOCOL = 2      # MAVLink2 on TELEM2
SERIAL2_BAUD = 921        # 921600 baud
```

---

## Memory Architecture

### LSRAM Allocation

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                      LSRAM Usage (MPFS250T: 1.5 MB)                         │
├─────────────────────────────────────────────────────────────────────────────┤
│                                                                             │
│   ┌─────────────────────────────────────────────────────────────────────┐  │
│   │  INS State Memory                                                   │  │
│   │  ├── Position (N,E,D)         3 × 32b =    96 bits                 │  │
│   │  ├── Velocity (N,E,D)         3 × 32b =    96 bits                 │  │
│   │  ├── Quaternion (w,x,y,z)     4 × 32b =   128 bits                 │  │
│   │  ├── Accel bias (x,y,z)       3 × 32b =    96 bits                 │  │
│   │  └── Gyro bias (x,y,z)        3 × 32b =    96 bits                 │  │
│   │                               ─────────────────────                 │  │
│   │                               Total:       512 bits                 │  │
│   │                               Depth:       2 (double buffer)        │  │
│   │                               LSRAM:       1 block (~1 KB)          │  │
│   └─────────────────────────────────────────────────────────────────────┘  │
│                                                                             │
│   ┌─────────────────────────────────────────────────────────────────────┐  │
│   │  Error State Memory                                                 │  │
│   │  └── δx (15 elements)        15 × 32b =   480 bits                 │  │
│   │                               Depth:       2                        │  │
│   │                               LSRAM:       1 block                  │  │
│   └─────────────────────────────────────────────────────────────────────┘  │
│                                                                             │
│   ┌─────────────────────────────────────────────────────────────────────┐  │
│   │  Covariance Matrix (P) - Symmetric, upper triangle stored          │  │
│   │  └── 120 unique elements     120 × 32b = 3840 bits                 │  │
│   │                               Depth:       2 (double buffer)        │  │
│   │                               LSRAM:       2 blocks (~2 KB)         │  │
│   └─────────────────────────────────────────────────────────────────────┘  │
│                                                                             │
│   ┌─────────────────────────────────────────────────────────────────────┐  │
│   │  State Transition Matrix (Φ) - Scratchpad                          │  │
│   │  └── 225 elements (15×15)    225 × 32b = 7200 bits                 │  │
│   │                               LSRAM:       2 blocks (~2 KB)         │  │
│   └─────────────────────────────────────────────────────────────────────┘  │
│                                                                             │
│   ┌─────────────────────────────────────────────────────────────────────┐  │
│   │  Process Noise Matrix (Q) - Diagonal, stored as ROM                │  │
│   │  └── 15 diagonal elements    15 × 32b =   480 bits                 │  │
│   │                               LSRAM:       1 block                  │  │
│   └─────────────────────────────────────────────────────────────────────┘  │
│                                                                             │
│   ┌─────────────────────────────────────────────────────────────────────┐  │
│   │  Altitude Profile FIFO                                              │  │
│   │  └── 128 entries × 128b      128 × 128b = 16384 bits               │  │
│   │                               LSRAM:       4 blocks (~4 KB)         │  │
│   └─────────────────────────────────────────────────────────────────────┘  │
│                                                                             │
│   ┌─────────────────────────────────────────────────────────────────────┐  │
│   │  IMU Calibration Parameters                                         │  │
│   │  ├── Gyro bias (3)            3 × 32b                              │  │
│   │  ├── Gyro scale (3)           3 × 32b                              │  │
│   │  ├── Accel bias (3)           3 × 32b                              │  │
│   │  ├── Accel scale (3)          3 × 32b                              │  │
│   │  └── Misalignment (9)         9 × 32b                              │  │
│   │                               LSRAM:       1 block                  │  │
│   └─────────────────────────────────────────────────────────────────────┘  │
│                                                                             │
│   ┌─────────────────────────────────────────────────────────────────────┐  │
│   │  Trigonometric LUT (sin/cos)                                        │  │
│   │  └── 2048 entries × 32b      2048 × 32b = 65536 bits               │  │
│   │                               LSRAM:       2 blocks (~8 KB)         │  │
│   └─────────────────────────────────────────────────────────────────────┘  │
│                                                                             │
│   ─────────────────────────────────────────────────────────────────────    │
│   Total LSRAM used:    ~14 blocks (~20 KB)                                 │
│   MPFS250T available:  1.5 MB                                              │
│   Utilization:         ~1.3%                                               │
│                                                                             │
└─────────────────────────────────────────────────────────────────────────────┘
```

### L2 Cache/Scratchpad Configuration

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                     L2 Memory Configuration (2 MB)                          │
├─────────────────────────────────────────────────────────────────────────────┤
│                                                                             │
│   Address Range        Size     Assignment         Notes                    │
│   ─────────────────────────────────────────────────────────────────────    │
│   0x0800_0000 -        512 KB   E51 Scratchpad    Uncached, deterministic  │
│   0x0807_FFFF                                     INS state, working vars   │
│                                                                             │
│   0x0808_0000 -        256 KB   Shared Memory     E51 ↔ U54 communication  │
│   0x080B_FFFF                                     INS state, TRN fixes      │
│                                                                             │
│   0x080C_0000 -        1.25 MB  U54 L2 Cache     Cached for Linux          │
│   0x081F_FFFF                                     Standard cache operation   │
│                                                                             │
└─────────────────────────────────────────────────────────────────────────────┘
```

### DDR4 Memory Map

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                     DDR4 Memory Map (2 GB example)                          │
├─────────────────────────────────────────────────────────────────────────────┤
│                                                                             │
│   Address Range          Size     Usage                                     │
│   ─────────────────────────────────────────────────────────────────────    │
│   0x8000_0000 -          1 GB    Linux kernel + userspace                  │
│   0xBFFF_FFFF                                                               │
│                                                                             │
│   0xC000_0000 -          256 MB  Altitude profile DMA buffer               │
│   0xCFFF_FFFF                    (for Pi 5 transfer)                        │
│                                                                             │
│   0xD000_0000 -          256 MB  State history buffer                      │
│   0xDFFF_FFFF                    (for logging/replay)                       │
│                                                                             │
│   0xE000_0000 -          512 MB  Reserved / Future use                     │
│   0xFFFF_FFFF                                                               │
│                                                                             │
└─────────────────────────────────────────────────────────────────────────────┘
```

---

## Register Maps

### APB Register Map

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                      APB Register Map                                        │
│                      Base Address: 0x6000_0000 (Fabric APB)                  │
├─────────────────────────────────────────────────────────────────────────────┤
│                                                                             │
│   Offset    Name                   R/W    Description                       │
│   ──────────────────────────────────────────────────────────────────────   │
│                                                                             │
│   === System Control ===                                                    │
│   0x000     CTRL                   R/W    [0] enable, [1] soft_reset       │
│   0x004     STATUS                 R      [0] running, [1] error, [2] lock │
│   0x008     VERSION                R      [31:16] major, [15:0] minor      │
│   0x00C     SCRATCH                R/W    Debug scratch register           │
│                                                                             │
│   === Interrupt Control ===                                                 │
│   0x010     IRQ_ENABLE             R/W    [0] imu, [1] kalman, [2] cov     │
│   0x014     IRQ_STATUS             R/W1C  [0] imu, [1] kalman, [2] cov     │
│   0x018     IRQ_RAW                R      Raw interrupt sources            │
│                                                                             │
│   === Sensor Configuration ===                                              │
│   0x020     IMU_CFG                R/W    [0] enable, [15:8] spi_div       │
│   0x024     BARO_CFG               R/W    [0] enable, [7:4] oversample     │
│   0x028     LIDAR_CFG              R/W    [0] enable, [15:8] baud_div      │
│   0x02C     SENSOR_STATUS          R      [0] imu_ok, [1] baro_ok, ...     │
│                                                                             │
│   === Calibrated Sensor Data (Read by E51) ===                              │
│   0x040     GYRO_X_CAL             R      Gyro X, calibrated (float32)     │
│   0x044     GYRO_Y_CAL             R      Gyro Y, calibrated (float32)     │
│   0x048     GYRO_Z_CAL             R      Gyro Z, calibrated (float32)     │
│   0x04C     ACCEL_X_CAL            R      Accel X, calibrated (float32)    │
│   0x050     ACCEL_Y_CAL            R      Accel Y, calibrated (float32)    │
│   0x054     ACCEL_Z_CAL            R      Accel Z, calibrated (float32)    │
│   0x058     BARO_PRESS             R      Barometric pressure (float32)    │
│   0x05C     BARO_TEMP              R      Barometer temperature (float32)  │
│   0x060     LIDAR_DIST             R      LiDAR distance (uint16)          │
│   0x064     LIDAR_STRENGTH         R      LiDAR signal strength (uint16)   │
│                                                                             │
│   === Covariance Propagation Control ===                                    │
│   0x080     COV_CTRL               R/W    [0] start propagation            │
│   0x084     COV_STATUS             R      [0] busy, [1] done               │
│                                                                             │
│   === Φ Matrix Write Interface (E51 writes) ===                            │
│   0x088     PHI_ADDR               W      Write address [7:0]              │
│   0x08C     PHI_DATA               W      Write data (float32)             │
│   0x090     PHI_CTRL               W      [0] write_enable                 │
│                                                                             │
│   === TRN Interface ===                                                     │
│   0x100     TRN_CTRL               R/W    [0] start Kalman update          │
│   0x104     TRN_STATUS             R      [0] busy, [1] done               │
│   0x108     TRN_POS_N              W      TRN position North (float32)     │
│   0x10C     TRN_POS_E              W      TRN position East (float32)      │
│   0x110     TRN_COV_NN             W      TRN covariance N-N (float32)     │
│   0x114     TRN_COV_EE             W      TRN covariance E-E (float32)     │
│   0x118     TRN_COV_NE             W      TRN covariance N-E (float32)     │
│                                                                             │
│   === Kalman Correction Output (E51 reads) ===                              │
│   0x140     CORR_POS_N             R      Position N correction (float32)  │
│   0x144     CORR_POS_E             R      Position E correction (float32)  │
│   0x148     CORR_POS_D             R      Position D correction (float32)  │
│   0x14C     CORR_VEL_N             R      Velocity N correction (float32)  │
│   0x150     CORR_VEL_E             R      Velocity E correction (float32)  │
│   0x154     CORR_VEL_D             R      Velocity D correction (float32)  │
│   0x158     CORR_ATT_X             R      Attitude X correction (float32)  │
│   0x15C     CORR_ATT_Y             R      Attitude Y correction (float32)  │
│   0x160     CORR_ATT_Z             R      Attitude Z correction (float32)  │
│   0x164     CORR_BIAS_AX           R      Accel bias X correction          │
│   0x168     CORR_BIAS_AY           R      Accel bias Y correction          │
│   0x16C     CORR_BIAS_AZ           R      Accel bias Z correction          │
│   0x170     CORR_BIAS_GX           R      Gyro bias X correction           │
│   0x174     CORR_BIAS_GY           R      Gyro bias Y correction           │
│   0x178     CORR_BIAS_GZ           R      Gyro bias Z correction           │
│                                                                             │
│   === Altitude Profile DMA ===                                              │
│   0x180     PROF_CTRL              R/W    [0] start DMA                    │
│   0x184     PROF_STATUS            R      [0] busy, [1] done, [15:8] count │
│   0x188     PROF_DDR_ADDR          R/W    DDR destination address          │
│   0x18C     PROF_COUNT             R/W    Number of samples to transfer    │
│                                                                             │
│   === Calibration Parameters ===                                            │
│   0x200     CAL_GYRO_BIAS_X        R/W    Gyro X bias (float32)            │
│   0x204     CAL_GYRO_BIAS_Y        R/W    Gyro Y bias (float32)            │
│   0x208     CAL_GYRO_BIAS_Z        R/W    Gyro Z bias (float32)            │
│   0x20C     CAL_GYRO_SCALE_X       R/W    Gyro X scale factor              │
│   0x210     CAL_GYRO_SCALE_Y       R/W    Gyro Y scale factor              │
│   0x214     CAL_GYRO_SCALE_Z       R/W    Gyro Z scale factor              │
│   0x218     CAL_ACCEL_BIAS_X       R/W    Accel X bias (float32)           │
│   0x21C     CAL_ACCEL_BIAS_Y       R/W    Accel Y bias (float32)           │
│   0x220     CAL_ACCEL_BIAS_Z       R/W    Accel Z bias (float32)           │
│   0x224     CAL_ACCEL_SCALE_X      R/W    Accel X scale factor             │
│   0x228     CAL_ACCEL_SCALE_Y      R/W    Accel Y scale factor             │
│   0x22C     CAL_ACCEL_SCALE_Z      R/W    Accel Z scale factor             │
│   0x230     CAL_APPLY              W      [0] apply calibration            │
│                                                                             │
│   === Debug / Raw Sensor Data ===                                           │
│   0x300     RAW_GYRO_X             R      Raw gyro X (int16)               │
│   0x304     RAW_GYRO_Y             R      Raw gyro Y (int16)               │
│   0x308     RAW_GYRO_Z             R      Raw gyro Z (int16)               │
│   0x30C     RAW_ACCEL_X            R      Raw accel X (int16)              │
│   0x310     RAW_ACCEL_Y            R      Raw accel Y (int16)              │
│   0x314     RAW_ACCEL_Z            R      Raw accel Z (int16)              │
│   0x318     IMU_TEMP               R      IMU temperature (int16)          │
│   0x31C     SAMPLE_COUNT           R      Total IMU samples (uint32)       │
│   0x320     ERROR_COUNT            R      Communication errors (uint32)    │
│   0x324     COV_CYCLE_COUNT        R      Covariance prop cycles           │
│   0x328     KALMAN_CYCLE_COUNT     R      Kalman update cycles             │
│                                                                             │
└─────────────────────────────────────────────────────────────────────────────┘
```

---

## Timing Budgets

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                         Timing Analysis                                      │
├─────────────────────────────────────────────────────────────────────────────┤
│                                                                             │
│   IMU Sample Period: 2.5 ms (400 Hz)                                       │
│                                                                             │
│   ┌─────────────────────────────────────────────────────────────────────┐  │
│   │  Operation                        Time        Clock    Margin       │  │
│   │  ───────────────────────────────────────────────────────────────────│  │
│   │  SPI burst read (IMU)             56 µs       100 MHz               │  │
│   │  IMU calibration                  0.2 µs      100 MHz               │  │
│   │  E51 INS propagation              ~80 µs      600 MHz  (SW)         │  │
│   │  Covariance prop (FPGA)           ~8 µs       150 MHz  (every 10th) │  │
│   │  ───────────────────────────────────────────────────────────────────│  │
│   │  Total per sample:                ~145 µs                           │  │
│   │  Available time:                  2500 µs                           │  │
│   │  Utilization:                     5.8%                              │  │
│   │  Headroom:                        2355 µs (94.2%)                   │  │
│   └─────────────────────────────────────────────────────────────────────┘  │
│                                                                             │
│   TRN Update (triggered by Pi 5, ~0.1-1 Hz):                               │
│   ┌─────────────────────────────────────────────────────────────────────┐  │
│   │  Operation                        Cycles      Time @ 150 MHz        │  │
│   │  ───────────────────────────────────────────────────────────────────│  │
│   │  Load TRN fix from registers      10          0.07 µs               │  │
│   │  Compute innovation (z - Hx)      5           0.03 µs               │  │
│   │  Compute S = HPH' + R (2×2)       40          0.27 µs               │  │
│   │  Invert S (2×2 direct)            25          0.17 µs               │  │
│   │  Compute K = PH'S⁻¹ (15×2)        ~250        1.67 µs               │  │
│   │  State correction (K×z)           ~50         0.33 µs               │  │
│   │  Covariance update (P - KSK')     ~400        2.67 µs               │  │
│   │  ───────────────────────────────────────────────────────────────────│  │
│   │  Total Kalman update:             ~780        ~5.2 µs               │  │
│   └─────────────────────────────────────────────────────────────────────┘  │
│                                                                             │
│   Profile DMA (128 samples × 128 bits):                                    │
│   ┌─────────────────────────────────────────────────────────────────────┐  │
│   │  AXI burst @ 150 MHz, 64-bit:     256 cycles  1.7 µs               │  │
│   └─────────────────────────────────────────────────────────────────────┘  │
│                                                                             │
│   MAVLink VISION_POSITION_ESTIMATE (30 Hz):                                │
│   ┌─────────────────────────────────────────────────────────────────────┐  │
│   │  Available period:                33.3 ms                           │  │
│   │  U54 processing:                  ~50 µs (negligible)               │  │
│   │  UART transmission @ 921600:      ~400 µs (36 bytes)                │  │
│   │  Headroom:                        32.8 ms (98.5%)                   │  │
│   └─────────────────────────────────────────────────────────────────────┘  │
│                                                                             │
└─────────────────────────────────────────────────────────────────────────────┘
```

---

## Resource Estimates

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                  PolarFire SoC Resource Utilization                         │
│                  Target: MPFS250T (Icicle Kit)                              │
├─────────────────────────────────────────────────────────────────────────────┤
│                                                                             │
│   Resource           Available    Estimated    Utilization                  │
│   ───────────────────────────────────────────────────────────────────────  │
│   Logic Elements     254,350      ~35,000      13.8%                       │
│   LSRAM (KB)         1,540        ~25          1.6%                        │
│   Math Blocks        784          ~60          7.7%                        │
│   µSRAM Blocks       1,120        ~20          1.8%                        │
│                                                                             │
│   ┌─────────────────────────────────────────────────────────────────────┐  │
│   │  Logic Element Breakdown:                                           │  │
│   │  ├── APB interface + registers          ~3,000 LEs                  │  │
│   │  ├── Sensor interfaces (SPI, UART)      ~2,500 LEs                  │  │
│   │  ├── IMU calibration                    ~1,500 LEs                  │  │
│   │  ├── INS acceleration engine            ~4,000 LEs                  │  │
│   │  ├── Covariance propagator              ~12,000 LEs                 │  │
│   │  │   ├── 15×15 matrix multiply          ~8,000                      │  │
│   │  │   ├── Matrix transpose               ~1,500                      │  │
│   │  │   └── Control logic                  ~2,500                      │  │
│   │  ├── Kalman update engine               ~8,000 LEs                  │  │
│   │  │   ├── Kalman gain calc               ~4,000                      │  │
│   │  │   ├── 2×2 inverse                    ~1,500                      │  │
│   │  │   └── State/cov update               ~2,500                      │  │
│   │  ├── Altitude profile manager           ~2,000 LEs                  │  │
│   │  └── Misc (clock, reset, debug)         ~2,000 LEs                  │  │
│   └─────────────────────────────────────────────────────────────────────┘  │
│                                                                             │
│   ┌─────────────────────────────────────────────────────────────────────┐  │
│   │  Math Block Breakdown:                                              │  │
│   │  ├── FP multiply units (pipelined)      32 blocks                   │  │
│   │  ├── FP add/subtract units              16 blocks                   │  │
│   │  ├── FP divide (for inverse)            4 blocks                    │  │
│   │  └── Misc (scaling, etc.)               8 blocks                    │  │
│   └─────────────────────────────────────────────────────────────────────┘  │
│                                                                             │
│   MPFS095T (Discovery Kit) Estimate:                                       │
│   ───────────────────────────────────────────────────────────────────────  │
│   Logic Elements:    ~35,000 / 95,000 = 36.8%                              │
│   Math Blocks:       ~60 / 336 = 17.9%                                     │
│   LSRAM:             ~25 KB / 480 KB = 5.2%                                │
│   → Fits, but reduced headroom for expansion                               │
│                                                                             │
└─────────────────────────────────────────────────────────────────────────────┘
```

---

## Power Analysis

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                         Power Estimation                                     │
├─────────────────────────────────────────────────────────────────────────────┤
│                                                                             │
│   PolarFire SoC MPFS250T (typical operation):                              │
│                                                                             │
│   Component                          Power (W)    Notes                     │
│   ───────────────────────────────────────────────────────────────────────  │
│   MSS (5× RISC-V cores @ 600 MHz)    0.8 - 1.2   E51 active, U54s partial  │
│   FPGA Fabric (13% util)             0.4 - 0.6   150 MHz DSP domain        │
│   LSRAM                              0.05        Minimal utilization        │
│   I/O (SPI, UART, MAVLink)           0.1         Low-speed interfaces       │
│   DDR4 Controller                    0.3 - 0.5   Moderate activity          │
│   ───────────────────────────────────────────────────────────────────────  │
│   Subtotal (PolarFire SoC):          1.7 - 2.5 W                           │
│                                                                             │
│   Compare to Zynq-7020:                                                     │
│   ───────────────────────────────────────────────────────────────────────  │
│   Zynq-7020 (similar config):        2.5 - 3.5 W                           │
│   PolarFire advantage:               ~30-40% lower                          │
│                                                                             │
│   Compare to Zynq UltraScale+:                                              │
│   ───────────────────────────────────────────────────────────────────────  │
│   ZU7EV (comparable fabric):         6 - 10 W                              │
│   PolarFire advantage:               ~70% lower                             │
│                                                                             │
│   System Power Budget (Group 1 UAS):                                        │
│   ───────────────────────────────────────────────────────────────────────  │
│   PolarFire SoC Navigation:          2.0 W                                 │
│   CubeOrange+ (estimated):           1.5 W                                 │
│   Raspberry Pi 5 (active):           4.0 W                                 │
│   Sensors (IMU, Baro, LiDAR):        0.5 W                                 │
│   ───────────────────────────────────────────────────────────────────────  │
│   Total Avionics:                    8.0 W                                 │
│                                                                             │
│   For 3S 2200mAh pack (24 Wh):                                             │
│   ├── Propulsion (90%):              21.6 Wh                               │
│   ├── Avionics (10%):                2.4 Wh                                │
│   └── Avionics flight time:          18 minutes                            │
│                                                                             │
│   With larger pack (4S 5000mAh, 74 Wh):                                    │
│   ├── Avionics budget (10%):         7.4 Wh                                │
│   └── Avionics flight time:          55 minutes                            │
│                                                                             │
└─────────────────────────────────────────────────────────────────────────────┘
```

---

## Build and Simulation

### Directory Structure

```
ins_trn_polarfire/
├── mss_config/
│   ├── MPFS_ICICLE_BASE_DESIGN.prjx    # Libero project
│   └── XML/
│       └── MPFS_ICICLE_MSS.cfg         # MSS Configurator output
│
├── e51_firmware/
│   ├── src/
│   ├── include/
│   └── Makefile
│
├── u54_linux/
│   ├── buildroot/                       # Buildroot config
│   ├── yocto/                           # Yocto meta-layers (alt)
│   └── applications/
│       ├── mavlink_handler/
│       └── pi5_interface/
│
├── fpga_fabric/
│   ├── hdl/                             # RTL source
│   ├── constraints/
│   │   ├── icicle_kit.pdc              # Pin constraints
│   │   └── timing.sdc                  # Timing constraints
│   └── ip/                              # IP cores
│
├── tb/
│   ├── sv/                              # SystemVerilog testbenches
│   └── python/                          # Python reference models
│
├── scripts/
│   ├── build_all.sh                     # Full build flow
│   ├── build_fabric.tcl                 # Libero synthesis
│   ├── build_mss.sh                     # MSS configuration
│   ├── build_linux.sh                   # Linux build
│   └── program.sh                       # Programming script
│
└── docs/
    └── architecture.md                  # This document
```

### Build Commands

```bash
# Prerequisites
# - Libero SoC v2024.1 or later
# - SoftConsole (for E51 firmware)
# - RISC-V toolchain (for E51/U54)
# - Buildroot or Yocto (for Linux)

# Full build flow
./scripts/build_all.sh

# Individual components:

# 1. Build FPGA fabric
cd fpga_fabric
libero SCRIPT:../scripts/build_fabric.tcl

# 2. Build MSS configuration (run once)
cd mss_config
./configure_mss.sh

# 3. Build E51 firmware
cd e51_firmware
make

# 4. Build Linux image
cd u54_linux/buildroot
make polarfire_soc_icicle_defconfig
make

# 5. Program device
./scripts/program.sh --fpga bitstream.stp --hss hss.hex
```

### Simulation

```bash
# RTL simulation with Modelsim
cd tb
vsim -do ../scripts/sim.tcl

# Python reference model comparison
cd tb/python
python3 run_comparison.py \
    --rtl_output ../sv/rtl_results.csv \
    --reference ins_reference.csv

# Hardware-in-the-loop test
cd scripts
./hil_test.sh --duration 60 --imu_rate 400
```

---

## References

1. Microchip PolarFire SoC FPGA Family Datasheet (DS60001675)
2. PolarFire SoC Icicle Kit User Guide (UG0936)
3. Mi-V RISC-V Ecosystem Documentation
4. Groves, P. D. (2013). Principles of GNSS, Inertial, and Multisensor Integrated Navigation Systems
5. MAVLink Protocol Specification v2.0
6. PX4 External Vision Documentation
7. ArduPilot External Navigation Documentation
8. Analog Devices ADIS16500 Datasheet

---

## License

MIT License - See LICENSE file for details.

---

## Revision History

| Version | Date | Author | Changes |
|---------|------|--------|---------|
| 1.0 | 2026-01-22 | - | Initial PolarFire SoC architecture |
