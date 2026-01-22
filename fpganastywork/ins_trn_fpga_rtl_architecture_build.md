# INS + TRN FPGA RTL Architecture

## GPS-Denied Navigation System for Zybo Z7-20

**Target Platform:** Xilinx Zynq-7020 (Zybo Z7-20)  
**Design Language:** SystemVerilog / Verilog  
**Clock Domain:** 100 MHz system clock, 200 MHz DSP clock  
**Interface:** AXI4-Lite to PS, SPI to sensors

---

## Table of Contents

1. [System Overview](#system-overview)
2. [Top-Level Block Diagram](#top-level-block-diagram)
3. [Module Hierarchy](#module-hierarchy)
4. [Clock and Reset Architecture](#clock-and-reset-architecture)
5. [Module Descriptions](#module-descriptions)
   - [IMU Interface](#imu-interface)
   - [INS Mechanization](#ins-mechanization)
   - [Covariance Propagation](#covariance-propagation)
   - [Altitude Profile Buffer](#altitude-profile-buffer)
   - [Kalman Update Engine](#kalman-update-engine)
6. [Data Flow](#data-flow)
7. [Memory Architecture](#memory-architecture)
8. [AXI Register Map](#axi-register-map)
9. [Timing Budgets](#timing-budgets)
10. [Resource Estimates](#resource-estimates)

---

## System Overview

This RTL implements the real-time components of an error-state Kalman filter for INS/TRN fusion. The FPGA handles high-rate sensor processing and state propagation, while the Raspberry Pi 5 handles terrain correlation and filter updates.

### Design Philosophy

- **Deterministic timing**: Fixed-latency pipelines for sensor processing
- **Numerical precision**: 32-bit IEEE 754 single-precision floating point
- **Modular architecture**: Each functional block is independently testable
- **PS/PL partitioning**: FPGA handles rate-critical math; ARM handles configuration and TRN

---

## Top-Level Block Diagram

```
┌─────────────────────────────────────────────────────────────────────────────────────┐
│                              ZYNQ PROCESSING SYSTEM (PS)                            │
│                                                                                     │
│   ┌─────────────┐     ┌─────────────┐     ┌─────────────────────────────────────┐   │
│   │  ARM Core 0 │     │  ARM Core 1 │     │           DDR3 Memory               │   │
│   │  (Config)   │     │  (Monitor)  │     │  - Covariance matrix backup         │   │
│   └──────┬──────┘     └─────────────┘     │  - Altitude profile history         │   │
│          │                                │  - DTED cache (managed by Pi)       │   │
│          │ AXI4-Lite                      └─────────────────────────────────────┘   │
└──────────┼──────────────────────────────────────────────────────────────────────────┘
           │
           │ AXI Interconnect
           │
┌──────────┴──────────────────────────────────────────────────────────────────────────┐
│                            PROGRAMMABLE LOGIC (PL)                                  │
│                                                                                     │
│  ┌────────────────────────────────────────────────────────────────────────────────┐ │
│  │                            ins_trn_top                                         │ │
│  │                                                                                │ │
│  │   ┌─────────────┐    ┌─────────────┐    ┌─────────────┐    ┌──────────────┐   │ │
│  │   │    SPI      │    │    IMU      │    │    INS      │    │  Covariance  │   │ │
│  │   │  Master     ├───►│  Interface  ├───►│   Mech      ├───►│  Propagation │   │ │
│  │   │             │    │             │    │             │    │              │   │ │
│  │   └──────┬──────┘    └─────────────┘    └──────┬──────┘    └──────┬───────┘   │ │
│  │          │                                     │                  │           │ │
│  │          │                                     ▼                  ▼           │ │
│  │   ┌──────┴──────┐    ┌─────────────┐    ┌─────────────┐    ┌──────────────┐   │ │
│  │   │    SPI      │    │   Baro      │    │   State     │    │   P Matrix   │   │ │
│  │   │  Master     ├───►│  Interface  ├───►│   RAM       │    │   BRAM       │   │ │
│  │   │  (Baro)     │    │             │    │  (15x1)     │    │  (15x15)     │   │ │
│  │   └─────────────┘    └─────────────┘    └──────┬──────┘    └──────┬───────┘   │ │
│  │                                                │                  │           │ │
│  │   ┌─────────────┐    ┌─────────────┐          │                  │           │ │
│  │   │   LiDAR     │    │  Altitude   │          ▼                  ▼           │ │
│  │   │  UART/I2C   ├───►│  Profile    │    ┌───────────────────────────────┐    │ │
│  │   │  Interface  │    │  Buffer     │    │       Kalman Update           │    │ │
│  │   └─────────────┘    └──────┬──────┘    │         Engine                │    │ │
│  │                             │           │  (triggered by PS on TRN fix) │    │ │
│  │                             │           └───────────────────────────────┘    │ │
│  │                             │                          │                     │ │
│  │                             ▼                          ▼                     │ │
│  │                      ┌─────────────┐          ┌──────────────┐               │ │
│  │                      │  To Pi 5    │          │  Corrected   │               │ │
│  │                      │  (via PS)   │          │  State Out   │               │ │
│  │                      └─────────────┘          └──────────────┘               │ │
│  │                                                                              │ │
│  └──────────────────────────────────────────────────────────────────────────────┘ │
│                                                                                    │
└────────────────────────────────────────────────────────────────────────────────────┘

External Interfaces:
                                    
    ┌─────────┐      ┌─────────┐      ┌─────────┐
    │ADIS16500│      │ BMP390  │      │ TF-Luna │
    │  IMU    │      │  Baro   │      │  LiDAR  │
    └────┬────┘      └────┬────┘      └────┬────┘
         │                │                │
         │ SPI            │ SPI/I2C        │ UART
         │                │                │
    ─────┴────────────────┴────────────────┴─────────► PMOD Connectors
```

---

## Module Hierarchy

```
ins_trn_top
├── clk_rst_gen                    # Clock wizard + reset synchronizers
│
├── axi_reg_bank                   # PS interface registers
│   └── reg_file (120 registers)
│
├── sensor_subsystem
│   ├── spi_master_imu             # ADIS16500 interface
│   │   ├── spi_phy
│   │   └── burst_controller
│   │
│   ├── spi_master_baro            # BMP390 interface
│   │   ├── spi_phy
│   │   └── compensation_engine    # Temperature/pressure compensation
│   │
│   ├── uart_rx_lidar              # TF-Luna interface
│   │   └── frame_parser
│   │
│   └── imu_calibration            # Bias subtraction, scale factor
│       ├── bias_ram
│       └── sf_multiplier
│
├── ins_mechanization
│   ├── attitude_update            # Quaternion integration
│   │   ├── quat_multiply
│   │   └── quat_normalize
│   │
│   ├── dcm_from_quat              # Direction cosine matrix
│   │
│   ├── specific_force_nav         # Transform accel to nav frame
│   │   └── matrix_vector_mult
│   │
│   ├── gravity_model              # WGS84 gravity calculation
│   │
│   └── nav_integration            # Velocity and position update
│       ├── velocity_integrator
│       └── position_integrator
│
├── error_state_propagation
│   ├── phi_matrix_gen             # State transition matrix
│   │   └── f_matrix_continuous
│   │
│   ├── q_matrix_gen               # Process noise matrix
│   │
│   └── covariance_propagator      # P = Φ*P*Φ' + Q
│       ├── matrix_multiply_15x15
│       ├── matrix_transpose
│       └── matrix_add
│
├── altitude_profile_manager
│   ├── profile_fifo               # Circular buffer for terrain samples
│   ├── timestamp_correlator       # Match samples to INS positions
│   └── dma_engine                 # Burst transfer to PS/DDR
│
├── kalman_update_engine
│   ├── innovation_calc            # z - H*x
│   ├── kalman_gain_calc           # K = P*H'*(H*P*H'+R)^-1
│   │   ├── matrix_multiply
│   │   └── matrix_inverse_2x2
│   ├── state_update               # x = x + K*(z - H*x)
│   └── covariance_update          # P = (I - K*H)*P
│
├── state_memory
│   ├── ins_state_ram              # Position, velocity, attitude
│   ├── error_state_ram            # 15-element error state
│   └── p_matrix_bram              # 15x15 covariance (120 unique)
│
└── output_interface
    ├── nav_output_formatter       # Pack state for readout
    └── interrupt_gen              # Signal PS on events
```

---

## Clock and Reset Architecture

```
                    ┌────────────────────────────────────┐
   External         │          clk_rst_gen               │
   100 MHz ────────►│                                    │
                    │   ┌─────────┐    ┌─────────────┐   │
                    │   │  MMCM   │    │   Reset     │   │
                    │   │         ├───►│   Sync      │   │
                    │   └────┬────┘    └──────┬──────┘   │
                    │        │                │          │
                    └────────┼────────────────┼──────────┘
                             │                │
              ┌──────────────┼────────────────┼──────────────┐
              │              │                │              │
              ▼              ▼                ▼              ▼
         ┌────────┐    ┌────────┐       ┌────────┐    ┌────────┐
         │clk_sys │    │clk_dsp │       │rst_sys │    │rst_dsp │
         │ 100MHz │    │ 200MHz │       │ (sync) │    │ (sync) │
         └────────┘    └────────┘       └────────┘    └────────┘
              │              │
              │              └──────────────────────────────┐
              │                                             │
    ┌─────────┴──────────────────────┐         ┌───────────┴────────────┐
    │      100 MHz Domain            │         │     200 MHz Domain     │
    │                                │         │                        │
    │  - AXI interfaces              │         │  - Matrix multiplies   │
    │  - Sensor interfaces           │         │  - Covariance prop     │
    │  - State machines              │         │  - Trig functions      │
    │  - Control logic               │         │  - FPU pipelines       │
    │                                │         │                        │
    └────────────────────────────────┘         └────────────────────────┘
```

### Reset Sequencing

```
Power-on
    │
    ▼
┌─────────────────┐
│ MMCM Lock Wait  │  (wait for clock stable)
│   (~1 ms)       │
└────────┬────────┘
         │
         ▼
┌─────────────────┐
│ Sync Reset      │  (8-cycle pulse, synchronized)
│ Assert          │
└────────┬────────┘
         │
         ▼
┌─────────────────┐
│ BRAM Init       │  (clear state memory)
│                 │
└────────┬────────┘
         │
         ▼
┌─────────────────┐
│ Sensor Config   │  (SPI writes to IMU/Baro)
│                 │
└────────┬────────┘
         │
         ▼
┌─────────────────┐
│ Operational     │
│                 │
└─────────────────┘
```

---

## Module Descriptions

### IMU Interface

**Module:** `spi_master_imu`  
**Purpose:** Read ADIS16500 at 400 Hz via burst mode  
**Clock:** 100 MHz (SPI clock derived: 2 MHz)

```
                         spi_master_imu
    ┌────────────────────────────────────────────────────────┐
    │                                                        │
    │   ┌──────────┐    ┌──────────────┐    ┌────────────┐  │
    │   │  Burst   │    │   SPI PHY    │    │   Output   │  │
 ───┼──►│  Control ├───►│              ├───►│   FIFO     ├──┼──► imu_data_valid
    │   │  FSM     │    │  CPOL=1      │    │            │  │    imu_gyro[2:0]
    │   │          │    │  CPHA=1      │    │  (16-deep) │  │    imu_accel[2:0]
    │   └──────────┘    │              │    └────────────┘  │
    │        │          └──────────────┘                    │
    │        │                 │                            │
    │        ▼                 ▼                            │
    │   ┌──────────┐    ┌──────────────┐                    │
    │   │  Timer   │    │   SPI Pins   │                    │
    │   │  (2.5ms) │    │  SCLK,MOSI   │                    │
    │   │          │    │  MISO,CS_N   │                    │
    │   └──────────┘    └──────────────┘                    │
    │                                                        │
    └────────────────────────────────────────────────────────┘
```

**Burst Read Sequence (ADIS16500):**

```
CS_N  ▔▔▔▔▔▔▔▔▔▔▔▔▔▔▔▔▔▔▔▔▔▔▔▔▔▔▔▔▔▔▔▔▔▔▔▔
              ▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁

SCLK  ────────────────────────────────────────
              ▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁
              │  16   │  16   │  16   │  16   │ ...
              │ bits  │ bits  │ bits  │ bits  │

MOSI  ────────[CMD]───[X]─────[X]─────[X]─────
               0x6800 (burst read command)

MISO  ────────[X]─────[DIAG]──[GX]────[GY]────[GZ]──[AX]──[AY]──[AZ]──
                      │       │       │       │     │     │     │
                      ▼       ▼       ▼       ▼     ▼     ▼     ▼
                    Status  GyroX  GyroY  GyroZ  AccX  AccY  AccZ
                    (16b)  (16b)  (16b)  (16b)  (16b) (16b) (16b)

Timeline:  ──────────────────────────────────────────────────────────►
           │◄─────── ~56 µs burst read ────────►│◄── 2.44 ms ──►│
           │                                     │   (idle)      │
           ├─────────────── 2.5 ms period (400 Hz) ─────────────┤
```

**State Machine:**

```
              ┌─────────────────────────────────────────────────┐
              │                                                 │
              ▼                                                 │
         ┌─────────┐    timer_2p5ms    ┌─────────┐             │
    ────►│  IDLE   ├──────────────────►│  ASSERT │             │
         │         │                   │   CS    │             │
         └─────────┘                   └────┬────┘             │
                                            │                  │
                                            │ cs_setup_done    │
                                            ▼                  │
                                       ┌─────────┐             │
                                       │  SEND   │             │
                                       │   CMD   │             │
                                       └────┬────┘             │
                                            │                  │
                                            │ cmd_done         │
                                            ▼                  │
                                       ┌─────────┐             │
                                       │  READ   │◄────┐       │
                                       │  WORD   ├─────┘       │
                                       └────┬────┘  word_count │
                                            │       < 7        │
                                            │                  │
                                            │ word_count == 7  │
                                            ▼                  │
                                       ┌─────────┐             │
                                       │ RELEASE │             │
                                       │   CS    ├─────────────┘
                                       └─────────┘
```

**Port List:**

```systemverilog
module spi_master_imu (
    // Clock and reset
    input  logic        clk,
    input  logic        rst_n,
    
    // SPI physical interface
    output logic        spi_sclk,
    output logic        spi_mosi,
    input  logic        spi_miso,
    output logic        spi_cs_n,
    
    // Configuration (from AXI registers)
    input  logic [7:0]  cfg_spi_div,        // Clock divider
    input  logic        cfg_enable,          // Enable sampling
    
    // Output data
    output logic        data_valid,          // Pulse on new sample
    output logic [15:0] gyro_x,              // Raw gyro X (LSB = 0.00625 °/s)
    output logic [15:0] gyro_y,
    output logic [15:0] gyro_z,
    output logic [15:0] accel_x,             // Raw accel X (LSB = 1.25 mg)
    output logic [15:0] accel_y,
    output logic [15:0] accel_z,
    output logic [15:0] temp,                // Die temperature
    output logic [15:0] status,              // Diagnostic status
    
    // Status
    output logic        sensor_error         // Checksum or comm error
);
```

---

### INS Mechanization

**Module:** `ins_mechanization`  
**Purpose:** Propagate navigation state from IMU measurements  
**Clock:** 200 MHz (DSP clock for floating-point operations)

```
                           ins_mechanization
    ┌──────────────────────────────────────────────────────────────────┐
    │                                                                  │
    │  ┌─────────────────────────────────────────────────────────────┐ │
    │  │                    Attitude Update                          │ │
    │  │                                                             │ │
    │  │   gyro ──►[bias sub]──►[Δθ calc]──►[quat mult]──►[normalize]│ │
    │  │              │                          │              │    │ │
    │  │              ▼                          ▼              ▼    │ │
    │  │          bias_gyro               q_body_nav        q_out   │ │
    │  │                                                             │ │
    │  └─────────────────────────────────────────────────────────────┘ │
    │                               │                                  │
    │                               ▼ q_body_nav                       │
    │  ┌─────────────────────────────────────────────────────────────┐ │
    │  │                  DCM from Quaternion                        │ │
    │  │                                                             │ │
    │  │   q ──►[q0²+q1²-q2²-q3²  2(q1q2-q0q3)    2(q1q3+q0q2)  ]   │ │
    │  │       [2(q1q2+q0q3)      q0²-q1²+q2²-q3² 2(q2q3-q0q1)  ]   │ │
    │  │       [2(q1q3-q0q2)      2(q2q3+q0q1)    q0²-q1²-q2²+q3²]   │ │
    │  │                                                             │ │
    │  └─────────────────────────────────────────────────────────────┘ │
    │                               │                                  │
    │                               ▼ C_b_n (3x3)                      │
    │  ┌─────────────────────────────────────────────────────────────┐ │
    │  │              Specific Force Navigation Frame                │ │
    │  │                                                             │ │
    │  │   accel ──►[bias sub]──►[C_b_n × f_b]──►[- gravity]──► a_n  │ │
    │  │                │              │               │             │ │
    │  │                ▼              ▼               ▼             │ │
    │  │           bias_accel      f_nav         gravity_ned         │ │
    │  │                                                             │ │
    │  └─────────────────────────────────────────────────────────────┘ │
    │                               │                                  │
    │                               ▼ a_n (nav frame accel)            │
    │  ┌─────────────────────────────────────────────────────────────┐ │
    │  │                 Navigation Integration                      │ │
    │  │                                                             │ │
    │  │   a_n ──►[∫ dt]──► v_n ──►[∫ dt]──► p_n                    │ │
    │  │            │         │       │         │                    │ │
    │  │            ▼         ▼       ▼         ▼                    │ │
    │  │         vel_prev  vel_out pos_prev  pos_out                 │ │
    │  │                                                             │ │
    │  │   Integration: Trapezoidal (2nd order)                      │ │
    │  │   v[k] = v[k-1] + (a[k] + a[k-1]) * dt/2                   │ │
    │  │   p[k] = p[k-1] + (v[k] + v[k-1]) * dt/2                   │ │
    │  │                                                             │ │
    │  └─────────────────────────────────────────────────────────────┘ │
    │                                                                  │
    └──────────────────────────────────────────────────────────────────┘
```

**Quaternion Update Detail:**

```
        Quaternion Integration (First-Order)
    ┌────────────────────────────────────────────────────────────┐
    │                                                            │
    │   Input: ω = [ωx, ωy, ωz] (corrected gyro, rad/s)         │
    │          q = [q0, q1, q2, q3] (current quaternion)         │
    │          dt = sample period                                │
    │                                                            │
    │   Step 1: Compute rotation angle                           │
    │   ┌──────────────────────────────────────────────────────┐ │
    │   │  |ω| = √(ωx² + ωy² + ωz²)                            │ │
    │   │  θ = |ω| × dt                                         │ │
    │   └──────────────────────────────────────────────────────┘ │
    │                                                            │
    │   Step 2: Compute delta quaternion                         │
    │   ┌──────────────────────────────────────────────────────┐ │
    │   │  if |ω| > ε:                                         │ │
    │   │      Δq0 = cos(θ/2)                                   │ │
    │   │      Δq1 = (ωx/|ω|) × sin(θ/2)                       │ │
    │   │      Δq2 = (ωy/|ω|) × sin(θ/2)                       │ │
    │   │      Δq3 = (ωz/|ω|) × sin(θ/2)                       │ │
    │   │  else:                                                │ │
    │   │      Δq = [1, ωx×dt/2, ωy×dt/2, ωz×dt/2]             │ │
    │   └──────────────────────────────────────────────────────┘ │
    │                                                            │
    │   Step 3: Quaternion multiplication                        │
    │   ┌──────────────────────────────────────────────────────┐ │
    │   │  q' = q ⊗ Δq                                         │ │
    │   │                                                       │ │
    │   │  q'0 = q0×Δq0 - q1×Δq1 - q2×Δq2 - q3×Δq3            │ │
    │   │  q'1 = q0×Δq1 + q1×Δq0 + q2×Δq3 - q3×Δq2            │ │
    │   │  q'2 = q0×Δq2 - q1×Δq3 + q2×Δq0 + q3×Δq1            │ │
    │   │  q'3 = q0×Δq3 + q1×Δq2 - q2×Δq1 + q3×Δq0            │ │
    │   └──────────────────────────────────────────────────────┘ │
    │                                                            │
    │   Step 4: Normalize                                        │
    │   ┌──────────────────────────────────────────────────────┐ │
    │   │  |q'| = √(q'0² + q'1² + q'2² + q'3²)                 │ │
    │   │  q_out = q' / |q'|                                    │ │
    │   └──────────────────────────────────────────────────────┘ │
    │                                                            │
    └────────────────────────────────────────────────────────────┘
```

**Pipeline Structure:**

```
    Cycle:   0    1    2    3    4    5    6    7    8    9   10   11
            ─────────────────────────────────────────────────────────────
    Stage 1: │ ω² │    │    │    │    │    │    │    │    │    │    │
    (mag²)   │calc│    │    │    │    │    │    │    │    │    │    │
            ─────────────────────────────────────────────────────────────
    Stage 2:      │√   │    │    │    │    │    │    │    │    │    │
    (sqrt)        │sum │    │    │    │    │    │    │    │    │    │
            ─────────────────────────────────────────────────────────────
    Stage 3:           │sin │    │    │    │    │    │    │    │    │
    (trig)             │cos │    │    │    │    │    │    │    │    │
            ─────────────────────────────────────────────────────────────
    Stage 4:                │Δq  │    │    │    │    │    │    │    │
    (delta q)               │form│    │    │    │    │    │    │    │
            ─────────────────────────────────────────────────────────────
    Stage 5:                     │q⊗Δq│    │    │    │    │    │    │
    (quat mult)                  │    │    │    │    │    │    │    │
            ─────────────────────────────────────────────────────────────
    Stage 6:                          │norm│    │    │    │    │    │
    (normalize)                       │    │    │    │    │    │    │
            ─────────────────────────────────────────────────────────────
    Stage 7:                               │DCM │    │    │    │    │
    (C_b_n)                                │calc│    │    │    │    │
            ─────────────────────────────────────────────────────────────
    Stage 8:                                    │C×f │    │    │    │
    (rotate)                                    │    │    │    │    │
            ─────────────────────────────────────────────────────────────
    Stage 9:                                         │-g  │    │    │
    (gravity)                                        │    │    │    │
            ─────────────────────────────────────────────────────────────
    Stage 10:                                             │∫v  │    │
    (vel int)                                             │    │    │
            ─────────────────────────────────────────────────────────────
    Stage 11:                                                  │∫p  │
    (pos int)                                                  │    │
            ─────────────────────────────────────────────────────────────
    
    Total latency: 11 cycles @ 200 MHz = 55 ns
    Throughput: 1 sample per cycle after pipeline fills
```

---

### Covariance Propagation

**Module:** `covariance_propagator`  
**Purpose:** Propagate error covariance matrix P = Φ*P*Φ' + Q  
**Clock:** 200 MHz

```
                        covariance_propagator
    ┌────────────────────────────────────────────────────────────────────┐
    │                                                                    │
    │   ┌─────────────────────────────────────────────────────────────┐  │
    │   │               Φ Matrix Generator                            │  │
    │   │                                                             │  │
    │   │   INS state ──►[F matrix]──►[I + F×dt]──► Φ (15×15)        │  │
    │   │                    │                                        │  │
    │   │                    ▼                                        │  │
    │   │              ┌─────────────────────────────────┐            │  │
    │   │              │         F Matrix                │            │  │
    │   │              │  ┌─                         ─┐  │            │  │
    │   │              │  │ 0    I    0    0    0    │  │            │  │
    │   │              │  │ 0    0  -[g×]  Cbn  0    │  │            │  │
    │   │              │  │ 0    0    0    0   -Cbn  │  │            │  │
    │   │              │  │ 0    0    0  -1/τa  0    │  │            │  │
    │   │              │  │ 0    0    0    0  -1/τg  │  │            │  │
    │   │              │  └─                         ─┘  │            │  │
    │   │              └─────────────────────────────────┘            │  │
    │   └─────────────────────────────────────────────────────────────┘  │
    │                               │                                    │
    │                               ▼ Φ                                  │
    │   ┌─────────────────────────────────────────────────────────────┐  │
    │   │              Matrix Multiplication Engine                   │  │
    │   │                                                             │  │
    │   │         ┌─────────┐      ┌─────────┐      ┌─────────┐      │  │
    │   │   P ───►│  P × Φ' │─────►│ Φ × ... │─────►│  + Q    │──► P'│  │
    │   │         │         │      │         │      │         │      │  │
    │   │         └─────────┘      └─────────┘      └─────────┘      │  │
    │   │              │                │                │            │  │
    │   │              ▼                ▼                ▼            │  │
    │   │         temp1 (15×15)   temp2 (15×15)    P_new (15×15)     │  │
    │   │                                                             │  │
    │   └─────────────────────────────────────────────────────────────┘  │
    │                                                                    │
    │   Memory: P stored as upper triangle (120 elements)                │
    │   Computation: Exploits symmetry to reduce operations by ~50%      │
    │                                                                    │
    └────────────────────────────────────────────────────────────────────┘
```

**Matrix Multiply Architecture:**

```
        Systolic Array for 15×15 Matrix Multiply
    ┌──────────────────────────────────────────────────────────────────┐
    │                                                                  │
    │   A matrix elements flow ───►                                    │
    │                                                                  │
    │   ┌─────┐  ┌─────┐  ┌─────┐  ┌─────┐       ┌─────┐             │
    │   │ PE  │──│ PE  │──│ PE  │──│ PE  │─ ─ ─ ─│ PE  │             │
    │   │ 0,0 │  │ 0,1 │  │ 0,2 │  │ 0,3 │       │ 0,14│             │
    │   └──┬──┘  └──┬──┘  └──┬──┘  └──┬──┘       └──┬──┘             │
    │      │        │        │        │             │                 │
    │   ┌──▼──┐  ┌──▼──┐  ┌──▼──┐  ┌──▼──┐       ┌──▼──┐             │
    │   │ PE  │──│ PE  │──│ PE  │──│ PE  │─ ─ ─ ─│ PE  │             │
    │   │ 1,0 │  │ 1,1 │  │ 1,2 │  │ 1,3 │       │ 1,14│             │
    │   └──┬──┘  └──┬──┘  └──┬──┘  └──┬──┘       └──┬──┘             │
    │      │        │        │        │             │                 │
    │      ▼        ▼        ▼        ▼             ▼                 │
    │      :        :        :        :             :                 │
    │      │        │        │        │             │                 │
    │   ┌──▼──┐  ┌──▼──┐  ┌──▼──┐  ┌──▼──┐       ┌──▼──┐             │
    │   │ PE  │──│ PE  │──│ PE  │──│ PE  │─ ─ ─ ─│ PE  │             │
    │   │14,0 │  │14,1 │  │14,2 │  │14,3 │       │14,14│             │
    │   └─────┘  └─────┘  └─────┘  └─────┘       └─────┘             │
    │                                                                  │
    │   ▲                                                              │
    │   │                                                              │
    │   B matrix elements flow                                         │
    │                                                                  │
    │   Each PE: c_out = c_in + a × b                                 │
    │   Pipeline depth: 15 + 15 - 1 = 29 cycles                       │
    │                                                                  │
    └──────────────────────────────────────────────────────────────────┘

    Processing Element (PE):
    ┌─────────────────────────────────────────┐
    │                     a_in ───┐           │
    │                             │           │
    │   b_in ──►[reg]──►[×]◄─────┘           │
    │              │      │                   │
    │              │      ▼                   │
    │              │   [+]◄── c_in            │
    │              │      │                   │
    │              ▼      ▼                   │
    │           b_out   c_out                 │
    │                                         │
    │   a_out ◄─────────[reg]◄── a_in        │
    │                                         │
    └─────────────────────────────────────────┘
```

**Sparse Φ Optimization:**

The state transition matrix Φ has significant structure that can be exploited:

```
    Φ matrix sparsity pattern (15×15):
    
           pos   vel   att   ba    bg
          [0:2] [3:5] [6:8] [9:11][12:14]
         ┌─────┬─────┬─────┬─────┬─────┐
    pos  │  I  │ I×dt│  0  │  0  │  0  │  [0:2]
         ├─────┼─────┼─────┼─────┼─────┤
    vel  │  0  │  I  │ G×dt│ C×dt│  0  │  [3:5]
         ├─────┼─────┼─────┼─────┼─────┤
    att  │  0  │  0  │  I  │  0  │-C×dt│  [6:8]
         ├─────┼─────┼─────┼─────┼─────┤
    ba   │  0  │  0  │  0  │  D  │  0  │  [9:11]
         ├─────┼─────┼─────┼─────┼─────┤
    bg   │  0  │  0  │  0  │  0  │  D  │  [12:14]
         └─────┴─────┴─────┴─────┴─────┘
    
    I = Identity (3×3)
    G = Gravity coupling -[g×] (skew-symmetric)
    C = DCM C_b_n (rotation matrix)
    D = Diagonal exp(-dt/τ)
    
    Non-zero 3×3 blocks: 9 out of 25
    → 64% sparsity can be exploited
```

---

### Altitude Profile Buffer

**Module:** `altitude_profile_manager`  
**Purpose:** Collect terrain elevation samples for TRN correlation  
**Clock:** 100 MHz

```
                      altitude_profile_manager
    ┌──────────────────────────────────────────────────────────────────┐
    │                                                                  │
    │  Inputs:                                                         │
    │  ┌──────────────┐  ┌──────────────┐  ┌──────────────┐           │
    │  │ baro_alt     │  │ lidar_agl    │  │ ins_position │           │
    │  │ (MSL)        │  │ (AGL)        │  │ (NED)        │           │
    │  └──────┬───────┘  └──────┬───────┘  └──────┬───────┘           │
    │         │                 │                 │                    │
    │         ▼                 ▼                 ▼                    │
    │  ┌─────────────────────────────────────────────────────────────┐ │
    │  │              Terrain Elevation Calculator                   │ │
    │  │                                                             │ │
    │  │   h_terrain = h_baro - h_agl                                │ │
    │  │                                                             │ │
    │  │   validity check:                                           │ │
    │  │   - baro valid                                              │ │
    │  │   - lidar valid (in range, good SNR)                        │ │
    │  │   - altitude change > threshold (moving)                    │ │
    │  │                                                             │ │
    │  └────────────────────────────┬────────────────────────────────┘ │
    │                               │                                  │
    │                               ▼                                  │
    │  ┌─────────────────────────────────────────────────────────────┐ │
    │  │                   Profile FIFO (Circular)                   │ │
    │  │                                                             │ │
    │  │   Depth: 64 samples                                         │ │
    │  │   Width: 128 bits per entry                                 │ │
    │  │                                                             │ │
    │  │   Entry format:                                             │ │
    │  │   ┌────────┬────────┬────────┬────────┬────────┬─────────┐ │ │
    │  │   │ h_terr │ pos_n  │ pos_e  │ time   │ valid  │ reserved│ │ │
    │  │   │ 32-bit │ 32-bit │ 32-bit │ 24-bit │ 1-bit  │  7-bit  │ │ │
    │  │   └────────┴────────┴────────┴────────┴────────┴─────────┘ │ │
    │  │                                                             │ │
    │  │   write_ptr ──────────────────────────────┐                 │ │
    │  │                                           ▼                 │ │
    │  │   ┌────┬────┬────┬────┬────┬────┬────┬────┬────┐           │ │
    │  │   │ 0  │ 1  │ 2  │ 3  │ 4  │... │ 61 │ 62 │ 63 │           │ │
    │  │   └────┴────┴────┴────┴────┴────┴────┴────┴────┘           │ │
    │  │   ▲                                                         │ │
    │  │   └────────────────────────────── read_ptr                  │ │
    │  │                                                             │ │
    │  └────────────────────────────┬────────────────────────────────┘ │
    │                               │                                  │
    │                               ▼                                  │
    │  ┌─────────────────────────────────────────────────────────────┐ │
    │  │                    DMA Engine                               │ │
    │  │                                                             │ │
    │  │   Triggered by PS when TRN correlation requested            │ │
    │  │   Burst transfers profile to DDR for Pi 5 access            │ │
    │  │                                                             │ │
    │  │   AXI-Stream interface to DMA controller                    │ │
    │  │                                                             │ │
    │  └─────────────────────────────────────────────────────────────┘ │
    │                                                                  │
    └──────────────────────────────────────────────────────────────────┘
```

---

### Kalman Update Engine

**Module:** `kalman_update_engine`  
**Purpose:** Apply TRN position fix to error state and covariance  
**Clock:** 200 MHz  
**Triggered:** By PS after TRN correlation completes

```
                         kalman_update_engine
    ┌──────────────────────────────────────────────────────────────────┐
    │                                                                  │
    │   Inputs from PS (via AXI):                                      │
    │   ┌─────────────┐  ┌─────────────┐  ┌─────────────┐             │
    │   │ trn_pos_n   │  │ trn_pos_e   │  │ trn_cov     │             │
    │   │ (fix north) │  │ (fix east)  │  │ (R matrix)  │             │
    │   └──────┬──────┘  └──────┬──────┘  └──────┬──────┘             │
    │          │                │                │                     │
    │          └────────────────┼────────────────┘                     │
    │                           │                                      │
    │                           ▼                                      │
    │   ┌─────────────────────────────────────────────────────────────┐│
    │   │              Step 1: Innovation Calculation                 ││
    │   │                                                             ││
    │   │   z = [trn_pos_n - ins_pos_n]   (measurement residual)     ││
    │   │       [trn_pos_e - ins_pos_e]                              ││
    │   │                                                             ││
    │   │   (Since error state δx ≈ 0, innovation ≈ z - H×0 = z)     ││
    │   │                                                             ││
    │   └────────────────────────────┬────────────────────────────────┘│
    │                                │                                 │
    │                                ▼                                 │
    │   ┌─────────────────────────────────────────────────────────────┐│
    │   │              Step 2: Kalman Gain Calculation                ││
    │   │                                                             ││
    │   │   H = [I₂ₓ₂  0₂ₓ₃  0₂ₓ₃  0₂ₓ₃  0₂ₓ₃]  (2×15 matrix)       ││
    │   │                                                             ││
    │   │   S = H × P × H' + R     (2×2 innovation covariance)        ││
    │   │     = P[0:1, 0:1] + R    (extract upper-left 2×2 of P)      ││
    │   │                                                             ││
    │   │   K = P × H' × S⁻¹       (15×2 Kalman gain)                 ││
    │   │     = P[:, 0:1] × S⁻¹    (first 2 columns of P × S⁻¹)      ││
    │   │                                                             ││
    │   │   ┌─────────────────────────────────────────────────────┐   ││
    │   │   │  2×2 Matrix Inverse (direct formula)                │   ││
    │   │   │                                                     │   ││
    │   │   │  S⁻¹ = 1/(s11×s22 - s12×s21) × [ s22  -s12]        │   ││
    │   │   │                                  [-s21   s11]        │   ││
    │   │   │                                                     │   ││
    │   │   └─────────────────────────────────────────────────────┘   ││
    │   │                                                             ││
    │   └────────────────────────────┬────────────────────────────────┘│
    │                                │                                 │
    │                                ▼                                 │
    │   ┌─────────────────────────────────────────────────────────────┐│
    │   │              Step 3: State Update                           ││
    │   │                                                             ││
    │   │   δx_new = K × z         (15×1 error state correction)      ││
    │   │                                                             ││
    │   │   Apply to INS state:                                       ││
    │   │   pos_ins  ← pos_ins  - δx_new[0:2]                        ││
    │   │   vel_ins  ← vel_ins  - δx_new[3:5]                        ││
    │   │   att_ins  ← att_ins  ⊗ δq(δx_new[6:8])                    ││
    │   │   bias_a   ← bias_a   - δx_new[9:11]                       ││
    │   │   bias_g   ← bias_g   - δx_new[12:14]                      ││
    │   │                                                             ││
    │   │   Reset error state:                                        ││
    │   │   δx ← 0                                                    ││
    │   │                                                             ││
    │   └────────────────────────────┬────────────────────────────────┘│
    │                                │                                 │
    │                                ▼                                 │
    │   ┌─────────────────────────────────────────────────────────────┐│
    │   │              Step 4: Covariance Update                      ││
    │   │                                                             ││
    │   │   Method: Joseph form (numerically stable)                  ││
    │   │                                                             ││
    │   │   P_new = (I - K×H) × P × (I - K×H)' + K × R × K'          ││
    │   │                                                             ││
    │   │   Simplified (since H is sparse):                           ││
    │   │   P_new = P - K × S × K'                                    ││
    │   │                                                             ││
    │   └─────────────────────────────────────────────────────────────┘│
    │                                                                  │
    │   Total cycles: ~450 @ 200 MHz = 2.25 µs                        │
    │                                                                  │
    └──────────────────────────────────────────────────────────────────┘
```

**Update Sequencing:**

```
    PS Trigger
        │
        ▼
    ┌─────────┐     ┌─────────┐     ┌─────────┐     ┌─────────┐
    │ Load z  │────►│Compute S│────►│Invert S │────►│Compute K│
    │ (2 cyc) │     │(~20 cyc)│     │(~15 cyc)│     │(~150cyc)│
    └─────────┘     └─────────┘     └─────────┘     └─────────┘
                                                         │
        ┌────────────────────────────────────────────────┘
        │
        ▼
    ┌─────────┐     ┌─────────┐     ┌─────────┐     ┌─────────┐
    │ δx = Kz │────►│ Update  │────►│ Update  │────►│  Done   │
    │(~30 cyc)│     │INS state│     │P matrix │     │ (IRQ)   │
    └─────────┘     │(~20 cyc)│     │(~200cyc)│     └─────────┘
                    └─────────┘     └─────────┘
```

---

## Data Flow

```
                              System Data Flow
    ┌─────────────────────────────────────────────────────────────────────────────┐
    │                                                                             │
    │   SENSORS                    FPGA                              PS / Pi 5    │
    │                                                                             │
    │   ┌─────────┐                                                               │
    │   │ADIS16500│    SPI     ┌────────────┐                                    │
    │   │  IMU    │───────────►│IMU Process │                                    │
    │   └─────────┘   2 MHz    │ + Calibrate│                                    │
    │                          └─────┬──────┘                                    │
    │                                │ 400 Hz                                     │
    │                                ▼                                            │
    │   ┌─────────┐            ┌────────────┐    ┌────────────┐                  │
    │   │ BMP390  │    SPI     │    INS     │    │ Covariance │                  │
    │   │  Baro   │───────────►│   Mech     │───►│ Propagate  │                  │
    │   └─────────┘   1 MHz    │            │    │            │                  │
    │                          └─────┬──────┘    └─────┬──────┘                  │
    │                                │                 │                          │
    │                                ▼                 ▼                          │
    │                          ┌────────────┐   ┌────────────┐                   │
    │                          │   State    │   │  P Matrix  │                   │
    │   ┌─────────┐            │    RAM     │   │   BRAM     │                   │
    │   │ TF-Luna │   UART     │ pos,vel,   │   │  (15×15)   │                   │
    │   │  LiDAR  │───────────►│ att,bias   │   │            │                   │
    │   └─────────┘  115200    └─────┬──────┘   └─────┬──────┘                   │
    │                                │                 │                          │
    │                                │                 │                          │
    │                          ┌─────▼─────────────────▼─────┐                   │
    │                          │      Kalman Update          │◄──────────────┐   │
    │                          │         Engine              │               │   │
    │                          └──────────────┬──────────────┘               │   │
    │                                         │                              │   │
    │                                         │ corrected state              │   │
    │                                         ▼                              │   │
    │                          ┌─────────────────────────────┐    ┌─────────┴──┐│
    │                          │      AXI Register Bank      │◄──►│  ARM Core  ││
    │                          │                             │    │            ││
    │                          │  - Config registers         │    │  - Init    ││
    │                          │  - State readout            │    │  - Monitor ││
    │                          │  - TRN fix input            │    │  - TRN req ││
    │                          │  - Profile DMA              │    │            ││
    │                          └──────────────┬──────────────┘    └────────────┘│
    │                                         │                        ▲        │
    │                                         │ altitude profile       │        │
    │                                         ▼                        │        │
    │                          ┌─────────────────────────────┐        │        │
    │                          │     DDR3 Memory             │        │        │
    │                          │                             │        │        │
    │                          │  - Profile buffer           │        │        │
    │                          │  - DTED cache               │◄───────┘        │
    │                          │  - State history            │    shared       │
    │                          └──────────────┬──────────────┘    memory       │
    │                                         │                        │        │
    │                                         │ SPI / USB              │        │
    │                                         ▼                        │        │
    │                                   ┌───────────┐          ┌───────┴──────┐ │
    │                                   │   Pi 5    │◄────────►│    DTED      │ │
    │                                   │           │          │   Storage    │ │
    │                                   │  - TRN    │          │   (SD card)  │ │
    │                                   │  - SLAM   │          └──────────────┘ │
    │                                   │  - Path   │                           │
    │                                   └───────────┘                           │
    │                                                                           │
    └───────────────────────────────────────────────────────────────────────────┘
```

---

## Memory Architecture

### BRAM Allocation

```
    ┌─────────────────────────────────────────────────────────────────┐
    │                    BRAM Usage Summary                           │
    │                                                                 │
    │   Total available (Z7-20): 140 × 36Kb = 630 KB                 │
    │                                                                 │
    │   ┌─────────────────────────────────────────────────────────┐  │
    │   │  INS State RAM                                          │  │
    │   │  ├── Position (N,E,D)      3 × 32b =   96 bits          │  │
    │   │  ├── Velocity (N,E,D)      3 × 32b =   96 bits          │  │
    │   │  ├── Quaternion (4)        4 × 32b =  128 bits          │  │
    │   │  ├── Accel bias (3)        3 × 32b =   96 bits          │  │
    │   │  └── Gyro bias (3)         3 × 32b =   96 bits          │  │
    │   │                            ────────────────────          │  │
    │   │                            Total:      512 bits          │  │
    │   │                            Depth:      2 (double buffer) │  │
    │   │                            BRAM:       1 × 36Kb          │  │
    │   └─────────────────────────────────────────────────────────┘  │
    │                                                                 │
    │   ┌─────────────────────────────────────────────────────────┐  │
    │   │  Error State RAM                                        │  │
    │   │  └── δx (15 elements)     15 × 32b =  480 bits          │  │
    │   │                            Depth:      2                 │  │
    │   │                            BRAM:       1 × 36Kb          │  │
    │   └─────────────────────────────────────────────────────────┘  │
    │                                                                 │
    │   ┌─────────────────────────────────────────────────────────┐  │
    │   │  P Matrix BRAM (symmetric, upper triangle)              │  │
    │   │  └── 120 unique elements  120 × 32b = 3840 bits         │  │
    │   │                            Depth:      2                 │  │
    │   │                            BRAM:       2 × 36Kb          │  │
    │   └─────────────────────────────────────────────────────────┘  │
    │                                                                 │
    │   ┌─────────────────────────────────────────────────────────┐  │
    │   │  Φ Matrix Scratchpad                                    │  │
    │   │  └── 225 elements (15×15) 225 × 32b = 7200 bits         │  │
    │   │                            BRAM:       2 × 36Kb          │  │
    │   └─────────────────────────────────────────────────────────┘  │
    │                                                                 │
    │   ┌─────────────────────────────────────────────────────────┐  │
    │   │  Q Matrix ROM                                           │  │
    │   │  └── 15 diagonal elements  15 × 32b =  480 bits         │  │
    │   │                            BRAM:       1 × 36Kb          │  │
    │   └─────────────────────────────────────────────────────────┘  │
    │                                                                 │
    │   ┌─────────────────────────────────────────────────────────┐  │
    │   │  Altitude Profile FIFO                                  │  │
    │   │  └── 64 entries × 128b    64 × 128b = 8192 bits         │  │
    │   │                            BRAM:       2 × 36Kb          │  │
    │   └─────────────────────────────────────────────────────────┘  │
    │                                                                 │
    │   ┌─────────────────────────────────────────────────────────┐  │
    │   │  IMU Calibration Parameters                             │  │
    │   │  ├── Gyro bias (3)         3 × 32b                      │  │
    │   │  ├── Gyro scale (3)        3 × 32b                      │  │
    │   │  ├── Accel bias (3)        3 × 32b                      │  │
    │   │  ├── Accel scale (3)       3 × 32b                      │  │
    │   │  └── Misalignment (9)      9 × 32b                      │  │
    │   │                            BRAM:       1 × 36Kb          │  │
    │   └─────────────────────────────────────────────────────────┘  │
    │                                                                 │
    │   ┌─────────────────────────────────────────────────────────┐  │
    │   │  Trig LUT (sin/cos)                                     │  │
    │   │  └── 1024 entries × 32b   1024 × 32b = 32768 bits       │  │
    │   │                            BRAM:       1 × 36Kb          │  │
    │   └─────────────────────────────────────────────────────────┘  │
    │                                                                 │
    │   Total BRAM used:  11 × 36Kb = 49.5 KB                        │
    │   Remaining:        129 × 36Kb = 580 KB (for future use)       │
    │                                                                 │
    └─────────────────────────────────────────────────────────────────┘
```

---

## AXI Register Map

```
    ┌─────────────────────────────────────────────────────────────────────────┐
    │                        AXI4-Lite Register Map                           │
    │                        Base Address: 0x4000_0000                        │
    │                                                                         │
    │   Offset   Name                    R/W    Description                   │
    │   ──────────────────────────────────────────────────────────────────    │
    │                                                                         │
    │   === Control Registers ===                                             │
    │   0x000    CTRL                    R/W    [0] enable, [1] reset         │
    │   0x004    STATUS                  R      [0] running, [1] error        │
    │   0x008    IMU_CFG                 R/W    [7:0] SPI div, [8] enable     │
    │   0x00C    BARO_CFG                R/W    [7:0] SPI div, [8] enable     │
    │   0x010    LIDAR_CFG               R/W    [7:0] baud div, [8] enable    │
    │   0x014    UPDATE_RATE             R/W    [15:0] IMU rate divider       │
    │   0x018    IRQ_ENABLE              R/W    [0] update, [1] error         │
    │   0x01C    IRQ_STATUS              R/C    [0] update, [1] error         │
    │                                                                         │
    │   === INS State Output (Read-Only) ===                                  │
    │   0x100    POS_N                   R      Position North (float32)      │
    │   0x104    POS_E                   R      Position East (float32)       │
    │   0x108    POS_D                   R      Position Down (float32)       │
    │   0x10C    VEL_N                   R      Velocity North (float32)      │
    │   0x110    VEL_E                   R      Velocity East (float32)       │
    │   0x114    VEL_D                   R      Velocity Down (float32)       │
    │   0x118    QUAT_W                  R      Quaternion W (float32)        │
    │   0x11C    QUAT_X                  R      Quaternion X (float32)        │
    │   0x120    QUAT_Y                  R      Quaternion Y (float32)        │
    │   0x124    QUAT_Z                  R      Quaternion Z (float32)        │
    │   0x128    ROLL                    R      Euler roll (float32, rad)     │
    │   0x12C    PITCH                   R      Euler pitch (float32, rad)    │
    │   0x130    YAW                     R      Euler yaw (float32, rad)      │
    │                                                                         │
    │   === Bias Estimates (Read-Only) ===                                    │
    │   0x140    BIAS_AX                 R      Accel X bias (float32)        │
    │   0x144    BIAS_AY                 R      Accel Y bias (float32)        │
    │   0x148    BIAS_AZ                 R      Accel Z bias (float32)        │
    │   0x14C    BIAS_GX                 R      Gyro X bias (float32)         │
    │   0x150    BIAS_GY                 R      Gyro Y bias (float32)         │
    │   0x154    BIAS_GZ                 R      Gyro Z bias (float32)         │
    │                                                                         │
    │   === Covariance Diagonal (Read-Only) ===                               │
    │   0x180    COV_POS                 R      Position std dev (float32, m) │
    │   0x184    COV_VEL                 R      Velocity std dev (m/s)        │
    │   0x188    COV_ATT                 R      Attitude std dev (rad)        │
    │                                                                         │
    │   === TRN Interface ===                                                 │
    │   0x200    TRN_CTRL                R/W    [0] start update              │
    │   0x204    TRN_STATUS              R      [0] busy, [1] done            │
    │   0x208    TRN_POS_N               W      TRN fix North (float32)       │
    │   0x20C    TRN_POS_E               W      TRN fix East (float32)        │
    │   0x210    TRN_COV_NN              W      TRN covariance (float32)      │
    │   0x214    TRN_COV_EE              W      TRN covariance (float32)      │
    │   0x218    TRN_COV_NE              W      TRN cross-covariance          │
    │                                                                         │
    │   === Altitude Profile DMA ===                                          │
    │   0x280    PROF_CTRL               R/W    [0] start DMA                 │
    │   0x284    PROF_STATUS             R      [0] busy, [7:0] count         │
    │   0x288    PROF_ADDR               R/W    DDR destination address       │
    │   0x28C    PROF_COUNT              R/W    Number of samples to xfer     │
    │                                                                         │
    │   === IMU Calibration (Write to apply) ===                              │
    │   0x300    CAL_GYRO_BIAS_X         R/W    Gyro X bias (float32)         │
    │   0x304    CAL_GYRO_BIAS_Y         R/W    Gyro Y bias (float32)         │
    │   0x308    CAL_GYRO_BIAS_Z         R/W    Gyro Z bias (float32)         │
    │   0x30C    CAL_ACCEL_BIAS_X        R/W    Accel X bias (float32)        │
    │   0x310    CAL_ACCEL_BIAS_Y        R/W    Accel Y bias (float32)        │
    │   0x314    CAL_ACCEL_BIAS_Z        R/W    Accel Z bias (float32)        │
    │   0x318    CAL_APPLY               W      [0] write 1 to apply cal      │
    │                                                                         │
    │   === Debug / Diagnostics ===                                           │
    │   0x3F0    RAW_GYRO_X              R      Raw gyro X (int16)            │
    │   0x3F4    RAW_GYRO_Y              R      Raw gyro Y (int16)            │
    │   0x3F8    RAW_GYRO_Z              R      Raw gyro Z (int16)            │
    │   0x3FC    RAW_ACCEL_X             R      Raw accel X (int16)           │
    │   0x400    RAW_ACCEL_Y             R      Raw accel Y (int16)           │
    │   0x404    RAW_ACCEL_Z             R      Raw accel Z (int16)           │
    │   0x408    SAMPLE_COUNT            R      Total IMU samples (uint32)    │
    │   0x40C    ERROR_COUNT             R      Communication errors (uint32) │
    │                                                                         │
    └─────────────────────────────────────────────────────────────────────────┘
```

---

## Timing Budgets

```
    ┌─────────────────────────────────────────────────────────────────────────┐
    │                         Timing Analysis                                 │
    │                                                                         │
    │   IMU Sample Period: 2.5 ms (400 Hz)                                   │
    │                                                                         │
    │   ┌─────────────────────────────────────────────────────────────────┐  │
    │   │  Operation                        Cycles    Time      Margin    │  │
    │   │  ───────────────────────────────────────────────────────────────│  │
    │   │  SPI burst read (7 words)         896       8.96 µs             │  │
    │   │  IMU calibration apply            10        0.10 µs             │  │
    │   │  INS mechanization pipeline       11        0.055 µs  *         │  │
    │   │  Covariance propagation           ~2000     10 µs     *         │  │
    │   │  ───────────────────────────────────────────────────────────────│  │
    │   │  Total per sample:                          ~20 µs              │  │
    │   │  Available time:                            2500 µs             │  │
    │   │  Utilization:                               0.8%                │  │
    │   │                                                                 │  │
    │   │  * = 200 MHz domain                                             │  │
    │   └─────────────────────────────────────────────────────────────────┘  │
    │                                                                         │
    │   TRN Update (triggered by PS, ~0.1 Hz):                               │
    │   ┌─────────────────────────────────────────────────────────────────┐  │
    │   │  Operation                        Cycles    Time                │  │
    │   │  ───────────────────────────────────────────────────────────────│  │
    │   │  Load TRN fix from registers      10        0.05 µs             │  │
    │   │  Compute innovation               5         0.025 µs            │  │
    │   │  Compute S (2×2)                  20        0.10 µs             │  │
    │   │  Invert S (2×2)                   15        0.075 µs            │  │
    │   │  Compute K (15×2)                 150       0.75 µs             │  │
    │   │  State update (δx = K×z)          30        0.15 µs             │  │
    │   │  Apply correction to INS          20        0.10 µs             │  │
    │   │  Covariance update (P -= KSK')    200       1.0 µs              │  │
    │   │  ───────────────────────────────────────────────────────────────│  │
    │   │  Total:                           ~450      ~2.25 µs            │  │
    │   └─────────────────────────────────────────────────────────────────┘  │
    │                                                                         │
    │   Profile DMA (64 samples × 128 bits):                                 │
    │   ┌─────────────────────────────────────────────────────────────────┐  │
    │   │  AXI burst @ 100 MHz, 64-bit:     128 cycles = 1.28 µs         │  │
    │   └─────────────────────────────────────────────────────────────────┘  │
    │                                                                         │
    └─────────────────────────────────────────────────────────────────────────┘
```

---

## Resource Estimates

```
    ┌─────────────────────────────────────────────────────────────────────────┐
    │                    Zynq-7020 Resource Utilization                       │
    │                                                                         │
    │   Resource        Available    Estimated    Utilization                 │
    │   ─────────────────────────────────────────────────────────────────     │
    │   LUTs            53,200       ~18,000      34%                         │
    │   Flip-Flops      106,400      ~12,000      11%                         │
    │   BRAM (36Kb)     140          11           8%                          │
    │   DSP48E1         220          48           22%                         │
    │                                                                         │
    │   ┌─────────────────────────────────────────────────────────────────┐  │
    │   │  DSP Breakdown:                                                 │  │
    │   │  ├── FP multiply units (pipelined)     24 DSPs                  │  │
    │   │  ├── FP add units                      12 DSPs                  │  │
    │   │  ├── Matrix MAC array                  8 DSPs                   │  │
    │   │  └── Misc (scaling, etc.)              4 DSPs                   │  │
    │   └─────────────────────────────────────────────────────────────────┘  │
    │                                                                         │
    │   ┌─────────────────────────────────────────────────────────────────┐  │
    │   │  LUT Breakdown:                                                 │  │
    │   │  ├── AXI infrastructure                ~3,000                   │  │
    │   │  ├── SPI/UART interfaces               ~1,500                   │  │
    │   │  ├── INS mechanization                 ~4,000                   │  │
    │   │  ├── Covariance propagation            ~5,000                   │  │
    │   │  ├── Kalman update                     ~3,000                   │  │
    │   │  └── Control/misc                      ~1,500                   │  │
    │   └─────────────────────────────────────────────────────────────────┘  │
    │                                                                         │
    │   Power estimate (typical):  ~1.2 W (PL) + ~0.5 W (PS) = ~1.7 W        │
    │                                                                         │
    └─────────────────────────────────────────────────────────────────────────┘
```

---

## Build and Simulation

### Directory Structure

```
ins_trn_fpga/
├── rtl/
│   ├── ins_trn_top.sv
│   ├── sensor_subsystem/
│   │   ├── spi_master_imu.sv
│   │   ├── spi_master_baro.sv
│   │   ├── uart_rx_lidar.sv
│   │   └── imu_calibration.sv
│   ├── ins_mechanization/
│   │   ├── ins_mechanization.sv
│   │   ├── attitude_update.sv
│   │   ├── quat_multiply.sv
│   │   ├── quat_normalize.sv
│   │   ├── dcm_from_quat.sv
│   │   └── nav_integration.sv
│   ├── error_state/
│   │   ├── phi_matrix_gen.sv
│   │   ├── q_matrix_gen.sv
│   │   └── covariance_propagator.sv
│   ├── kalman/
│   │   ├── kalman_update_engine.sv
│   │   ├── matrix_inverse_2x2.sv
│   │   └── state_corrector.sv
│   ├── memory/
│   │   ├── ins_state_ram.sv
│   │   ├── error_state_ram.sv
│   │   └── p_matrix_bram.sv
│   └── common/
│       ├── fp_multiply.sv
│       ├── fp_add.sv
│       ├── fp_sqrt.sv
│       └── trig_lut.sv
├── tb/
│   ├── tb_ins_trn_top.sv
│   ├── tb_ins_mechanization.sv
│   ├── imu_stimulus.sv
│   └── reference_model/
│       └── ins_reference.py
├── constraints/
│   └── zybo_z7.xdc
├── scripts/
│   ├── build.tcl
│   └── sim.tcl
└── docs/
    └── architecture.md
```

### Simulation Commands

```bash
# Run RTL simulation with Vivado
vivado -mode batch -source scripts/sim.tcl

# Run Python reference model comparison
python tb/reference_model/ins_reference.py --input imu_data.csv --output ref_trajectory.csv

# Compare RTL vs reference
python tb/compare_results.py rtl_output.csv ref_trajectory.csv
```

### Synthesis Commands

```bash
# Full build flow
vivado -mode batch -source scripts/build.tcl

# Generate bitstream only
vivado -mode batch -source scripts/build.tcl -tclargs bitstream_only
```

---

## References

1. Groves, P. D. (2013). *Principles of GNSS, Inertial, and Multisensor Integrated Navigation Systems*
2. Savage, P. G. (2000). *Strapdown Analytics*
3. Analog Devices ADIS16500 Datasheet
4. Xilinx UG585 - Zynq-7000 Technical Reference Manual

---

## License

MIT License - See LICENSE file for details.

---

## Contact

For questions or contributions, open an issue on GitHub.
