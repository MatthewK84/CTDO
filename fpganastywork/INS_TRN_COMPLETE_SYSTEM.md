# INS + TRN Complete Navigation System

## GPS-Denied Navigation for Zybo Z7-20 + Raspberry Pi 5 + Pico 2W

---

# Table of Contents

1. [System Overview](#system-overview)
2. [Hardware Configuration](#hardware-configuration)
3. [FPGA RTL Modules](#fpga-rtl-modules)
4. [Raspberry Pi 5 - Terrain Correlation](#raspberry-pi-5-terrain-correlation)
5. [Raspberry Pi Pico 2W - Sensor Hub](#raspberry-pi-pico-2w-sensor-hub)
6. [Build Instructions](#build-instructions)
7. [Testing](#testing)

---

# System Overview

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                         SYSTEM ARCHITECTURE                                  │
│                                                                             │
│   SENSORS              ZYBO Z7-20                    COMPUTE               │
│  ┌─────────┐          ┌─────────────────────┐       ┌─────────────┐        │
│  │ADIS16500│──SPI────►│                     │       │             │        │
│  │  IMU    │          │   INS Mechanization │       │   Pi 5      │        │
│  └─────────┘          │   Kalman Filter     │◄─────►│   TERCOM    │        │
│  ┌─────────┐          │   Covariance Prop   │  SPI  │   DTED      │        │
│  │ BMP390  │──SPI────►│                     │       │             │        │
│  │ Baro    │          └─────────────────────┘       └──────┬──────┘        │
│  └─────────┘                    │                          │               │
│  ┌─────────┐                    │                          │               │
│  │TF-Luna  │──UART───►          │                          │               │
│  │ LiDAR   │                    │                   ┌──────▼──────┐        │
│  └─────────┘                    │                   │  Pico 2W    │        │
│  ┌─────────┐                    └──────────────────►│  Telemetry  │        │
│  │RM3100   │──SPI────►                              │  WiFi Link  │        │
│  │  Mag    │                                        └─────────────┘        │
│  └─────────┘                                                               │
└─────────────────────────────────────────────────────────────────────────────┘
```

---

# Hardware Configuration

## Bill of Materials

| Component | Model | Interface | Cost |
|-----------|-------|-----------|------|
| FPGA Board | Zybo Z7-20 | - | $300 |
| Compute | Raspberry Pi 5 8GB | SPI/USB | $80 |
| Telemetry | Raspberry Pi Pico 2W | UART | $7 |
| IMU | ADIS16500 | SPI | $450 |
| Barometer | BMP390 | SPI | $15 |
| LiDAR | TF-Luna | UART | $40 |
| Magnetometer | RM3100 | SPI | $45 |
| **Total** | | | **~$940** |

## Pin Assignments (Zybo Z7-20)

```
PMOD JA (IMU - ADIS16500):
  JA1: SCLK
  JA2: MOSI
  JA3: MISO
  JA4: CS_N
  JA7: DR (Data Ready)

PMOD JB (Baro - BMP390):
  JB1: SCLK
  JB2: MOSI
  JB3: MISO
  JB4: CS_N

PMOD JC (LiDAR - TF-Luna):
  JC1: UART_TX
  JC2: UART_RX

PMOD JD (Mag - RM3100):
  JD1: SCLK
  JD2: MOSI
  JD3: MISO
  JD4: CS_N
```

---

# FPGA RTL Modules

## Directory Structure

```
ins_trn_fpga/
├── rtl/
│   ├── ins_trn_top.sv
│   ├── clk_rst_gen.sv
│   ├── axi_reg_bank.sv
│   ├── sensor/
│   │   ├── spi_master_imu.sv
│   │   ├── spi_master_baro.sv
│   │   ├── uart_rx_lidar.sv
│   │   └── imu_calibration.sv
│   ├── ins/
│   │   ├── ins_mechanization.sv
│   │   ├── attitude_update.sv
│   │   ├── quat_multiply.sv
│   │   └── dcm_from_quat.sv
│   ├── filter/
│   │   ├── covariance_propagator.sv
│   │   ├── phi_matrix_gen.sv
│   │   ├── kalman_update_engine.sv
│   │   └── matrix_inverse_2x2.sv
│   ├── profile/
│   │   └── altitude_profile_manager.sv
│   └── math/
│       ├── fp_multiply.sv
│       ├── fp_add.sv
│       ├── fp_sqrt.sv
│       └── int16_to_float.sv
├── tb/
│   └── tb_ins_trn_top.sv
├── constraints/
│   └── zybo_z7_20.xdc
└── scripts/
    └── build.tcl
```

---

## ins_trn_top.sv

```systemverilog
//=============================================================================
// INS + TRN Navigation System - Top Level
// Target: Xilinx Zynq-7020 (Zybo Z7-20)
//=============================================================================

`timescale 1ns / 1ps

module ins_trn_top #(
    parameter C_S_AXI_DATA_WIDTH = 32,
    parameter C_S_AXI_ADDR_WIDTH = 12
)(
    input  logic        sys_clk_i,
    input  logic        sys_rst_n_i,
    
    // AXI4-Lite Interface (directly to PS)
    input  logic [C_S_AXI_ADDR_WIDTH-1:0] s_axi_awaddr,
    input  logic        s_axi_awvalid,
    output logic        s_axi_awready,
    input  logic [C_S_AXI_DATA_WIDTH-1:0] s_axi_wdata,
    input  logic        s_axi_wvalid,
    output logic        s_axi_wready,
    output logic [1:0]  s_axi_bresp,
    output logic        s_axi_bvalid,
    input  logic        s_axi_bready,
    input  logic [C_S_AXI_ADDR_WIDTH-1:0] s_axi_araddr,
    input  logic        s_axi_arvalid,
    output logic        s_axi_arready,
    output logic [C_S_AXI_DATA_WIDTH-1:0] s_axi_rdata,
    output logic [1:0]  s_axi_rresp,
    output logic        s_axi_rvalid,
    input  logic        s_axi_rready,
    
    // IMU SPI (ADIS16500)
    output logic        imu_spi_sclk,
    output logic        imu_spi_mosi,
    input  logic        imu_spi_miso,
    output logic        imu_spi_cs_n,
    input  logic        imu_dr,
    
    // Barometer SPI (BMP390)
    output logic        baro_spi_sclk,
    output logic        baro_spi_mosi,
    input  logic        baro_spi_miso,
    output logic        baro_spi_cs_n,
    
    // LiDAR UART (TF-Luna)
    input  logic        lidar_uart_rx,
    output logic        lidar_uart_tx,
    
    // Magnetometer SPI (RM3100)
    output logic        mag_spi_sclk,
    output logic        mag_spi_mosi,
    input  logic        mag_spi_miso,
    output logic        mag_spi_cs_n,
    
    // Pi 5 Communication SPI (directly connected to PS SPI)
    // Directly via MIO pins
    
    // Interrupt
    output logic        irq,
    
    // Debug
    output logic [3:0]  led
);

    // Clock and Reset
    logic clk_100m, clk_200m;
    logic rst_100m_n, rst_200m_n;
    logic clk_locked;
    
    // IMU Data
    logic        imu_valid;
    logic [31:0] gyro_x, gyro_y, gyro_z;
    logic [31:0] accel_x, accel_y, accel_z;
    
    // Baro Data
    logic        baro_valid;
    logic [31:0] baro_alt;
    
    // LiDAR Data
    logic        lidar_valid;
    logic [31:0] lidar_agl;
    
    // INS State
    logic        ins_valid;
    logic [31:0] pos_n, pos_e, pos_d;
    logic [31:0] vel_n, vel_e, vel_d;
    logic [31:0] quat_w, quat_x, quat_y, quat_z;
    
    // TRN Interface
    logic        trn_fix_valid;
    logic [31:0] trn_pos_n, trn_pos_e;
    logic [31:0] trn_cov_nn, trn_cov_ee, trn_cov_ne;
    logic        trn_busy, trn_done;
    
    // Error State
    logic [31:0] error_state [0:14];
    
    // Covariance
    logic [31:0] cov_pos_std, cov_vel_std, cov_att_std;
    
    // Configuration
    logic        cfg_enable;
    logic [7:0]  cfg_imu_div, cfg_baro_div;
    logic [31:0] cfg_gyro_bias [0:2];
    logic [31:0] cfg_accel_bias [0:2];
    
    //=========================================================================
    // Clock Generation
    //=========================================================================
    
    clk_rst_gen u_clk_rst (
        .clk_in       (sys_clk_i),
        .rst_n_in     (sys_rst_n_i),
        .clk_100m_o   (clk_100m),
        .clk_200m_o   (clk_200m),
        .locked_o     (clk_locked),
        .rst_100m_n_o (rst_100m_n),
        .rst_200m_n_o (rst_200m_n)
    );
    
    //=========================================================================
    // AXI Register Bank
    //=========================================================================
    
    axi_reg_bank #(
        .C_S_AXI_DATA_WIDTH(C_S_AXI_DATA_WIDTH),
        .C_S_AXI_ADDR_WIDTH(C_S_AXI_ADDR_WIDTH)
    ) u_axi_regs (
        .s_axi_aclk    (clk_100m),
        .s_axi_aresetn (rst_100m_n),
        .s_axi_awaddr  (s_axi_awaddr),
        .s_axi_awvalid (s_axi_awvalid),
        .s_axi_awready (s_axi_awready),
        .s_axi_wdata   (s_axi_wdata),
        .s_axi_wvalid  (s_axi_wvalid),
        .s_axi_wready  (s_axi_wready),
        .s_axi_bresp   (s_axi_bresp),
        .s_axi_bvalid  (s_axi_bvalid),
        .s_axi_bready  (s_axi_bready),
        .s_axi_araddr  (s_axi_araddr),
        .s_axi_arvalid (s_axi_arvalid),
        .s_axi_arready (s_axi_arready),
        .s_axi_rdata   (s_axi_rdata),
        .s_axi_rresp   (s_axi_rresp),
        .s_axi_rvalid  (s_axi_rvalid),
        .s_axi_rready  (s_axi_rready),
        
        // Config outputs
        .cfg_enable_o     (cfg_enable),
        .cfg_imu_div_o    (cfg_imu_div),
        .cfg_baro_div_o   (cfg_baro_div),
        .cfg_gyro_bias_o  (cfg_gyro_bias),
        .cfg_accel_bias_o (cfg_accel_bias),
        
        // Status inputs
        .pos_n_i      (pos_n),
        .pos_e_i      (pos_e),
        .pos_d_i      (pos_d),
        .vel_n_i      (vel_n),
        .vel_e_i      (vel_e),
        .vel_d_i      (vel_d),
        .quat_w_i     (quat_w),
        .quat_x_i     (quat_x),
        .quat_y_i     (quat_y),
        .quat_z_i     (quat_z),
        .cov_pos_i    (cov_pos_std),
        .cov_vel_i    (cov_vel_std),
        .cov_att_i    (cov_att_std),
        
        // TRN interface
        .trn_fix_valid_o (trn_fix_valid),
        .trn_pos_n_o     (trn_pos_n),
        .trn_pos_e_o     (trn_pos_e),
        .trn_cov_nn_o    (trn_cov_nn),
        .trn_cov_ee_o    (trn_cov_ee),
        .trn_cov_ne_o    (trn_cov_ne),
        .trn_busy_i      (trn_busy),
        .trn_done_i      (trn_done),
        
        .irq_o (irq)
    );
    
    //=========================================================================
    // IMU Interface
    //=========================================================================
    
    spi_master_imu u_imu (
        .clk          (clk_100m),
        .rst_n        (rst_100m_n),
        .spi_sclk     (imu_spi_sclk),
        .spi_mosi     (imu_spi_mosi),
        .spi_miso     (imu_spi_miso),
        .spi_cs_n     (imu_spi_cs_n),
        .data_ready   (imu_dr),
        .cfg_div      (cfg_imu_div),
        .cfg_enable   (cfg_enable),
        .data_valid   (imu_valid),
        .gyro_x       (gyro_x),
        .gyro_y       (gyro_y),
        .gyro_z       (gyro_z),
        .accel_x      (accel_x),
        .accel_y      (accel_y),
        .accel_z      (accel_z)
    );
    
    //=========================================================================
    // Barometer Interface
    //=========================================================================
    
    spi_master_baro u_baro (
        .clk          (clk_100m),
        .rst_n        (rst_100m_n),
        .spi_sclk     (baro_spi_sclk),
        .spi_mosi     (baro_spi_mosi),
        .spi_miso     (baro_spi_miso),
        .spi_cs_n     (baro_spi_cs_n),
        .cfg_div      (cfg_baro_div),
        .cfg_enable   (cfg_enable),
        .data_valid   (baro_valid),
        .altitude     (baro_alt)
    );
    
    //=========================================================================
    // LiDAR Interface
    //=========================================================================
    
    uart_rx_lidar u_lidar (
        .clk          (clk_100m),
        .rst_n        (rst_100m_n),
        .uart_rx      (lidar_uart_rx),
        .uart_tx      (lidar_uart_tx),
        .cfg_enable   (cfg_enable),
        .data_valid   (lidar_valid),
        .distance     (lidar_agl)
    );
    
    //=========================================================================
    // INS Mechanization
    //=========================================================================
    
    ins_mechanization u_ins (
        .clk          (clk_200m),
        .rst_n        (rst_200m_n),
        .imu_valid    (imu_valid),
        .gyro_x       (gyro_x),
        .gyro_y       (gyro_y),
        .gyro_z       (gyro_z),
        .accel_x      (accel_x),
        .accel_y      (accel_y),
        .accel_z      (accel_z),
        .cfg_dt       (32'h3A83126F),  // 0.001s
        .state_valid  (ins_valid),
        .pos_n        (pos_n),
        .pos_e        (pos_e),
        .pos_d        (pos_d),
        .vel_n        (vel_n),
        .vel_e        (vel_e),
        .vel_d        (vel_d),
        .quat_w       (quat_w),
        .quat_x       (quat_x),
        .quat_y       (quat_y),
        .quat_z       (quat_z),
        .correction_valid (trn_done),
        .correction       (error_state)
    );
    
    //=========================================================================
    // Covariance Propagation
    //=========================================================================
    
    covariance_propagator u_cov (
        .clk          (clk_200m),
        .rst_n        (rst_200m_n),
        .trigger      (ins_valid),
        .quat_w       (quat_w),
        .quat_x       (quat_x),
        .quat_y       (quat_y),
        .quat_z       (quat_z),
        .cfg_dt       (32'h3A83126F),
        .cov_pos_std  (cov_pos_std),
        .cov_vel_std  (cov_vel_std),
        .cov_att_std  (cov_att_std)
    );
    
    //=========================================================================
    // Kalman Update Engine
    //=========================================================================
    
    kalman_update_engine u_kalman (
        .clk          (clk_200m),
        .rst_n        (rst_200m_n),
        .fix_valid    (trn_fix_valid),
        .fix_pos_n    (trn_pos_n),
        .fix_pos_e    (trn_pos_e),
        .fix_cov_nn   (trn_cov_nn),
        .fix_cov_ee   (trn_cov_ee),
        .fix_cov_ne   (trn_cov_ne),
        .ins_pos_n    (pos_n),
        .ins_pos_e    (pos_e),
        .error_state  (error_state),
        .busy         (trn_busy),
        .done         (trn_done)
    );
    
    //=========================================================================
    // Altitude Profile Manager
    //=========================================================================
    
    altitude_profile_manager u_profile (
        .clk          (clk_100m),
        .rst_n        (rst_100m_n),
        .baro_valid   (baro_valid),
        .baro_alt     (baro_alt),
        .lidar_valid  (lidar_valid),
        .lidar_agl    (lidar_agl),
        .ins_pos_n    (pos_n),
        .ins_pos_e    (pos_e),
        .cfg_enable   (cfg_enable)
    );
    
    //=========================================================================
    // Debug LEDs
    //=========================================================================
    
    assign led[0] = cfg_enable;
    assign led[1] = imu_valid;
    assign led[2] = trn_done;
    assign led[3] = clk_locked;

endmodule
```

---

## spi_master_imu.sv

```systemverilog
//=============================================================================
// SPI Master for ADIS16500 IMU
//=============================================================================

`timescale 1ns / 1ps

module spi_master_imu (
    input  logic        clk,
    input  logic        rst_n,
    
    // SPI pins
    output logic        spi_sclk,
    output logic        spi_mosi,
    input  logic        spi_miso,
    output logic        spi_cs_n,
    
    // Control
    input  logic        data_ready,
    input  logic [7:0]  cfg_div,
    input  logic        cfg_enable,
    
    // Output (float32)
    output logic        data_valid,
    output logic [31:0] gyro_x,
    output logic [31:0] gyro_y,
    output logic [31:0] gyro_z,
    output logic [31:0] accel_x,
    output logic [31:0] accel_y,
    output logic [31:0] accel_z
);

    // ADIS16500 burst read command
    localparam [15:0] BURST_CMD = 16'h6800;
    localparam BURST_WORDS = 10;
    
    // State machine
    typedef enum logic [3:0] {
        IDLE, WAIT_DR, CS_ASSERT, SEND_CMD,
        READ_WORD, CS_DEASSERT, CONVERT, OUTPUT
    } state_t;
    state_t state, next_state;
    
    // Counters
    logic [7:0]  clk_div_cnt;
    logic        spi_tick;
    logic [4:0]  bit_cnt;
    logic [3:0]  word_cnt;
    logic [3:0]  setup_cnt;
    
    // Shift registers
    logic [15:0] tx_shift;
    logic [15:0] rx_shift;
    logic [15:0] rx_data [0:9];
    
    // Data ready sync
    logic [2:0]  dr_sync;
    wire         dr_rise = dr_sync[1] & ~dr_sync[2];
    
    always_ff @(posedge clk or negedge rst_n)
        if (!rst_n) dr_sync <= 3'b0;
        else dr_sync <= {dr_sync[1:0], data_ready};
    
    // Clock divider
    always_ff @(posedge clk or negedge rst_n)
        if (!rst_n) begin
            clk_div_cnt <= 8'd0;
            spi_tick <= 1'b0;
        end else if (clk_div_cnt >= cfg_div) begin
            clk_div_cnt <= 8'd0;
            spi_tick <= 1'b1;
        end else begin
            clk_div_cnt <= clk_div_cnt + 1'b1;
            spi_tick <= 1'b0;
        end
    
    // State machine
    always_ff @(posedge clk or negedge rst_n)
        if (!rst_n) state <= IDLE;
        else state <= next_state;
    
    always_comb begin
        next_state = state;
        case (state)
            IDLE:       if (cfg_enable) next_state = WAIT_DR;
            WAIT_DR:    if (!cfg_enable) next_state = IDLE;
                        else if (dr_rise) next_state = CS_ASSERT;
            CS_ASSERT:  if (setup_cnt >= 4'd8) next_state = SEND_CMD;
            SEND_CMD:   if (spi_tick && bit_cnt == 5'd15) next_state = READ_WORD;
            READ_WORD:  if (spi_tick && bit_cnt == 5'd15 && word_cnt >= BURST_WORDS-1)
                            next_state = CS_DEASSERT;
            CS_DEASSERT:if (setup_cnt >= 4'd8) next_state = CONVERT;
            CONVERT:    next_state = OUTPUT;
            OUTPUT:     next_state = WAIT_DR;
            default:    next_state = IDLE;
        endcase
    end
    
    // Bit counter
    always_ff @(posedge clk or negedge rst_n)
        if (!rst_n) bit_cnt <= 5'd0;
        else if (state == CS_ASSERT) bit_cnt <= 5'd0;
        else if (spi_tick && (state == SEND_CMD || state == READ_WORD))
            bit_cnt <= (bit_cnt == 5'd15) ? 5'd0 : bit_cnt + 1'b1;
    
    // Word counter
    always_ff @(posedge clk or negedge rst_n)
        if (!rst_n) word_cnt <= 4'd0;
        else if (state == CS_ASSERT) word_cnt <= 4'd0;
        else if (state == READ_WORD && spi_tick && bit_cnt == 5'd15)
            word_cnt <= word_cnt + 1'b1;
    
    // Setup counter
    always_ff @(posedge clk or negedge rst_n)
        if (!rst_n) setup_cnt <= 4'd0;
        else if (state == CS_ASSERT || state == CS_DEASSERT) begin
            if (spi_tick) setup_cnt <= setup_cnt + 1'b1;
        end else setup_cnt <= 4'd0;
    
    // TX shift register
    always_ff @(posedge clk or negedge rst_n)
        if (!rst_n) tx_shift <= 16'd0;
        else if (state == CS_ASSERT) tx_shift <= BURST_CMD;
        else if (spi_tick && state == SEND_CMD)
            tx_shift <= {tx_shift[14:0], 1'b0};
    
    // RX shift register
    always_ff @(posedge clk or negedge rst_n)
        if (!rst_n) rx_shift <= 16'd0;
        else if (spi_tick && (state == SEND_CMD || state == READ_WORD))
            rx_shift <= {rx_shift[14:0], spi_miso};
    
    // Store received words
    always_ff @(posedge clk or negedge rst_n)
        if (!rst_n) begin
            for (int i = 0; i < 10; i++) rx_data[i] <= 16'd0;
        end else if (state == READ_WORD && spi_tick && bit_cnt == 5'd15)
            rx_data[word_cnt] <= rx_shift;
    
    // SPI outputs
    always_ff @(posedge clk or negedge rst_n)
        if (!rst_n) begin
            spi_cs_n <= 1'b1;
            spi_sclk <= 1'b1;
        end else begin
            spi_cs_n <= ~(state == CS_ASSERT || state == SEND_CMD || 
                          state == READ_WORD || state == CS_DEASSERT);
            if (state == SEND_CMD || state == READ_WORD)
                if (spi_tick) spi_sclk <= ~spi_sclk;
            else spi_sclk <= 1'b1;
        end
    
    assign spi_mosi = tx_shift[15];
    
    // Convert raw data to float32 (simplified)
    // ADIS16500: Gyro = 0.00625 °/s/LSB, Accel = 1.25 mg/LSB
    always_ff @(posedge clk or negedge rst_n)
        if (!rst_n) begin
            data_valid <= 1'b0;
            gyro_x <= 32'd0; gyro_y <= 32'd0; gyro_z <= 32'd0;
            accel_x <= 32'd0; accel_y <= 32'd0; accel_z <= 32'd0;
        end else if (state == OUTPUT) begin
            data_valid <= 1'b1;
            // Burst order: DIAG, GYRO_X, GYRO_Y, GYRO_Z, ACCEL_X, ACCEL_Y, ACCEL_Z, TEMP, CNT, CRC
            gyro_x <= {16'd0, rx_data[1]};  // Placeholder - need int16_to_float
            gyro_y <= {16'd0, rx_data[2]};
            gyro_z <= {16'd0, rx_data[3]};
            accel_x <= {16'd0, rx_data[4]};
            accel_y <= {16'd0, rx_data[5]};
            accel_z <= {16'd0, rx_data[6]};
        end else data_valid <= 1'b0;

endmodule
```

---

## ins_mechanization.sv

```systemverilog
//=============================================================================
// INS Strapdown Mechanization
//=============================================================================

`timescale 1ns / 1ps

module ins_mechanization (
    input  logic        clk,
    input  logic        rst_n,
    
    // IMU input
    input  logic        imu_valid,
    input  logic [31:0] gyro_x, gyro_y, gyro_z,
    input  logic [31:0] accel_x, accel_y, accel_z,
    
    // Config
    input  logic [31:0] cfg_dt,
    
    // State output
    output logic        state_valid,
    output logic [31:0] pos_n, pos_e, pos_d,
    output logic [31:0] vel_n, vel_e, vel_d,
    output logic [31:0] quat_w, quat_x, quat_y, quat_z,
    
    // Corrections
    input  logic        correction_valid,
    input  logic [31:0] correction [0:14]
);

    // Constants
    localparam [31:0] GRAVITY = 32'h411CF5C3;  // 9.81
    localparam [31:0] ONE = 32'h3F800000;      // 1.0
    localparam [31:0] HALF = 32'h3F000000;     // 0.5
    
    // State machine
    typedef enum logic [3:0] {
        IDLE, QUAT_DELTA, QUAT_MULT, QUAT_NORM,
        DCM_CALC, ROTATE_ACCEL, GRAVITY_SUB,
        VEL_INT, POS_INT, OUTPUT
    } state_t;
    state_t state, next_state;
    
    // Internal state
    logic [31:0] q_w, q_x, q_y, q_z;
    logic [31:0] v_n, v_e, v_d;
    logic [31:0] p_n, p_e, p_d;
    
    // DCM
    logic [31:0] dcm [0:2][0:2];
    
    // Delta quaternion
    logic [31:0] dq_w, dq_x, dq_y, dq_z;
    
    // Accel in nav frame
    logic [31:0] a_n, a_e, a_d;
    
    // State machine
    always_ff @(posedge clk or negedge rst_n)
        if (!rst_n) state <= IDLE;
        else state <= next_state;
    
    always_comb begin
        next_state = state;
        case (state)
            IDLE:        if (imu_valid) next_state = QUAT_DELTA;
            QUAT_DELTA:  next_state = QUAT_MULT;
            QUAT_MULT:   next_state = QUAT_NORM;
            QUAT_NORM:   next_state = DCM_CALC;
            DCM_CALC:    next_state = ROTATE_ACCEL;
            ROTATE_ACCEL:next_state = GRAVITY_SUB;
            GRAVITY_SUB: next_state = VEL_INT;
            VEL_INT:     next_state = POS_INT;
            POS_INT:     next_state = OUTPUT;
            OUTPUT:      next_state = IDLE;
            default:     next_state = IDLE;
        endcase
    end
    
    // Initialize state
    initial begin
        q_w = ONE;
        q_x = 32'd0; q_y = 32'd0; q_z = 32'd0;
        v_n = 32'd0; v_e = 32'd0; v_d = 32'd0;
        p_n = 32'd0; p_e = 32'd0; p_d = 32'd0;
    end
    
    // Quaternion update (simplified first-order)
    // dq = [1, wx*dt/2, wy*dt/2, wz*dt/2]
    // TODO: Implement proper quaternion integration with FP units
    
    // DCM from quaternion
    // C = f(q) - 9 elements from quaternion
    // TODO: Implement DCM computation
    
    // Rotate acceleration
    // a_nav = C * a_body
    // TODO: Matrix-vector multiply
    
    // Subtract gravity
    // a_nav_d = a_nav_d - g
    // TODO: FP subtract
    
    // Velocity integration
    // v = v + a * dt
    // TODO: FP multiply-add
    
    // Position integration
    // p = p + v * dt
    // TODO: FP multiply-add
    
    // Apply corrections
    always_ff @(posedge clk or negedge rst_n)
        if (!rst_n) begin
            // Reset to initial
        end else if (correction_valid) begin
            // Subtract corrections from state
            // p -= correction[0:2]
            // v -= correction[3:5]
            // Apply attitude correction to quaternion
        end
    
    // Outputs
    assign state_valid = (state == OUTPUT);
    assign pos_n = p_n;
    assign pos_e = p_e;
    assign pos_d = p_d;
    assign vel_n = v_n;
    assign vel_e = v_e;
    assign vel_d = v_d;
    assign quat_w = q_w;
    assign quat_x = q_x;
    assign quat_y = q_y;
    assign quat_z = q_z;

endmodule
```

---

## kalman_update_engine.sv

```systemverilog
//=============================================================================
// Kalman Update Engine
//=============================================================================

`timescale 1ns / 1ps

module kalman_update_engine (
    input  logic        clk,
    input  logic        rst_n,
    
    // TRN fix
    input  logic        fix_valid,
    input  logic [31:0] fix_pos_n, fix_pos_e,
    input  logic [31:0] fix_cov_nn, fix_cov_ee, fix_cov_ne,
    
    // Current INS
    input  logic [31:0] ins_pos_n, ins_pos_e,
    
    // Output
    output logic [31:0] error_state [0:14],
    output logic        busy,
    output logic        done
);

    typedef enum logic [3:0] {
        IDLE, CALC_Z, LOAD_P, CALC_S, INV_S,
        CALC_K, CALC_DX, UPDATE_P, OUTPUT, DONE
    } state_t;
    state_t state, next_state;
    
    // Innovation
    logic [31:0] z [0:1];
    
    // S matrix (2x2)
    logic [31:0] s [0:1][0:1];
    logic [31:0] s_inv [0:1][0:1];
    
    // Kalman gain (15x2)
    logic [31:0] k [0:14][0:1];
    
    // State machine
    always_ff @(posedge clk or negedge rst_n)
        if (!rst_n) state <= IDLE;
        else state <= next_state;
    
    always_comb begin
        next_state = state;
        case (state)
            IDLE:     if (fix_valid) next_state = CALC_Z;
            CALC_Z:   next_state = LOAD_P;
            LOAD_P:   next_state = CALC_S;
            CALC_S:   next_state = INV_S;
            INV_S:    next_state = CALC_K;
            CALC_K:   next_state = CALC_DX;
            CALC_DX:  next_state = UPDATE_P;
            UPDATE_P: next_state = OUTPUT;
            OUTPUT:   next_state = DONE;
            DONE:     next_state = IDLE;
            default:  next_state = IDLE;
        endcase
    end
    
    assign busy = (state != IDLE);
    assign done = (state == DONE);
    
    // Innovation: z = trn_pos - ins_pos
    always_ff @(posedge clk)
        if (state == CALC_Z) begin
            // z[0] = fix_pos_n - ins_pos_n (FP subtract)
            // z[1] = fix_pos_e - ins_pos_e
            z[0] <= fix_pos_n;  // Placeholder
            z[1] <= fix_pos_e;
        end
    
    // S = P[0:1,0:1] + R
    // S_inv = inv(S) using 2x2 formula
    // K = P[:,0:1] * S_inv
    // dx = K * z
    // P = P - K*S*K'
    
    // Output error state
    initial begin
        for (int i = 0; i < 15; i++)
            error_state[i] = 32'd0;
    end

endmodule
```

---

## covariance_propagator.sv

```systemverilog
//=============================================================================
// Covariance Propagator
//=============================================================================

`timescale 1ns / 1ps

module covariance_propagator (
    input  logic        clk,
    input  logic        rst_n,
    
    input  logic        trigger,
    input  logic [31:0] quat_w, quat_x, quat_y, quat_z,
    input  logic [31:0] cfg_dt,
    
    output logic [31:0] cov_pos_std,
    output logic [31:0] cov_vel_std,
    output logic [31:0] cov_att_std
);

    // P matrix storage (upper triangle, 120 elements)
    logic [31:0] p_matrix [0:119];
    
    // State machine
    typedef enum logic [3:0] {
        IDLE, GEN_PHI, GEN_Q, MULT_P_PHIT,
        MULT_PHI_TEMP, ADD_Q, EXTRACT, DONE
    } state_t;
    state_t state, next_state;
    
    // Initialize P with typical values
    initial begin
        // Position variance: 100 m^2
        p_matrix[0] = 32'h42C80000;
        p_matrix[16] = 32'h42C80000;
        p_matrix[31] = 32'h42C80000;
        // Velocity variance: 1 (m/s)^2
        p_matrix[45] = 32'h3F800000;
        p_matrix[58] = 32'h3F800000;
        p_matrix[70] = 32'h3F800000;
        // Attitude variance: 0.01 rad^2
        p_matrix[81] = 32'h3C23D70A;
        p_matrix[91] = 32'h3C23D70A;
        p_matrix[100] = 32'h3C23D70A;
        // Bias variances: small
        // ... initialize rest to 0 or small values
    end
    
    always_ff @(posedge clk or negedge rst_n)
        if (!rst_n) state <= IDLE;
        else state <= next_state;
    
    always_comb begin
        next_state = state;
        case (state)
            IDLE:         if (trigger) next_state = GEN_PHI;
            GEN_PHI:      next_state = GEN_Q;
            GEN_Q:        next_state = MULT_P_PHIT;
            MULT_P_PHIT:  next_state = MULT_PHI_TEMP;
            MULT_PHI_TEMP:next_state = ADD_Q;
            ADD_Q:        next_state = EXTRACT;
            EXTRACT:      next_state = DONE;
            DONE:         next_state = IDLE;
            default:      next_state = IDLE;
        endcase
    end
    
    // Extract standard deviations (sqrt of diagonal)
    // cov_pos_std = sqrt((P[0,0] + P[1,1] + P[2,2])/3)
    always_ff @(posedge clk or negedge rst_n)
        if (!rst_n) begin
            cov_pos_std <= 32'h41200000;  // 10 m
            cov_vel_std <= 32'h3F800000;  // 1 m/s
            cov_att_std <= 32'h3DCCCCCD;  // 0.1 rad
        end else if (state == EXTRACT) begin
            // TODO: Compute actual std from P diagonal
        end

endmodule
```

---

## altitude_profile_manager.sv

```systemverilog
//=============================================================================
// Altitude Profile Manager
//=============================================================================

`timescale 1ns / 1ps

module altitude_profile_manager (
    input  logic        clk,
    input  logic        rst_n,
    
    input  logic        baro_valid,
    input  logic [31:0] baro_alt,
    input  logic        lidar_valid,
    input  logic [31:0] lidar_agl,
    input  logic [31:0] ins_pos_n, ins_pos_e,
    input  logic        cfg_enable,
    
    // DMA interface (to PS)
    input  logic        dma_read_en,
    output logic [127:0] dma_data,
    output logic [5:0]  fifo_count
);

    // Profile entry: terrain_alt(32) + pos_n(32) + pos_e(32) + timestamp(24) + valid(8)
    localparam FIFO_DEPTH = 64;
    
    logic [127:0] profile_fifo [0:FIFO_DEPTH-1];
    logic [5:0]   write_ptr, read_ptr;
    
    // Terrain elevation = baro_alt - lidar_agl
    logic [31:0] terrain_alt;
    logic        sample_valid;
    
    // Compute terrain elevation when both sensors valid
    always_ff @(posedge clk or negedge rst_n)
        if (!rst_n) begin
            terrain_alt <= 32'd0;
            sample_valid <= 1'b0;
        end else if (baro_valid && lidar_valid && cfg_enable) begin
            // terrain_alt = baro_alt - lidar_agl (FP subtract)
            terrain_alt <= baro_alt;  // Placeholder
            sample_valid <= 1'b1;
        end else sample_valid <= 1'b0;
    
    // Timestamp counter
    logic [23:0] timestamp;
    always_ff @(posedge clk or negedge rst_n)
        if (!rst_n) timestamp <= 24'd0;
        else timestamp <= timestamp + 1'b1;
    
    // Write to FIFO
    always_ff @(posedge clk or negedge rst_n)
        if (!rst_n) write_ptr <= 6'd0;
        else if (sample_valid) begin
            profile_fifo[write_ptr] <= {8'hFF, timestamp, ins_pos_e, ins_pos_n, terrain_alt};
            write_ptr <= write_ptr + 1'b1;
        end
    
    // Read from FIFO (DMA)
    always_ff @(posedge clk or negedge rst_n)
        if (!rst_n) read_ptr <= 6'd0;
        else if (dma_read_en) read_ptr <= read_ptr + 1'b1;
    
    assign dma_data = profile_fifo[read_ptr];
    assign fifo_count = write_ptr - read_ptr;

endmodule
```

---

## Constraints File (zybo_z7_20.xdc)

```tcl
## Clock
set_property -dict {PACKAGE_PIN K17 IOSTANDARD LVCMOS33} [get_ports sys_clk_i]
create_clock -period 8.000 -name sys_clk [get_ports sys_clk_i]

## Reset
set_property -dict {PACKAGE_PIN Y16 IOSTANDARD LVCMOS33} [get_ports sys_rst_n_i]

## PMOD JA - IMU (ADIS16500)
set_property -dict {PACKAGE_PIN N15 IOSTANDARD LVCMOS33} [get_ports imu_spi_sclk]
set_property -dict {PACKAGE_PIN L14 IOSTANDARD LVCMOS33} [get_ports imu_spi_mosi]
set_property -dict {PACKAGE_PIN K16 IOSTANDARD LVCMOS33} [get_ports imu_spi_miso]
set_property -dict {PACKAGE_PIN K14 IOSTANDARD LVCMOS33} [get_ports imu_spi_cs_n]
set_property -dict {PACKAGE_PIN N16 IOSTANDARD LVCMOS33} [get_ports imu_dr]

## PMOD JB - Barometer (BMP390)
set_property -dict {PACKAGE_PIN V8  IOSTANDARD LVCMOS33} [get_ports baro_spi_sclk]
set_property -dict {PACKAGE_PIN W8  IOSTANDARD LVCMOS33} [get_ports baro_spi_mosi]
set_property -dict {PACKAGE_PIN U7  IOSTANDARD LVCMOS33} [get_ports baro_spi_miso]
set_property -dict {PACKAGE_PIN V7  IOSTANDARD LVCMOS33} [get_ports baro_spi_cs_n]

## PMOD JC - LiDAR (TF-Luna)
set_property -dict {PACKAGE_PIN V15 IOSTANDARD LVCMOS33} [get_ports lidar_uart_tx]
set_property -dict {PACKAGE_PIN W15 IOSTANDARD LVCMOS33} [get_ports lidar_uart_rx]

## PMOD JD - Magnetometer (RM3100)
set_property -dict {PACKAGE_PIN T14 IOSTANDARD LVCMOS33} [get_ports mag_spi_sclk]
set_property -dict {PACKAGE_PIN T15 IOSTANDARD LVCMOS33} [get_ports mag_spi_mosi]
set_property -dict {PACKAGE_PIN P14 IOSTANDARD LVCMOS33} [get_ports mag_spi_miso]
set_property -dict {PACKAGE_PIN R14 IOSTANDARD LVCMOS33} [get_ports mag_spi_cs_n]

## LEDs
set_property -dict {PACKAGE_PIN M14 IOSTANDARD LVCMOS33} [get_ports {led[0]}]
set_property -dict {PACKAGE_PIN M15 IOSTANDARD LVCMOS33} [get_ports {led[1]}]
set_property -dict {PACKAGE_PIN G14 IOSTANDARD LVCMOS33} [get_ports {led[2]}]
set_property -dict {PACKAGE_PIN D18 IOSTANDARD LVCMOS33} [get_ports {led[3]}]

## Interrupt
set_property -dict {PACKAGE_PIN L15 IOSTANDARD LVCMOS33} [get_ports irq]
```

---

# Raspberry Pi 5 - Terrain Correlation

## Directory Structure

```
pi5_tercom/
├── src/
│   ├── main.py
│   ├── tercom.py
│   ├── dted_handler.py
│   ├── fpga_interface.py
│   └── nav_filter.py
├── data/
│   └── dted/
├── config/
│   └── config.yaml
├── requirements.txt
└── README.md
```

---

## main.py

```python
#!/usr/bin/env python3
"""
INS + TRN Navigation System - Pi 5 Main Application
Handles terrain correlation and FPGA communication
"""

import time
import signal
import logging
import yaml
import numpy as np
from pathlib import Path
from threading import Thread, Event

from tercom import TERCOMCorrelator
from dted_handler import DTEDHandler
from fpga_interface import FPGAInterface
from nav_filter import NavFilterManager

logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)


class NavigationSystem:
    """Main navigation system controller"""
    
    def __init__(self, config_path: str = "config/config.yaml"):
        self.config = self._load_config(config_path)
        self.shutdown_event = Event()
        
        # Initialize components
        self.fpga = FPGAInterface(
            spi_bus=self.config['fpga']['spi_bus'],
            spi_device=self.config['fpga']['spi_device'],
            spi_speed=self.config['fpga']['spi_speed']
        )
        
        self.dted = DTEDHandler(
            dted_path=self.config['dted']['path'],
            cache_size=self.config['dted']['cache_size']
        )
        
        self.tercom = TERCOMCorrelator(
            dted_handler=self.dted,
            search_radius=self.config['tercom']['search_radius'],
            grid_spacing=self.config['tercom']['grid_spacing'],
            min_profile_length=self.config['tercom']['min_profile_length']
        )
        
        self.nav_manager = NavFilterManager()
        
        # State
        self.current_position = np.zeros(3)  # NED
        self.current_velocity = np.zeros(3)
        self.position_uncertainty = np.eye(3) * 100  # 10m std
        
        logger.info("Navigation system initialized")
    
    def _load_config(self, path: str) -> dict:
        """Load configuration from YAML file"""
        with open(path, 'r') as f:
            return yaml.safe_load(f)
    
    def run(self):
        """Main run loop"""
        logger.info("Starting navigation system")
        
        # Register signal handlers
        signal.signal(signal.SIGINT, self._signal_handler)
        signal.signal(signal.SIGTERM, self._signal_handler)
        
        # Start FPGA communication thread
        fpga_thread = Thread(target=self._fpga_loop, daemon=True)
        fpga_thread.start()
        
        # Start terrain correlation thread
        tercom_thread = Thread(target=self._tercom_loop, daemon=True)
        tercom_thread.start()
        
        # Main loop - status reporting
        while not self.shutdown_event.is_set():
            self._report_status()
            time.sleep(1.0)
        
        logger.info("Navigation system shutdown complete")
    
    def _fpga_loop(self):
        """FPGA communication loop - runs at high rate"""
        rate = self.config['fpga']['poll_rate']
        period = 1.0 / rate
        
        while not self.shutdown_event.is_set():
            start = time.time()
            
            try:
                # Read INS state from FPGA
                state = self.fpga.read_ins_state()
                if state is not None:
                    self.current_position = state['position']
                    self.current_velocity = state['velocity']
                    self.position_uncertainty = state['covariance']
                
                # Read altitude profile
                profile = self.fpga.read_altitude_profile()
                if profile is not None and len(profile) > 0:
                    self.tercom.add_profile_samples(profile)
                
            except Exception as e:
                logger.error(f"FPGA communication error: {e}")
            
            # Rate control
            elapsed = time.time() - start
            if elapsed < period:
                time.sleep(period - elapsed)
    
    def _tercom_loop(self):
        """Terrain correlation loop - runs at lower rate"""
        rate = self.config['tercom']['update_rate']
        period = 1.0 / rate
        
        while not self.shutdown_event.is_set():
            start = time.time()
            
            try:
                # Check if we have enough profile samples
                if self.tercom.has_sufficient_profile():
                    # Run correlation
                    result = self.tercom.correlate(
                        estimated_position=self.current_position[:2],
                        position_uncertainty=self.position_uncertainty[:2, :2]
                    )
                    
                    if result is not None and result['confidence'] > 0.7:
                        # Send fix to FPGA
                        self.fpga.send_trn_fix(
                            pos_n=result['position'][0],
                            pos_e=result['position'][1],
                            cov_nn=result['covariance'][0, 0],
                            cov_ee=result['covariance'][1, 1],
                            cov_ne=result['covariance'][0, 1]
                        )
                        logger.info(f"TRN fix applied: {result['position']}, "
                                   f"std: {np.sqrt(np.diag(result['covariance']))}")
                    
                    # Clear profile for next correlation
                    self.tercom.clear_profile()
                
            except Exception as e:
                logger.error(f"TERCOM error: {e}")
            
            # Rate control
            elapsed = time.time() - start
            if elapsed < period:
                time.sleep(period - elapsed)
    
    def _report_status(self):
        """Report system status"""
        pos_std = np.sqrt(np.diag(self.position_uncertainty))
        logger.info(f"Position: N={self.current_position[0]:.1f}m, "
                   f"E={self.current_position[1]:.1f}m, "
                   f"D={self.current_position[2]:.1f}m | "
                   f"Uncertainty: {pos_std[0]:.1f}m")
    
    def _signal_handler(self, signum, frame):
        """Handle shutdown signals"""
        logger.info("Shutdown signal received")
        self.shutdown_event.set()


if __name__ == "__main__":
    nav = NavigationSystem()
    nav.run()
```

---

## tercom.py

```python
"""
TERCOM (Terrain Contour Matching) Correlator
Implements terrain-relative navigation correlation algorithm
"""

import numpy as np
from typing import Optional, Dict, List, Tuple
from dataclasses import dataclass
from scipy.ndimage import map_coordinates
from scipy.optimize import minimize
import logging

logger = logging.getLogger(__name__)


@dataclass
class ProfileSample:
    """Single terrain profile sample"""
    terrain_elevation: float  # meters MSL
    position_n: float         # meters, relative
    position_e: float         # meters, relative
    timestamp: float          # seconds


@dataclass
class CorrelationResult:
    """Result of terrain correlation"""
    position: np.ndarray      # [N, E] in meters
    covariance: np.ndarray    # 2x2 covariance matrix
    confidence: float         # 0-1 confidence score
    correlation_score: float  # Raw correlation metric


class TERCOMCorrelator:
    """
    Terrain Contour Matching Correlator
    
    Correlates measured terrain profile against stored DTED to estimate position.
    Uses Mean Absolute Difference (MAD) or Normalized Cross-Correlation (NCC).
    """
    
    def __init__(
        self,
        dted_handler,
        search_radius: float = 500.0,    # meters
        grid_spacing: float = 10.0,       # meters
        min_profile_length: int = 20,     # samples
        correlation_method: str = 'mad'   # 'mad' or 'ncc'
    ):
        self.dted = dted_handler
        self.search_radius = search_radius
        self.grid_spacing = grid_spacing
        self.min_profile_length = min_profile_length
        self.correlation_method = correlation_method
        
        # Profile buffer
        self.profile_samples: List[ProfileSample] = []
        
        logger.info(f"TERCOM initialized: search_radius={search_radius}m, "
                   f"grid_spacing={grid_spacing}m")
    
    def add_profile_samples(self, samples: List[Dict]):
        """Add terrain profile samples from FPGA"""
        for s in samples:
            self.profile_samples.append(ProfileSample(
                terrain_elevation=s['terrain_alt'],
                position_n=s['pos_n'],
                position_e=s['pos_e'],
                timestamp=s['timestamp']
            ))
    
    def has_sufficient_profile(self) -> bool:
        """Check if we have enough samples for correlation"""
        return len(self.profile_samples) >= self.min_profile_length
    
    def clear_profile(self):
        """Clear profile buffer"""
        self.profile_samples = []
    
    def correlate(
        self,
        estimated_position: np.ndarray,
        position_uncertainty: np.ndarray
    ) -> Optional[CorrelationResult]:
        """
        Perform terrain correlation
        
        Args:
            estimated_position: [N, E] estimated position in meters
            position_uncertainty: 2x2 position covariance matrix
            
        Returns:
            CorrelationResult or None if correlation failed
        """
        if not self.has_sufficient_profile():
            logger.warning("Insufficient profile samples")
            return None
        
        # Extract profile data
        measured_heights = np.array([s.terrain_elevation for s in self.profile_samples])
        rel_positions_n = np.array([s.position_n for s in self.profile_samples])
        rel_positions_e = np.array([s.position_e for s in self.profile_samples])
        
        # Make positions relative to first sample
        rel_positions_n -= rel_positions_n[0]
        rel_positions_e -= rel_positions_e[0]
        
        # Generate search grid
        search_std = np.sqrt(np.max(np.diag(position_uncertainty))) * 3
        search_range = min(search_std, self.search_radius)
        
        n_grid = int(2 * search_range / self.grid_spacing) + 1
        grid_n = np.linspace(-search_range, search_range, n_grid)
        grid_e = np.linspace(-search_range, search_range, n_grid)
        
        # Correlation surface
        correlation_surface = np.zeros((n_grid, n_grid))
        
        logger.debug(f"Searching {n_grid}x{n_grid} grid, range ±{search_range:.0f}m")
        
        # Evaluate correlation at each grid point
        for i, dn in enumerate(grid_n):
            for j, de in enumerate(grid_e):
                # Candidate position
                candidate_n = estimated_position[0] + dn
                candidate_e = estimated_position[1] + de
                
                # Get stored terrain heights at measured locations
                stored_heights = []
                for k in range(len(measured_heights)):
                    query_n = candidate_n + rel_positions_n[k]
                    query_e = candidate_e + rel_positions_e[k]
                    
                    h = self.dted.get_elevation(query_n, query_e)
                    if h is not None:
                        stored_heights.append(h)
                    else:
                        stored_heights.append(np.nan)
                
                stored_heights = np.array(stored_heights)
                
                # Compute correlation metric
                valid = ~np.isnan(stored_heights)
                if np.sum(valid) < self.min_profile_length // 2:
                    correlation_surface[i, j] = np.inf if self.correlation_method == 'mad' else -1
                    continue
                
                if self.correlation_method == 'mad':
                    # Mean Absolute Difference (lower = better)
                    correlation_surface[i, j] = np.mean(
                        np.abs(measured_heights[valid] - stored_heights[valid])
                    )
                else:
                    # Normalized Cross-Correlation (higher = better)
                    m = measured_heights[valid]
                    s = stored_heights[valid]
                    m = m - np.mean(m)
                    s = s - np.mean(s)
                    correlation_surface[i, j] = np.sum(m * s) / (
                        np.sqrt(np.sum(m**2) * np.sum(s**2)) + 1e-10
                    )
        
        # Find best match
        if self.correlation_method == 'mad':
            best_idx = np.unravel_index(np.argmin(correlation_surface), correlation_surface.shape)
            best_score = correlation_surface[best_idx]
            # Convert MAD to pseudo-confidence (lower MAD = higher confidence)
            confidence = np.exp(-best_score / 10.0)  # Tunable parameter
        else:
            best_idx = np.unravel_index(np.argmax(correlation_surface), correlation_surface.shape)
            best_score = correlation_surface[best_idx]
            confidence = max(0, best_score)
        
        # Refined position from grid
        best_dn = grid_n[best_idx[0]]
        best_de = grid_e[best_idx[1]]
        
        # Subgrid refinement using quadratic fit
        if 0 < best_idx[0] < n_grid-1 and 0 < best_idx[1] < n_grid-1:
            # Fit quadratic surface around minimum
            dn_refined, de_refined = self._subgrid_refinement(
                correlation_surface, best_idx, grid_n, grid_e
            )
            best_dn = dn_refined
            best_de = de_refined
        
        # Estimate uncertainty from correlation surface curvature
        covariance = self._estimate_covariance(
            correlation_surface, best_idx, grid_n, grid_e
        )
        
        result = CorrelationResult(
            position=np.array([estimated_position[0] + best_dn, 
                              estimated_position[1] + best_de]),
            covariance=covariance,
            confidence=confidence,
            correlation_score=best_score
        )
        
        logger.info(f"TERCOM result: offset=[{best_dn:.1f}, {best_de:.1f}]m, "
                   f"confidence={confidence:.2f}")
        
        return result
    
    def _subgrid_refinement(
        self,
        surface: np.ndarray,
        best_idx: Tuple[int, int],
        grid_n: np.ndarray,
        grid_e: np.ndarray
    ) -> Tuple[float, float]:
        """Subgrid refinement using quadratic interpolation"""
        i, j = best_idx
        
        # 3x3 region around minimum
        region = surface[i-1:i+2, j-1:j+2]
        
        # Quadratic fit coefficients
        # f(x,y) = a*x^2 + b*y^2 + c*x*y + d*x + e*y + f
        # Minimum at: x = -(2*b*d - c*e) / (4*a*b - c^2)
        #             y = -(2*a*e - c*d) / (4*a*b - c^2)
        
        # Simple finite difference approximation
        dn_offset = (region[0, 1] - region[2, 1]) / (
            2 * (region[0, 1] + region[2, 1] - 2*region[1, 1]) + 1e-10
        )
        de_offset = (region[1, 0] - region[1, 2]) / (
            2 * (region[1, 0] + region[1, 2] - 2*region[1, 1]) + 1e-10
        )
        
        # Clamp to ±0.5 grid cells
        dn_offset = np.clip(dn_offset, -0.5, 0.5)
        de_offset = np.clip(de_offset, -0.5, 0.5)
        
        spacing = grid_n[1] - grid_n[0]
        return grid_n[i] + dn_offset * spacing, grid_e[j] + de_offset * spacing
    
    def _estimate_covariance(
        self,
        surface: np.ndarray,
        best_idx: Tuple[int, int],
        grid_n: np.ndarray,
        grid_e: np.ndarray
    ) -> np.ndarray:
        """Estimate position covariance from correlation surface curvature"""
        i, j = best_idx
        spacing = grid_n[1] - grid_n[0]
        
        # Hessian approximation
        if 0 < i < len(grid_n)-1 and 0 < j < len(grid_e)-1:
            d2f_dn2 = (surface[i+1, j] + surface[i-1, j] - 2*surface[i, j]) / spacing**2
            d2f_de2 = (surface[i, j+1] + surface[i, j-1] - 2*surface[i, j]) / spacing**2
            d2f_dnde = (surface[i+1, j+1] + surface[i-1, j-1] - 
                       surface[i+1, j-1] - surface[i-1, j+1]) / (4 * spacing**2)
            
            hessian = np.array([[d2f_dn2, d2f_dnde],
                               [d2f_dnde, d2f_de2]])
            
            # Covariance ≈ inverse of Hessian (scaled)
            try:
                cov = np.linalg.inv(hessian + np.eye(2) * 1e-6)
                # Ensure positive definite
                eigvals = np.linalg.eigvalsh(cov)
                if np.any(eigvals < 0):
                    cov = np.eye(2) * spacing**2
            except np.linalg.LinAlgError:
                cov = np.eye(2) * spacing**2
        else:
            cov = np.eye(2) * spacing**2
        
        # Minimum uncertainty based on grid spacing
        min_var = (spacing / 3)**2
        cov = np.maximum(cov, np.eye(2) * min_var)
        
        return cov
```

---

## dted_handler.py

```python
"""
DTED (Digital Terrain Elevation Data) Handler
Loads and caches terrain elevation data
"""

import numpy as np
from pathlib import Path
from typing import Optional, Dict, Tuple
from functools import lru_cache
import struct
import logging

logger = logging.getLogger(__name__)


class DTEDHandler:
    """
    Handler for DTED (Digital Terrain Elevation Data) files
    
    Supports DTED Level 0, 1, 2 formats.
    Caches tiles for efficient access.
    """
    
    # DTED resolutions (arc-seconds)
    DTED_LEVELS = {
        0: 30,   # ~900m
        1: 3,    # ~90m
        2: 1     # ~30m
    }
    
    def __init__(
        self,
        dted_path: str,
        cache_size: int = 10,
        default_level: int = 2
    ):
        self.dted_path = Path(dted_path)
        self.cache_size = cache_size
        self.default_level = default_level
        
        # Tile cache: key = (lat, lon, level), value = elevation array
        self.tile_cache: Dict[Tuple[int, int, int], np.ndarray] = {}
        self.cache_order = []  # LRU tracking
        
        # Reference point for local NED frame
        self.ref_lat = None
        self.ref_lon = None
        
        logger.info(f"DTED handler initialized: path={dted_path}")
    
    def set_reference(self, lat: float, lon: float):
        """Set reference point for NED coordinate conversion"""
        self.ref_lat = lat
        self.ref_lon = lon
        logger.info(f"Reference set: lat={lat:.6f}, lon={lon:.6f}")
    
    def get_elevation(self, north: float, east: float, level: int = None) -> Optional[float]:
        """
        Get terrain elevation at position
        
        Args:
            north: North position in meters (relative to reference)
            east: East position in meters (relative to reference)
            level: DTED level (default: self.default_level)
            
        Returns:
            Elevation in meters MSL, or None if not available
        """
        if self.ref_lat is None or self.ref_lon is None:
            logger.error("Reference point not set")
            return None
        
        level = level or self.default_level
        
        # Convert NED to lat/lon
        lat, lon = self._ned_to_latlon(north, east)
        
        # Get elevation from DTED
        return self._get_elevation_latlon(lat, lon, level)
    
    def get_elevation_grid(
        self,
        center_n: float,
        center_e: float,
        size: float,
        resolution: float
    ) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        """
        Get grid of terrain elevations
        
        Args:
            center_n: Center north position (meters)
            center_e: Center east position (meters)
            size: Grid size (meters, square)
            resolution: Grid resolution (meters)
            
        Returns:
            Tuple of (north_coords, east_coords, elevation_grid)
        """
        n_points = int(size / resolution) + 1
        
        north = np.linspace(center_n - size/2, center_n + size/2, n_points)
        east = np.linspace(center_e - size/2, center_e + size/2, n_points)
        
        elevations = np.zeros((n_points, n_points))
        
        for i, n in enumerate(north):
            for j, e in enumerate(east):
                elev = self.get_elevation(n, e)
                elevations[i, j] = elev if elev is not None else np.nan
        
        return north, east, elevations
    
    def _ned_to_latlon(self, north: float, east: float) -> Tuple[float, float]:
        """Convert NED position to lat/lon"""
        # Approximate conversion (valid for small distances)
        # 1 degree latitude ≈ 111,111 meters
        # 1 degree longitude ≈ 111,111 * cos(lat) meters
        
        lat_offset = north / 111111.0
        lon_offset = east / (111111.0 * np.cos(np.radians(self.ref_lat)))
        
        return self.ref_lat + lat_offset, self.ref_lon + lon_offset
    
    def _get_elevation_latlon(self, lat: float, lon: float, level: int) -> Optional[float]:
        """Get elevation at lat/lon from DTED"""
        # Determine tile coordinates
        tile_lat = int(np.floor(lat))
        tile_lon = int(np.floor(lon))
        
        # Load tile (from cache or file)
        tile = self._load_tile(tile_lat, tile_lon, level)
        if tile is None:
            return None
        
        # Interpolate within tile
        resolution = self.DTED_LEVELS[level] / 3600.0  # degrees
        
        lat_idx = (lat - tile_lat) / resolution
        lon_idx = (lon - tile_lon) / resolution
        
        # Bilinear interpolation
        return self._bilinear_interpolate(tile, lat_idx, lon_idx)
    
    def _load_tile(self, lat: int, lon: int, level: int) -> Optional[np.ndarray]:
        """Load DTED tile from cache or file"""
        key = (lat, lon, level)
        
        # Check cache
        if key in self.tile_cache:
            # Move to end of LRU list
            self.cache_order.remove(key)
            self.cache_order.append(key)
            return self.tile_cache[key]
        
        # Load from file
        tile = self._read_dted_file(lat, lon, level)
        if tile is None:
            return None
        
        # Add to cache
        self.tile_cache[key] = tile
        self.cache_order.append(key)
        
        # Evict old entries if cache full
        while len(self.cache_order) > self.cache_size:
            old_key = self.cache_order.pop(0)
            del self.tile_cache[old_key]
        
        return tile
    
    def _read_dted_file(self, lat: int, lon: int, level: int) -> Optional[np.ndarray]:
        """Read DTED file from disk"""
        # DTED filename format: n35/w106/n35_w106_1arc_v3.dt2
        lat_dir = f"{'n' if lat >= 0 else 's'}{abs(lat):02d}"
        lon_dir = f"{'w' if lon < 0 else 'e'}{abs(lon):03d}"
        
        # Try different filename formats
        filename_patterns = [
            f"{lat_dir}_{lon_dir}_{level}arc.dt{level}",
            f"{'n' if lat >= 0 else 's'}{abs(lat):02d}{'w' if lon < 0 else 'e'}{abs(lon):03d}.dt{level}",
        ]
        
        for pattern in filename_patterns:
            filepath = self.dted_path / lat_dir / lon_dir / pattern
            if filepath.exists():
                return self._parse_dted_file(filepath, level)
        
        logger.warning(f"DTED tile not found: lat={lat}, lon={lon}, level={level}")
        return None
    
    def _parse_dted_file(self, filepath: Path, level: int) -> np.ndarray:
        """Parse DTED file format"""
        # DTED format: UHL + DSI + ACC + data records
        # Simplified parser - real implementation needs full format support
        
        resolution = 3600 // self.DTED_LEVELS[level] + 1  # points per degree
        
        try:
            with open(filepath, 'rb') as f:
                # Skip headers (UHL=80, DSI=648, ACC=2700)
                f.seek(80 + 648 + 2700)
                
                # Read elevation data
                elevations = np.zeros((resolution, resolution), dtype=np.int16)
                
                for col in range(resolution):
                    # Skip record header (8 bytes)
                    f.seek(8, 1)
                    
                    # Read column data
                    for row in range(resolution):
                        data = f.read(2)
                        if len(data) == 2:
                            elev = struct.unpack('>h', data)[0]
                            elevations[row, col] = elev
                    
                    # Skip record checksum (4 bytes)
                    f.seek(4, 1)
                
                logger.debug(f"Loaded DTED tile: {filepath}")
                return elevations.astype(np.float32)
                
        except Exception as e:
            logger.error(f"Error reading DTED file {filepath}: {e}")
            return None
    
    def _bilinear_interpolate(self, grid: np.ndarray, y: float, x: float) -> float:
        """Bilinear interpolation on grid"""
        y0, x0 = int(np.floor(y)), int(np.floor(x))
        y1, x1 = min(y0 + 1, grid.shape[0] - 1), min(x0 + 1, grid.shape[1] - 1)
        
        fy, fx = y - y0, x - x0
        
        val = (grid[y0, x0] * (1 - fx) * (1 - fy) +
               grid[y0, x1] * fx * (1 - fy) +
               grid[y1, x0] * (1 - fx) * fy +
               grid[y1, x1] * fx * fy)
        
        return float(val)
```

---

## fpga_interface.py

```python
"""
FPGA Interface
Communicates with Zybo Z7-20 via SPI
"""

import numpy as np
import spidev
import struct
from typing import Optional, Dict, List
import logging

logger = logging.getLogger(__name__)


class FPGAInterface:
    """
    Interface to Zynq FPGA via SPI
    
    Reads INS state, altitude profiles
    Writes TRN fixes
    """
    
    # Register addresses (match axi_reg_bank.sv)
    REG_CTRL       = 0x000
    REG_STATUS     = 0x004
    REG_POS_N      = 0x100
    REG_POS_E      = 0x104
    REG_POS_D      = 0x108
    REG_VEL_N      = 0x10C
    REG_VEL_E      = 0x110
    REG_VEL_D      = 0x114
    REG_QUAT_W     = 0x118
    REG_QUAT_X     = 0x11C
    REG_QUAT_Y     = 0x120
    REG_QUAT_Z     = 0x124
    REG_COV_POS    = 0x180
    REG_COV_VEL    = 0x184
    REG_COV_ATT    = 0x188
    REG_TRN_CTRL   = 0x200
    REG_TRN_POS_N  = 0x208
    REG_TRN_POS_E  = 0x20C
    REG_TRN_COV_NN = 0x210
    REG_TRN_COV_EE = 0x214
    REG_TRN_COV_NE = 0x218
    REG_PROF_CTRL  = 0x280
    REG_PROF_STATUS = 0x284
    REG_PROF_DATA  = 0x290
    
    def __init__(
        self,
        spi_bus: int = 0,
        spi_device: int = 0,
        spi_speed: int = 1000000
    ):
        self.spi = spidev.SpiDev()
        self.spi.open(spi_bus, spi_device)
        self.spi.max_speed_hz = spi_speed
        self.spi.mode = 0
        
        logger.info(f"FPGA interface initialized: SPI{spi_bus}.{spi_device} @ {spi_speed}Hz")
    
    def close(self):
        """Close SPI connection"""
        self.spi.close()
    
    def read_ins_state(self) -> Optional[Dict]:
        """Read current INS state from FPGA"""
        try:
            # Read position
            pos_n = self._read_float(self.REG_POS_N)
            pos_e = self._read_float(self.REG_POS_E)
            pos_d = self._read_float(self.REG_POS_D)
            
            # Read velocity
            vel_n = self._read_float(self.REG_VEL_N)
            vel_e = self._read_float(self.REG_VEL_E)
            vel_d = self._read_float(self.REG_VEL_D)
            
            # Read quaternion
            quat_w = self._read_float(self.REG_QUAT_W)
            quat_x = self._read_float(self.REG_QUAT_X)
            quat_y = self._read_float(self.REG_QUAT_Y)
            quat_z = self._read_float(self.REG_QUAT_Z)
            
            # Read covariance (diagonal std)
            cov_pos = self._read_float(self.REG_COV_POS)
            cov_vel = self._read_float(self.REG_COV_VEL)
            cov_att = self._read_float(self.REG_COV_ATT)
            
            # Build covariance matrix (diagonal approximation)
            covariance = np.diag([cov_pos**2, cov_pos**2, cov_pos**2,
                                  cov_vel**2, cov_vel**2, cov_vel**2,
                                  cov_att**2, cov_att**2, cov_att**2])
            
            return {
                'position': np.array([pos_n, pos_e, pos_d]),
                'velocity': np.array([vel_n, vel_e, vel_d]),
                'quaternion': np.array([quat_w, quat_x, quat_y, quat_z]),
                'covariance': covariance[:3, :3]  # Position only for TRN
            }
            
        except Exception as e:
            logger.error(f"Error reading INS state: {e}")
            return None
    
    def read_altitude_profile(self) -> Optional[List[Dict]]:
        """Read altitude profile samples from FPGA"""
        try:
            # Check profile count
            status = self._read_reg(self.REG_PROF_STATUS)
            count = status & 0xFF
            
            if count == 0:
                return None
            
            samples = []
            for _ in range(count):
                # Read 128-bit profile entry
                # Format: terrain_alt(32) + pos_n(32) + pos_e(32) + timestamp(24) + valid(8)
                data = self._read_profile_entry()
                if data is not None:
                    samples.append(data)
            
            return samples
            
        except Exception as e:
            logger.error(f"Error reading altitude profile: {e}")
            return None
    
    def send_trn_fix(
        self,
        pos_n: float,
        pos_e: float,
        cov_nn: float,
        cov_ee: float,
        cov_ne: float
    ) -> bool:
        """Send TRN position fix to FPGA"""
        try:
            # Write fix data
            self._write_float(self.REG_TRN_POS_N, pos_n)
            self._write_float(self.REG_TRN_POS_E, pos_e)
            self._write_float(self.REG_TRN_COV_NN, cov_nn)
            self._write_float(self.REG_TRN_COV_EE, cov_ee)
            self._write_float(self.REG_TRN_COV_NE, cov_ne)
            
            # Trigger update
            self._write_reg(self.REG_TRN_CTRL, 0x01)
            
            logger.debug(f"TRN fix sent: N={pos_n:.1f}, E={pos_e:.1f}")
            return True
            
        except Exception as e:
            logger.error(f"Error sending TRN fix: {e}")
            return False
    
    def _read_reg(self, addr: int) -> int:
        """Read 32-bit register"""
        cmd = [0x00, (addr >> 8) & 0xFF, addr & 0xFF, 0x00, 0x00, 0x00, 0x00]
        response = self.spi.xfer2(cmd)
        return struct.unpack('<I', bytes(response[3:7]))[0]
    
    def _write_reg(self, addr: int, value: int):
        """Write 32-bit register"""
        data = struct.pack('<I', value)
        cmd = [0x01, (addr >> 8) & 0xFF, addr & 0xFF] + list(data)
        self.spi.xfer2(cmd)
    
    def _read_float(self, addr: int) -> float:
        """Read IEEE 754 float from register"""
        raw = self._read_reg(addr)
        return struct.unpack('<f', struct.pack('<I', raw))[0]
    
    def _write_float(self, addr: int, value: float):
        """Write IEEE 754 float to register"""
        raw = struct.unpack('<I', struct.pack('<f', value))[0]
        self._write_reg(addr, raw)
    
    def _read_profile_entry(self) -> Optional[Dict]:
        """Read single profile entry"""
        # Trigger read
        self._write_reg(self.REG_PROF_CTRL, 0x01)
        
        # Read 4 x 32-bit words
        terrain_alt = self._read_float(self.REG_PROF_DATA + 0)
        pos_n = self._read_float(self.REG_PROF_DATA + 4)
        pos_e = self._read_float(self.REG_PROF_DATA + 8)
        ts_valid = self._read_reg(self.REG_PROF_DATA + 12)
        
        timestamp = (ts_valid >> 8) & 0xFFFFFF
        valid = ts_valid & 0xFF
        
        if valid == 0:
            return None
        
        return {
            'terrain_alt': terrain_alt,
            'pos_n': pos_n,
            'pos_e': pos_e,
            'timestamp': timestamp / 100000.0  # Convert to seconds
        }
```

---

## config.yaml

```yaml
# INS + TRN Navigation System Configuration

fpga:
  spi_bus: 0
  spi_device: 0
  spi_speed: 1000000  # 1 MHz
  poll_rate: 100      # Hz

dted:
  path: "/home/pi/data/dted"
  cache_size: 10
  default_level: 2

tercom:
  update_rate: 0.1          # Hz (every 10 seconds)
  search_radius: 500        # meters
  grid_spacing: 10          # meters
  min_profile_length: 20    # samples
  correlation_method: "mad" # "mad" or "ncc"

reference:
  latitude: 35.0           # degrees
  longitude: -106.0        # degrees
  altitude: 1500           # meters MSL

logging:
  level: "INFO"
  file: "/var/log/ins_trn.log"
```

---

## requirements.txt

```
numpy>=1.24.0
scipy>=1.10.0
spidev>=3.5
PyYAML>=6.0
```

---

# Raspberry Pi Pico 2W - Sensor Hub

## Directory Structure

```
pico_telemetry/
├── src/
│   ├── main.c
│   ├── telemetry.c
│   ├── backup_imu.c
│   └── wifi_link.c
├── include/
│   ├── telemetry.h
│   ├── backup_imu.h
│   └── wifi_link.h
├── CMakeLists.txt
└── pico_sdk_import.cmake
```

---

## main.c

```c
/**
 * INS + TRN Navigation System
 * Raspberry Pi Pico 2W - Telemetry and Backup IMU
 */

#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"
#include "hardware/uart.h"
#include "hardware/spi.h"
#include "hardware/gpio.h"
#include "hardware/timer.h"

#include "telemetry.h"
#include "backup_imu.h"
#include "wifi_link.h"

// Pin definitions
#define UART_PI5_TX_PIN 0
#define UART_PI5_RX_PIN 1
#define UART_PI5_BAUD   115200

#define STATUS_LED_PIN  25

// UART for Pi 5 communication
#define UART_PI5        uart0

// Global state
typedef struct {
    float position[3];      // NED meters
    float velocity[3];      // NED m/s
    float attitude[3];      // Roll, pitch, yaw radians
    float pos_uncertainty;  // meters
    uint32_t timestamp;     // milliseconds
    uint8_t nav_mode;       // 0=INS only, 1=INS+TRN
    uint8_t system_status;  // Bit flags
} nav_state_t;

static nav_state_t g_nav_state;
static uint32_t g_last_update = 0;

// Forward declarations
void init_hardware(void);
void process_pi5_data(void);
void update_telemetry(void);
void blink_status(void);

int main() {
    stdio_init_all();
    
    printf("INS+TRN Pico 2W Telemetry Starting...\n");
    
    init_hardware();
    
    // Initialize WiFi
    if (cyw43_arch_init()) {
        printf("WiFi init failed!\n");
        return -1;
    }
    
    // Initialize subsystems
    telemetry_init();
    backup_imu_init();
    wifi_link_init();
    
    printf("System initialized\n");
    
    // Main loop
    uint32_t last_telemetry = 0;
    uint32_t last_blink = 0;
    
    while (true) {
        uint32_t now = to_ms_since_boot(get_absolute_time());
        
        // Process incoming data from Pi 5
        process_pi5_data();
        
        // Update telemetry at 10 Hz
        if (now - last_telemetry >= 100) {
            update_telemetry();
            last_telemetry = now;
        }
        
        // Blink status LED
        if (now - last_blink >= 500) {
            blink_status();
            last_blink = now;
        }
        
        // Read backup IMU (if main IMU fails)
        backup_imu_update();
        
        // Small delay to prevent busy-waiting
        sleep_ms(1);
    }
    
    return 0;
}

void init_hardware(void) {
    // Initialize status LED
    gpio_init(STATUS_LED_PIN);
    gpio_set_dir(STATUS_LED_PIN, GPIO_OUT);
    
    // Initialize UART for Pi 5 communication
    uart_init(UART_PI5, UART_PI5_BAUD);
    gpio_set_function(UART_PI5_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(UART_PI5_RX_PIN, GPIO_FUNC_UART);
    
    // Set UART format: 8N1
    uart_set_format(UART_PI5, 8, 1, UART_PARITY_NONE);
    uart_set_fifo_enabled(UART_PI5, true);
    
    printf("Hardware initialized\n");
}

void process_pi5_data(void) {
    // Check for incoming data from Pi 5
    static uint8_t rx_buffer[256];
    static uint8_t rx_idx = 0;
    
    while (uart_is_readable(UART_PI5)) {
        uint8_t c = uart_getc(UART_PI5);
        
        // Simple packet format: $NAV,pos_n,pos_e,pos_d,vel_n,vel_e,vel_d,roll,pitch,yaw,unc,mode*CS\n
        if (c == '$') {
            rx_idx = 0;
        }
        
        rx_buffer[rx_idx++] = c;
        
        if (c == '\n' || rx_idx >= sizeof(rx_buffer) - 1) {
            rx_buffer[rx_idx] = '\0';
            
            // Parse packet
            if (strncmp((char*)rx_buffer, "$NAV,", 5) == 0) {
                float pos_n, pos_e, pos_d;
                float vel_n, vel_e, vel_d;
                float roll, pitch, yaw;
                float unc;
                int mode;
                
                int parsed = sscanf((char*)rx_buffer + 5,
                    "%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%d",
                    &pos_n, &pos_e, &pos_d,
                    &vel_n, &vel_e, &vel_d,
                    &roll, &pitch, &yaw,
                    &unc, &mode);
                
                if (parsed == 11) {
                    g_nav_state.position[0] = pos_n;
                    g_nav_state.position[1] = pos_e;
                    g_nav_state.position[2] = pos_d;
                    g_nav_state.velocity[0] = vel_n;
                    g_nav_state.velocity[1] = vel_e;
                    g_nav_state.velocity[2] = vel_d;
                    g_nav_state.attitude[0] = roll;
                    g_nav_state.attitude[1] = pitch;
                    g_nav_state.attitude[2] = yaw;
                    g_nav_state.pos_uncertainty = unc;
                    g_nav_state.nav_mode = mode;
                    g_nav_state.timestamp = to_ms_since_boot(get_absolute_time());
                    
                    g_last_update = g_nav_state.timestamp;
                }
            }
            
            rx_idx = 0;
        }
    }
}

void update_telemetry(void) {
    uint32_t now = to_ms_since_boot(get_absolute_time());
    
    // Check for stale data
    if (now - g_last_update > 1000) {
        g_nav_state.system_status |= 0x01;  // Data stale flag
    } else {
        g_nav_state.system_status &= ~0x01;
    }
    
    // Send telemetry over WiFi
    telemetry_send(&g_nav_state);
    
    // Log to SD card (if available)
    telemetry_log(&g_nav_state);
}

void blink_status(void) {
    static bool led_on = false;
    
    // Blink pattern indicates status
    // Fast blink = error
    // Slow blink = normal
    // Solid = TRN active
    
    if (g_nav_state.system_status & 0x01) {
        // Error - fast blink
        led_on = !led_on;
    } else if (g_nav_state.nav_mode == 1) {
        // TRN active - solid
        led_on = true;
    } else {
        // Normal - slow blink
        led_on = !led_on;
    }
    
    cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, led_on);
}
```

---

## telemetry.h

```c
#ifndef TELEMETRY_H
#define TELEMETRY_H

#include <stdint.h>

typedef struct {
    float position[3];
    float velocity[3];
    float attitude[3];
    float pos_uncertainty;
    uint32_t timestamp;
    uint8_t nav_mode;
    uint8_t system_status;
} nav_state_t;

void telemetry_init(void);
void telemetry_send(const nav_state_t *state);
void telemetry_log(const nav_state_t *state);

#endif
```

---

## telemetry.c

```c
/**
 * Telemetry transmission and logging
 */

#include <stdio.h>
#include <string.h>
#include "telemetry.h"
#include "wifi_link.h"

static char tx_buffer[512];

void telemetry_init(void) {
    printf("Telemetry initialized\n");
}

void telemetry_send(const nav_state_t *state) {
    // Format telemetry packet
    int len = snprintf(tx_buffer, sizeof(tx_buffer),
        "{\"t\":%lu,\"pos\":[%.2f,%.2f,%.2f],\"vel\":[%.2f,%.2f,%.2f],"
        "\"att\":[%.3f,%.3f,%.3f],\"unc\":%.1f,\"mode\":%d,\"status\":%d}\n",
        state->timestamp,
        state->position[0], state->position[1], state->position[2],
        state->velocity[0], state->velocity[1], state->velocity[2],
        state->attitude[0], state->attitude[1], state->attitude[2],
        state->pos_uncertainty,
        state->nav_mode,
        state->system_status
    );
    
    // Send over WiFi
    wifi_send(tx_buffer, len);
}

void telemetry_log(const nav_state_t *state) {
    // TODO: Log to SD card
    // For now, just print to console
    printf("NAV: pos=[%.1f,%.1f,%.1f] unc=%.1f mode=%d\n",
        state->position[0], state->position[1], state->position[2],
        state->pos_uncertainty, state->nav_mode);
}
```

---

## wifi_link.c

```c
/**
 * WiFi communication link
 */

#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"
#include "lwip/udp.h"
#include "lwip/ip_addr.h"

#include "wifi_link.h"

// WiFi configuration
#define WIFI_SSID       "NAV_TELEMETRY"
#define WIFI_PASSWORD   "navigation123"
#define UDP_PORT        5000
#define BROADCAST_IP    "255.255.255.255"

static struct udp_pcb *udp_pcb = NULL;
static ip_addr_t broadcast_addr;

void wifi_link_init(void) {
    // Enable station mode
    cyw43_arch_enable_sta_mode();
    
    printf("Connecting to WiFi: %s\n", WIFI_SSID);
    
    // Connect to WiFi
    int result = cyw43_arch_wifi_connect_timeout_ms(
        WIFI_SSID, WIFI_PASSWORD, 
        CYW43_AUTH_WPA2_AES_PSK, 
        30000
    );
    
    if (result != 0) {
        printf("WiFi connection failed: %d\n", result);
        return;
    }
    
    printf("WiFi connected\n");
    
    // Create UDP socket
    udp_pcb = udp_new();
    if (udp_pcb == NULL) {
        printf("Failed to create UDP socket\n");
        return;
    }
    
    // Set broadcast address
    IP4_ADDR(&broadcast_addr, 255, 255, 255, 255);
    
    printf("WiFi link initialized, broadcasting on port %d\n", UDP_PORT);
}

void wifi_send(const char *data, int len) {
    if (udp_pcb == NULL) return;
    
    // Allocate pbuf
    struct pbuf *p = pbuf_alloc(PBUF_TRANSPORT, len, PBUF_RAM);
    if (p == NULL) return;
    
    // Copy data
    memcpy(p->payload, data, len);
    
    // Send
    udp_sendto(udp_pcb, p, &broadcast_addr, UDP_PORT);
    
    // Free pbuf
    pbuf_free(p);
}
```

---

## CMakeLists.txt

```cmake
cmake_minimum_required(VERSION 3.13)

include(pico_sdk_import.cmake)

project(pico_telemetry C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

pico_sdk_init()

add_executable(pico_telemetry
    src/main.c
    src/telemetry.c
    src/backup_imu.c
    src/wifi_link.c
)

target_include_directories(pico_telemetry PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(pico_telemetry
    pico_stdlib
    pico_cyw43_arch_lwip_threadsafe_background
    hardware_spi
    hardware_uart
    hardware_gpio
    hardware_timer
)

pico_enable_stdio_usb(pico_telemetry 1)
pico_enable_stdio_uart(pico_telemetry 0)

pico_add_extra_outputs(pico_telemetry)
```

---

# Build Instructions

## FPGA (Vivado)

```bash
# Create project
cd ins_trn_fpga
vivado -mode batch -source scripts/build.tcl

# Or interactive
vivado &
# File -> New Project -> RTL Project
# Add sources from rtl/
# Add constraints from constraints/
# Run Synthesis, Implementation, Generate Bitstream
```

## Raspberry Pi 5

```bash
# Install dependencies
sudo apt update
sudo apt install python3-pip python3-venv
cd pi5_tercom
python3 -m venv venv
source venv/bin/activate
pip install -r requirements.txt

# Run
python src/main.py
```

## Raspberry Pi Pico 2W

```bash
# Setup Pico SDK
cd ~
git clone https://github.com/raspberrypi/pico-sdk.git
cd pico-sdk
git submodule update --init
export PICO_SDK_PATH=~/pico-sdk

# Build
cd pico_telemetry
mkdir build && cd build
cmake ..
make -j4

# Flash
# Hold BOOTSEL, connect USB, release
# Copy pico_telemetry.uf2 to RPI-RP2 drive
```

---

# Testing

## Unit Tests

```bash
# FPGA simulation
cd ins_trn_fpga
vivado -mode batch -source scripts/sim.tcl

# Python tests
cd pi5_tercom
pytest tests/
```

## Integration Test

1. Load FPGA bitstream
2. Start Pi 5 application
3. Start Pico telemetry
4. Monitor WiFi telemetry output
5. Verify TRN fixes are applied

## Performance Targets

| Metric | Target | Notes |
|--------|--------|-------|
| IMU Rate | 400 Hz | ADIS16500 burst mode |
| INS Update | 400 Hz | Position, velocity, attitude |
| TRN Update | 0.1 Hz | Every 10 seconds |
| Position Accuracy | <30m CEP | After TRN fix |
| Telemetry Rate | 10 Hz | WiFi broadcast |

---

# License

MIT License - See LICENSE for details.

---

# References

1. Groves, P.D. *Principles of GNSS, Inertial, and Multisensor Integrated Navigation Systems*
2. Savage, P.G. *Strapdown Analytics*
3. ADIS16500 Datasheet - Analog Devices
4. Xilinx UG585 - Zynq-7000 TRM
5. Raspberry Pi Pico SDK Documentation
