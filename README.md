# Projects

This repository contains an inventory management system for drone components and a Link 16 simulation gateway for UAS interoperability testing.

---

## Repository Contents

```
CTDO/
├── inventory.py                              # Drone inventory management application
├── requirements.txt                          # Python dependencies
├── AFCENT_CTDO_Drone_Component_Laydown.xlsx  # Master inventory data
├── README.md
└── droneaction/                              # Link 16 Simulation Gateway
    ├── Link16_Defs.h
    ├── BitPacker.h
    ├── BitPacker.cpp
    └── main.cpp
```

---

## Drone Component Inventory Manager

A Streamlit web application for managing drone component inventory with real-time tracking, low-stock alerts, and order request generation.

### Features

- Multi-sheet dashboard displaying all inventory categories in organized tabs
- Low stock highlighting for quantities under threshold
- Image storage capability for each inventory item (SQLite backend)
- Automated order request generation for items at critical stock levels
- File upload support to refresh inventory data
- Search and filter functionality across all columns

### Inventory Categories

| Sheet | Description |
|-------|-------------|
| On Hand Drone Parts List | General drone components (124 items) |
| On Hand Battery Inventory | Battery stock (18 items) |
| Trapper Part List | Trapper system components (14 items) |
| SICA BTU BOM | SICA Bill of Materials (70 items) |
| SICA Tools List | SICA tooling inventory (79 items) |
| REDDI Part List | REDDI system components (52 items) |
| BlackBird Part List | BlackBird system components (42 items) |
| Open Purchase Request 1 | Pending orders (50 items) |
| Open Purchase Request 2 | Pending orders (77 items) |

### Streamlit Application Note

Note: The SQLite database for images and order requests is created at runtime. On Streamlit Cloud free tier, this data resets when the application enters sleep mode. It's free. Get over it. 

---

## Link 16 Simulation Gateway (MAVLink-to-DIS)

A C++ software gateway that translates simulated UAS telemetry into Distributed Interactive Simulation (DIS) protocol packets. The system formats data into Link 16 J-Series messages (J2.2 Air PPLI), encapsulates them per SISO-STD-002-2021, and transmits them as DIS Signal PDUs over UDP.

This software enables commercial or simulated drones to interoperate with military simulation networks and Ground Control Stations that utilize DIS without requiring classified cryptographic hardware.

### Compliance and Standards

| Standard | Description |
|----------|-------------|
| SISO-STD-002-2021 | Standard for Link 16 Simulation |
| IEEE 1278.1 | Distributed Interactive Simulation Application Protocols |
| MIL-STD-6016 (Simulated) | Unclassified open-simulation quantization for J-Series message packing |

### Gateway Components

| File | Purpose |
|------|---------|
| Link16_Defs.h | DIS PDU structures and SISO-STD-002 Simulation Headers |
| BitPacker.h | Bit-manipulation logic header |
| BitPacker.cpp | Bit-packing algorithms for 75-bit J-Series words |
| main.cpp | Entry point, network socket management, simulation loop |

### Prerequisites

- C++ compiler supporting C++11 or higher
- POSIX-compliant networking libraries

### Compilation

Linux (Ubuntu/Debian/CentOS):
```bash
sudo apt-get update && sudo apt-get install g++
g++ main.cpp BitPacker.cpp -o gateway
```

Raspberry Pi OS:
```bash
sudo apt-get install build-essential
g++ main.cpp BitPacker.cpp -o gateway
```

macOS:
```bash
xcode-select --install
clang++ main.cpp BitPacker.cpp -o gateway
```

Windows (via WSL):
```powershell
wsl --install
```

Then in the Ubuntu terminal:
```bash
sudo apt-get update && sudo apt-get install g++
cd /mnt/c/Users/<your-path>/CTDO/droneaction
g++ main.cpp BitPacker.cpp -o gateway
```

### Configuration

Edit `main.cpp` to set your target GCS address:

```cpp
const char* GCS_IP = "192.168.1.100";
const int GCS_PORT = 3000;
```

### Execution

```bash
./gateway
```

The program transmits DIS Signal PDUs at approximately 10 Hz. Terminal output confirms packet transmission and displays simulated coordinate data.

### Verification

To verify output without a dedicated GCS:

1. Install Wireshark on the target machine
2. Start a capture on the relevant network interface
3. Apply display filter: `dis`
4. Inspect incoming Signal PDU packets
5. Expand the Data field to view the SISO-STD-002 Link 16 Simulation Header and J-Series payload

---

## Disclaimer

The Link 16 Simulation Gateway is intended strictly for educational, research, and simulation purposes. It implements unclassified simulation standards defined by SISO-STD-002. It does not contain, process, or generate classified cryptographic variables, waveforms, or restricted military data. Do not attempt to interface with operational cryptographic terminals (CCI) without proper authorization and hardware certification.

---
