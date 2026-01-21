# Link 16 Simulation Gateway (MAVLink-to-DIS)

A C++ software gateway that translates simulated unmanned aerial system (UAS) telemetry into Distributed Interactive Simulation (DIS) protocol packets. The system formats data into Link 16 J-Series messages (J2.2 Air PPLI), encapsulates them per SISO-STD-002-2021, and transmits them as DIS Signal PDUs over UDP.

This software enables commercial or simulated drones to interoperate with military simulation networks and Ground Control Stations (GCS) that utilize DIS—without requiring classified cryptographic hardware.

---

## Compliance & Standards

| Standard | Description |
|----------|-------------|
| **SISO-STD-002-2021** | Standard for Link 16 Simulation |
| **IEEE 1278.1** | Distributed Interactive Simulation (DIS) Application Protocols |
| **MIL-STD-6016 (Simulated)** | Unclassified, open-simulation quantization for J-Series message packing |

---

## Project Structure

```
├── Link16_Defs.h    # DIS PDU structures and SISO-STD-002 Simulation Headers
├── BitPacker.h      # Bit-manipulation logic (header)
├── BitPacker.cpp    # Bit-packing algorithms for 75-bit J-Series words
└── main.cpp         # Entry point, network socket, simulation loop
```

---

## Prerequisites

- C++ compiler supporting **C++11** or higher
- POSIX-compliant networking libraries (`<arpa/inet.h>`, `<unistd.h>`)

---

## Compilation

### Linux (Ubuntu / Debian / CentOS)

```bash
sudo apt-get update && sudo apt-get install g++
g++ main.cpp BitPacker.cpp -o gateway
```

### Raspberry Pi OS

```bash
sudo apt-get install build-essential
g++ main.cpp BitPacker.cpp -o gateway
```

### macOS

```bash
xcode-select --install
clang++ main.cpp BitPacker.cpp -o gateway
```

### Windows (via WSL)

This software uses POSIX networking headers. Use Windows Subsystem for Linux:

```powershell
# In PowerShell (Admin)
wsl --install
```

Then in the Ubuntu terminal:

```bash
sudo apt-get update && sudo apt-get install g++
cd /mnt/c/Users/<your-path>
g++ main.cpp BitPacker.cpp -o gateway
```

---

## Usage

### 1. Configure

Edit `main.cpp` to set your target GCS address:

```cpp
const char* GCS_IP = "192.168.1.100";
const int GCS_PORT = 3000;
```

### 2. Run

```bash
./gateway
```

The program transmits DIS Signal PDUs at approximately **10 Hz**. Terminal output confirms packet transmission and displays simulated coordinate data.

---

## Verification

To verify output without a dedicated GCS:

1. Install [Wireshark](https://www.wireshark.org/) on the target machine
2. Start a capture on the relevant network interface
3. Apply display filter: `dis`
4. Inspect incoming **Signal PDU** packets
5. Expand the **Data** field to view the SISO-STD-002 Link 16 Simulation Header and J-Series payload

---

## Disclaimer

> **This software is intended strictly for educational, research, and simulation purposes.**
>
> It implements unclassified simulation standards defined by SISO-STD-002. It does not contain, process, or generate classified cryptographic variables, waveforms, or restricted military data.
>
> **Do not attempt to interface with operational cryptographic terminals (CCI) without proper authorization and hardware certification.**

---

## License

*Add your license here*
