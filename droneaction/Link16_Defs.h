// Link16_Defs.h
// Definitions based on SISO-STD-002-2021 and IEEE 1278.1 (DIS)

#ifndef LINK16_DEFS_H
#define LINK16_DEFS_H

#include <cstdint>

// --- Constants ---
// From SISO-STD-002-2021 Table 7
const uint16_t DIS_ENCODING_RAW_BINARY = 0x4000; // Bits 14-15 = 1 (Raw Binary)
const uint16_t TDL_TYPE_LINK16 = 100;            // Link 16 Standardized Format

// --- Standard DIS PDU Header ---
struct PDUHeader {
    uint8_t protocol_version = 7; // DIS PDU Version 7
    uint8_t exercise_id = 1;      // Exercise ID
    uint8_t pdu_type = 26;        // 26 = Signal PDU
    uint8_t protocol_family = 4;  // 4 = Radio Communications
    uint32_t timestamp = 0;       // Timestamp (placeholder)
    uint16_t length = 0;          // Total PDU length (bytes)
    uint16_t padding = 0;
};

// --- Signal PDU Base Structure ---
// Represents the fixed fields of the Signal PDU before the dynamic data.
struct SignalPDUBase {
    PDUHeader header;
    uint16_t site_id = 101;       // Simulation Site ID
    uint16_t app_id = 1;          // Application ID
    uint16_t entity_id = 1;       // Entity ID
    uint16_t radio_id = 1;        // Radio ID on this entity
    uint16_t encoding_scheme;     // Encoding Class (Raw Binary) + Word Count
    uint16_t tdl_type;            // 100 = Link 16
    uint32_t sample_rate = 0;     // 0 for data
    uint16_t data_length;         // Length of Data field in BITS
    uint16_t samples = 0;         // 0 for data
};

// --- Link 16 Simulation Network Header ---
// From SISO-STD-002-2021 Table 8
// Total Size: 160 bits (20 bytes)
struct Link16SimHeader {
    uint16_t npg_number;          // Network Participation Group (e.g. 6)
    uint8_t  net_number;          // Stacked Net Number
    uint8_t  tsec_cvll;           // Crypto Variable (255 = Wildcard)
    uint8_t  msec_cvll;           // Msg Security Crypto (255 = Wildcard)
    uint8_t  msg_type;            // 0 = JTIDS Header/Messages
    uint8_t  siso_version;        // 1 = SISO-STD-002-2021
    uint8_t  link16_version;      // 0 = No Statement
    uint32_t timeslot_id;         // 0xFFFFFFFF = Wildcard
    uint32_t tx_time_sec;         // 0xFFFFFFFF = Wildcard
    uint32_t tx_time_frac;        // 0xFFFFFFFF = Wildcard
};

#endif // LINK16_DEFS_H
