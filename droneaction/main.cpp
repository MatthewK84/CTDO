// main.cpp
// MAVLink-to-DIS Gateway (Simulated)
// Compile with: g++ main.cpp BitPacker.cpp -o gateway

#include <iostream>
#include <vector>
#include <cstring>
#include <unistd.h>      // For usleep
#include <arpa/inet.h>   // For sockets
#include "Link16_Defs.h"
#include "BitPacker.h"

int main() {
    // --- Configuration ---
    const char* GCS_IP = "192.168.1.100"; // Change to your GCS IP
    const int GCS_PORT = 3000;            // Standard DIS UDP Port

    // --- Socket Setup ---
    int sock = socket(AF_INET, SOCK_DGRAM, 0);
    if (sock < 0) {
        perror("Socket creation failed");
        return 1;
    }

    sockaddr_in dest_addr;
    memset(&dest_addr, 0, sizeof(dest_addr));
    dest_addr.sin_family = AF_INET;
    dest_addr.sin_port = htons(GCS_PORT);
    inet_pton(AF_INET, GCS_IP, &dest_addr.sin_addr);

    std::cout << "Starting Link 16 Gateway..." << std::endl;
    std::cout << "Target GCS: " << GCS_IP << ":" << GCS_PORT << std::endl;

    // --- Simulation Variables (Replace with MAVLink listener) ---
    double drone_lat = 34.0522;
    double drone_lon = -118.2437;
    double drone_alt = 1500.0;

    // --- Main Loop ---
    while (true) {
        // 1. Generate the J-Series Payload (The "Letter")
        BitPacker packer;
        packer.packJ2_2(drone_lat, drone_lon, drone_alt);
        std::vector<uint8_t> j_message = packer.getData();

        // 2. Prepare Link 16 Simulation Header (The "Envelope")
        Link16SimHeader sim_header;
        memset(&sim_header, 0, sizeof(sim_header));
        sim_header.npg_number = htons(6);        // NPG 6 (PPLI)
        sim_header.tsec_cvll = 255;              // Wildcard (Unclassified)
        sim_header.msec_cvll = 255;              // Wildcard (Unclassified)
        sim_header.siso_version = 1;             // SISO-STD-002-2021
        sim_header.timeslot_id = 0xFFFFFFFF;     // Wildcard
        sim_header.tx_time_sec = 0xFFFFFFFF;     // Wildcard
        sim_header.tx_time_frac = 0xFFFFFFFF;    // Wildcard

        // 3. Prepare DIS Signal PDU (The "Mail Truck")
        SignalPDUBase pdu;
        memset(&pdu, 0, sizeof(pdu));
        
        pdu.header.protocol_version = 7;
        pdu.header.pdu_type = 26; // Signal PDU
        pdu.header.protocol_family = 4;
        pdu.tdl_type = htons(TDL_TYPE_LINK16);
        
        // Encoding Scheme Calculation:
        // Bits 0-13: Number of Words (2 words in our J2.2)
        // Bits 14-15: Encoding Class (1 = Raw Binary)
        // 0100 0000 0000 0010 = 0x4002
        uint16_t encoding_val = 0x4002;
        pdu.encoding_scheme = htons(encoding_val);

        // Calculate Data Length in BITS
        // (Header Bytes + Message Bytes) * 8
        int total_data_bytes = sizeof(Link16SimHeader) + j_message.size();
        pdu.data_length = htons(total_data_bytes * 8);

        // Calculate Total PDU Length in BYTES
        pdu.header.length = htons(sizeof(SignalPDUBase) + total_data_bytes);

        // 4. Serialize into one buffer
        std::vector<uint8_t> packet;
        
        // Append PDU Base
        uint8_t* pdu_ptr = (uint8_t*)&pdu;
        packet.insert(packet.end(), pdu_ptr, pdu_ptr + sizeof(SignalPDUBase));
        
        // Append Link 16 Sim Header
        uint8_t* header_ptr = (uint8_t*)&sim_header;
        packet.insert(packet.end(), header_ptr, header_ptr + sizeof(Link16SimHeader));
        
        // Append J-Message Bits
        packet.insert(packet.end(), j_message.begin(), j_message.end());

        // 5. Send
        sendto(sock, packet.data(), packet.size(), 0, (const sockaddr*)&dest_addr, sizeof(dest_addr));

        // 6. Simulate Movement (10Hz)
        drone_lat += 0.0001; 
        usleep(100000); 
    }

    close(sock);
    return 0;
}
