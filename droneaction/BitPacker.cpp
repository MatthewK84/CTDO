// BitPacker.cpp
#include "BitPacker.h"

BitPacker::BitPacker() {
    current_bit = 0;
    // Initialize with 1 word (10 bytes / 80 bits)
    buffer.resize(10, 0); 
}

void BitPacker::addWord() {
    buffer.resize(buffer.size() + 10, 0);
}

void BitPacker::pack(uint64_t value, int num_bits) {
    for (int i = 0; i < num_bits; i++) {
        // Check if the i-th bit of 'value' is set
        if ((value >> i) & 1) {
            int byte_pos = current_bit / 8;
            // Link 16 Simulation standard often uses Big Endian byte order
            // We fill from MSB (7) down to LSB (0) inside the byte
            int bit_pos = 7 - (current_bit % 8); 
            
            buffer[byte_pos] |= (1 << bit_pos);
        }
        current_bit++;
    }
}

void BitPacker::packJ2_2(double lat, double lon, double alt_ft) {
    // --- WORD 1: Header (35 bits + padding) ---
    pack(0, 2);  // Initial Word (00)
    pack(2, 5);  // Label 2 (PPLI)
    pack(2, 3);  // Sublabel 2 (Air)
    
    // In a real J-Series message, Source Track Number (STN) would go here.
    // We will simulate STN = 011 (octal)
    pack(9, 15); // STN
    
    // Pad the rest of the 75-bit word + 5 bit parity to reach 80 bits
    pack(0, 50); // Padding
    pack(0, 5);  // Parity (Simulated as 0)

    addWord(); // Move to Word 2

    // --- WORD 2: Extension (Position) ---
    pack(2, 2);  // Extension Word (10)

    // Latitude (24 bits)
    // Scale: 180 degrees / 2^24
    uint32_t lat_encoded = (uint32_t)((lat + 90.0) * (pow(2.0, 24) / 180.0));
    pack(lat_encoded, 24);

    // Longitude (25 bits)
    // Scale: 360 degrees / 2^25
    uint32_t lon_encoded = (uint32_t)((lon + 180.0) * (pow(2.0, 25) / 360.0));
    pack(lon_encoded, 25);

    // Altitude (14 bits)
    // Scale: -2000 to 60000 ft range mapped to 14 bits
    uint32_t alt_encoded = (uint32_t)((alt_ft + 2000.0) * (pow(2.0, 14) / 62000.0));
    pack(alt_encoded, 14);

    // Padding for remainder of word
    pack(0, 10); 
    pack(0, 5);  // Parity
}

std::vector<uint8_t> BitPacker::getData() {
    return buffer;
}
