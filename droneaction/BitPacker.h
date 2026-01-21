// BitPacker.h
// Helper class to pack C++ types into 75-bit J-Series words

#ifndef BITPACKER_H
#define BITPACKER_H

#include <vector>
#include <cstdint>
#include <cmath>

class BitPacker {
private:
    std::vector<uint8_t> buffer;
    int current_bit;

public:
    BitPacker();

    // Resize buffer to add a new 75-bit word (padded to 80 bits/10 bytes)
    void addWord();

    // Pack specific bits into the buffer
    // value: The integer data to pack
    // num_bits: How many bits this field occupies
    void pack(uint64_t value, int num_bits);

    // Create a J2.2 Air PPLI Payload using simulation standards
    // lat/lon: Decimal degrees
    // alt_ft: Altitude in feet
    void packJ2_2(double lat, double lon, double alt_ft);

    // Retrieve the raw byte buffer for transmission
    std::vector<uint8_t> getData();
};

#endif // BITPACKER_H
