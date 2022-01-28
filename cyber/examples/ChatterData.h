#ifndef MICROBUF_MSG_CHATTERDATA_H
#define MICROBUF_MSG_CHATTERDATA_H
// Microbuf Message: ChatterData
// Version 1
#include "microbuf.h"

struct ChatterData_struct_t {
    static constexpr size_t data_size = 24;

    uint32_t timestamp{};
    uint32_t lidar_timestamp{};
    uint32_t seq{};
    uint32_t dump{};
    
    microbuf::array<uint8_t,24> as_bytes() const {
        microbuf::array<uint8_t,24> bytes {};
        microbuf::insert_bytes<0>(bytes, microbuf::gen_fixarray(4));
        microbuf::insert_bytes<1>(bytes, microbuf::gen_uint32(timestamp));
        microbuf::insert_bytes<6>(bytes, microbuf::gen_uint32(lidar_timestamp));
        microbuf::insert_bytes<11>(bytes, microbuf::gen_uint32(seq));
        microbuf::insert_bytes<16>(bytes, microbuf::gen_uint32(dump));
        microbuf::append_crc(bytes);
        return bytes;
    }
    
    bool from_bytes(const microbuf::array<uint8_t,24>& bytes) {
        bool worked = microbuf::check_fixarray<0>(bytes, 4);
        if(!worked) { return false; }
        worked = microbuf::parse_uint32<1>(bytes, timestamp);
        if(!worked) { return false; }
        worked = microbuf::parse_uint32<6>(bytes, lidar_timestamp);
        if(!worked) { return false; }
        worked = microbuf::parse_uint32<11>(bytes, seq);
        if(!worked) { return false; }
        worked = microbuf::parse_uint32<16>(bytes, dump);
        if(!worked) { return false; }
        worked = microbuf::verify_crc(bytes);
        return worked;
    }
};


#endif // MICROBUF_MSG_CHATTERDATA_H
