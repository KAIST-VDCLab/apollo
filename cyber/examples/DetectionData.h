#ifndef MICROBUF_MSG_DETECTIONDATA_H
#define MICROBUF_MSG_DETECTIONDATA_H
// Microbuf Message: DetectionData
// Version 1
#include "microbuf.h"

struct DetectionData_struct_t {
    static constexpr size_t data_size = 828;

    uint8_t send;
    uint16_t object_ID[10];
    uint8_t Flags[10];
    uint8_t classification[10];
    uint32_t class_age[10];
    float center_x[10];
    float center_y[10];
    float angle[10];
    float angle_sigma[10];
    float rel_vel_x[10];
    float rel_vel_y[10];
    float rel_vel_x_sigma[10];
    float rel_vel_y_sigma[10];
    float abs_vel_x[10];
    float abs_Vel_y[10];
    float abs_vel_x_sigma[10];
    float abs_vel_y_sigma[10];
    float bbox_x[10];
    float bbox_y[10];
    
    microbuf::array<uint8_t,828> as_bytes() const {
        microbuf::array<uint8_t,828> bytes {};
        microbuf::insert_bytes<0>(bytes, microbuf::gen_array16(181));
        microbuf::insert_bytes<3>(bytes, microbuf::gen_uint8(send));
        microbuf::insert_bytes<5>(bytes, microbuf::gen_multiple<10>(object_ID, microbuf::gen_uint16));
        microbuf::insert_bytes<35>(bytes, microbuf::gen_multiple<10>(Flags, microbuf::gen_uint8));
        microbuf::insert_bytes<55>(bytes, microbuf::gen_multiple<10>(classification, microbuf::gen_uint8));
        microbuf::insert_bytes<75>(bytes, microbuf::gen_multiple<10>(class_age, microbuf::gen_uint32));
        microbuf::insert_bytes<125>(bytes, microbuf::gen_multiple<10>(center_x, microbuf::gen_float32));
        microbuf::insert_bytes<175>(bytes, microbuf::gen_multiple<10>(center_y, microbuf::gen_float32));
        microbuf::insert_bytes<225>(bytes, microbuf::gen_multiple<10>(angle, microbuf::gen_float32));
        microbuf::insert_bytes<275>(bytes, microbuf::gen_multiple<10>(angle_sigma, microbuf::gen_float32));
        microbuf::insert_bytes<325>(bytes, microbuf::gen_multiple<10>(rel_vel_x, microbuf::gen_float32));
        microbuf::insert_bytes<375>(bytes, microbuf::gen_multiple<10>(rel_vel_y, microbuf::gen_float32));
        microbuf::insert_bytes<425>(bytes, microbuf::gen_multiple<10>(rel_vel_x_sigma, microbuf::gen_float32));
        microbuf::insert_bytes<475>(bytes, microbuf::gen_multiple<10>(rel_vel_y_sigma, microbuf::gen_float32));
        microbuf::insert_bytes<525>(bytes, microbuf::gen_multiple<10>(abs_vel_x, microbuf::gen_float32));
        microbuf::insert_bytes<575>(bytes, microbuf::gen_multiple<10>(abs_Vel_y, microbuf::gen_float32));
        microbuf::insert_bytes<625>(bytes, microbuf::gen_multiple<10>(abs_vel_x_sigma, microbuf::gen_float32));
        microbuf::insert_bytes<675>(bytes, microbuf::gen_multiple<10>(abs_vel_y_sigma, microbuf::gen_float32));
        microbuf::insert_bytes<725>(bytes, microbuf::gen_multiple<10>(bbox_x, microbuf::gen_float32));
        microbuf::insert_bytes<775>(bytes, microbuf::gen_multiple<10>(bbox_y, microbuf::gen_float32));
        microbuf::append_crc(bytes);
        return bytes;
    }
    
    bool from_bytes(const microbuf::array<uint8_t,828>& bytes) {
        bool worked = microbuf::check_array16<0>(bytes, 181);
        if(!worked) { return false; }
        worked = microbuf::parse_uint8<3>(bytes, send);
        if(!worked) { return false; }
        worked = microbuf::parse_multiple<10,5>(bytes, object_ID, microbuf::parse_uint16<0>);
        if(!worked) { return false; }
        worked = microbuf::parse_multiple<10,35>(bytes, Flags, microbuf::parse_uint8<0>);
        if(!worked) { return false; }
        worked = microbuf::parse_multiple<10,55>(bytes, classification, microbuf::parse_uint8<0>);
        if(!worked) { return false; }
        worked = microbuf::parse_multiple<10,75>(bytes, class_age, microbuf::parse_uint32<0>);
        if(!worked) { return false; }
        worked = microbuf::parse_multiple<10,125>(bytes, center_x, microbuf::parse_float32<0>);
        if(!worked) { return false; }
        worked = microbuf::parse_multiple<10,175>(bytes, center_y, microbuf::parse_float32<0>);
        if(!worked) { return false; }
        worked = microbuf::parse_multiple<10,225>(bytes, angle, microbuf::parse_float32<0>);
        if(!worked) { return false; }
        worked = microbuf::parse_multiple<10,275>(bytes, angle_sigma, microbuf::parse_float32<0>);
        if(!worked) { return false; }
        worked = microbuf::parse_multiple<10,325>(bytes, rel_vel_x, microbuf::parse_float32<0>);
        if(!worked) { return false; }
        worked = microbuf::parse_multiple<10,375>(bytes, rel_vel_y, microbuf::parse_float32<0>);
        if(!worked) { return false; }
        worked = microbuf::parse_multiple<10,425>(bytes, rel_vel_x_sigma, microbuf::parse_float32<0>);
        if(!worked) { return false; }
        worked = microbuf::parse_multiple<10,475>(bytes, rel_vel_y_sigma, microbuf::parse_float32<0>);
        if(!worked) { return false; }
        worked = microbuf::parse_multiple<10,525>(bytes, abs_vel_x, microbuf::parse_float32<0>);
        if(!worked) { return false; }
        worked = microbuf::parse_multiple<10,575>(bytes, abs_Vel_y, microbuf::parse_float32<0>);
        if(!worked) { return false; }
        worked = microbuf::parse_multiple<10,625>(bytes, abs_vel_x_sigma, microbuf::parse_float32<0>);
        if(!worked) { return false; }
        worked = microbuf::parse_multiple<10,675>(bytes, abs_vel_y_sigma, microbuf::parse_float32<0>);
        if(!worked) { return false; }
        worked = microbuf::parse_multiple<10,725>(bytes, bbox_x, microbuf::parse_float32<0>);
        if(!worked) { return false; }
        worked = microbuf::parse_multiple<10,775>(bytes, bbox_y, microbuf::parse_float32<0>);
        if(!worked) { return false; }
        worked = microbuf::verify_crc(bytes);
        return worked;
    }
};


#endif // MICROBUF_MSG_DETECTIONDATA_H
