function bytes = serialize_ChatterData(timestamp, lidar_timestamp, seq, dump)
% serialize_ChatterData Serialize microbuf message ChatterData (version 1)

bytes = repmat(uint8(0), 1, 24);

bytes(1:1) = microbuf.gen_fixarray(4);

bytes(2:6) = microbuf.gen_uint32(timestamp);

bytes(7:11) = microbuf.gen_uint32(lidar_timestamp);

bytes(12:16) = microbuf.gen_uint32(seq);

bytes(17:21) = microbuf.gen_uint32(dump);

bytes(22:24) = microbuf.gen_uint16(microbuf.crc16_aug_ccitt(bytes, 21));

end