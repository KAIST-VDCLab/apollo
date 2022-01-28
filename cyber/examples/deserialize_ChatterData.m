function [err, timestamp, lidar_timestamp, seq, dump] = deserialize_ChatterData(bytes, bytes_length)
% deserialize_ChatterData Parse microbuf message ChatterData (version 1)

err = true;

timestamp = uint32(0);
lidar_timestamp = uint32(0);
seq = uint32(0);
dump = uint32(0);

if bytes_length < 24
    return
end

idx = 1;

[idx, err] = microbuf.check_fixarray(bytes, bytes_length, 4, idx);
if err ;return; end

[idx, err, timestamp] = microbuf.parse_uint32(bytes, bytes_length, idx);
if err ;return; end
[idx, err, lidar_timestamp] = microbuf.parse_uint32(bytes, bytes_length, idx);
if err ;return; end
[idx, err, seq] = microbuf.parse_uint32(bytes, bytes_length, idx);
if err ;return; end
[idx, err, dump] = microbuf.parse_uint32(bytes, bytes_length, idx);
if err ;return; end
[err] = microbuf.check_crc(bytes, bytes_length, idx);

end