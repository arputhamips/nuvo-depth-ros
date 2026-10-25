std_msgs/Header header
string device_uuid
string run_uuid
uint64 capture_epoch
uint64 capture_sequence
uint64 configuration_epoch
uint64 stream_sequence
int64 source_capture_time_ns
string source_capture_clock
string calibration_sha256
uint8 calibration_authority
# local_boottime, receipt, replay_rebased, or replay_clock.
# Receipt timestamps are arrival time and do not establish acquisition latency.
string timestamp_mode
int64 timestamp_uncertainty_ns
bool replay
