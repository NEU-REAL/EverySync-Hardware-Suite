nc 10.72.127.200 7501


<!-- get_sensor_info -->
set_config_param timestamp_mode TIME_FROM_SYNC_PULSE_IN
<!-- set_config_param -->
set_config_param multipurpose_io_mode INPUT_NMEA_UART
<!-- set_config_param -->
set_config_param sync_pulse_in_polarity ACTIVE_HIGH
<!-- set_config_param -->
set_config_param nmea_in_polarity ACTIVE_HIGH
<!-- set_config_param -->
set_config_param nmea_baud_rate BAUD_9600
<!-- set_config_param -->
set_config_param nmea_leap_seconds 37
set_config_param nmea_leap_seconds 0
<!-- set_config_param -->
reinitialize
<!-- reinitialize -->
write_config_txt
<!-- write_config_txt -->

get_time_info

{"timestamp": {"time": 1683439681.306032, "mode": "TIME_FROM_SYNC_PULSE_IN", "time_options": {"ptp_1588": 627, "sync_pulse_in": 1683439680, "internal_osc": 617}}, "sync_pulse_in": {"locked": 1, "polarity": "ACTIVE_HIGH", "diagnostics": {"last_period_nsec": 0, "count": 616, "count_unfiltered": 565}}, "multipurpose_io": {"mode": "INPUT_NMEA_UART", "sync_pulse_out": {"polarity": "ACTIVE_HIGH", "frequency_hz": 1, "angle_deg": 360, "pulse_width_ms": 0}, "nmea": {"locked": 1, "polarity": "ACTIVE_HIGH", "ignore_valid_char": 0, "baud_rate": "BAUD_9600", "leap_seconds": 0, "diagnostics": {"decoding": {"utc_decoded_count": 593, "date_decoded_count": 593, "not_valid_count": 0, "last_read_message": "GPRMC,060800.863771,A,2237.496474,N,11356.089515,E,0.0,225.5,070523,2.3,W,A*23"}, "io_checks": {"start_char_count": 593, "char_count": 48034, "bit_count": 145563, "bit_count_unfiltered": 145563}}}}}
{"timestamp": {"time": 1683439767.405309, "mode": "TIME_FROM_SYNC_PULSE_IN", "time_options": {"ptp_1588": 717, "sync_pulse_in": 1683439767, "internal_osc": 706}}, "sync_pulse_in": {"locked": 1, "polarity": "ACTIVE_HIGH", "diagnostics": {"last_period_nsec": 0, "count": 706, "count_unfiltered": 648}}, "multipurpose_io": {"mode": "INPUT_NMEA_UART", "sync_pulse_out": {"polarity": "ACTIVE_HIGH", "frequency_hz": 1, "angle_deg": 360, "pulse_width_ms": 0}, "nmea": {"locked": 1, "polarity": "ACTIVE_HIGH", "ignore_valid_char": 0, "baud_rate": "BAUD_9600", "leap_seconds": 0, "diagnostics": {"decoding": {"utc_decoded_count": 683, "date_decoded_count": 683, "not_valid_count": 0, "last_read_message": "GPRMC,060927.000027,A,2237.496474,N,11356.089515,E,0.0,225.5,070523,2.3,W,A*23"}, "io_checks": {"start_char_count": 683, "char_count": 55324, "bit_count": 167550, "bit_count_unfiltered": 167550}}}}}

