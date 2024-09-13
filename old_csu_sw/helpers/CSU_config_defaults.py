from typing import NamedTuple


class CSU_motor_config_defaults(NamedTuple):
    motor_type: int = 10
    nominal_current_ma: int = 1000
    output_current_limit_ma: int = 1000
    number_of_pole_pairs: int = 7
    thermal_time_constant_winding_ms: int = 100
    torque_constant_uNm_A: int = 100
    max_motor_speed_rpm: int = 1000

class CSU_gear_config_defaults(NamedTuple):
    gear_reduction_numberator: int = 1
    gear_reduction_denominator: int = 1
    gear_max_input_speed_rpm: int = 1000
    orientation: int = 1

class CSU_digital_incremental_encoder_config_defaults(NamedTuple):
    number_of_pulses_per_turn: int = 1000
    encoder_type: int = 1
    direction: int = 1
    method: int = 1

class CSU_ssi_encoder_config_defaults(NamedTuple):
    data_rate: int = 1000000
    number_of_bits: int = 24
    encoding_type: int = 1
    direction: int = 1
    check_frame: int = 1
    timeout_time_ms: int = 1000
    number_of_multi_turn_bits: int = 12
    number_of_single_turn_bits: int = 12

