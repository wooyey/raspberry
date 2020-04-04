from enum import IntEnum

class Device(IntEnum):
    ADDRESS = 0x76  # ???
    ID = 0x60

class Register(IntEnum):  
    ID = 0xd0
    RESET = 0xe0
    CTRL_HUM = 0xf2
    CTRL_MEAS = 0xf4
    CONFIG = 0xf5
    STATUS = 0xf3
    CALIB_00 = 0x88
    CALIB_25 = 0xa1
    CALIB_26 = 0xe1
    CALIB_41 = 0xf0
    HUM_LSB = 0xfe
    HUM_MSB = 0xfd
    TEMP_XLSB = 0xfc
    TEMP_LSB = 0xfb
    TEMP_MSB = 0xfa
    PRESS_XLSB = 0xf9
    PRESS_LSB = 0xf8
    PRESS_MSB = 0xf7

class Ctrl_hum(IntEnum):
    SKIPPED = 0b000
    OVERSAMPLING_1 = 0b001
    OVERSAMPLING_2 = 0b010
    OVERSAMPLING_4 = 0b011
    OVERSAMPLING_8 = 0b100
    OVERSAMPLING_16 = 0b101

class Status(IntEnum):
    MEASURING = 0b1000
    IM_UPDATE = 0b001

class Ctrl_meas(IntEnum):
    _osrs_p_offset = 3
    _osrs_t_offset = 6
    PRESSURE_SKIPPED = 0b000 << _osrs_p_offset
    PRESSURE_OVERSAMPLING_1 = 0b001 << _osrs_p_offset
    PRESSURE_OVERSAMPLING_2 = 0b010 << _osrs_p_offset
    PRESSURE_OVERSAMPLING_4 = 0b011 << _osrs_p_offset
    PRESSURE_OVERSAMPLING_8 = 0b100 << _osrs_p_offset
    PRESSURE_OVERSAMPLING_16 = 0b101 << _osrs_p_offset
    TEMPERATURE_SKIPPED = 0b000 << _osrs_t_offset
    TEMPERATURE_OVERSAMPLING_1 = 0b001 << _osrs_t_offset
    TEMPERATURE_OVERSAMPLING_2 = 0b010 << _osrs_t_offset
    TEMPERATURE_OVERSAMPLING_4 = 0b011 << _osrs_t_offset
    TEMPERATURE_OVERSAMPLING_8 = 0b100 << _osrs_t_offset
    TEMPERATURE_OVERSAMPLING_16 = 0b101 << _osrs_t_offset
    SLEEP = 0b00
    FORCED = 0b01
    NORMAL = 0b11

class Config(IntEnum):
    _t_sb_offset = 6
    _filter_offset = 2
    STANDBY_0 = 0b000 << _t_sb_offset
    STANDBY_62 = 0b001 << _t_sb_offset
    STANDBY_125 = 0b010 << _t_sb_offset
    STANDBY_250 = 0b011 << _t_sb_offset
    STANDBY_500 = 0b100 << _t_sb_offset
    STANDBY_1000 = 0b101 << _t_sb_offset
    STANDBY_10 = 0b110 << _t_sb_offset
    STANDBY_20 = 0b111 << _t_sb_offset
    FILTER_OFF = 0b000 << _filter_offset
    FILTER_2 = 0b001 << _filter_offset
    FILTER_4 = 0b010 << _filter_offset
    FILTER_8 = 0b011 << _filter_offset
    FILTER_16 = 0b100 << _filter_offset
    SPI_ENABLE = 0b1
    SPI_DISABLE = 0b0

