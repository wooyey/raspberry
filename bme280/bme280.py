from enum import IntEnum

class Device(IntEnum):
    ADDRESS = 0x76
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
    MEASURING = 0b100
    IM_UPDATE = 0b001

class Osrs_p(IntEnum):
    offset = 3
    SKIPPED = 0b000 << offset
    OVERSAMPLING_1 = 0b001 << offset
    OVERSAMPLING_2 = 0b010 << offset
    OVERSAMPLING_4 = 0b011 << offset
    OVERSAMPLING_8 = 0b100 << offset
    OVERSAMPLING_16 = 0b101 << offset

class Osrs_t(IntEnum):
    offset = 6
    SKIPPED = 0b000 << offset
    OVERSAMPLING_1 = 0b001 << offset
    OVERSAMPLING_2 = 0b010 << offset
    OVERSAMPLING_4 = 0b011 << offset
    OVERSAMPLING_8 = 0b100 << offset
    OVERSAMPLING_16 = 0b101 << offset

class Mode(IntEnum):
    SLEEP = 0b00
    FORCED = 0b01
    NORMAL = 0b11


