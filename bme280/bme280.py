#!/usr/bin/env python
from enum import IntEnum
import ctypes
import smbus
import click

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
    CALIB_34 = 0xe7
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

    WEATHER_PROFILE = OVERSAMPLING_1

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

    WEATHER_PROFILE = PRESSURE_OVERSAMPLING_1 | \
                      TEMPERATURE_OVERSAMPLING_1 | \
                      FORCED

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

    WEATHER_PROFILE = FILTER_OFF

class NotSupportedDevice(Exception):
    pass

class Bme280(object):

    PROFILE = {
        'weather': {
            Register.CONFIG: Config.FILTER_OFF,
            Register.CTRL_MEAS: Ctrl_meas.FORCED | \
                                Ctrl_meas.PRESSURE_OVERSAMPLING_1 | \
                                Ctrl_meas.TEMPERATURE_OVERSAMPLING_1,
            Register.CTRL_HUM: Ctrl_hum.OVERSAMPLING_1
        }
    }

    def __init__(self, bus_addr, addr, profile='weather'):
        self.bus = smbus.SMBus(bus_addr)
        self.addr = addr
        self.profile = profile
        self._check_device_id()
        self._setup_readings()
        self.calib = self._get_calibration_data()


    def _setup_readings(self):
        for addr, value in self.PROFILE[self.profile].items():
            self.bus.write_byte_data(self.addr, addr, value)

    def _check_device_id(self):
        device_id = self.bus.read_byte_data(self.addr, Register.ID)
        if device_id != Device.ID:
            raise NotSupportedDevice(f"Device ID mismatch: {device_id} != {Device.ID}")

    def _get_calibration_data(self):
        data = self.bus.read_i2c_block_data(self.addr, Register.CALIB_00, 26) + \
            self.bus.read_i2c_block_data(self.addr, Register.CALIB_26, 8)

        result = {
            'T1': ctypes.c_ushort(data[0] | data[1] << 8).value,
            'T2': ctypes.c_short(data[2] | data[3] << 8).value,
            'T3': ctypes.c_short(data[4] | data[5] << 8).value,
            'P1': ctypes.c_ushort(data[6] | data[7] << 8).value,
            'P2': ctypes.c_short(data[8] | data[9] << 8).value,
            'P3': ctypes.c_short(data[10] | data[11] << 8).value,
            'P4': ctypes.c_short(data[12] | data[13] << 8).value,
            'P5': ctypes.c_short(data[14] | data[15] << 8).value,
            'P6': ctypes.c_short(data[16] | data[17] << 8).value,
            'P7': ctypes.c_short(data[18] | data[19] << 8).value,
            'P8': ctypes.c_short(data[20] | data[21] << 8).value,
            'P9': ctypes.c_short(data[22] | data[23] << 8).value,
            'H1': ctypes.c_ushort(data[26]).value,
            'H2': ctypes.c_short(data[27] | data[28] << 8).value,
            'H3': ctypes.c_ubyte(data[29]).value,
            'H4': ctypes.c_short(data[30] << 4| data[31]).value,
            'H5': ctypes.c_short(data[31] >> 4| data[32] << 4).value,
            'H6': ctypes.c_byte(data[33]).value
        }

        return result

    def compensate_temperature(self, value):
        v1 = (value / 16384 - self.calib['T1'] / 1024) * self.calib['T2']
        v2 = (value / 131072 - self.calib['T1'] / 8192) ** 2 * self.calib['T3']
        self.t_fine = int(v1 + v2)
        return self.t_fine / 5120

    def compensate_pressure(self, value):
        t1 = (self.t_fine / 2) - 64000
        t2 = t1 ** 2 * self.calib['P6'] / 32768
        t2 += t1 * self.calib['P5'] * 2
        t2 = (t2 / 4) + self.calib['P4'] * 65536
        t1 = self.calib['P3'] * t1 ** 2 / 524288 + self.calib['P2'] * t1 / 524288
        t1 = (1 + t1 / 32768) * self.calib['P1']
        if t1 == 0:
            return 0

        p = 1048576 - value
        p = (p - (t2 / 4096)) * 6250 / t1
        t1 = self.calib['P9'] * p ** 2 / 2147483648
        t2 = p * self.calib['P8'] / 32768
        p = p + (t1 + t2 + self.calib['P7']) / 16
        return p

    def compensate_humidity(self, value):
        h = self.t_fine - 76800
        h = (value - (self.calib['H4'] * 64 + self.calib['H5'] / 16384 * h)) * \
            (self.calib['H2'] / 65536) * (1 + self.calib['H6'] / 67108864 * h * (1 + self.calib['H3'] / 67108864 * h))
        h = h * (1 - self.calib['H1'] * h / 524288)

        if h > 100:
            return 100
        if h < 0:
            return 0
        return h

    def get_readings(self):
        data = self.bus.read_i2c_block_data(self.addr, Register.PRESS_MSB, 8)
        temperature = self.compensate_temperature(
            data[3] << 12 | data[4] << 4 | data[5] >> 4) # order matters - first compensate temp
        pressure = self.compensate_pressure(
            data[0] << 12 | data[1] << 4 | data[2] >> 4)
        humidity = self.compensate_humidity(data[6] << 8 | data[7])

        return {"readings": f"temp: {temperature}, pressure: {pressure}, humidity: {humidity}",
                "calibration": self.calib,
                "temp": data[0:3]}



def connect(bus=1, addr=Device.ADDRESS):
    return Bme280(bus, addr)

@click.command()
def main():
    """Simple app to read data from BME280 sensor"""
    dev = connect()
    result = dev.get_readings()
    print(f"Result: {result}")

if __name__ == '__main__':
    main()