#!/usr/bin/env python
from enum import IntEnum
import math
from datetime import datetime
from time import sleep
import struct
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
    CALIB_26 = 0xe1
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

class NotSupportedDevice(Exception):
    pass

class AttributeDict(dict):
    __getattr__ = dict.__getitem__
    __setattr__ = dict.__setitem__

class Bme280(object):

    MAX_HUMIDITY = 100
    MIN_HUMIDITY = 0

    PROFILE = {
        'weather': {
            Register.CONFIG: Config.FILTER_OFF,
            Register.CTRL_MEAS: Ctrl_meas.FORCED | \
                                Ctrl_meas.PRESSURE_OVERSAMPLING_16 | \
                                Ctrl_meas.TEMPERATURE_OVERSAMPLING_1,
            Register.CTRL_HUM: Ctrl_hum.OVERSAMPLING_1
        }
    }

    def __init__(self, bus_addr, addr, altitude = 217, profile='weather'):
        self.bus = smbus.SMBus(bus_addr)
        self.addr = addr
        self.profile = profile
        self._check_device_id()
        self._last_read = datetime.now()
        self.update_period = 1
        self._setup_readings()
        self._get_calibration_data()
        self._temperature = None
        self._pressure = None
        self._humidity = None
        self.altitude = altitude

    @property
    def temperature(self):
        """The compensated temperature in degrees celsius."""
        self._get_readings()
        return self._temperature

    @property
    def pressure(self):
        """The compensated pressure in hPa."""
        self._get_readings()
        return self._pressure

    @property
    def humidity(self):
        """The compensated humidity in %."""
        self._get_readings()
        return self._humidity

    @property
    def sea_level_pressure(self):
        """The Sea Level calculates pressure."""
        self._get_readings()
        sea_level = self._pressure * \
            math.pow(
                (1 - (0.0065 * self.altitude) / (self._temperature + 0.0065 * self.altitude + 273.15)),
                -5.257)
        return sea_level

    def _setup_readings(self):
        for addr, value in self.PROFILE[self.profile].items():
            self.bus.write_byte_data(self.addr, addr, value)
        sleep(0.002)

    def _check_device_id(self):
        device_id = self.bus.read_byte_data(self.addr, Register.ID)
        if device_id != Device.ID:
            raise NotSupportedDevice(f"Device ID mismatch, found: {device_id}")

    def _get_calibration_data(self):
        # Read 25 bytes and 7 bytes with calibration coefficients
        # 25th byte is not used
        data = self.bus.read_i2c_block_data(self.addr, Register.CALIB_00, 26) + \
            self.bus.read_i2c_block_data(self.addr, Register.CALIB_26, 7)

        coeff_map = {
            'temperature': {
                'prefix': 'T',
                'data': data[0:6],
                'structure': '<Hhh'   # see struct.unpack() format string
            },
            'pressure': {
                'prefix': 'P',
                'data': data[6:24],
                'structure': '<Hhhhhhhhh'
            },
            'humidity': {
                'prefix': 'H',
                'data': data[25:],
                'structure': '<BhBBhb'   # need some tricks here later on
            }
        }
        coeffs = AttributeDict()
        for key, conf in coeff_map.items():
            registers = list(struct.unpack(conf['structure'], bytes(conf['data'])))
            for idx, value in enumerate(registers, start=1):
                coeffs[f"{conf['prefix']}{idx}"] = value

        # Some weird number notation from BME280 docs to solve:
        #
        # 0xE4 / 0xE5[3:0] | dig_H4 [11:4] / [3:0] | signed short
        # 0xE5[7:4] / 0xE6 | dig_H5 [3:0] / [11:4] | signed short
        coeffs.H4 = (coeffs.H4 << 4 | coeffs.H5 & 0xF)  # 4 LSBs from next byte
        coeffs.H5 = coeffs.H5 >> 4

        self.coeffs = coeffs
        return coeffs

    def compensate_temperature(self, value):
        var1 = (value / 16384 - self.coeffs.T1 / 1024) * self.coeffs.T2
        var2 = (value / 131072 - self.coeffs.T1 / 8192) * \
             (value / 131072 - self.coeffs.T1 / 8192) * self.coeffs.T3
        self._t_fine = int(var1 + var2)
        return self._t_fine / 5120

    def compensate_pressure(self, value):
        var1 = self._t_fine / 2 - 64000
        var2 = var1 * var1 * self.coeffs.P6 / 32768
        var2 = var2 + var1 * self.coeffs.P5 * 2
        var2 = var2 / 4 + self.coeffs.P4 * 65536
        var3 = self.coeffs.P3 * var1 * var1 / 524288
        var1 = (var3 + self.coeffs['P2'] * var1) / 524288
        var1 = (1 + var1 / 32768) * self.coeffs.P1
        if not var1:
            return 0

        pressure = 1048576 - value
        pressure = (pressure - var2 / 4096) * 6250 / var1
        var1 = self.coeffs.P9 * pressure * pressure / 2147483648
        var2 = pressure * self.coeffs.P8 / 32768
        pressure = pressure + (var1 + var2 + self.coeffs.P7) / 16
        return pressure / 100

    def compensate_humidity(self, value):
        h = self._t_fine - 76800
        h = (value - (self.coeffs.H4 * 64 + self.coeffs.H5 / 16384 * h)) * \
            (self.coeffs.H2 / 65536) * (1 + self.coeffs.H6 / 67108864 * h * \
                (1 + self.coeffs.H3 / 67108864 * h))
        h = h * (1 - self.coeffs.H1 * h / 524288)

        if h > Bme280.MAX_HUMIDITY:
            return Bme280.MAX_HUMIDITY
        if h < Bme280.MIN_HUMIDITY:
            return Bme280.MIN_HUMIDITY

        return h

    def _get_readings(self):
        now = datetime.now()
        if self._temperature and ((now - self._last_read).total_seconds() < self.update_period):
            return

        data = self.bus.read_i2c_block_data(self.addr, Register.PRESS_MSB, 8)
        self._temperature = self.compensate_temperature(
            data[3] << 12 | data[4] << 4 | data[5] >> 4) # order matters - first compensate temp
        self._pressure = self.compensate_pressure(
            data[0] << 12 | data[1] << 4 | data[2] >> 4)
        self._humidity = self.compensate_humidity(data[6] << 8 | data[7])

        self._last_read = now


def connect(bus=1, addr=Device.ADDRESS):
    return Bme280(bus, addr)

@click.command()
def main():
    """Simple app to read data from BME280 sensor"""
    dev = connect()
    print(f"Temperature: {dev.temperature:.1f} C")
    print(f"Pressure: {dev.pressure:.1f} [absolute] / {dev.sea_level_pressure:.1f} [sea level] hPa")
    print(f"Humidity: {dev.humidity:3.1f} %")

if __name__ == '__main__':
    main()
