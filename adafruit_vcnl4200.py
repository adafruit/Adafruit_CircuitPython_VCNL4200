# SPDX-FileCopyrightText: Copyright (c) 2024 Liz Clark, Tim Cocks for Adafruit Industries
#
# SPDX-License-Identifier: MIT
"""
`adafruit_vcnl4200`
================================================================================

CircuitPython driver for the Adafruit VCNL4200 Long Distance IR Proximity and Light Sensor


* Author(s): Liz Clark, Tim Cocks

Implementation Notes
--------------------

**Hardware:**

* `Adafruit VCNL4200 Long Distance IR Proximity and Light Sensor <https://www.adafruit.com/product/6064>`_

**Software and Dependencies:**

* Adafruit CircuitPython firmware for the supported boards:
  https://circuitpython.org/downloads

* Adafruit's Bus Device library: https://github.com/adafruit/Adafruit_CircuitPython_BusDevice
* Adafruit's Register library: https://github.com/adafruit/Adafruit_CircuitPython_Register
"""

import time

from adafruit_bus_device.i2c_device import I2CDevice
from adafruit_register.i2c_bit import ROBit, RWBit
from adafruit_register.i2c_bits import RWBits
from adafruit_register.i2c_struct import ROUnaryStruct, Struct, UnaryStruct
from micropython import const

try:
    import typing

    from busio import I2C
except ImportError:
    pass

__version__ = "0.0.0+auto.0"
__repo__ = "https://github.com/adafruit/Adafruit_CircuitPython_VCNL4200.git"

_I2C_ADDRESS = const(0x51)  # Default I2C address for VCNL4200

# Register addresses
_ALS_CONF = const(0x00)  # ALS configuration register
_ALS_THDH = const(0x01)  # ALS high threshold register
_ALS_THDL = const(0x02)  # ALS low threshold register
_PS_CONF12 = const(0x03)  # Proximity sensor configuration register 1 & 2
_PS_CONF3MS = const(0x04)  # Proximity sensor configuration register 3 & MS
_PS_CANC_LVL = const(0x05)  # Proximity cancellation level register
_PS_THDL = const(0x06)  # Proximity sensor low threshold register
_PS_THDH = const(0x07)  # Proximity sensor high threshold register
_PS_DATA = const(0x08)  # Proximity sensor data register
_ALS_DATA = const(0x09)  # ALS data register
_WHITE_DATA = const(0x0A)  # White sensor register
_INT_FLAG = const(0x0D)  # Interrupt flag register
_ID = const(0x0E)  # Device ID register

# Interrupt flags
_INTFLAG_PROX_UPFLAG = const(0x80)  # Proximity code saturation flag
_INTFLAG_PROX_SPFLAG = const(0x40)  # Proximity sunlight protection flag
_INTFLAG_ALS_LOW = const(0x20)  # ALS interrupt flag
_INTFLAG_ALS_HIGH = const(0x10)  # Proximity interrupt flag
_INTFLAG_PROX_CLOSE = const(0x02)  # Proximity THDH trigger
_INTFLAG_PROX_AWAY = const(0x01)  # Proximity THDL trigger

# ALS Integration Time settings
ALS_IT = {
    "50MS": const(0x00),  # 50 ms integration time
    "100MS": const(0x01),  # 100 ms integration time
    "200MS": const(0x02),  # 200 ms integration time
    "400MS": const(0x03),  # 400 ms integration time
}

# ALS Persistence settings
ALS_PERS = {
    "1": const(0x00),  # ALS persistence 1 conversion
    "2": const(0x01),  # ALS persistence 2 conversions
    "4": const(0x02),  # ALS persistence 4 conversions
    "8": const(0x03),  # ALS persistence 8 conversions
}

# Proximity Sensor Integration Time settings
PS_IT = {
    "1T": const(0x00),  # Proximity integration time 1T
    "2T": const(0x01),  # Proximity integration time 2T
    "3T": const(0x02),  # Proximity integration time 3T
    "4T": const(0x03),  # Proximity integration time 4T
    "8T": const(0x04),  # Proximity integration time 8T
    "9T": const(0x05),  # Proximity integration time 9T
}

# Proximity Sensor Persistence settings
PS_PERS = {
    "1": const(0x00),  # Proximity persistence 1 conversion
    "2": const(0x01),  # Proximity persistence 2 conversions
    "3": const(0x02),  # Proximity persistence 3 conversions
    "4": const(0x03),  # Proximity persistence 4 conversions
}

# Proximity Sensor Duty settings
PS_DUTY = {
    "1_160": const(0x00),  # Proximity duty cycle 1/160
    "1_320": const(0x01),  # Proximity duty cycle 1/320
    "1_640": const(0x02),  # Proximity duty cycle 1/640
    "1_1280": const(0x03),  # Proximity duty cycle 1/1280
}

# Proximity Sensor Interrupt settings
PS_INT = {
    "DISABLE": const(0x00),  # Proximity interrupt disabled
    "CLOSE": const(0x01),  # Proximity interrupt when an object is close
    "AWAY": const(0x02),  # Proximity interrupt when an object is away
    "BOTH": const(0x03),  # Proximity interrupt for both close and away
}

# LED Current settings
LED_I = {
    "50MA": const(0x00),  # LED current 50mA
    "75MA": const(0x01),  # LED current 75mA
    "100MA": const(0x02),  # LED current 100mA
    "120MA": const(0x03),  # LED current 120mA
    "140MA": const(0x04),  # LED current 140mA
    "160MA": const(0x05),  # LED current 160mA
    "180MA": const(0x06),  # LED current 180mA
    "200MA": const(0x07),  # LED current 200mA
}

# Proximity Sensor Multi Pulse settings
PS_MPS = {
    "1": const(0x00),  # Proximity multi pulse 1
    "2": const(0x01),  # Proximity multi pulse 2
    "4": const(0x02),  # Proximity multi pulse 4
    "8": const(0x03),  # Proximity multi pulse 8
}


class Adafruit_VCNL4200:
    # Device ID expected value for VCNL4200
    _DEVICE_ID = 0x1058

    # Register for ALS configuration
    als_shutdown = RWBits(1, _ALS_CONF, 0)  # ALS shutdown bit
    als_integration_time = RWBits(2, _ALS_CONF, 6)  # ALS integration time bits
    als_persistance = RWBits(2, _ALS_CONF, 2)  # ALS persistence bits
    als_low_threshold = UnaryStruct(_ALS_THDL, "<H")
    als_high_threshold = UnaryStruct(_ALS_THDH, "<H")
    prox_active_force = RWBit(_PS_CONF3MS, 3)
    prox_duty = RWBits(2, _PS_CONF12, 6)
    prox_hd = RWBit(_PS_CONF12, 11)
    prox_integration_time = RWBits(3, _PS_CONF12, 1)
    prox_interrupt = RWBits(2, _PS_CONF12, 8)
    prox_persistence = RWBits(2, _PS_CONF12, 4)
    prox_shutdown = RWBit(_PS_CONF12, 0)  # Bit 0: PS_SD (Proximity Sensor Shutdown)
    proximity = ROUnaryStruct(_PS_DATA, "<H")
    lux = ROUnaryStruct(_ALS_DATA, "<H")
    white_light = ROUnaryStruct(_WHITE_DATA, "<H")  # 16-bit register for white light data
    sunlight_cancellation = RWBit(_PS_CONF3MS, 0)  # Bit 0: PS_CONF3MS sunlight cancellation enable
    _als_int_en = RWBits(1, _ALS_CONF, 1)  # Bit 1: ALS interrupt enable
    _als_int_switch = RWBits(1, _ALS_CONF, 5)  # Bit 5: ALS interrupt channel selection (white/ALS)
    _proximity_int_en = RWBits(1, _PS_CONF12, 0)
    _prox_trigger = RWBit(_PS_CONF3MS, 2)

    def __init__(self, i2c: I2C, addr: int = _I2C_ADDRESS) -> None:
        self.i2c_device = I2CDevice(i2c, addr)
        for _ in range(2):
            with self.i2c_device as i2c:
                # Manually read the device ID register (0x0E, 2 bytes)
                buffer = bytearray(2)
                i2c.write_then_readinto(bytes([_ID]), buffer)
                device_id = int.from_bytes(buffer, "little")
                # Check if it matches expected device ID
                if device_id == self._DEVICE_ID:
                    break
                else:
                    raise RuntimeError("Device ID mismatch.")
        try:
            self.als_shutdown = False
            self.als_integration_time = ALS_IT["50MS"]
            self.als_persistence = ALS_PERS["1"]
            self.als_low_threshold = 0
            self.als_high_threshold = 0xFFFF
            self.set_interrupt(enabled=False, white_channel=False)
            self.prox_duty = PS_DUTY["1_160"]
            self.prox_shutdown = False
            self.prox_integration_time = PS_IT["1T"]
            self.prox_persistence = PS_PERS["1"]
        except Exception as error:
            raise RuntimeError(f"Failed to initialize: {error}") from error

    def set_interrupt(self, enabled, white_channel):
        """Configure ALS interrupt settings, enabling or disabling
        the interrupt and selecting the interrupt channel."""
        try:
            self._als_int_en = enabled
            self._als_int_switch = white_channel
            return True
        except OSError:
            return False

    def trigger_prox(self):
        """Triggers a single proximity measurement manually in active force mode."""
        try:
            self._prox_trigger = True
            return True
        except OSError:
            return False
