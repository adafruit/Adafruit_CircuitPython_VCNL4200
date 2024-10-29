# SPDX-FileCopyrightText: Copyright (c) 2024 Liz Clark, Tim Cocks for Adafruit Industries
#
# SPDX-License-Identifier: MIT

# Written by Liz Clark & Tim Cocks (Adafruit Industries) with OpenAI ChatGPT 4o May 13, 2024 build
# https://help.openai.com/en/articles/6825453-chatgpt-release-notes

# https://chatgpt.com/share/6720fdc1-2bc0-8006-8f7c-f0ff11189ea9
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
    als_low_threshold = UnaryStruct(_ALS_THDL, "<H")
    als_high_threshold = UnaryStruct(_ALS_THDH, "<H")
    prox_hd = RWBit(_PS_CONF12, 11)
    prox_shutdown = RWBit(_PS_CONF12, 0)  # Bit 0: PS_SD (Proximity Sensor Shutdown)
    proximity = ROUnaryStruct(_PS_DATA, "<H")
    lux = ROUnaryStruct(_ALS_DATA, "<H")
    white_light = ROUnaryStruct(_WHITE_DATA, "<H")  # 16-bit register for white light data
    _als_int_en = RWBits(1, _ALS_CONF, 1)  # Bit 1: ALS interrupt enable
    _als_int_switch = RWBits(1, _ALS_CONF, 5)  # Bit 5: ALS interrupt channel selection (white/ALS)
    _proximity_int_en = RWBits(1, _PS_CONF12, 0)
    _prox_trigger = RWBit(_PS_CONF3MS, 2)
    _PROX_INTERRUPT_MASK = 0b00000011
    _PROX_SUN_CANCEL_MASK = 0b00000001  # Bit 0 in PS_CONF3MS register
    _PROX_SUNLIGHT_DOUBLE_IMMUNITY_MASK = 0b00000010
    _PROX_ACTIVE_FORCE_MASK = 0b00001000
    _PROX_SMART_PERSISTENCE_MASK = 0b00010000
    _ALS_PERSISTENCE_MASK = 0b00001100
    _ALS_INTEGRATION_TIME_MASK = 0b11000000
    _PROX_DUTY_MASK = 0b11000000
    _PROX_INTEGRATION_TIME_MASK = 0b00001110
    _PROX_PERSISTENCE_MASK = 0b00110000

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

    # Internal I2C utility functions
    def _read_register(self, register, length=1):
        """Read the specified number of bytes from a register."""
        with self.i2c_device as i2c:
            buffer = bytearray(length)
            i2c.write_then_readinto(bytes([register]), buffer)
            return buffer

    def _write_register(self, register, data):
        """Write bytes to the specified register."""
        with self.i2c_device as i2c:
            i2c.write(bytes([register]) + data)

    @property
    def prox_interrupt(self):
        """Get the current interrupt mode for the proximity sensor (PS) as a mode name."""
        PS_INT_REVERSE = {value: key for key, value in PS_INT.items()}
        buffer = self._read_register(_PS_CONF12, 2)
        mode_value = buffer[1] & self._PROX_INTERRUPT_MASK
        # Return the mode name if available, otherwise return "Unknown"
        return PS_INT_REVERSE.get(mode_value, "Unknown")

    @prox_interrupt.setter
    def prox_interrupt(self, mode):
        """Set the interrupt mode for the proximity sensor (PS) using PS_INT values."""
        if mode not in PS_INT.values():
            raise ValueError("Invalid interrupt mode")
        buffer = self._read_register(_PS_CONF12, 2)
        buffer[1] = (buffer[1] & ~self._PROX_INTERRUPT_MASK) | (mode & self._PROX_INTERRUPT_MASK)
        self._write_register(_PS_CONF12, buffer)

    @property
    def prox_duty(self):
        """Get the current proximity sensor duty cycle setting as a setting name."""
        # Reverse lookup dictionary for PS_DUTY
        PS_DUTY_REVERSE = {value: key for key, value in PS_DUTY.items()}
        # Read PS_CONF12 as a 2-byte register
        buffer = self._read_register(_PS_CONF12, 2)
        # Extract bits 6–7 from the first byte and map to setting name
        duty_value = (buffer[0] & self._PROX_DUTY_MASK) >> 6
        return PS_DUTY_REVERSE.get(duty_value, "Unknown")

    @prox_duty.setter
    def prox_duty(self, setting):
        """Set the proximity sensor duty cycle using a setting name from PS_DUTY."""
        if setting not in PS_DUTY.values():
            raise ValueError(f"Invalid proximity duty cycle setting: {setting}")

        # Read the current 2-byte value of PS_CONF12
        buffer = self._read_register(_PS_CONF12, 2)
        # Clear bits 6–7 in the first byte, then set to the new duty cycle
        buffer[0] = (buffer[0] & ~self._PROX_DUTY_MASK) | (setting << 6)
        # Write the modified 2-byte value back to PS_CONF12
        self._write_register(_PS_CONF12, buffer)

    @property
    def prox_sun_cancellation(self):
        """Sunlight cancellation for the proximity sensor."""
        # Read the PS_CONF3MS register (2 bytes)
        buffer = self._read_register(_PS_CONF3MS, 2)
        # Check if the sunlight cancellation enable bit is set
        return bool(buffer[0] & self._PROX_SUN_CANCEL_MASK)

    @prox_sun_cancellation.setter
    def prox_sun_cancellation(self, enable):
        # Read the current register value
        buffer = self._read_register(_PS_CONF3MS, 2)
        # Update the bit based on the enable parameter
        if enable:
            buffer[0] |= self._PROX_SUN_CANCEL_MASK  # Set the bit
        else:
            buffer[0] &= ~self._PROX_SUN_CANCEL_MASK  # Clear the bit
        # Write the updated value back to the register
        self._write_register(_PS_CONF3MS, buffer)

    @property
    def prox_sunlight_double_immunity(self):
        """Get the current state of double sunlight immunity for the proximity sensor (PS)."""
        # Read the PS_CONF3MS register (2 bytes)
        buffer = self._read_register(_PS_CONF3MS, 2)
        # Check if the double sunlight immunity enable bit is set
        return bool(buffer[0] & self._PROX_SUNLIGHT_DOUBLE_IMMUNITY_MASK)

    @prox_sunlight_double_immunity.setter
    def prox_sunlight_double_immunity(self, enable):
        """Enable or disable double sunlight immunity for the proximity sensor (PS)."""
        # Read the current register value
        buffer = self._read_register(_PS_CONF3MS, 2)
        # Update the bit based on the enable parameter
        if enable:
            buffer[0] |= self._PROX_SUNLIGHT_DOUBLE_IMMUNITY_MASK  # Set the bit
        else:
            buffer[0] &= ~self._PROX_SUNLIGHT_DOUBLE_IMMUNITY_MASK  # Clear the bit
        # Write the updated value back to the register
        self._write_register(_PS_CONF3MS, buffer)

    @property
    def prox_active_force(self):
        """Get the current state of active force mode for the proximity sensor (PS)."""
        # Read the PS_CONF3MS register (2 bytes)
        buffer = self._read_register(_PS_CONF3MS, 2)
        # Check if the active force mode enable bit is set
        return bool(buffer[0] & self._PROX_ACTIVE_FORCE_MASK)

    @prox_active_force.setter
    def prox_active_force(self, enable):
        """Enable or disable active force mode for the proximity sensor (PS)."""
        # Read the current register value
        buffer = self._read_register(_PS_CONF3MS, 2)
        # Update the bit based on the enable parameter
        if enable:
            buffer[0] |= self._PROX_ACTIVE_FORCE_MASK  # Set the bit
        else:
            buffer[0] &= ~self._PROX_ACTIVE_FORCE_MASK  # Clear the bit
        # Write the updated value back to the register
        self._write_register(_PS_CONF3MS, buffer)

    @property
    def prox_smart_persistence(self):
        """Get the current state of smart persistence for the proximity sensor (PS)."""
        # Read the PS_CONF3MS register (2 bytes)
        buffer = self._read_register(_PS_CONF3MS, 2)
        # Check if the smart persistence enable bit is set
        return bool(buffer[0] & self._PROX_SMART_PERSISTENCE_MASK)

    @prox_smart_persistence.setter
    def prox_smart_persistence(self, enable):
        """Enable or disable smart persistence for the proximity sensor (PS)."""
        # Read the current register value
        buffer = self._read_register(_PS_CONF3MS, 2)
        # Update the bit based on the enable parameter
        if enable:
            buffer[0] |= self._PROX_SMART_PERSISTENCE_MASK  # Set the bit
        else:
            buffer[0] &= ~self._PROX_SMART_PERSISTENCE_MASK  # Clear the bit
        # Write the updated value back to the register
        self._write_register(_PS_CONF3MS, buffer)

    @property
    def als_integration_time(self):
        """Get the current ALS integration time setting as an integer value."""
        # Reverse lookup dictionary for ALS_IT
        ALS_IT_REVERSE = {value: key for key, value in ALS_IT.items()}
        # Read ALS_CONF as a 2-byte register
        buffer = self._read_register(_ALS_CONF, 2)
        # Extract bits 6–7 (integration time)
        integration_value = (buffer[0] & self._ALS_INTEGRATION_TIME_MASK) >> 6
        # Map the result to the setting name, defaulting to "Unknown" if unmatched
        return ALS_IT_REVERSE.get(integration_value, "Unknown")

    @als_integration_time.setter
    def als_integration_time(self, it):
        """Set the ALS integration time using a valid integer setting."""
        if it not in ALS_IT.values():
            raise ValueError(f"Invalid ALS integration time setting: {it}")

        # Read current ALS_CONF as a 2-byte register
        buffer = self._read_register(_ALS_CONF, 2)
        # Clear bits 6–7, then set the new value
        buffer[0] = (buffer[0] & ~self._ALS_INTEGRATION_TIME_MASK) | (it << 6)
        # Write back both bytes to ALS_CONF
        self._write_register(_ALS_CONF, buffer)

    @property
    def als_persistence(self):
        """Get the current ALS persistence setting as a human-readable name."""
        # Reverse lookup dictionary for ALS_PERS
        ALS_PERS_REVERSE = {value: key for key, value in ALS_PERS.items()}
        # Read ALS_CONF as a 2-byte register
        buffer = self._read_register(_ALS_CONF, 2)
        # Extract bits 2–3 and map to persistence level
        persistence_value = (buffer[0] & self._ALS_PERSISTENCE_MASK) >> 2
        return ALS_PERS_REVERSE.get(persistence_value, "Unknown")

    @als_persistence.setter
    def als_persistence(self, pers):
        """Set the ALS persistence level using an integer value from ALS_PERS."""
        if pers not in ALS_PERS.values():
            raise ValueError(f"Invalid ALS persistence setting: {pers}")
        # Read the current 2-byte value of ALS_CONF
        buffer = self._read_register(_ALS_CONF, 2)
        # Clear bits 2–3, then set the new persistence value
        buffer[0] = (buffer[0] & ~self._ALS_PERSISTENCE_MASK) | (pers << 2)
        # Write the modified 2-byte value back to ALS_CONF
        self._write_register(_ALS_CONF, buffer)

    @property
    def prox_integration_time(self):
        """Get the current proximity sensor integration time as a setting name."""
        # Reverse lookup dictionary for PS_IT
        PS_IT_REVERSE = {value: key for key, value in PS_IT.items()}
        # Read PS_CONF12 as a 2-byte register
        buffer = self._read_register(_PS_CONF12, 2)
        # Extract bits 1–3 (proximity integration time) and map to setting name
        it_value = (buffer[0] & self._PROX_INTEGRATION_TIME_MASK) >> 1
        return PS_IT_REVERSE.get(it_value, "Unknown")

    @prox_integration_time.setter
    def prox_integration_time(self, setting):
        """Set the proximity sensor integration time using a setting name from PS_IT."""
        if setting not in PS_IT.values():
            raise ValueError(f"Invalid proximity integration time setting: {setting}")

        # Read the current 2-byte value of PS_CONF12
        buffer = self._read_register(_PS_CONF12, 2)
        # Clear bits 1–3 in the first byte, then set to the new integration time
        buffer[0] = (buffer[0] & ~self._PROX_INTEGRATION_TIME_MASK) | (setting << 1)
        # Write the modified 2-byte value back to PS_CONF12
        self._write_register(_PS_CONF12, buffer)

    @property
    def prox_persistence(self):
        """Get the current proximity sensor persistence setting as a setting name."""
        # Reverse lookup dictionary for PS_PERS
        PS_PERS_REVERSE = {value: key for key, value in PS_PERS.items()}
        # Read PS_CONF12 as a 2-byte register
        buffer = self._read_register(_PS_CONF12, 2)
        # Extract bits 4–5 (proximity persistence) and map to setting name
        pers_value = (buffer[0] & self._PROX_PERSISTENCE_MASK) >> 4
        return PS_PERS_REVERSE.get(pers_value, "Unknown")

    @prox_persistence.setter
    def prox_persistence(self, setting):
        """Set the proximity sensor persistence level using a setting name from PS_PERS."""
        if setting not in PS_PERS.values():
            raise ValueError(f"Invalid proximity persistence setting: {setting}")

        # Read the current 2-byte value of PS_CONF12
        buffer = self._read_register(_PS_CONF12, 2)
        # Clear bits 4–5 in the first byte, then set to the new persistence level
        buffer[0] = (buffer[0] & ~self._PROX_PERSISTENCE_MASK) | (setting << 4)
        # Write the modified 2-byte value back to PS_CONF12
        self._write_register(_PS_CONF12, buffer)
