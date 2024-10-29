# SPDX-FileCopyrightText: 2024 Liz Clark for Adafruit Industries
#
# SPDX-License-Identifier: MIT
"""VCNL4200 Full Test"""

import time

import board
import busio

from adafruit_vcnl4200 import PS_INT, Adafruit_VCNL4200, LED_I

# Initialize I2C and the VCNL4200 sensor
i2c = board.I2C()
sensor = Adafruit_VCNL4200(i2c)

print(f"Proximity Integration Mode: {sensor.prox_integration_time}")
print(f"Proximity Persistence Mode: {sensor.prox_persistence}")
print(f"Proximity Duty Cycle: {sensor.prox_duty}")
print(f"Lux Persistence Mode: {sensor.als_persistence}")
print(f"Lux Integration Mode: {sensor.als_integration_time}")
sensor.prox_interrupt = PS_INT["DISABLE"]
print(f"Proximity Interrupt Mode: {sensor.prox_interrupt}")
sensor.prox_sun_cancellation = False
print(f"Proximity Sun Cancellation: {sensor.prox_sun_cancellation}")
sensor.prox_sunlight_double_immunity = False
print(f"Proximity Sunlight Double Immunity: {sensor.prox_sunlight_double_immunity}")
sensor.prox_active_force = False
print(f"Proximity Active Force: {sensor.prox_active_force}")
sensor.prox_smart_persistence = False
print(f"Proximity Smart Persistence: {sensor.prox_smart_persistence}")
sensor.prox_led_current = LED_I['75MA']
print(f"Proximity IR LED Current: {sensor.prox_led_current}")
print(f"Proximity is: {sensor.proximity}")
print(f"Ambient is: {sensor.lux}")

while True:
    pass
