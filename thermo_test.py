# SPDX-FileCopyrightText: 2021 ladyada for Adafruit Industries
# SPDX-License-Identifier: MIT

import board
import digitalio
import time

import adafruit_max31856

# Create sensor object, communicating over the board's default SPI bus
spi = board.SPI()
print("initialized SPI")
# allocate a CS pin and set the direction
cs = digitalio.DigitalInOut(board.D5)
# cs.direction = digitalio.Direction.OUTPUT
print("Setup chip select")
# create a thermocouple object with the above
thermocouple = adafruit_max31856.MAX31856(spi, cs)
print("Created object")
# print the temperature!
while True:
  print(thermocouple.temperature)
  time.sleep(0.5)
