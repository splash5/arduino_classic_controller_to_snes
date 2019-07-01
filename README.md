# arduino_classic_controller_to_snes
Turn a Classic Controller into an SNES controller with Arduino Nano

All the classic controller I2C timing is measured with SNES Classic Mini. (I2C speed is 200KHz)
Some non-official classic controller (like PS2 to classic controller) need 200KHz speed, official one can run with 300KHz.
But both can not work properly with 400KHz speed.

Schematic
![image](https://github.com/splash5/arduino_classic_controller_to_snes/raw/master/Schematic.jpg)

IMPORTANT:
Remember to disconnect 5V from SNES when you plug Arduino Nano into a USB port for safety.
