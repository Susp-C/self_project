import os
import sys
import time
import spidev
import logging
import numpy as np

class RaspberryPi:
    def __init__(self, spi=spidev.SpiDev(0,0), spi_freq=40000000, rst=27, dc=25, bl=18, bl_freq=1000, i2c=None, i2c_freq=100000):
        try:
            import gpiod
            self.gpiod = gpiod
        except ImportError:
            logging.error("Failed to import gpiod. Please install it using: apt-get install python3-libgpiod")
            sys.exit(1)

        self.np=np
        self.RST_PIN = rst
        self.DC_PIN = dc
        self.BL_PIN = bl
        self.SPEED = spi_freq
        self.BL_freq = bl_freq
        
        # Connect to RP1 GPIO chip on Pi 5 (usually gpiochip4)
        try:
            self.chip = self.gpiod.Chip("gpiochip4")
        except Exception as e:
            logging.error(f"Cannot open gpiochip4: {e}")
            sys.exit(1)

        # Get lines for LCD
        self.rst_line = self.chip.get_line(self.RST_PIN)
        self.dc_line = self.chip.get_line(self.DC_PIN)
        self.bl_line = self.chip.get_line(self.BL_PIN)

        # Request lines as output
        self.rst_line.request(consumer="lcd_rst", type=self.gpiod.LINE_REQ_DIR_OUT)
        self.dc_line.request(consumer="lcd_dc", type=self.gpiod.LINE_REQ_DIR_OUT)
        self.bl_line.request(consumer="lcd_bl", type=self.gpiod.LINE_REQ_DIR_OUT)
        
        # Default backlight to HIGH
        self.bl_line.set_value(1)
        
        # Initialize SPI
        self.SPI = spi
        if self.SPI != None:
            self.SPI.max_speed_hz = spi_freq
            self.SPI.mode = 0b00

    def digital_write(self, pin, value):
        if pin == self.DC_PIN:
            self.dc_line.set_value(value)
        elif pin == self.RST_PIN:
            self.rst_line.set_value(value)
        elif pin == self.BL_PIN:
            self.bl_line.set_value(value)

    def digital_read(self, pin):
        if pin == self.DC_PIN:
            return self.dc_line.get_value()
        elif pin == self.RST_PIN:
            return self.rst_line.get_value()
        elif pin == self.BL_PIN:
            return self.bl_line.get_value()
        return 0

    def delay_ms(self, delaytime):
        time.sleep(delaytime / 1000.0)

    def spi_writebyte(self, data):
        if self.SPI != None:
            self.SPI.writebytes(data)

    def bl_DutyCycle(self, duty):
        # gpiod does not natively support PWM 
        # Leave empty to avoid crashing when parent functions call it
        pass
        
    def bl_Frequency(self, freq):
        pass
           
    def module_init(self):
        # Variables requested during init were handled in __init__ for gpiod
        if self.SPI != None:
            self.SPI.max_speed_hz = self.SPEED
            self.SPI.mode = 0b00     
        return 0

    def module_exit(self):
        logging.debug("spi end")
        if self.SPI != None:
            self.SPI.close()
        
        logging.debug("gpio cleanup...")
        self.rst_line.set_value(1)
        self.dc_line.set_value(0)
        time.sleep(0.001)
        self.bl_line.set_value(1)
        
        self.rst_line.release()
        self.dc_line.release()
        self.bl_line.release()

### END OF FILE ###
