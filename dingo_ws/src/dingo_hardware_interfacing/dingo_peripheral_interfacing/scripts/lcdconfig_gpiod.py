import os
import sys
import time
import spidev
import logging
import numpy as np


class RaspberryPi:
    def __init__(self, spi=spidev.SpiDev(0, 0), spi_freq=40000000,
                 rst=27, dc=25, bl=18, bl_freq=1000,
                 i2c=None, i2c_freq=100000):
        try:
            import gpiod
            self.gpiod = gpiod
        except ImportError:
            print("[lcdconfig_gpiod] ERROR: failed to import gpiod", flush=True)
            print("[lcdconfig_gpiod] Try: apt-get install python3-libgpiod gpiod", flush=True)
            sys.exit(1)

        self.np = np
        self.RST_PIN = rst
        self.DC_PIN = dc
        self.BL_PIN = bl
        self.SPEED = spi_freq
        self.BL_freq = bl_freq

        self.chip = None
        last_error = None

        # In this container gpiochip0 works, while gpiochip4 gives ioctl error.
        # Try gpiochip0 first.
        for chip_name in ["gpiochip0", "gpiochip4", "gpiochip10", "gpiochip11", "gpiochip12", "gpiochip13"]:
            try:
                chip = self.gpiod.Chip(chip_name)
                # Test whether the needed BCM lines exist on this chip.
                chip.get_line(self.RST_PIN)
                chip.get_line(self.DC_PIN)
                chip.get_line(self.BL_PIN)
                self.chip = chip
                print("[lcdconfig_gpiod] using " + chip_name, flush=True)
                break
            except Exception as e:
                last_error = e
                try:
                    chip.close()
                except Exception:
                    pass

        if self.chip is None:
            print("[lcdconfig_gpiod] ERROR: cannot open a usable gpiochip", flush=True)
            print("[lcdconfig_gpiod] Last error: " + str(last_error), flush=True)
            sys.exit(1)

        try:
            self.rst_line = self.chip.get_line(self.RST_PIN)
            self.dc_line = self.chip.get_line(self.DC_PIN)
            self.bl_line = self.chip.get_line(self.BL_PIN)
            print("[lcdconfig_gpiod] got GPIO lines RST=" + str(self.RST_PIN) +
                  " DC=" + str(self.DC_PIN) +
                  " BL=" + str(self.BL_PIN), flush=True)
        except Exception as e:
            print("[lcdconfig_gpiod] ERROR getting GPIO lines: " + str(e), flush=True)
            sys.exit(1)

        try:
            self.rst_line.request(consumer="lcd_rst", type=self.gpiod.LINE_REQ_DIR_OUT)
            self.dc_line.request(consumer="lcd_dc", type=self.gpiod.LINE_REQ_DIR_OUT)
            self.bl_line.request(consumer="lcd_bl", type=self.gpiod.LINE_REQ_DIR_OUT)
            print("[lcdconfig_gpiod] requested GPIO lines as output", flush=True)
        except Exception as e:
            print("[lcdconfig_gpiod] ERROR requesting GPIO lines: " + str(e), flush=True)
            print("[lcdconfig_gpiod] Maybe another LCD process is still holding the GPIO lines.", flush=True)
            sys.exit(1)

        try:
            self.bl_line.set_value(1)
            print("[lcdconfig_gpiod] backlight set HIGH", flush=True)
        except Exception as e:
            print("[lcdconfig_gpiod] ERROR setting backlight: " + str(e), flush=True)
            sys.exit(1)

        self.SPI = spi
        if self.SPI is not None:
            self.SPI.max_speed_hz = spi_freq
            self.SPI.mode = 0b00
            print("[lcdconfig_gpiod] SPI initialized", flush=True)

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
        if self.SPI is not None:
            self.SPI.writebytes(data)

    def bl_DutyCycle(self, duty):
        # gpiod v1 does not provide PWM directly.
        # Keep backlight on.
        try:
            self.bl_line.set_value(1 if duty > 0 else 0)
        except Exception:
            pass

    def bl_Frequency(self, freq):
        pass

    def module_init(self):
        if self.SPI is not None:
            self.SPI.max_speed_hz = self.SPEED
            self.SPI.mode = 0b00
        return 0

    def module_exit(self):
        logging.debug("spi end")
        if self.SPI is not None:
            self.SPI.close()

        logging.debug("gpio cleanup")
        try:
            self.rst_line.set_value(1)
            self.dc_line.set_value(0)
            self.bl_line.set_value(1)
            time.sleep(0.001)
            self.rst_line.release()
            self.dc_line.release()
            self.bl_line.release()
            self.chip.close()
        except Exception:
            pass


# END OF FILE
