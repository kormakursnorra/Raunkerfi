import sys
import time
import board
import digitalio

import adafruit_ssd1306
import adafruit_tcs34725

from adafruit_display_text.bitmap_label import Label
from PIL import Image, ImageDraw, ImageFont

from displayio import Group
from terminalio import FONT

WIDTH = 128
HEIGHT = 64
BORDER = 5

class MainController:

    def __init__( self, oled_reset ):
        self.m_mainGroup = Group()
        self.m_i2c = board.I2C()
        self.m_sensor = adafruit_tcs34725.TCS34725( self.m_i2c )
        self.m_screen = adafruit_ssd1306.SSD1306_I2C(WIDTH, HEIGHT, self.m_i2c, addr=0x3C, reset=oled_reset)

        self.color_output_label = Label(FONT, text="", scale=2)
        self.temperature_output_label = Label(FONT, text="", scale=2)
        self.lux_output_label = Label(FONT, text="", scale=2)


    def run( self ) -> int:
        self.m_screen.fill(0)
        self.m_screen.show()

        self.color_output_label.anchor_point = (0, 0)
        self.color_output_label.anchored_position = (
            4,
            board.DISPLAY.height // 2 - 60,
        )
        self.temperature_output_label.anchor_point = (0, 0)
        self.temperature_output_label.anchored_position = (
            4,
            board.DISPLAY.height // 2 - 0,
        )
        self.lux_output_label.anchor_point = (0, 0)
        self.lux_output_label.anchored_position = (
            4,
            board.DISPLAY.height // 2 + 20,
        )
        
        self.m_mainGroup.append(self.color_output_label)
        self.m_mainGroup.append(self.temperature_output_label)
        self.m_mainGroup.append(self.lux_output_label)

        board.DISPLAY.root_group = self.m_mainGroup

        while self.m_sensor.active:
            if self.processSensorData() < 0:
                return -1
            time.sleep(0.5)

            if self.displayData() < 0:
                return -1

        return 0

    def processSensorData( self ) -> int:
        self.color_output_label.text = f"RGB color 3-tuple:\n{self.m_sensor.color_rgb_bytes}"
        self.temperature_output_label.text = f"Temp: {self.m_sensor.color_temperature}K"
        self.lux_output_label.text = f"Lux: {self.m_sensor.lux}"
        return 0

    def displayData( self ) -> int:
        self.m_screen.show()
        return 0




def main():
    oled_reset = None
    try:
        oled_reset = digitalio.DigitalInOut(board.D4)
    except Exception as e:
        pass

    controller = MainController( oled_reset=oled_reset )
    retVal = controller.run()
    if retVal < 0:
        sys.exit( 1 )
    else:
        sys.exit( 0 )

if __name__ == "__main__":
    main()