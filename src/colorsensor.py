import sys
import time
import board
import digitalio

import adafruit_ssd1306
import adafruit_tcs34725

from adafruit_display_text.bitmap_label import Label
from PIL import Image, ImageDraw, ImageFont

class MainController:
    WIDTH = 128
    HEIGHT = 64
    BORDER = 5
    RESET_PIN = board.D4
    FONT = ImageFont.load_default()

    def __init__( self):

        oled_reset = None
        try:
            oled_reset = digitalio.DigitalInOut(self.RESET_PIN)
        except Exception as e:
            pass

        self.m_i2c = board.I2C()
        self.m_sensor = adafruit_tcs34725.TCS34725( self.m_i2c )
        self.m_screen = adafruit_ssd1306.SSD1306_I2C(
            MainController.WIDTH, MainController.HEIGHT, self.m_i2c, addr=0x3C, reset=oled_reset
        )   

        self.color_output_label = Label(MainController.FONT, text="", scale=2)
        self.temperature_output_label = Label(MainController.FONT, text="", scale=2)
        self.lux_output_label = Label(MainController.FONT, text="", scale=2)

        self.m_img = Image.new("RGB", (128, 64), color="black")
        self.m_draw = ImageDraw.Draw(self.m_img)

    def run( self ) -> int:

        self.m_sensor.integration_time = 150
        self.m_sensor.gain = 4

        self.m_screen.fill(0)
        self.m_screen.show()

        self.color_output_label.anchor_point = (0, 0)
        self.color_output_label.anchored_position = (
            4,
            MainController.HEIGHT // 2 - 60,
        )
        self.temperature_output_label.anchor_point = (0, 0)
        self.temperature_output_label.anchored_position = (
            4,
            MainController.HEIGHT // 2 - 0,
        )
        self.lux_output_label.anchor_point = (0, 0)
        self.lux_output_label.anchored_position = (
            4,
            MainController.HEIGHT // 2 + 20,
        )
        
        self.color_output_label
        self.temperature_output_label
        self.lux_output_label


        while self.m_sensor.active:
            if self.processSensorData() < 0:
                return -1


            if self.displayData() < 0:
                return -1

            time.sleep(0.5)
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
    controller = MainController()
    retCode = controller.run()
    if retCode < 0:
        sys.exit( 1 )
    else:
        sys.exit( 0 )

if __name__ == "__main__":
    main()