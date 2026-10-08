import sys
import time
import board
import digitalio

import adafruit_ssd1306
import adafruit_tcs34725

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
        self.m_image = Image.new("1", (self.m_screen.width, self.m_screen.height))
        # Get drawing object to draw on image.
        self.m_draw = ImageDraw.Draw(self.m_image)
        self.m_output = ""

    def run( self ) -> int:

        self.m_sensor.integration_time = 150
        self.m_sensor.gain = 4


        while True:
            self.m_screen.fill(0)
            self.m_screen.show()
            if self.processSensorData() < 0:
                return -1
            if self.displayData() < 0:
                return -1
            time.sleep(0.5)



    def processSensorData( self ) -> int:
        color = self.m_sensor.color
        color_rgb = self.m_sensor.color_rgb_bytes
        print(f"RGB color as 8 bits per channel int: #{color:02X} or as 3-tuple: {color_rgb}")
        temp = self.m_sensor.color_temperature
        lux = self.m_sensor.lux
        print(f"Temperature: {temp}K Lux: {lux}\n")

        self.m_output = f"RGB:{color_rgb[0]},{color_rgb[1]},{color_rgb[2]}"
        return 0


    def displayData( self ) -> int:
        print("In displayData")
        print(self.m_output)
        self.m_draw.text((0,0), self.m_output, fill=255)
        self.m_screen.image(self.m_image)
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
