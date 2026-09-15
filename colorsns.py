import board
from time import sleep
import busio
import adafruit_ssd1306
import adafruit_tcs34725
from PIL import Image, ImageDraw, ImageFont

i2c = busio.I2C(board.SCL, board.SDA)
display = adafruit_ssd1306.SSD1306_I2C(128, 64, i2c)
sensor = adafruit_tcs34725.TCS34725(i2c)

display.fill(0)
display.show()

while True:
    image = Image.new("1", (display.width, display.height))
    draw = ImageDraw.Draw(image)
    color = sensor.color_rgb_bytes
    temperature = sensor.color_temperature
    lux = sensor.lux
    draw.text((0,0), f'RGB:{color[0]},{color[1]},{color[2]}', fill=255)
    try:
        draw.text((0,8), f'Temperature: {temperature}', fill=255)
    except:
        draw.text((0,8 ), 'Temperature: error', fill=255)
    draw.text((0,16), f'Lux: {lux}', fill=255)
    display.image(image)
    display.show()
    sleep(1)
