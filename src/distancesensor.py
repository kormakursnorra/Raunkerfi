import busio
import digitalio
import board
import adafruit_mcp3xxx.mcp3008 as MCP
from adafruit_mcp3xxx.analog_in import AnalogIn

# create the spi bus
spi = busio.SPI(clock=board.SCLK, MISO=board.MISO, MOSI=board.MOSI)

# create the cs (chip select)
cs = digitalio.DigitalInOut(board.D8)

# create the mcp object
mcp = MCP.MCP3008(spi, cs)

# create an analog input channel on pin 0
chan = AnalogIn(mcp, MCP.P0)

def voltage_to_cm(voltage):
    if voltage <= 0:
        return None
    return 60.374 * (voltage ** -1.16)

print('Press enter to scan, q to quit')
while (user := input()) != 'q':
    voltage = chan.voltage
    distance_cm = voltage_to_cm(voltage)

    print(f'Raw ADC Value: {chan.value}')
    print(f'ADC Voltage: {voltage:.3f}V')
    print(f'Distance: {distance_cm:.1f} cm')
