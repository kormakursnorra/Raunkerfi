import time
import board
import adafruit_icm20x

i2c = board.I2C()
# The library defaults to 0x69; pass the address i2cdetect showed you
imu = adafruit_icm20x.ICM20948(i2c, address=0x69)

while True:
    ax, ay, az = imu.acceleration   # m/s^2
    gx, gy, gz = imu.gyro           # rad/s
    mx, my, mz = imu.magnetic       # microtesla
    print(f"accel: {ax:6.2f} {ay:6.2f} {az:6.2f}")
    print(f"gyro:  {gx:6.2f} {gy:6.2f} {gz:6.2f}")
    print(f"mag:   {mx:6.1f} {my:6.1f} {mz:6.1f}")
    time.sleep(0.5)
