


#   Requires `bmp180.py` library from https://github.com/micropython-IMU/micropython-bmp180
import machine
import time
from bmp180 import BMP180


def getPresure():
  try:
      i2c = machine.I2C(scl=machine.Pin(22), sda=machine.Pin(21))
      bmp180 = BMP180(i2c)
      p = bmp180.pressure*0.00750062
      time.sleep(1)
         
  except Exception as e: 
      time.sleep(1)
      print("Error: ", str(e))
      p = 750
  
  return p


