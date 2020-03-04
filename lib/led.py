
import machine
import time

LED0 = machine.Pin(4, machine.Pin.OUT)


def blink():
  LED0.on()
  time.sleep_ms(50)
  LED0.off()
  time.sleep_ms(300)
  




