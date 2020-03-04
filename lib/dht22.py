

import dht
import machine
d = dht.DHT22(machine.Pin(14))
import time



def dhtTemp():
  time.sleep(1)
  d.measure()
  temp = d.temperature()
  #print('Temperature: %3.1f C' %temp)
  return temp


def dhtHum():
  time.sleep(1)
  d.measure()
  hum = d.humidity()
  #print(hum)
  return hum


