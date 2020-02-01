import dht
import machine
d = dht.DHT22(machine.Pin(14))




def dhtTemp():
  d.measure()
  temp = d.temperature()
  #print('Temperature: %3.1f C' %temp)
  return temp


def dhtHum():
  d.measure()
  hum = d.humidity()
  #print('Humidity: %3.1f %%' %hum)
  return hum

