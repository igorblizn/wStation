
import time
import co2
import dht22
import ds18
import pressure


while True:
  print(pressure.getPresure())

  print(ds18.ds18Temp())
  print(dht22.dhtHum())
  print(dht22.dhtTemp())

  print(co2.getCO2())
  time.sleep(100)

