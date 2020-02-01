import machine, onewire, ds18x20, time

ds_pin = machine.Pin(13)
ds_sensor = ds18x20.DS18X20(onewire.OneWire(ds_pin))

roms = ds_sensor.scan()

def ds18Temp():
  ds_sensor.convert_temp()
  time.sleep(1)
  temp = 0
  for rom in roms:
    temp = ds_sensor.read_temp(rom)
  if (temp == 85): 
    temp = 0  
  return temp
