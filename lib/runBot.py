

import gc
import led
import sys
import api
import machine
import co2
from machine import Timer

#import time
import dht22
import ds18
import pressure

telegram = api.TelegramBot('964820596:AAHFec_OfTebXv4TuboSRFQ-Bl9rGiWY5Nk')

def message_handler(message):
  
    #if chk.if_digit(message[2]):
  #print('start message handler')
  if message[2] == '/temp':
      led.blink()
      tem = str(dht22.dhtTemp())
      telegram.send(message[0], 'dht22 temp =>' + tem)
      print(message[2])
  else: 
    if message[2] == '/hum':
      led.blink()
      tem = str(dht22.dhtHum())
      telegram.send(message[0], 'dht22 hum =>' + tem)
      print(message[2])
    else:
      if message[2] == '/out':
        led.blink()
        tem = str(ds18.ds18Temp())
        telegram.send(message[0], 'ds18 temp =>' + tem)
        print(message[2])
      else:
        if message[2] == '/press':
          led.blink()
          tem = str(pressure.getPresure())
          telegram.send(message[0], 'pressure =>' + tem)
          print(message[2])
        else:
          if message[2] == '/co2':
            led.blink()
            tem = str(co2.getCO2())
            telegram.send(message[0], 'co2 =>' + tem)
            print(message[2])
          else:
            led.blink()
            telegram.send(message[0], 'received  text => ' + message[2] + ' /temp /hum /out /press /co2')
            print(message[2])
        

  gc.collect()


def run():
  print('!start!')
  telegram.listen(message_handler)
  print('!stop!')



