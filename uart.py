from machine import UART
import time
i=0
while i<10:
  i=i+1
  try:
    uart=UART(1, 9600, rx=15, tx=13)
    
    uart.init(9600, bits=8, parity=None, stop=1,timeout=1)
    

    result=uart.write("\xff\x01\x86\x00\x00\x00\x00\x00\x79")
    time.sleep_ms(100)
    print(uart.read(9))

    #uart.deinit()
     

  except Exception:
    print('Exception')
