
from machine import UART

    
def getCO2():    
    uart=UART(1, 9600, rx=15, tx=12)
    
    uart.init(9600, bits=8, parity=None, stop=1,timeout=200)
    
    uart.write(b'\xff\x01\x86\x00\x00\x00\x00\x00y')
    s = uart.readline(9)

    return ((s[2]*256)+s[3])
    

