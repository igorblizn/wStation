


import socket 
import machine
import led

#HTML to send to browsers
html = """<!DOCTYPE html>
<html>
<head> <title>ESP8266 LED ON/OFF</title> </head>

<form>
LED0: 
<button name="LED" value="ON0" type="submit">LED ON</button>
<button name="LED" value="OFF0" type="submit">LED OFF</button><br><br>

</form>
</html>
"""
led.blink()
#Setup PINS
LED0 = machine.Pin(4, machine.Pin.OUT)


#Setup Socket WebServer
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.bind(('', 80))
s.listen(5)
while True:
    conn, addr = s.accept()
    print("Got a connection from %s" % str(addr))
    request = conn.recv(1024)
    print("Content = %s" % str(request))
    request = str(request)
    LEDON0 = request.find('/?LED=ON0')
    LEDOFF0 = request.find('/?LED=OFF0')
    print("Data: " + str(LEDON0))
    #print("Data2: " + str(LEDOFF0))
    
    if LEDON0 == 6:
        print('TURN LED0 ON')
        LED0.on()
    if LEDOFF0 == 6:
        print('TURN LED0 OFF')
        LED0.off()

    response = html
    conn.send(response)
    conn.close()



