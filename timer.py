


from machine import Timer
import dht22
import sendData as db
import machine
import runBot as bot
import ds18
import pressure
import co2

tim = Timer(8)
tim2 = Timer(9)

def sendToDB(t):
    print("send data to db")
    hum  = str(dht22.dhtHum())
    print(hum)
    temp = str(dht22.dhtTemp())
    print(temp)
    ds18T = str(ds18.ds18Temp())
    print(ds18T)
    press = str(pressure.getPresure())
    print(press)
    co = str(co2.getCO2())
    print(co)
    db.send(temp, hum, ds18T, press, co)

def runTelegramBot(t):
    print('run BOT')
    bot.run()

tim.init(period=239999, mode=Timer.PERIODIC, callback=sendToDB)
tim2.init(period=31921, mode=Timer.PERIODIC, callback=runTelegramBot)


