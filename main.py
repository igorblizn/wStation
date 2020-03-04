
import machine

print("try to run bot")
button = machine.Pin(2, machine.Pin.IN, machine.Pin.PULL_UP)
print(button.value())
if button.value() == 0:
    print('MAIN:run BOT')
    #import runBot
    import timer

