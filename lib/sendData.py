
import urequests

def send(dhtT, dhtH, ds18T, pressure, co):
  url = "https://api.thingspeak.com/update?api_key=LOO986MXGL8CV4M1"
  field1 = "&field1=" + dhtT
  field2 = "&field2=" + dhtH
  field3 = "&field3=" + ds18T
  field4 = "&field4=" + pressure
  field5 = "&field5=" + co
  #print(url)
  print(url + field1 + field2 + field3 + field4 + field5)
  response = urequests.get(url + field1 + field2 + field3 + field4 + field5)
  response.close()
  #print('closed in sendData')


