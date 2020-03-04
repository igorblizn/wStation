

def if_digit(x):
  try:
    float(x)
    #print("true")
    return True
  except ValueError:
    #print("false")
    return False
