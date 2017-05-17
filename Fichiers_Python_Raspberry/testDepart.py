import time
import RPi.GPIO as GP
GP.setmode(GP.BOARD)
GP.setup(11,GP.OUT)

while True :
	GP.output(11,True)
	time.sleep(2)
	GP.output(11,False)
	time.sleep(2)


