
# coding: utf8
import fonctions
import serial
import threading,time
from math import atan,pi
import RPi.GPIO as GP

GP.setmode(GP.BOARD)
GP.setup(11,GP.OUT)
GP.setup(13,GP.IN)

move = serial.Serial("/dev/moteur",9600,timeout = 1)
capteur = serial.Serial("/dev/capteur",115200,timeout = 1)
actionneur = serial.Serial("/dev/actionneur",9600,timeout = 1)

input_value = False
GP.output(11,True)
while(not input_value) :
	input_value = GP.input(13)
GP.output(11,False)


l = [("aller",(150,0)),("noStop",0),("aller",(0,-90)),("aller",(130,0)),("attraperBas",(150,0)),("endNoStop",0),("aller",(450,0)),("aller",(0,-90)),("aller",(670,0)),("noStop",0),("aller",(0,90)),("endNoStop",0),("aller",(100,0)),("deposerBas",(670,0))]
#l = [("aller",(700,0)),("attraperBas",1),("aller",(0,90)),("aller",(700,0)),("aller",(90,0)),("deposerBas",1)]*4
#test aller et retour

#l = [("aller",(200,0)), ("attraperBas",1), ("aller",(200,0)), ("deposerBas",1), ("aller",(-500,0)), ("attraperBas",1), ("aller",(600,0)), ("deposerBas",1)]
finished = 1
position = (0,0,0)
arret = False
arretable = True

class execution(threading.Thread) :
	def __init__(self):
		threading.Thread.__init__(self)
		self.running = True
	
	def run(self) :
		global arretable
		global arret
		global finished #il faut préciser qu'on se sert de la varaible globale
		while self.running :
			time.sleep(0.5)
			print(arret)
			if l and arret == False :
				finished=0
                		command=l.pop(0)
				print(command)
                		if command[0]=="aller" : #à terme il faudra créer un tableau du type t= ["avancer","tourner"] et regarder t[command]
					aller(command[1])
					time.sleep(5)
                		elif command[0]=="attraperBas":
					time.sleep(1)
					print("OK")
                			pince(8)
                    			time.sleep(3)
					finished = 1
				elif command[0] == "deposerBas":
					time.sleep(1)
					print("OK")
					pince(7)
					time.sleep(2)
					finished = 1
				elif command[0] == "noStop":
					arretable = False
				elif command[0] == "endNoStop":
					arretable = True
					arret = False
	def top(self) : 
		self.running = False


class serialRead(threading.Thread):
	def __init__(self):
		threading.Thread.__init__(self)
	        self.running = True
	        
	def run(self):

	        replyTest = 0
        	global finished
         	global position
         	while self.running :
			time.sleep(1)
	             	replyTest = move.readline()
        	     	replyTest = replyTest[:-2]

             		if(len(replyTest) > 0 and replyTest == "debut"):
                 		replyCommand = move.readline()
      		           	replyCommand = replyCommand[:-2]
                	 	replyArgCount = move.readline()

                		if(replyArgCount[0] == 'd'):
                     			break
	               		replyArgCount = int(replyArgCount[:-2])# une commande peut éventuellement renvoyer plusieurs valeurs
        	        	Targ = []
                		for i in range(replyArgCount):
                     			Targ.append(move.readline()[:-2])
               			if replyCommand == "5":
                     			finished = 1
                		elif replyCommand == "6":
    					print("Valeur :"+Targ[0]+"\n")
                		elif replyCommand == 140:
                     			x = 256*ord(Targ[3])+ord(Targ[2])
                     			y = 256*ord(Targ[5])+ord(Targ[4])
                     			angle = 256*ord(Targ[1])+ord(Targ[0])
                     			position = (x,y,angle)
   	def top(self):
		self.running = False

distance_tab=[-1,-1,-1]
class capteurDist(threading.Thread):
    def run(self):
    	global arretable
    	global arret
        while True:
            replyCommand = capteur.readline()
            if replyCommand != '':
                Targ = replyCommand.decode(encoding='UTF-8').split(',')
                update_capt(Targ)
                print(distance_tab[1])
		if(0 < distance_tab[1]  < 200):
	    		if(arret == False and arretable == True) :
	    			arret = True
	    			print(arret)
	    			aller((0,0))
	    	elif(arret == True):
	    		arret = False
	    		print(arret) 
def pince(a):
    actionneur.write(chr(a)+chr(0))

def cmd(f,args):
	t=(f,args)
	l.append(t)
	print(l)

def attraper():
    pince.write(chr(0)+chr(0))

def reposer():
    pince.write(chr(1)+chr(0))

def deplacer(t):
    x_mm = t[0]
    y_mm = t[1]
    x_clicks = x_mm * cpmm
    y_clicks = y_mm * cpmm
    deplacer_aux((x_clicks, y_clicks, angle))

def deplacer_aux(t): ##x,y en clicks, angle en millième de radians. Déplacement rectiligne entre deux points
    x,y,angle = t
    readPos()

    ##calcul de l'angle à donner en consigne
    pos_x, pos_y, pos_angle = position
    delta_x = x - pos_x
    delta_y = y - pos_y
    norme = pow((delta_x**2+delta_y**2),1/2)
    if delta_y > 0 :
        cos_cons_angle = delta_x / norme
        cons_angle = arccos(cos_cons_angle)
    else :
        cos_cons_angle = delta_x / norme
        cons_angle = -arccos(cos_cons_angle)
    delta_angle = cons_angle - pos_angle
    ##fin du calcul

    cmd(3,delta_angle) ## le robot tourne de delta_angle
    cmd(1,(norme,4000)) ## le robot avance de norme à la vitesse 4000
    cmd(3,angle-cons_angle) ## le robot se met dans l'angle donné en consigne

def aller(t): ##distance en mm, angle en °, case6.
	distance,angle = t
	distance += 32768
	angle += 32768
	distance=min(distance,256**2-1)
	distance=max(0,distance)
	angle = min(angle,256**2-1)
	angle = max(0,angle)
	Arg1 = distance/256
	Arg0 = distance%256
	Arg3 = angle/256
	Arg2 = angle%256

	move.write(chr(6)+chr(4) +chr(Arg0) +chr(Arg1) +chr(Arg2) +chr(Arg3))

def stop(): ##arrêt en cas d'obstacle, case 10
	move.write(chr(10)+chr(0))
	
def reprise(): ##reprise du mouvement une fois l'obstacle parti, case 11
	move.write(chr(11)+chr(0))
	
def avancer(t): ##case 1
	distance,speed = t
	distance += 32768
	speed += 32768
	distance=min(distance,256**2-1)
	distance=max(0,distance)
	speed=min(speed,256**2-1)
	speed=max(0,speed)
	Arg1 = distance/256
	#Arg0 = distance - 256*Arg1
	Arg0=distance%256
	Arg3 = speed/256
	#Arg2 = speed - 256*Arg3
	Arg2 = speed %256
	move.write(chr(1)+chr(4)+chr(Arg0)+chr(Arg1)+chr(Arg2)+chr(Arg3))

def r(): #case2
	move.write(chr(2)+chr(0))

def re(): #case999
	move.write(chr(9)+chr(0))

def tourner(angle): #case3
	angle += 32768
	if angle > 256**2 - 1:
		angle = 256**2 - 1
	if angle < 0 :
		angle = 0
	Arg1 = angle/256
	Arg0 = angle - 256*Arg1
	move.write(chr(3)+chr(2)+chr(Arg0)+chr(Arg1))

def setNewTarget(t): #case4, x et y en clicks
	x,y=t
	x += 32768
	if x > 256**2 - 1:
		x = 256**2 - 1
	if x < 0 :
		x = 0
	Arg1 = x/256
	Arg0 = x - 256*Arg1
	y += 32768
	if y > 256**2 - 1:
		y = 256**2 - 1
	if y < 0 :
		y = 0
	Arg3 = y/256
	Arg2 = y - 256*Arg3
	move.write(chr(4)+chr(4)+chr(Arg0)+chr(Arg1)+chr(Arg2)+chr(Arg3))

def hasArrived(): #case5
	move.write(chr(5)+chr(0))

def readPos(): #case140
	move.write(chr(140)+chr(0))

def test():
	fonctions.cmd(1,(300,400))
	fonctions.cmd(3,3140)
	fonctions.cmd(1,(300,400))
	fonctions.cmd(3,-3140)
	fonctions.cmd(6,(150,45))

def update_capt(t):
    if(len(t)< 4) :
        return
    global distance_tab
    captIndex= int(t[0])
    timeStamp = int(t[1])
    rangeStatus = int(t[2])
    dist = int(t[3])
    if (rangeStatus==0 and captIndex < 2):
        distance_tab[captIndex]=dist
        #print(distance_tab)

    elif captIndex < 2:
        distance_tab[captIndex]=-500



def recherche_tube():
    global distance_tab

    dPince = 100
    dCapteurs = 35
    dCentre = 210

    gauche = distance_tab[2]
    centre=distance_tab[1]
    droite = distance_tab[0]

    if min(distance_tab)>300:
        print("pas d'objet en face")
        return -1
    if centre>gauche+40:

        angle = pi/2-atan((gauche+dCentre)/dCapteurs)
        print("objet à gauche :" )
        aller((0,int(180*angle/pi)))
        return 0
    if centre>droite+40:

        angle = -(pi/2-atan((droite+dCentre)/dCapteurs))
        print("objet à droite : :")
        aller((0,int(180*angle/pi)))
        return 0

    if droite>centre+40:
        if gauche>centre+40:
            print("objet en face")
            aller((centre-dPince,0))
            return 1
        else:
            angle = (pi/2-atan((gauche+dCentre)/dCapteurs))/2
            print("centre gauche:")
            aller((0,int(180*angle/pi)))
            return 0
    else:
        if gauche>centre+40:
            angle = -(pi/2-atan((droite+dCentre)/dCapteurs))/2
            aller((0,int(180*angle/pi)))
            print("centre droit :"  )
            return 0
        else:
            print("gros objet")
            return -1

capteurDist().start()
##serialRead().start()
execution().start()
