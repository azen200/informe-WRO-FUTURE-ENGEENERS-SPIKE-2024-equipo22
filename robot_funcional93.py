from pybricks.hubs import PrimeHub
from pybricks.pupdevices import Motor, ColorSensor, UltrasonicSensor, ForceSensor
from pybricks.parameters import Button, Color, Direction, Port, Side, Stop
from pybricks.robotics import DriveBase
from pybricks.tools import wait, StopWatch

class Robot():
    def __init__(self):
        self.spike = PrimeHub()
        self.spike.imu.ready()
        self.spike.imu.reset_heading(0)
        self.motor_direccion = Motor(Port.B,positive_direction=Direction.COUNTERCLOCKWISE, gears=None, reset_angle=True, profile=None ) 
        self.motor_traccion = Motor(Port.A, positive_direction=Direction.CLOCKWISE, gears=None, reset_angle=True, profile=None)
        self.contador = StopWatch()
        self.sensorcolor = ColorSensor(Port.D)
        self.sensorizquierda = UltrasonicSensor(Port.F)
        self.sensorderecha = UltrasonicSensor(Port.C)

        # VARIABLES
        self.control_direccion = None

    def ultrasonido_elegido(self):
        if self.control_direccion == 1:
            return self.sensorderecha.distance()
        elif self.control_direccion == -1:
            return self.sensorizquierda.distance()
        
    def que_color_es(self):
        colores_hsv = self.sensorcolor.hsv()
        h, s, v = colores_hsv.h, colores_hsv.s, colores_hsv.v

        naranja_lona = [352,76,100]
        violeta_lona = [234,64,67]
        blanco_lona = [0,0,100]

        naranja_h, naranja_s, naranja_v = naranja_lona[0], naranja_lona[1], naranja_lona[2]
        violeta_h, violeta_s, violeta_v = violeta_lona[0], violeta_lona[1], violeta_lona[2]
        blanco_h, blanco_s, blanco_v = blanco_lona[0], blanco_lona[1], blanco_lona[2]

        sensibilidad_nar, sensibilidad_viol, sensibilidad_blan = 1, 1, 1

        formula_naranja=((h-naranja_h)**2+(s-naranja_s)**2+(v-naranja_v)**2)*sensibilidad_nar
        formula_violeta=((h-violeta_h)**2+(s-violeta_s)**2+(v-violeta_v)**2)*sensibilidad_viol
        formula_blanco=((h-blanco_h)**2+(s-blanco_s)**2+(v-blanco_v)**2)*sensibilidad_blan

        if formula_naranja < formula_violeta and formula_naranja < formula_blanco:
            return("Naranja")
        elif formula_violeta<formula_blanco:
            return("Violeta")
        else:
            return("Blanco")
   
    def pip(self):
        self.spike.speaker.beep(frequency=400, duration=20)

    def contador_obtener(self):
        self.contador.time()

    def contador_resetear(self):
        self.contador.reset()

    def timeout(self, tiempo_max):
        self.contador.time()
        if self.contador.time() > tiempo_max:
            return True
        else: return False

    def angulo_resetear(self):
        self.spike.imu.reset_heading(0)
    
    def angulo_obtener(self):
        angulo = self.spike.imu.heading()
        return angulo
    
    def direccion_centrar(self):
        self.contador_resetear()
        while (not self.motor_direccion.stalled()) or (self.timeout(2500)): # Defino limite izquierda
            print(self.contador.time())
            self.motor_direccion.run(-150)
        self.motor_direccion.hold()
        self.pip()
        maximo_izquierda = self.motor_direccion.angle()
        
        self.contador_resetear()
        while (not self.motor_direccion.stalled()) or (self.timeout(2500)): # Defino limite derecha
            print(self.contador.time())
            self.motor_direccion.run(150)
        self.motor_direccion.hold()
        self.pip()
        maximo_derecha = self.motor_direccion.angle()
        
        centro_absoluto = (maximo_derecha + maximo_izquierda) / 2
        print(maximo_izquierda, maximo_derecha, centro_absoluto)
        self.motor_direccion.run_target(150, centro_absoluto, then=Stop.HOLD, wait=True)
        self.pip()
        
        self.limite_derecha = maximo_derecha - centro_absoluto
        self.limite_izquierda = maximo_izquierda - centro_absoluto

        print(self.limite_derecha, self.limite_izquierda)
    
        self.motor_direccion.reset_angle(0)

    def direccion_controlar(self, angulo_deseado, kp, velocidad):
        angulo_actual = self.angulo_obtener()

        error = angulo_deseado - angulo_actual
        correccion = error * kp

        if correccion != 0:
            self.motor_direccion.run_target(200, correccion, then=Stop.BRAKE, wait=False)
        else:
            self.motor_direccion.brake()
        
        self.motor_traccion.run(velocidad)
    def giro_forza(self):
        return self.sensorFRENTE.distance()



# ------------------- CONFIGURACIÓN INICIAL -----------------------

robotito = Robot()

robotito.angulo_resetear()

robotito.direccion_centrar()

direccion = 0

robotito.control_direccion = -1

# ----------------- PRIMER AVANCE PARA DEFINIR LA DIRECCION ------------------

robotito.contador_resetear()

while not robotito.timeout(10000): 
    color_detectado = robotito.que_color_es() # DETECTAMOS el color de la primera linea
    robotito.direccion_controlar(direccion, 1.2, 600)
    if color_detectado == "Naranja" or robotito.que_color_es() == "Violeta":
        robotito.motor_traccion.hold()
        if color_detectado == "Naranja":
            robotito.control_direccion = 1
        if color_detectado == "Violeta":
            robotito.control_direccion = -1
        break

robotito.pip()



while robotito.ultrasonido_elegido() < 900:  # Mientras que nuestro sensor elegido nos de una distancia menor a 90cm, avanzamos
    
        robotito.direccion_controlar(direccion, 1.2, 2000)

robotito.pip()

# ------------- LUEGO DE DEFININA LA DIRECCIÓN, HACEMOS LAS VUELTAS --------------

while True:
    if robotito.control_direccion == -1:
        direccion -= 90
    else:
        direccion += 90

    robotito.contador_resetear()

    contador_grados_inicio = robotito.motor_traccion.angle()
    while not robotito.motor_traccion.angle() > contador_grados_inicio + 1500: # PRIMERO, AVANZAMOS 2.2seg para llegar a estar al lado de la pared
        robotito.direccion_controlar(direccion, 1.1, 2000)
    robotito.pip()

    while robotito.ultrasonido_elegido() < 900: # Mientras que nuestro sensor elegido nos de una distancia menor a 90cm, avanzamos
        robotito.direccion_controlar(direccion, 1.2, 2000)

    robotito.pip()



    if abs(direccion) > 989:
        robotito.pip()
        break

robotito.motor_traccion.hold()

#----------------- ULTIMO AVANCE PARA TERMINAR EN LA SECCION DE INICIO ------------------

if robotito.control_direccion == -1:
    direccion -= 90
else:
    direccion += 90

contador_grados_inicio = robotito.motor_traccion.angle()
while not robotito.motor_traccion.angle() > contador_grados_inicio + 1500:
    robotito.direccion_controlar(direccion, 0.65, 2000)

robotito.motor_traccion.hold()

