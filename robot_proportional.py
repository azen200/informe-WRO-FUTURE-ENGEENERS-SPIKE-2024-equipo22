from pybricks.hubs import PrimeHub
from pybricks.pupdevices import Motor, ColorSensor, UltrasonicSensor, ForceSensor
from pybricks.parameters import Button, Color, Direction, Port, Side, Stop
from pybricks.robotics import DriveBase
from pybricks.tools import wait, StopWatch
from pupremote import PUPRemoteHub


pr = PUPRemoteHub(Port.E)
pr.add_channel('cam', to_hub_fmt='hhh')
hub = PrimeHub()


class Robot():
    def __init__(self):
        self.spike = PrimeHub()
        self.spike.imu.ready()
        self.spike.imu.reset_heading(0)
        self.motor_direccion = Motor(Port.B, positive_direction=Direction.COUNTERCLOCKWISE)
        self.motor_traccion = Motor(Port.A, positive_direction=Direction.CLOCKWISE)
        self.timer1 = StopWatch()
        self.sensorcolor = ColorSensor(Port.D)
        self.sensorizquierda = UltrasonicSensor(Port.F)
        self.sensorderecha = UltrasonicSensor(Port.C)

        # VARIABLES
        self.sentido_giro = 1  # 1 derecha -1 izquierda , inicia derecha por las dudas

    def distancia_pared_adentro(self):
        if self.sentido_giro == 1:
            return self.sensorderecha.distance()
        elif self.sentido_giro == -1:
            return self.sensorizquierda.distance()

    def distancia_pared_afuera(self):
        if self.sentido_giro == -1:
            return self.sensorderecha.distance()
        elif self.sentido_giro == 1:
            return self.sensorizquierda.distance()

    def que_color_es(self):
        colores_hsv = self.sensorcolor.hsv()
        h, s, v = colores_hsv.h, colores_hsv.s, colores_hsv.v

        naranja_lona = [352, 76, 100]
        violeta_lona = [234, 64, 67]
        blanco_lona = [0, 0, 100]

        naranja_h, naranja_s, naranja_v = naranja_lona[0], naranja_lona[1], naranja_lona[2]
        violeta_h, violeta_s, violeta_v = violeta_lona[0], violeta_lona[1], violeta_lona[2]
        blanco_h, blanco_s, blanco_v = blanco_lona[0], blanco_lona[1], blanco_lona[2]

        sensibilidad_nar, sensibilidad_viol, sensibilidad_blan = 1, 1, 1

        formula_naranja = ((h - naranja_h) ** 2 + (s - naranja_s) ** 2 + (v - naranja_v) ** 2) * sensibilidad_nar
        formula_violeta = ((h - violeta_h) ** 2 + (s - violeta_s) ** 2 + (v - violeta_v) ** 2) * sensibilidad_viol
        formula_blanco = ((h - blanco_h) ** 2 + (s - blanco_s) ** 2 + (v - blanco_v) ** 2) * sensibilidad_blan

        if formula_naranja < formula_violeta and formula_naranja < formula_blanco:
            return "Naranja"
        elif formula_violeta < formula_blanco:
            return "Violeta"
        else:
            return "Blanco"

    def pip(self):
        self.spike.speaker.beep(frequency=400, duration=20)

    def angulo_resetear(self):
        self.spike.imu.reset_heading(0)

    def angulo_obtener(self):
        angulo = self.spike.imu.heading()
        return angulo

    def direccion_centrar(self):
        self.timer1.reset()
        while not self.motor_direccion.stalled() or self.timer1.time() > 2500:  # Defino limite izquierda
            print(self.timer1.time())
            self.motor_direccion.run(-150)
        self.motor_direccion.hold()
        self.pip()
        maximo_izquierda = self.motor_direccion.angle()

        self.timer1.reset()
        while not self.motor_direccion.stalled() or self.timer1.time() < 2500:  # Defino limite derecha
            print(self.timer1.time())
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
    
    
    def avance_derecho(self, angulo_deseado, kpangulo, velocidad, distancia_deseada, kpdistance):
        # print("avance derecho")
        # Obtener el ángulo actual del robot
        angulo_actual = self.angulo_obtener()
        
        # Calcular el error de orientación
        error_angulo = angulo_deseado - angulo_actual
        correccion_angulo = error_angulo * kpangulo
        
        # Obtener la distancia actual a la pared usando el sensor de ultrasonido
        distancia_actual = self.distancia_pared_afuera()
        if distancia_actual > 1000:
            kpdistance = 0
        
        # Calcular el error de distancia
        error_distancia = distancia_deseada - distancia_actual
        correccion_distancia = error_distancia * kpdistance
        if self.sentido_giro == -1:
            correccion_distancia = -correccion_distancia
        
        # Combinar las correcciones de ángulo y distancia
        
        correccion_total = correccion_angulo + correccion_distancia
        if correccion_total > self.limite_derecha:
            correccion_total = 80
        if correccion_total  < self.limite_izquierda:
            correccion_total = -80
        # print(correccion_total,  correccion_angulo, correccion_distancia)
        
        # Aplicar la corrección al motor de dirección
        if correccion_total != 0:
            self.motor_direccion.run_target(200, correccion_total, then=Stop.BRAKE, wait=False)
        else:
            self.motor_direccion.brake()
        
        # Mover el robot hacia adelante
        self.motor_traccion.run(velocidad)

    def avance_derecho_ang(self, angulo_apuntado, kpangulo, velocidad):
        # print("avance derecho")
        # Obtener el ángulo actual del robot
        angulo_actual = self.angulo_obtener()
        
        # Calcular el error de orientación
        error_angulo = angulo_apuntado - angulo_actual
        correccion_angulo = error_angulo * kpangulo
        
        # Combinar las correcciones de ángulo y distancia
        correccion_total = correccion_angulo
        if correccion_total > self.limite_derecha:
            correccion_total = 80
        if correccion_total  < self.limite_izquierda:
            correccion_total = -80
        # print(correccion_total,  correccion_angulo, correccion_distancia)
        
        # Aplicar la corrección al motor de dirección
        if correccion_total != 0:
            self.motor_direccion.run_target(200, correccion_total, then=Stop.BRAKE, wait=False)
        else:
            self.motor_direccion.brake()
        
        # Mover el robot hacia adelante
        self.motor_traccion.run(velocidad)

    def giro_derecha(self, angulo_giro_d, kp, velocidad):
        angulo_actual = self.angulo_obtener()
        error = angulo_giro_d - angulo_actual
        correccion = error * kp
        if correccion != 0:
            self.motor_direccion.run_target(200, correccion, then=Stop.BRAKE, wait=False)
        else:
            self.motor_direccion.brake()
        self.motor_traccion.run(velocidad)

    def giro_izquierda(self, angulo_giro_i, kp, velocidad):
        angulo_actual = self.angulo_obtener()
        error = angulo_giro_i - angulo_actual
        correccion = error * kp
        print(correccion)
        if correccion != 0:
            self.motor_direccion.run_target(200, correccion, then=Stop.BRAKE, wait=False)
        else:
            self.motor_direccion.brake()
        self.motor_traccion.run(velocidad)
    
    def parar_robot(self):
        while True:
            robotito.avance_derecho(angulo_deseado=0, kpangulo=1.5, velocidad=0, distancia_deseada=300, kpdistance=0.7)
            print("Deteniendo el robot")
            # Detiene el motor de tracción
            self.motor_traccion.stop(Stop.BRAKE)
            # Detiene el motor de dirección (si está en movimiento)
            self.motor_direccion.stop(Stop.HOLD)
            wait(2000)
            break
    
    def esquivar_derecha(self):
        while True:
            self.avance_derecho_ang(0,1.5, 150)
            wait(750)
            self.giro_derecha(60, 1.5, 150)
            wait(1500)
            self.avance_derecho_ang( 0, 0.6, 150)
            wait(1500)
            break

    def esquivar_izquierda(self):
        while True:
            self.avance_derecho_ang( 0, 1.5, 150)
            wait(750)
            self.giro_izquierda( -60, 1.5, 150)
            wait(1500)
            self.avance_derecho_ang( 0, 0.6, 150)
            wait(1500)
            break
    

# ------------------- CONFIGURACIÓN INICIAL -----------------------

robotito = Robot()

robotito.angulo_resetear()

robotito.direccion_centrar()

direccion = 0

robotito.sentido_giro = 1


# prueba comunicacion camara GV 25 8
# while 1:
#     id_blob_elegido, x_blob_elegido, y_blob_elegido = pr.call('cam')
#     print(id_blob_elegido, x_blob_elegido, y_blob_elegido)
# fin prueba comunicacion camara GV 25 8
# ----------------- PRIMER AVANCE PARA DEFINIR LA DIRECCION ------------------

robotito.timer1.reset()
while robotito.timer1.time() < 40000:

    # print(f"Distancia Pared Afuera: {robotito.distancia_pared_afuera()}")
    # color_detectado = robotito.que_color_es()  # DETECTAMOS el color de la primera linea
    robotito.avance_derecho(angulo_deseado=0, kpangulo=1.5, velocidad=150, distancia_deseada=300, kpdistance=0.7)
    
    id_blob_elegido, x_blob_elegido, y_blob_elegido = pr.call('cam')
    print(id_blob_elegido, x_blob_elegido, y_blob_elegido)

    if id_blob_elegido != -1:
        robotito.motor_traccion.brake()
        robotito.pip()
        wait(2000)
        if  id_blob_elegido == 0:
            robotito.esquivar_derecha()
        elif  id_blob_elegido == 1:
            robotito.esquivar_izquierda()
   

    

robotito.pip()
