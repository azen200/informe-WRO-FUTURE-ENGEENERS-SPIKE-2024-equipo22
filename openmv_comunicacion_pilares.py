import sensor
import time
import math

# Seteamos la comunicación con Pybricks
from pupremote import PUPRemoteSensor
pr = PUPRemoteSensor()
# Set up data channel, named 'cam' with signed half ints h to send speed and turn_rate.
pr.add_channel('cam',to_hub_fmt='hhh')

# Umbrales para la deteccion de colores
umbrales = [
    (40, 60, 40, 70, 10, 60),  # Umbral en color CIELab para el pilar rojo
    (5, 80, -50, -10, -15, 15),  # Umbral en color CIELab para el pilar verde
]

# Iniciamos la camarita
sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QQVGA) # Resolucion menor a QVGA → Nos da 74fps, en QVGA nos daba 35fps
sensor.skip_frames(time=2000)
sensor.set_auto_gain(False)  # Probablemente debe estar desactivado para el seguimiento de color
sensor.set_auto_whitebal(False)  # Probablemente debe estar desactivado para el seguimiento de color
clock = time.clock()

# Contadores de identificación únicos para cada color

while True:
    blob_elegido = None
    x_blob_elegido = 0
    y_blob_elegido = 0
    id_blob_elegido = None

    clock.tick()
    img = sensor.snapshot() # Tomamos una nueva imagen cada vez que se reinicia el bucle

    contador = 0  # Contador para iterar sobre los umbrales (simple)

    while contador < len(umbrales):
        umbral_recorrido = umbrales[contador]
        blobs = img.find_blobs([umbral_recorrido], pixels_threshold=300, area_threshold=400)

        for blob in blobs:
            if contador == 0:  # Red blobs
                id_unica = 0
                color = (255, 0, 0)  # Rojo
            elif contador == 1:  # green blobs
                id_unica = 1
                color = (0, 255, 0)  # Verde
            if blob_elegido == None or blob.cy() > blob_elegido.cy():
                blob_elegido = blob
                id_blob_elegido = id_unica
                x_blob_elegido = int(blob.cx())
                y_blob_elegido = int(blob.cy())

            # Acá podemos elegir si nuestro blob es circular o no
            if blob.elongation() > 0.7:
                img.draw_edges(blob.min_corners(), color=color)
                img.draw_line(blob.major_axis_line(), color=color)
                img.draw_line(blob.minor_axis_line(), color=color)

            # Eb cada blob le dibujamos si respectivo rectangulo
            img.draw_rectangle(blob.rect(), color=color)
            img.draw_cross(blob.cx(), blob.cy(), color=color)
            img.draw_string(blob.x(), blob.y() - 10, str(id_unica), color=color)

        contador += 1  # Incrementa el contador para pasar al siguiente umbral

    # Acá dibujamos la palabra Elegido en el blob más cercano
    #if blob_elegido != None:
    #    img.draw_string(int(x_blob_elegido), int(y_blob_elegido) - 20, str("Elegido " + str(id_blob_elegido)), color=(255, 255, 255))

    # En caso de no encontrar ningún blob, enviamos -1 para indicar que hubo un error
    if blob_elegido == None:
        id_blob_elegido, x_blob_elegido, y_blob_elegido = -1, -1, -1

    print(id_blob_elegido, x_blob_elegido, y_blob_elegido)

    # Enviamos ID del blob elegido junto a las coordenadas de su centroide
    pr.update_channel('cam', id_blob_elegido, x_blob_elegido, y_blob_elegido)
    pr.process()

    print(clock.fps())
