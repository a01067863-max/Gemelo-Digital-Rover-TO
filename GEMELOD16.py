print("Gemelo digital iniciado - Fase 2: Lectura serial en tiempo real")

import math
import re
from collections import deque

try:
    import serial
except ImportError:
    print("ERROR: No se encontró la librería pyserial.")
    print("Instálala con: pip install pyserial")
    raise

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

import csv
import os
import sys
import subprocess
from pathlib import Path
from datetime import datetime
# ==========================
# CONFIGURACIÓN DE REGISTRO
# ==========================
# IMPORTANTE:
# Usamos rutas absolutas basadas en la ubicación de este archivo .py.
# Así, aunque ejecutes con "Run and Debug", los CSV siempre se guardan
# junto al programa y no en una carpeta escondida de VS Code.
BASE_DIR = Path(__file__).resolve().parent

CARPETA_REGISTROS = BASE_DIR / "registros_trayectorias"
CARPETA_REGISTROS.mkdir(exist_ok=True)

nombre_sesion = datetime.now().strftime("trayectoria_%Y-%m-%d_%H-%M-%S.csv")

ARCHIVO_ACTUAL = CARPETA_REGISTROS / nombre_sesion
ARCHIVO_ULTIMA = CARPETA_REGISTROS / "ultima_trayectoria.csv"

# Bandera para saber si ya empezamos a escribir la última trayectoria
# de esta ejecución. La primera muestra sobrescribe el archivo anterior;
# las siguientes se agregan. Esto permite que la última trayectoria se
# actualice en tiempo real y no dependa de que el programa cierre perfecto.
ultima_trayectoria_inicializada = False

COLUMNAS = [
    "fecha_pc",
    "timestamp_ms",
    "x_m",
    "y_m",
    "yaw_deg",
    "vx_cm_s",
    "vy_cm_s",
    "flow",
    "lat",
    "lng",
    "temperatura_c",
    "humedad_pct",
    "presion_hpa",
    "soil_pct",
    "voltaje_v",
    "corriente_ma",
    "distancia_cm",
    "raw_line"
]

datos_sesion = []

# =========================
# CONFIGURACIÓN SERIAL
# =========================

# Cambia este puerto según el COM que te aparezca en Arduino IDE / Administrador de dispositivos.
PUERTO_SERIAL = "COM7"
BAUDRATE = 115200
TIMEOUT_SERIAL = 0.02

# Cantidad máxima de líneas que se leen por ciclo de actualización.
# El bloque completo tiene varias líneas: ESP-NOW, POS, VEL, GPS, ENV, PWR, DIST.
LINEAS_POR_CICLO = 60


# =========================
# CONFIGURACIÓN GENERAL
# =========================

DELAY_VISUALIZACION = 0.05
MAX_POINTS = 120

LONGITUD_FLECHA_2D = 0.12
LONGITUD_FLECHA_3D = 0.25

# Dimensiones visuales del módulo 3D
LARGO_MODULO = 0.25
ANCHO_MODULO = 0.14
ALTO_MODULO = 0.08

# Configuración del rastro 3D
# - RASTRO_3D_MAX_POINTS controla cuántos puntos recientes se ven en el 3D.
# - GRAFICAR_3D_CADA_N controla cada cuántas muestras se guarda un punto en el rastro 3D.
# - VENTANA_3D_METROS controla el tamaño visible alrededor del rover.
RASTRO_3D_MAX_POINTS = 50
GRAFICAR_3D_CADA_N = 2
VENTANA_3D_METROS = 1.20

# Configuración del rumbo/yaw en la gráfica.
# Para graficar usamos un rumbo continuo y evitamos saltos falsos de 359° a 0°.
MARGEN_GRAFICA_RUMBO = 15


# =========================
# DATOS PARA VISUALIZACIÓN
# =========================

times = deque(maxlen=MAX_POINTS)
x_values = deque(maxlen=MAX_POINTS)
y_values = deque(maxlen=MAX_POINTS)
rumbo_values = deque(maxlen=MAX_POINTS)
cambio_rumbo_values = deque(maxlen=MAX_POINTS)

# Trayectoria completa para la gráfica 2D y métricas generales
trayectoria_x = []
trayectoria_y = []
trayectoria_z = []

# Rastro móvil para el modelo 3D.
# Aquí NO se guarda toda la trayectoria, solo los últimos puntos.
trayectoria_3d_x = deque(maxlen=RASTRO_3D_MAX_POINTS)
trayectoria_3d_y = deque(maxlen=RASTRO_3D_MAX_POINTS)
trayectoria_3d_z = deque(maxlen=RASTRO_3D_MAX_POINTS)


# =========================
# VARIABLES DE MOVIMIENTO
# =========================

distancia_recorrida_total = 0.0

x_anterior = None
y_anterior = None
rumbo_anterior = None
rumbo_continuo_anterior = None

tiempo_inicial_ms = None
muestra_actual = 0
flecha_orientacion_2d = None


# =========================
# PARSEO DEL FORMATO SERIAL NUEVO
# =========================

# Formato esperado aproximado:
# [ESP-NOW] t=1587ms
#   POS    x=0.00 m  y=0.00 m  yaw=-0.1 deg
#   VEL    vx=0.0 cm/s  vy=0.0 cm/s  flow=OK
#   GPS    lat=0.000000  lng=0.000000
#   ENV    T=0.0C  H=0%  P=0.0 hPa  soil=0%
#   PWR    0.00V  0.0 mA
#   DIST   0.0 cm

NUM = r"[-+]?(?:\d+(?:\.\d*)?|\.\d+)"

# Este prefijo aparece cuando copias desde el Serial Monitor de Arduino:
# 23:02:14.941 ->
# Cuando Python lee directo del puerto, normalmente NO aparece, pero lo soportamos por seguridad.
RE_PREFIX = re.compile(r"^\d{2}:\d{2}:\d{2}\.\d{3}\s*->\s*")

RE_TIME = re.compile(r"\[ESP-NOW\]\s*t=(\d+)ms", re.IGNORECASE)

RE_POS = re.compile(
    rf"POS\s+x=({NUM})\s*m\s+y=({NUM})\s*m\s+yaw=({NUM})\s*deg",
    re.IGNORECASE
)

RE_VEL = re.compile(
    rf"VEL\s+vx=({NUM})\s*cm/s\s+vy=({NUM})\s*cm/s\s+flow=([A-Za-z]+)",
    re.IGNORECASE
)

RE_GPS = re.compile(
    rf"GPS\s+lat=({NUM})\s+lng=({NUM})",
    re.IGNORECASE
)

RE_ENV = re.compile(
    rf"ENV\s+T=({NUM})C\s+H=({NUM})%\s+P=({NUM})\s*hPa\s+soil=({NUM})%",
    re.IGNORECASE
)

RE_PWR = re.compile(
    rf"PWR\s+({NUM})V\s+({NUM})\s*mA",
    re.IGNORECASE
)

RE_DIST = re.compile(
    rf"DIST\s+({NUM})\s*cm",
    re.IGNORECASE
)

paquete_en_proceso = {}


def limpiar_linea(raw_line):
    """
    Limpia saltos de línea y elimina el prefijo del Serial Monitor si existe.
    """

    line = raw_line.strip()
    line = RE_PREFIX.sub("", line)
    return line.strip()


def procesar_linea_serial(line):
    """
    Procesa una línea del serial.
    Cuando llega DIST, se asume que el bloque está completo y regresa un paquete.
    Si todavía no está completo, regresa None.
    """

    global paquete_en_proceso

    if not line:
        return None

    # Inicio de un bloque nuevo
    coincidencia = RE_TIME.search(line)
    if coincidencia:
        paquete_en_proceso = {
            "timestamp_ms": int(coincidencia.group(1)),
            "raw_line": line
        }
        return None

    # Si por alguna razón llega una línea antes del encabezado, no rompemos el programa.
    if not paquete_en_proceso:
        paquete_en_proceso = {
            "raw_line": line
        }
    else:
        # Guardamos el bloque original completo para que también quede evidencia en el CSV.
        paquete_en_proceso["raw_line"] = paquete_en_proceso.get("raw_line", "") + " | " + line

    # POS
    coincidencia = RE_POS.search(line)
    if coincidencia:
        paquete_en_proceso["x_m"] = float(coincidencia.group(1))
        paquete_en_proceso["y_m"] = float(coincidencia.group(2))
        paquete_en_proceso["yaw_deg"] = float(coincidencia.group(3))
        return None

    # VEL
    coincidencia = RE_VEL.search(line)
    if coincidencia:
        paquete_en_proceso["vx_cms"] = float(coincidencia.group(1))
        paquete_en_proceso["vy_cms"] = float(coincidencia.group(2))
        paquete_en_proceso["flow"] = coincidencia.group(3).upper()
        return None

    # GPS
    coincidencia = RE_GPS.search(line)
    if coincidencia:
        paquete_en_proceso["gps_lat"] = float(coincidencia.group(1))
        paquete_en_proceso["gps_lng"] = float(coincidencia.group(2))
        return None

    # ENV
    coincidencia = RE_ENV.search(line)
    if coincidencia:
        paquete_en_proceso["temp_c"] = float(coincidencia.group(1))
        paquete_en_proceso["hum_pct"] = float(coincidencia.group(2))
        paquete_en_proceso["pressure_hpa"] = float(coincidencia.group(3))
        paquete_en_proceso["soil_pct"] = float(coincidencia.group(4))
        return None

    # PWR
    coincidencia = RE_PWR.search(line)
    if coincidencia:
        paquete_en_proceso["voltage_v"] = float(coincidencia.group(1))
        paquete_en_proceso["current_ma"] = float(coincidencia.group(2))
        return None

    # DIST marca el final útil del bloque
    coincidencia = RE_DIST.search(line)
    if coincidencia:
        paquete_en_proceso["distance_cm"] = float(coincidencia.group(1))

        campos_obligatorios = [
            "timestamp_ms",
            "x_m",
            "y_m",
            "yaw_deg",
            "vx_cms",
            "vy_cms",
            "flow",
            "distance_cm"
        ]

        if all(campo in paquete_en_proceso for campo in campos_obligatorios):
            paquete = paquete_en_proceso.copy()
            paquete_en_proceso = {}
            return paquete

    return None


def paquete_valido(paquete):
    """
    Filtro básico para evitar graficar basura si llega una línea incompleta.
    """

    x = paquete.get("x_m")
    y = paquete.get("y_m")
    yaw = paquete.get("yaw_deg")

    if x is None or y is None or yaw is None:
        return False

    if not math.isfinite(x) or not math.isfinite(y) or not math.isfinite(yaw):
        return False

    # Límite amplio solo para detectar datos claramente corruptos.
    if abs(x) > 1000 or abs(y) > 1000:
        return False

    return True


def leer_paquetes_disponibles(ser):
    """
    Lee varias líneas disponibles del serial y devuelve todos los paquetes completos.
    """

    paquetes = []

    for _ in range(LINEAS_POR_CICLO):
        try:
            raw = ser.readline()
        except serial.SerialException as error:
            print("Error leyendo del puerto serial:", error)
            return paquetes

        if not raw:
            break

        line = raw.decode("utf-8", errors="ignore")
        line = limpiar_linea(line)

        paquete = procesar_linea_serial(line)

        if paquete is not None and paquete_valido(paquete):
            paquetes.append(paquete)

    return paquetes


# =========================
# FUNCIONES DE RUMBO Y MOVIMIENTO Y CARGA DE DATOS AL PRINCIPIO     
# =========================
def escribir_fila_csv(ruta, fila, modo="a"):
    """
    Escribe una fila en un archivo CSV usando siempre las mismas columnas.
    Si el archivo es nuevo o se abre en modo "w", también escribe encabezados.
    """

    archivo_nuevo = (not ruta.exists()) or modo == "w"
    fila_limpia = {col: fila.get(col, "") for col in COLUMNAS}

    with open(ruta, mode=modo, newline="", encoding="utf-8") as archivo:
        writer = csv.DictWriter(archivo, fieldnames=COLUMNAS)

        if archivo_nuevo:
            writer.writeheader()

        writer.writerow(fila_limpia)


def guardar_dato_csv(fila):
    """
    Guarda una fila de datos en:
    1. El CSV histórico de la sesión actual.
    2. ultima_trayectoria.csv, actualizado en tiempo real.

    Esto arregla el problema de Run and Debug:
    aunque detengas el programa con Stop, la última trayectoria ya quedó
    guardada punto por punto.
    """

    global ultima_trayectoria_inicializada

    # Archivo único de esta corrida
    escribir_fila_csv(ARCHIVO_ACTUAL, fila, modo="a")

    # Archivo que el gemelo cargará como "última trayectoria" en la siguiente corrida.
    # En la primera muestra de esta ejecución se reemplaza el archivo anterior.
    # En las siguientes muestras se agregan filas.
    modo_ultima = "a" if ultima_trayectoria_inicializada else "w"
    escribir_fila_csv(ARCHIVO_ULTIMA, fila, modo=modo_ultima)
    ultima_trayectoria_inicializada = True

    datos_sesion.append(fila)


def cargar_ultima_trayectoria():
    """
    Carga la última trayectoria guardada.
    Regresa dos listas: x_ultima, y_ultima.
    """

    x_ultima = []
    y_ultima = []

    if not ARCHIVO_ULTIMA.exists():
        return x_ultima, y_ultima

    try:
        with open(ARCHIVO_ULTIMA, mode="r", newline="", encoding="utf-8") as archivo:
            reader = csv.DictReader(archivo)

            for fila in reader:
                try:
                    x = float(fila["x_m"])
                    y = float(fila["y_m"])

                    x_ultima.append(x)
                    y_ultima.append(y)

                except:
                    pass

    except Exception as e:
        print(f"No se pudo cargar la última trayectoria: {e}")

    return x_ultima, y_ultima


def abrir_archivo(ruta):
    """
    Abre el archivo CSV al terminar el programa.
    En Windows normalmente se abre con Excel.
    """

    try:
        if os.name == "nt":
            os.startfile(ruta)
        elif sys.platform == "darwin":
            subprocess.Popen(["open", ruta])
        else:
            subprocess.Popen(["xdg-open", ruta])

    except Exception as e:
        print(f"No se pudo abrir el archivo automáticamente: {e}")


def finalizar_registro():
    """
    Se ejecuta al final del programa.
    La última trayectoria ya se fue guardando en tiempo real.
    Aquí solo se informa la ubicación de los archivos y se abre el CSV de la sesión.
    """

    if len(datos_sesion) == 0:
        print("No se guardaron datos nuevos porque no se recibió información.")
        print(f"Si existía una trayectoria anterior, sigue en: {ARCHIVO_ULTIMA}")
        return

    try:
        print("\nRegistro finalizado correctamente.")
        print(f"Archivo de esta sesión: {ARCHIVO_ACTUAL}")
        print(f"Última trayectoria actualizada en tiempo real: {ARCHIVO_ULTIMA}")

        abrir_archivo(ARCHIVO_ACTUAL)

    except Exception as e:
        print(f"Error al finalizar el registro: {e}")


def calcular_vector_rumbo(rumbo_grados, longitud):
    """
    Convierte rumbo en grados a vector dx, dy.

    Convención:
    0°   = arriba en Y
    90°  = derecha en X
    180° = abajo
    270° = izquierda
    """

    angulo_rad = math.radians(rumbo_grados)

    dx = longitud * math.sin(angulo_rad)
    dy = longitud * math.cos(angulo_rad)

    return dx, dy


def calcular_estado_movimiento(desplazamiento, cambio_rumbo):
    """
    Clasifica el movimiento del módulo.
    """

    if desplazamiento < 0.002:
        return "DETENIDO"

    if abs(cambio_rumbo) >= 25:
        return "GIRO BRUSCO"

    elif desplazamiento < 0.015:
        return "MOVIMIENTO LENTO"

    else:
        return "MOVIMIENTO NORMAL"


def calcular_cambio_rumbo(rumbo_actual, rumbo_anterior):
    """
    Calcula el cambio angular real entre dos rumbos,
    evitando errores al cruzar 0° / 360°.

    Ejemplo:
    359° -> 0.5° no es -358.5°, sino +1.5°.
    0.5° -> 359° no es +358.5°, sino -1.5°.
    """

    if rumbo_anterior is None:
        return 0.0

    cambio = rumbo_actual - rumbo_anterior

    if cambio > 180:
        cambio -= 360
    elif cambio < -180:
        cambio += 360

    return cambio


def calcular_rumbo_continuo(rumbo_actual, rumbo_continuo_anterior):
    """
    Convierte el rumbo en un rumbo continuo para graficar.
    Soporta yaw en 0-360 y también yaw con valores negativos cercanos a 0.
    """

    if rumbo_continuo_anterior is None:
        return rumbo_actual

    ultimo_rumbo_normalizado = rumbo_continuo_anterior % 360
    cambio_real = calcular_cambio_rumbo(rumbo_actual, ultimo_rumbo_normalizado)

    return rumbo_continuo_anterior + cambio_real


def ajustar_eje_rumbo(ax, valores_rumbo):
    """
    Ajusta el eje Y de la gráfica de rumbo continuo.
    Ya no se fuerza de 0 a 360 porque eso reintroduce el salto visual.
    """

    if len(valores_rumbo) == 0:
        return

    minimo = min(valores_rumbo)
    maximo = max(valores_rumbo)

    if math.isclose(minimo, maximo):
        ax.set_ylim(minimo - MARGEN_GRAFICA_RUMBO, maximo + MARGEN_GRAFICA_RUMBO)
    else:
        ax.set_ylim(minimo - MARGEN_GRAFICA_RUMBO, maximo + MARGEN_GRAFICA_RUMBO)


# =========================
# FUNCIONES PARA MODELO 3D
# =========================

def crear_vertices_modulo_3d(x, y, z, rumbo):
    """
    Crea los vértices de una caja 3D orientada según el rumbo.
    """

    theta = math.radians(rumbo)

    # Vector hacia adelante según rumbo
    forward_x = math.sin(theta)
    forward_y = math.cos(theta)

    # Vector hacia la derecha del módulo
    right_x = math.cos(theta)
    right_y = -math.sin(theta)

    largo_2 = LARGO_MODULO / 2
    ancho_2 = ANCHO_MODULO / 2

    z_base = z
    z_top = z + ALTO_MODULO

    p1 = (
        x + forward_x * largo_2 + right_x * ancho_2,
        y + forward_y * largo_2 + right_y * ancho_2,
        z_base
    )
    p2 = (
        x + forward_x * largo_2 - right_x * ancho_2,
        y + forward_y * largo_2 - right_y * ancho_2,
        z_base
    )
    p3 = (
        x - forward_x * largo_2 - right_x * ancho_2,
        y - forward_y * largo_2 - right_y * ancho_2,
        z_base
    )
    p4 = (
        x - forward_x * largo_2 + right_x * ancho_2,
        y - forward_y * largo_2 + right_y * ancho_2,
        z_base
    )

    p5 = (p1[0], p1[1], z_top)
    p6 = (p2[0], p2[1], z_top)
    p7 = (p3[0], p3[1], z_top)
    p8 = (p4[0], p4[1], z_top)

    caras = [
        [p1, p2, p3, p4],  # base
        [p5, p6, p7, p8],  # techo
        [p1, p2, p6, p5],  # frente
        [p2, p3, p7, p6],
        [p3, p4, p8, p7],
        [p4, p1, p5, p8]
    ]

    return caras


def actualizar_modelo_3d(ax, x, y, z, rumbo):
    """
    Limpia y redibuja la vista 3D del módulo.

    El rastro 3D usa una ventana móvil:
    solo muestra los últimos RASTRO_3D_MAX_POINTS puntos.
    Además, los límites de la gráfica siguen al rover para que no se salga de la vista.
    """

    ax.cla()

    # =========================
    # RASTRO 3D RECIENTE
    # =========================

    if len(trayectoria_3d_x) > 0:
        ax.plot(
            list(trayectoria_3d_x),
            list(trayectoria_3d_y),
            list(trayectoria_3d_z),
            marker="o",
            linewidth=2,
            markersize=4,
            label="Rastro 3D reciente"
        )

    # =========================
    # MODELO 3D DEL MÓDULO
    # =========================

    caras = crear_vertices_modulo_3d(x, y, z, rumbo)

    modulo_3d = Poly3DCollection(
        caras,
        alpha=0.55,
        edgecolor="black"
    )

    ax.add_collection3d(modulo_3d)

    # =========================
    # FLECHA 3D DE DIRECCIÓN
    # =========================

    dx, dy = calcular_vector_rumbo(rumbo, LONGITUD_FLECHA_3D)

    ax.quiver(
        x,
        y,
        z + ALTO_MODULO,
        dx,
        dy,
        0,
        length=1,
        normalize=False
    )

    # Punto actual del rover
    ax.scatter(x, y, z, s=45, label="Posición actual")

    # =========================
    # CONFIGURACIÓN VISUAL
    # =========================

    ax.set_title(
        f"Modelo 3D del módulo | Rastro: últimos {RASTRO_3D_MAX_POINTS} puntos"
    )
    ax.set_xlabel("X global (m)")
    ax.set_ylabel("Y global (m)")
    ax.set_zlabel("Z (m)")

    # Ventana móvil centrada en la posición actual.
    mitad_ventana = VENTANA_3D_METROS / 2

    ax.set_xlim(x - mitad_ventana, x + mitad_ventana)
    ax.set_ylim(y - mitad_ventana, y + mitad_ventana)
    ax.set_zlim(0, 0.5)

    ax.view_init(elev=25, azim=-60)
    ax.grid(True)
    ax.legend(loc="upper left")


# =========================
# ACTUALIZACIÓN DEL GEMELO DIGITAL
# =========================

def actualizar_gemelo_con_paquete(paquete):
    """
    Toma un paquete completo del serial y actualiza todas las gráficas.
    """

    global distancia_recorrida_total
    global x_anterior, y_anterior, rumbo_anterior, rumbo_continuo_anterior
    global tiempo_inicial_ms, muestra_actual, flecha_orientacion_2d

    timestamp_ms = paquete["timestamp_ms"]

    if tiempo_inicial_ms is None:
        tiempo_inicial_ms = timestamp_ms

    tiempo_s = (timestamp_ms - tiempo_inicial_ms) / 1000.0

    x = paquete["x_m"]
    y = paquete["y_m"]
    z = 0.0
    rumbo = paquete["yaw_deg"]

    vx = paquete.get("vx_cms", 0.0)
    vy = paquete.get("vy_cms", 0.0)
    flow = paquete.get("flow", "NA")
    distancia_sensor_cm = paquete.get("distance_cm", 0.0)

    gps_lat = paquete.get("gps_lat", 0.0)
    gps_lng = paquete.get("gps_lng", 0.0)
    temp_c = paquete.get("temp_c", 0.0)
    hum_pct = paquete.get("hum_pct", 0.0)
    pressure_hpa = paquete.get("pressure_hpa", 0.0)
    soil_pct = paquete.get("soil_pct", 0.0)
    voltage_v = paquete.get("voltage_v", 0.0)
    current_ma = paquete.get("current_ma", 0.0)
    raw_line = paquete.get("raw_line", "")

    muestra_actual += 1

    # =========================
    # DESPLAZAMIENTO ENTRE MUESTRAS
    # =========================

    if x_anterior is not None and y_anterior is not None:
        desplazamiento_muestra = math.sqrt((x - x_anterior) ** 2 + (y - y_anterior) ** 2)
    else:
        desplazamiento_muestra = 0.0

    distancia_recorrida_total += desplazamiento_muestra

    # =========================
    # CAMBIO DE RUMBO
    # =========================

    cambio_rumbo = calcular_cambio_rumbo(rumbo, rumbo_anterior)

    # Rumbo continuo solo para la gráfica.
    rumbo_continuo = calcular_rumbo_continuo(rumbo, rumbo_continuo_anterior)

    estado_movimiento = calcular_estado_movimiento(
        desplazamiento_muestra,
        cambio_rumbo
    )

    # =========================
    # GUARDAR DATOS EN CSV
    # =========================

    fila_csv = {
        "fecha_pc": datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
        "timestamp_ms": timestamp_ms,
        "x_m": x,
        "y_m": y,
        "yaw_deg": rumbo,
        "vx_cm_s": vx,
        "vy_cm_s": vy,
        "flow": flow,
        "lat": gps_lat,
        "lng": gps_lng,
        "temperatura_c": temp_c,
        "humedad_pct": hum_pct,
        "presion_hpa": pressure_hpa,
        "soil_pct": soil_pct,
        "voltaje_v": voltage_v,
        "corriente_ma": current_ma,
        "distancia_cm": distancia_sensor_cm,
        "raw_line": raw_line
    }
    guardar_dato_csv(fila_csv)

    # =========================
    # GUARDAR DATOS PARA VISUALIZACIÓN
    # =========================

    times.append(tiempo_s)
    x_values.append(x)
    y_values.append(y)
    rumbo_values.append(rumbo_continuo)
    cambio_rumbo_values.append(cambio_rumbo)

    trayectoria_x.append(x)
    trayectoria_y.append(y)
    trayectoria_z.append(z)

    # =========================
    # GUARDAR RASTRO 3D LIMITADO
    # =========================

    if muestra_actual % GRAFICAR_3D_CADA_N == 0:
        trayectoria_3d_x.append(x)
        trayectoria_3d_y.append(y)
        trayectoria_3d_z.append(z)

    # Asegura que desde la primera muestra aparezca algo en 3D.
    if len(trayectoria_3d_x) == 0:
        trayectoria_3d_x.append(x)
        trayectoria_3d_y.append(y)
        trayectoria_3d_z.append(z)

    # =========================
    # ACTUALIZAR TRAYECTORIA 2D
    # =========================

    line_trayectoria.set_data(trayectoria_x, trayectoria_y)
    punto_modulo.set_data([x], [y])

    if flecha_orientacion_2d is not None:
        try:
            flecha_orientacion_2d.remove()
        except ValueError:
            pass

    dx_2d, dy_2d = calcular_vector_rumbo(rumbo, LONGITUD_FLECHA_2D)

    flecha_orientacion_2d = ax_trayectoria.arrow(
        x,
        y,
        dx_2d,
        dy_2d,
        head_width=0.035,
        head_length=0.035,
        length_includes_head=True
    )

    # =========================
    # ACTUALIZAR MODELO 3D
    # =========================

    actualizar_modelo_3d(ax_3d, x, y, z, rumbo)

    # =========================
    # ACTUALIZAR GRÁFICAS
    # =========================

    line_rumbo.set_data(times, rumbo_values)

    line_x.set_data(times, x_values)
    line_y.set_data(times, y_values)

    line_cambio_rumbo.set_data(times, cambio_rumbo_values)

    # =========================
    # DESPLAZAMIENTO DESDE EL INICIO
    # =========================

    if len(trayectoria_x) > 1:
        dx_total = trayectoria_x[-1] - trayectoria_x[0]
        dy_total = trayectoria_y[-1] - trayectoria_y[0]
        distancia_directa = math.sqrt(dx_total ** 2 + dy_total ** 2)
    else:
        dx_total = 0.0
        dy_total = 0.0
        distancia_directa = 0.0

    # =========================
    # PANEL DE INFORMACIÓN
    # =========================

    texto_estado.set_text(
        f"ESTADO DEL MÓDULO\n\n"
        f"Modo: SERIAL EN TIEMPO REAL + 3D\n"
        f"Puerto: {PUERTO_SERIAL}\n"
        f"Baudrate: {BAUDRATE}\n"
        f"Muestra: {muestra_actual}\n"
        f"Tiempo ESP-NOW: {tiempo_s:.2f} s\n"
        f"Rastro 3D visible: {len(trayectoria_3d_x)} / {RASTRO_3D_MAX_POINTS} puntos\n"
        f"Última trayectoria cargada: {len(x_ultima)} puntos\n\n"

        f"POSICIÓN GLOBAL\n"
        f"X: {x:.3f} m\n"
        f"Y: {y:.3f} m\n"
        f"Z: {z:.3f} m\n\n"

        f"ORIENTACIÓN\n"
        f"Yaw sensor: {rumbo:.1f}°\n"
        f"Yaw gráfica: {rumbo_continuo:.1f}°\n"
        f"Cambio rumbo: {cambio_rumbo:+.1f}°\n\n"

        f"MOVIMIENTO\n"
        f"VX: {vx:.1f} cm/s\n"
        f"VY: {vy:.1f} cm/s\n"
        f"Flow: {flow}\n"
        f"Dist. sensor: {distancia_sensor_cm:.1f} cm\n"
        f"ΔX: {dx_total:.3f} m\n"
        f"ΔY: {dy_total:.3f} m\n"
        f"Dist. directa: {distancia_directa:.3f} m\n"
        f"Dist. recorrida: {distancia_recorrida_total:.3f} m\n\n"

        f"AMBIENTE / GPS / POTENCIA\n"
        f"GPS: {gps_lat:.6f}, {gps_lng:.6f}\n"
        f"T: {temp_c:.1f} °C | H: {hum_pct:.0f}%\n"
        f"P: {pressure_hpa:.1f} hPa | Soil: {soil_pct:.0f}%\n"
        f"PWR: {voltage_v:.2f} V | {current_ma:.1f} mA\n\n"

        f"ESTADO DINÁMICO\n"
        f"Desp. muestra: {desplazamiento_muestra:.4f} m\n"
        f"Estado: {estado_movimiento}"
    )

    # =========================
    # REESCALAR GRÁFICAS 2D
    # =========================

    ax_trayectoria.relim()
    ax_trayectoria.autoscale_view()
    ax_trayectoria.axis("equal")

    ax_rumbo.relim()
    ax_rumbo.autoscale_view()
    ajustar_eje_rumbo(ax_rumbo, rumbo_values)

    ax_posicion.relim()
    ax_posicion.autoscale_view()

    ax_cambio_rumbo.relim()
    ax_cambio_rumbo.autoscale_view()

    # =========================
    # GUARDAR VALORES ANTERIORES
    # =========================

    x_anterior = x
    y_anterior = y
    rumbo_anterior = rumbo
    rumbo_continuo_anterior = rumbo_continuo

    # =========================
    # TERMINAL
    # =========================

    print(
        f"Muestra {muestra_actual} | "
        f"t: {tiempo_s:.2f} s | "
        f"X: {x:.3f} m | "
        f"Y: {y:.3f} m | "
        f"Yaw sensor: {rumbo:.1f}° | "
        f"Yaw gráfica: {rumbo_continuo:.1f}° | "
        f"Cambio rumbo: {cambio_rumbo:+.1f}° | "
        f"VX: {vx:.1f} cm/s | "
        f"VY: {vy:.1f} cm/s | "
        f"Flow: {flow} | "
        f"Distancia total: {distancia_recorrida_total:.3f} m | "
        f"Rastro 3D: {len(trayectoria_3d_x)}/{RASTRO_3D_MAX_POINTS} | "
        f"Estado: {estado_movimiento}"
    )


# =========================
# CARGAR ÚLTIMA TRAYECTORIA GUARDADA
# =========================

x_ultima, y_ultima = cargar_ultima_trayectoria()

if len(x_ultima) > 0:
    print(f"Última trayectoria cargada: {len(x_ultima)} puntos.")
else:
    print("No hay una última trayectoria guardada todavía.")


# =========================
# CONFIGURACIÓN DE FIGURA
# =========================

plt.ion()

fig = plt.figure(figsize=(17, 9))
try:
    fig.canvas.manager.set_window_title("Digital Twin Rover - Serial en tiempo real")
except Exception:
    pass

ax_trayectoria = fig.add_subplot(2, 3, 1)
ax_rumbo = fig.add_subplot(2, 3, 2)
ax_3d = fig.add_subplot(2, 3, 3, projection="3d")
ax_posicion = fig.add_subplot(2, 3, 4)
ax_cambio_rumbo = fig.add_subplot(2, 3, 5)
ax_info = fig.add_subplot(2, 3, 6)

ax_info.axis("off")


# =========================
# CONFIGURACIÓN DE GRÁFICAS
# =========================

line_ultima_trayectoria, = ax_trayectoria.plot(
    x_ultima,
    y_ultima,
    linestyle="--",
    linewidth=1.5,
    marker=".",
    markersize=3,
    label="Última trayectoria"
)

line_trayectoria, = ax_trayectoria.plot(
    [],
    [],
    marker="o",
    linewidth=2,
    label="Trayectoria actual"
)

punto_modulo, = ax_trayectoria.plot(
    [],
    [],
    marker="o",
    markersize=10,
    label="Rover actual"
)

line_rumbo, = ax_rumbo.plot([], [], marker="o", label="Rumbo corregido")

line_x, = ax_posicion.plot([], [], marker="o", label="X")
line_y, = ax_posicion.plot([], [], marker="o", label="Y")

line_cambio_rumbo, = ax_cambio_rumbo.plot([], [], marker="o", label="Cambio de rumbo")

ax_trayectoria.set_title("Trayectoria global 2D")
ax_trayectoria.set_xlabel("X global (m)")
ax_trayectoria.set_ylabel("Y global (m)")
ax_trayectoria.grid(True)
ax_trayectoria.axis("equal")
ax_trayectoria.legend()

ax_rumbo.set_title("Rumbo / Yaw corregido para gráfica")
ax_rumbo.set_xlabel("Tiempo real ESP-NOW (s)")
ax_rumbo.set_ylabel("Rumbo continuo (°)")
ax_rumbo.grid(True)
ax_rumbo.legend()

ax_posicion.set_title("Posición global X/Y")
ax_posicion.set_xlabel("Tiempo real ESP-NOW (s)")
ax_posicion.set_ylabel("Posición (m)")
ax_posicion.grid(True)
ax_posicion.legend()

ax_cambio_rumbo.set_title("Cambio de rumbo por muestra")
ax_cambio_rumbo.set_xlabel("Tiempo real ESP-NOW (s)")
ax_cambio_rumbo.set_ylabel("Cambio rumbo (°)")
ax_cambio_rumbo.grid(True)
ax_cambio_rumbo.legend()

texto_estado = ax_info.text(
    0.03,
    0.97,
    "ESTADO DEL MÓDULO\n\nEsperando datos del puerto serial...",
    fontsize=9,
    verticalalignment="top",
    bbox=dict(boxstyle="round", facecolor="white", edgecolor="black")
)

fig.subplots_adjust(
    left=0.05,
    right=0.97,
    top=0.93,
    bottom=0.08,
    wspace=0.35,
    hspace=0.38
)


# =========================
# LOOP PRINCIPAL EN TIEMPO REAL
# =========================

ser = None

try:
    print(f"Abriendo puerto serial {PUERTO_SERIAL} a {BAUDRATE} baudios...")
    ser = serial.Serial(PUERTO_SERIAL, BAUDRATE, timeout=TIMEOUT_SERIAL)
    ser.reset_input_buffer()
    print("Puerto serial abierto correctamente.")
    print("Esperando bloques tipo [ESP-NOW]...")

    while plt.fignum_exists(fig.number):
        paquetes = leer_paquetes_disponibles(ser)

        # Si llegaron varios paquetes en un mismo ciclo, procesamos todos para no atrasarnos.
        for paquete in paquetes:
            actualizar_gemelo_con_paquete(paquete)

        plt.pause(DELAY_VISUALIZACION)

except serial.SerialException as error:
    print("No se pudo abrir o leer el puerto serial.")
    print(error)
    print("Revisa que:")
    print("1. El puerto en PUERTO_SERIAL sea correcto.")
    print("2. El Serial Monitor de Arduino esté cerrado.")
    print("3. El baudrate del ESP sea 115200.")

except KeyboardInterrupt:
    print("Lectura detenida por el usuario.")

finally:
    if ser is not None and ser.is_open:
        ser.close()
        print("Puerto serial cerrado.")

    finalizar_registro()

    print("Gemelo digital cerrado.")
    plt.ioff()
    if plt.fignum_exists(fig.number):
        plt.show()
