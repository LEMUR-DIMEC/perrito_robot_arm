"""
CONTROLADOR DE MOTOR DYNAMIXEL CON MONITOREO EN TIEMPO REAL
Autor: Asistente AI
Descripción: Sistema completo para control de motor Dynamixel MX-64AT con agarre suave,
             control PID, detección de contacto y visualización de datos en tiempo real.
"""

import os
import time
import matplotlib.pyplot as plt
import numpy as np
from dynamixel_sdk import *  # Librería oficial Dynamixel SDK

# =============================================================================
# CONFIGURACIÓN DE PARÁMETROS DEL MOTOR DYNAMIXEL
# =============================================================================

# Parámetros de comunicación
PROTOCOL_VERSION = 1.0           # Versión del protocolo Dynamixel (1.0 para MX-64AT)
DXL_ID = 1                       # ID del motor a controlar
BAUDRATE = 57600                 # Velocidad de comunicación serial
DEVICENAME = 'COM5'              # Puerto serial (COM5 para Windows)

# Direcciones de memoria del motor MX-64AT (Protocolo 1.0)
ADDR_MX_TORQUE_ENABLE = 24       # Dirección para habilitar/deshabilitar torque
ADDR_MX_GOAL_POSITION = 30       # Dirección para escribir posición objetivo
ADDR_MX_PRESENT_POSITION = 36    # Dirección para leer posición actual
ADDR_MX_MOVING_SPEED = 32        # Dirección para establecer velocidad de movimiento
ADDR_MX_PRESENT_SPEED = 38       # Dirección para leer velocidad actual
ADDR_MX_TORQUE_LIMIT = 10        # Dirección para establecer límite de torque (seguridad)
ADDR_MX_PRESENT_LOAD = 40        # Dirección para leer carga actual (torque)
ADDR_MX_PRESENT_VOLTAGE = 42     # Dirección para leer voltaje actual

# =============================================================================
# PARÁMETROS DE CONFIGURACIÓN DEL PROYECTO
# =============================================================================

# Límites y valores operativos
DXL_MAX_LOAD_VALUE = 1023        # Valor máximo de carga del motor (0-1023)
SAFE_TORQUE_LIMIT = 500          # Límite máximo de torque para operación segura
GOAL_SPEED = 50                  # Velocidad de movimiento para agarre suave
GRIPPER_OPEN_POS = 100           # Posición inicial (pinza abierta)
GRIPPER_CLOSE_POS_OVERSHOT = 3000 # Posición de cierre exagerada para forzar detección
LOAD_THRESHOLD = 200             # Umbral de carga para detectar contacto (aprox. 10%)

# Parámetros del controlador PID
KP = 0.5                         # Ganancia proporcional - responde al error actual
KI = 0.02                        # Ganancia integral - elimina error acumulado
KD = 0.6                         # Ganancia derivativa - amortigua oscilaciones
INTEGRAL_MAX = 500               # Límite anti-windup para término integral
POSITION_ADJUST_MAX = 10         # Máximo ajuste de posición por ciclo PID

# =============================================================================
# INICIALIZACIÓN DE COMUNICACIÓN Y ESTRUCTURAS DE DATOS
# =============================================================================

# Inicializar manejadores de puerto y paquetes
portHandler = PortHandler(DEVICENAME)      # Maneja comunicación serial
packetHandler = PacketHandler(PROTOCOL_VERSION)  # Maneja protocolo Dynamixel

# Listas para almacenamiento de datos de monitoreo
time_data = []                    # Marcas de tiempo de cada medición
position_data = []               # Valores de posición del motor
current_data = []                # Valores de corriente (proxy mediante carga)
torque_data = []                 # Valores de torque aplicado
voltage_data = []                # Valores de voltaje del motor
events = []                      # Lista de eventos importantes (tiempo, descripción)

# =============================================================================
# FUNCIÓN: soft_grip - AGARRE SUAVE CON DETECCIÓN DE CONTACTO Y CONTROL PID
# =============================================================================

def soft_grip(load_threshold, goal_speed, overshot_pos):
    """
    Realiza el agarre suave de un objeto con detección de contacto por carga
    y mantiene la fuerza mediante control PID.
    
    Parámetros de entrada:
    - load_threshold (int): Umbral de carga para detectar contacto (0-1023)
    - goal_speed (int): Velocidad de movimiento del motor (0-1023)
    - overshot_pos (int): Posición de cierre exagerada para forzar detección
    
    Flujo de la función:
    1. Configura límites de seguridad y velocidad
    2. Inicia movimiento hacia posición de cierre
    3. Monitorea carga continuamente hasta detectar contacto
    4. Bloquea posición al detectar contacto
    5. Aplica control PID para mantener fuerza constante
    """
    
    # Acceso a variables globales de almacenamiento de datos
    global time_data, position_data, current_data, torque_data, voltage_data, events
    
    # Reinicializar listas de datos para nuevo ciclo
    time_data = []
    position_data = []
    current_data = []
    torque_data = []
    voltage_data = []
    events = []
    
    # Registrar tiempo de inicio y evento inicial
    start_time = time.time()
    events.append((0, "Inicio agarre"))
    
    # -------------------------------------------------------------------------
    # FASE 1: CONFIGURACIÓN DE SEGURIDAD Y MOVIMIENTO INICIAL
    # -------------------------------------------------------------------------
    
    # Establecer límite de torque para seguridad (evita daños por sobrecarga)
    packetHandler.write2ByteTxRx(portHandler, DXL_ID, ADDR_MX_TORQUE_LIMIT, SAFE_TORQUE_LIMIT)
    print(f"\nLímite de Torque (seguridad) establecido en: {SAFE_TORQUE_LIMIT}")

    # Establecer velocidad para movimiento de agarre suave
    packetHandler.write2ByteTxRx(portHandler, DXL_ID, ADDR_MX_MOVING_SPEED, goal_speed)

    # Enviar comando de posición de cierre exagerada
    dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(
        portHandler, DXL_ID, ADDR_MX_GOAL_POSITION, overshot_pos)
    
    # Verificar éxito del comando
    if dxl_comm_result != COMM_SUCCESS or dxl_error != 0:
        print("Error al enviar posición de agarre.")
        return

    print(f"Iniciando agarre suave hacia posición: {overshot_pos} (Velocidad: {goal_speed})")

    # -------------------------------------------------------------------------
    # FASE 2: DETECCIÓN DE CONTACTO POR MONITOREO DE CARGA
    # -------------------------------------------------------------------------
    
    start_time_phase1 = time.time()
    contact_detected = False
    contact_time = None
    
    # Bucle de monitoreo con timeout de seguridad (5 segundos)
    while (time.time() - start_time_phase1) < 5:
        
        # Calcular tiempo transcurrido desde inicio
        current_time = time.time() - start_time
        
        # Leer valor de carga actual del motor (Address 40)
        present_load_raw, dxl_comm_result, dxl_error = packetHandler.read2ByteTxRx(
            portHandler, DXL_ID, ADDR_MX_PRESENT_LOAD)

        # Verificar éxito de lectura
        if dxl_comm_result != COMM_SUCCESS or dxl_error != 0:
            print("Error leyendo carga. Deteniendo agarre.")
            break
            
        # Procesar valor de carga: el motor reporta 0-1023, donde 0-511 es un sentido
        # y 512-1023 es el opuesto. Calculamos magnitud absoluta.
        load_magnitude = present_load_raw
        if load_magnitude > 1023:
            load_magnitude = present_load_raw - 1024
        load_magnitude = abs(load_magnitude)

        # Leer posición actual del motor
        present_position, _, _ = packetHandler.read2ByteTxRx(
            portHandler, DXL_ID, ADDR_MX_PRESENT_POSITION)
        
        # Leer voltaje actual del motor (Address 42)
        present_voltage, _, _ = packetHandler.read1ByteTxRx(
            portHandler, DXL_ID, ADDR_MX_PRESENT_VOLTAGE)
        
        # Almacenar todos los datos en listas para posterior análisis
        time_data.append(current_time)
        position_data.append(present_position)
        current_data.append(load_magnitude)      # Usar carga como proxy de corriente
        torque_data.append(load_magnitude)       # Mismo valor para torque
        voltage_data.append(present_voltage * 0.1)  # Convertir a voltios (0.1V por unidad)
        
        # Verificar si se supera el umbral de carga (detección de contacto)
        if not contact_detected and load_magnitude >= load_threshold:
            print(f"\n--- CONTACTO DETECTADO ---")
            print(f"Carga actual ({load_magnitude}) >= Umbral ({load_threshold}). Deteniendo.")
            contact_detected = True
            contact_time = current_time
            events.append((current_time, "Contacto detectado"))
            break

        # Mostrar progreso en consola
        print(f"Posición: {present_position}, Carga: {load_magnitude}. Buscando contacto...", end='\r')
        time.sleep(0.02)  # Pequeño retardo para estabilidad

    # -------------------------------------------------------------------------
    # FASE 3: BLOQUEO DE POSICIÓN TRAS DETECCIÓN DE CONTACTO
    # -------------------------------------------------------------------------
    
    # Leer posición actual para bloquearla
    final_position_to_lock = packetHandler.read2ByteTxRx(
        portHandler, DXL_ID, ADDR_MX_PRESENT_POSITION)[0]
    
    # Establecer posición actual como nueva posición objetivo (bloqueo)
    packetHandler.write2ByteTxRx(
        portHandler, DXL_ID, ADDR_MX_GOAL_POSITION, final_position_to_lock)
    
    print(f"\nObjeto agarrado y posición bloqueada en: {final_position_to_lock}")
    events.append((time.time() - start_time, "Posición bloqueada"))

    # -------------------------------------------------------------------------
    # FASE 4: CONTROL PID PARA MANTENIMIENTO DE FUERZA CONSTANTE
    # -------------------------------------------------------------------------
    
    print("\n[MODO PID] Entrando en modo de mantenimiento de fuerza...")
    pid_start_time = time.time()
    events.append((pid_start_time - start_time, "Inicio PID"))
    
    # Inicialización de variables del controlador PID
    integral_error = 0           # Acumulador de error integral
    last_error = 0               # Error del ciclo anterior para cálculo derivativo
    current_goal_position = final_position_to_lock  # Posición actual como punto de partida
    
    # Parámetros de duración y seguimiento de valores extremos
    pid_duration_seconds = 10    # Tiempo total de operación del PID
    max_torque = 0               # Seguimiento de torque máximo
    min_voltage = float('inf')   # Seguimiento de voltaje mínimo
    
    # Bucle principal de control PID
    while (time.time() - pid_start_time) < pid_duration_seconds:
        current_time = time.time() - start_time
        time.sleep(0.05)  # Frecuencia de muestreo de 20Hz

        # Lectura de sensores para realimentación
        present_load_raw = packetHandler.read2ByteTxRx(
            portHandler, DXL_ID, ADDR_MX_PRESENT_LOAD)[0]
        load_magnitude = abs(present_load_raw)
        
        # Actualizar torque máximo registrado
        if load_magnitude > max_torque:
            max_torque = load_magnitude
        
        # Leer posición actual
        present_position, _, _ = packetHandler.read2ByteTxRx(
            portHandler, DXL_ID, ADDR_MX_PRESENT_POSITION)
        
        # Leer voltaje actual
        present_voltage, _, _ = packetHandler.read1ByteTxRx(
            portHandler, DXL_ID, ADDR_MX_PRESENT_VOLTAGE)
        
        # Convertir y almacenar voltaje
        voltage_value = present_voltage * 0.1
        if voltage_value < min_voltage:
            min_voltage = voltage_value
        
        # Almacenar datos para gráficos
        time_data.append(current_time)
        position_data.append(present_position)
        current_data.append(load_magnitude)
        torque_data.append(load_magnitude)
        voltage_data.append(voltage_value)
        
        # ---------------------------------------------------------------------
        # CÁLCULO DEL CONTROLADOR PID
        # ---------------------------------------------------------------------
        
        # Error = Setpoint (carga deseada) - Valor actual
        error = load_threshold - load_magnitude
        
        # Término Proporcional: respuesta inmediata al error actual
        p_term = KP * error
        
        # Término Integral: acumula error para eliminar error estacionario
        integral_error += error
        # Aplicar límite anti-windup
        integral_error = max(min(integral_error, INTEGRAL_MAX), -INTEGRAL_MAX)
        i_term = KI * integral_error
        
        # Término Derivativo: predice tendencia futura del error
        derivative_error = error - last_error
        d_term = KD * derivative_error
        
        # Calcular ajuste de posición total
        position_adjustment = int(p_term + i_term + d_term)
        # Limitar ajuste para evitar movimientos bruscos
        position_adjustment = max(min(position_adjustment, POSITION_ADJUST_MAX), -POSITION_ADJUST_MAX)

        # Aplicar ajuste a la posición objetivo
        current_goal_position += position_adjustment
        
        # Enviar nueva posición objetivo al motor
        packetHandler.write2ByteTxRx(
            portHandler, DXL_ID, ADDR_MX_GOAL_POSITION, current_goal_position)
        
        # Actualizar error para próximo ciclo
        last_error = error
        
        # Mostrar información de depuración
        print(f"Carga Actual: {load_magnitude}, Error: {error:.2f}, "
              f"Ajuste Pos: {position_adjustment}, Posición Final: {current_goal_position}", end='\r')

    # Registrar eventos finales del ciclo PID
    pid_end_time = time.time() - start_time
    events.append((pid_end_time, "Fin PID"))
    events.append((pid_end_time, f"Torque max: {max_torque}"))
    events.append((pid_end_time, f"Voltaje min: {min_voltage:.1f}V"))

    print(f"\n[FIN PID] Agarre estabilizado en posición {current_goal_position} "
          f"por {pid_duration_seconds} segundos.")

# =============================================================================
# FUNCIÓN: plot_results - GENERACIÓN DE GRÁFICOS DE ANÁLISIS
# =============================================================================

def plot_results():
    """
    Genera y muestra 4 gráficos con los datos recolectados durante la operación,
    incluyendo marcas de eventos clave y estadísticas importantes.
    
    Gráficos generados:
    1. Posición vs Tiempo
    2. Corriente (Carga) vs Tiempo  
    3. Torque vs Tiempo
    4. Voltaje vs Tiempo
    
    La función no recibe parámetros y utiliza las variables globales de datos.
    """
    
    # Verificar que existan datos para graficar
    if not time_data:
        print("No hay datos para graficar")
        return
        
    # Crear figura con 4 subgráficos (2x2)
    plt.figure(figsize=(16, 12))
    
    # -------------------------------------------------------------------------
    # GRÁFICO 1: POSICIÓN vs TIEMPO
    # -------------------------------------------------------------------------
    plt.subplot(2, 2, 1)
    plt.plot(time_data, position_data, 'b-', linewidth=2, label='Posición')
    plt.title('Posición del Motor vs Tiempo')
    plt.xlabel('Tiempo (s)')
    plt.ylabel('Posición')
    plt.grid(True, alpha=0.3)
    
    # -------------------------------------------------------------------------
    # GRÁFICO 2: CORRIENTE vs TIEMPO
    # -------------------------------------------------------------------------
    plt.subplot(2, 2, 2)
    plt.plot(time_data, current_data, 'r-', linewidth=2, label='Corriente')
    plt.title('Corriente (Carga) vs Tiempo')
    plt.xlabel('Tiempo (s)')
    plt.ylabel('Corriente (Carga)')
    plt.grid(True, alpha=0.3)
    
    # -------------------------------------------------------------------------
    # GRÁFICO 3: TORQUE vs TIEMPO
    # -------------------------------------------------------------------------
    plt.subplot(2, 2, 3)
    plt.plot(time_data, torque_data, 'g-', linewidth=2, label='Torque')
    plt.title('Torque vs Tiempo')
    plt.xlabel('Tiempo (s)')
    plt.ylabel('Torque')
    plt.grid(True, alpha=0.3)
    
    # -------------------------------------------------------------------------
    # GRÁFICO 4: VOLTAJE vs TIEMPO
    # -------------------------------------------------------------------------
    plt.subplot(2, 2, 4)
    plt.plot(time_data, voltage_data, 'm-', linewidth=2, label='Voltaje')
    plt.title('Voltaje vs Tiempo')
    plt.xlabel('Tiempo (s)')
    plt.ylabel('Voltaje (V)')
    plt.grid(True, alpha=0.3)
    
    # -------------------------------------------------------------------------
    # MARCADO DE EVENTOS CLAVE EN TODOS LOS GRÁFICOS
    # -------------------------------------------------------------------------
    
    # Paleta de colores para diferentes eventos
    colors = ['red', 'blue', 'green', 'orange', 'purple', 'brown', 'pink', 'gray']
    
    # Añadir líneas verticales para cada evento registrado
    for i, (event_time, event_desc) in enumerate(events):
        color = colors[i % len(colors)]  # Ciclar colores si hay muchos eventos
        
        # Añadir línea vertical en los 4 subgráficos
        for subplot_num in range(1, 5):
            plt.subplot(2, 2, subplot_num)
            plt.axvline(x=event_time, color=color, linestyle='--', alpha=0.7, linewidth=1.5)
            
            # Añadir etiqueta descriptiva solo en el primer gráfico
            if subplot_num == 1:
                plt.text(event_time, plt.ylim()[1] * 0.95, event_desc, 
                        rotation=90, verticalalignment='top', fontsize=8,
                        bbox=dict(boxstyle="round,pad=0.3", facecolor=color, alpha=0.2))
    
    # -------------------------------------------------------------------------
    # PANEL DE ESTADÍSTICAS
    # -------------------------------------------------------------------------
    
    # Calcular estadísticas importantes
    max_position = max(position_data) if position_data else 0
    max_torque = max(torque_data) if torque_data else 0
    min_voltage = min(voltage_data) if voltage_data else 0
    total_time = time_data[-1] if time_data else 0
    
    # Añadir panel informativo en la parte inferior
    plt.figtext(0.02, 0.02, 
               f"Estadísticas clave:\n"
               f"- Posición máxima: {max_position}\n"
               f"- Torque máximo: {max_torque}\n"
               f"- Voltaje mínimo: {min_voltage:.1f}V\n"
               f"- Tiempo total: {total_time:.1f}s",
               bbox=dict(boxstyle="round,pad=0.5", facecolor="lightgray", alpha=0.7),
               fontsize=9)
    
    # Ajustar layout y mostrar gráficos
    plt.tight_layout(rect=[0, 0.05, 1, 0.95])
    plt.show()

# =============================================================================
# PROGRAMA PRINCIPAL - FLUJO DE EJECUCIÓN
# =============================================================================

# Bloque try-finally para garantizar cierre seguro del puerto
try:
    # Registrar tiempo de inicio del programa
    program_start_time = time.time()
    
    # -------------------------------------------------------------------------
    # PASO 1: INICIALIZACIÓN DE COMUNICACIÓN CON EL MOTOR
    # -------------------------------------------------------------------------
    
    # Abrir puerto serial
    if portHandler.openPort():
        print("Puerto abierto correctamente")
    else:
        print("No se pudo abrir el puerto")
        quit()

    # Configurar velocidad de comunicación
    if portHandler.setBaudRate(BAUDRATE):
        print("Baudrate configurado correctamente")
    else:
        print("No se pudo configurar el baudrate")
        quit()

    # Habilitar torque del motor
    dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(
        portHandler, DXL_ID, ADDR_MX_TORQUE_ENABLE, 1)
    
    # Verificar éxito de habilitación de torque
    if dxl_comm_result != COMM_SUCCESS or dxl_error != 0:
        print("Error al habilitar torque.")
    else:
        print("Torque habilitado")

    # -------------------------------------------------------------------------
    # PASO 2: APERTURA INICIAL DE LA PINZA
    # -------------------------------------------------------------------------
    
    print("\n[PASO 1] Abriendo pinza a posición inicial...")
    dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(
        portHandler, DXL_ID, ADDR_MX_GOAL_POSITION, GRIPPER_OPEN_POS)
    time.sleep(2)  # Esperar a que complete el movimiento

    # -------------------------------------------------------------------------
    # PASO 3: EJECUCIÓN DEL AGARRE SUAVE CON CONTROL PID
    # -------------------------------------------------------------------------
    
    print("\n[PASO 2] Ejecutando función de agarre suave...")
    soft_grip(LOAD_THRESHOLD, GOAL_SPEED, GRIPPER_CLOSE_POS_OVERSHOT)

    # -------------------------------------------------------------------------
    # PASO 4: ESPERA DE COMANDO DEL USUARIO
    # -------------------------------------------------------------------------
    
    # Registrar evento de presión de botón
    button_press_time = time.time() - program_start_time
    events.append((button_press_time, "Usuario presiona 0"))
    
    # Esperar input del usuario para soltar
    tiempo = input("Apretar 0 para soltar...")
    time.sleep(int(tiempo))

    # -------------------------------------------------------------------------
    # PASO 5: LIBERACIÓN DEL OBJETO
    # -------------------------------------------------------------------------
    
    print("\n[PASO 3] Soltando objeto...")
    
    # Registrar evento de liberación
    release_time = time.time() - program_start_time
    events.append((release_time, "Soltar objeto"))
    
    # Enviar comando para abrir pinza
    dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(
        portHandler, DXL_ID, ADDR_MX_GOAL_POSITION, GRIPPER_OPEN_POS)
    time.sleep(2)  # Esperar a que complete el movimiento

# =============================================================================
# BLOQUE FINALLY: LIMPIEZA Y CIERRE SEGURO
# =============================================================================

finally:
    """
    Este bloque se ejecuta siempre, garantizando el cierre seguro del puerto
    independientemente de si el programa termina normalmente o con error.
    """
    
    # Deshabilitar torque del motor (importante para seguridad)
    packetHandler.write1ByteTxRx(portHandler, DXL_ID, ADDR_MX_TORQUE_ENABLE, 0)
    
    # Cerrar puerto serial
    portHandler.closePort()
    print("\nPuerto cerrado.")
    
    # Generar y mostrar gráficos con los datos recolectados
    print("Generando gráficos...")
    plot_results()
    print("Fin del script.")

# =============================================================================
# FIN DEL CÓDIGO
# =============================================================================