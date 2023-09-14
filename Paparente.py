from machine import Pin, ADC, deepsleep
from time import ticks_ms
import math

# Definición de funciones
def calcular_offset(sensor_adc, tiempo_ventana):
    sumatoria_valores = 0
    cantidad_muestras = 0
    tiempo_inicio = ticks_ms()

    while True:
        valor_sensor = sensor_adc.read()
        valor_voltaje = 3.3 * valor_sensor / 4095
        sumatoria_valores += valor_voltaje
        cantidad_muestras += 1

        tiempo_actual = ticks_ms()
        if tiempo_actual - tiempo_inicio >= tiempo_ventana:
            promedio = sumatoria_valores / cantidad_muestras
            return promedio

def calcular_potencia_aparente(sensor_adc, promedio_offset, sensibilidad, tiempo_ventana):
    tiempo_inicio = ticks_ms()
    sumatoria_diferencias_cuadrado = 0
    cantidad_muestras = 0

    while True:
        valor_sensor = sensor_adc.read()
        valor_voltaje = 3.3 * valor_sensor / 4095
        diferencia = valor_voltaje - promedio_offset
        sumatoria_diferencias_cuadrado += diferencia ** 2
        cantidad_muestras += 1

        tiempo_actual = ticks_ms()
        if tiempo_actual - tiempo_inicio >= tiempo_ventana:
            corriente_rms = (1 / sensibilidad) * (sumatoria_diferencias_cuadrado / cantidad_muestras) ** 0.5
            voltaje_rms = 120
            potencia_aparente = corriente_rms * voltaje_rms
            return potencia_aparente

# Configuración de pines
pin_sensor = Pin(34)
adc_sensor = ADC(pin_sensor)
adc_sensor.atten(ADC.ATTN_11DB)

pin_boton = Pin(0, Pin.IN)

# Definición de ventana de tiempo (en milisegundos)
ventana_tiempo = 200

# Datos del sensor
sensibilidad_sensor = 0.185

# Cálculo del offset
offset_promedio = calcular_offset(adc_sensor, ventana_tiempo)
print("El voltaje promedio es:", offset_promedio, "V")

# Cálculo de la potencia aparente
potencia_aparente_calculada = calcular_potencia_aparente(adc_sensor, offset_promedio, sensibilidad_sensor, ventana_tiempo)
print("La Potencia Aparente es:", potencia_aparente_calculada, "VA")

# Comprobación si se presiona el botón para entrar en el modo de sueño profundo
if pin_boton.value() == 1:
    deepsleep(1000)
