from machine import Pin
from machine import ADC
from time import ticks_ms
from machine import deepsleep

# Configura el pin del sensor y la resolución del ADC
sensor_pin = Pin(34)
adc = ADC(sensor_pin)
adc.atten(ADC.ATTN_11DB)

# Configura el pin del botón
button_pin = Pin(0, Pin.IN)

# Inicializa las variables para el cálculo del promedio
sum_of_values = 0
count = 0

# Define la ventana de tiempo (en milisegundos)
ventana_tiempo = 200

# Obtiene el tiempo actual en milisegundos
start_time = ticks_ms()

while True:
    # Obtiene el valor del sensor
    sensor_value = adc.read()
    voltage = 3.3 * sensor_value / 4095

    # Acumula los valores
    sum_of_values += voltage
    count += 1

    # Verifica si ha pasado la ventana de tiempo
    current_time = ticks_ms()
    if current_time - start_time >= ventana_tiempo:
        # Calcula el promedio
        promedio = sum_of_values / count

        # Imprime el voltaje promedio
        print('Voltaje promedio:', promedio)

        # Restablece las variables para el siguiente ciclo
        sum_of_values = 0
        count = 0
        start_time = current_time
        break

    # Comprueba si se ha presionado el botón para entrar en el modo de sueño profundo
    if button_pin.value() == 1:
        deepsleep(1000)
