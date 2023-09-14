from machine import Pin, ADC, deepsleep, lightsleep
from time import sleep

# Configura el pin del sensor y la resolución del ADC
sensor_pin = Pin(34)
adc = ADC(sensor_pin)
adc.atten(ADC.ATTN_11DB)

# Configura el pin del botón
button_pin = Pin(0, Pin.IN)

# Lee el valor del sensor
sensor_value = adc.read()
sensor_voltage = 3.3 * sensor_value / 4095

# Imprime el valor del sensor
print('Raw value:', sensor_value, 'and voltage:', sensor_voltage)

# Verifica si se ha presionado el botón
if button_pin.value() == 1:
    # Si se presiona el botón, entra en modo de sueño profundo durante 1000 ms (1 segundo)
    deepsleep(1000)
else:
    # Si no se presiona el botón, entra en modo de sueño ligero
    lightsleep()
