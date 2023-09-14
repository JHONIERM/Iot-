from machine import Pin
from machine import ADC
from time import sleep
from machine import deepsleep

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

# Verifica si se ha presionado el botón para entrar en el modo de sueño profundo
if button_pin.value() == 1:
    deepsleep(1000)
