from machine import ADC, Pin
import time

# Configurar el pin ADC para la medición de voltaje
adc = ADC(Pin(34))
adc.atten(ADC.ATTN_11DB)  # Configurar la atenuación a 11 dB para el rango completo (0-3.3V)

while True:
    voltaje = adc.read() / 4095 * 3.3  # Convertir la lectura del ADC a voltaje
    print("Voltaje:", voltaje)
    time.sleep_ms(1000)  # Dormir durante 1000 milisegundos (1 segundo)