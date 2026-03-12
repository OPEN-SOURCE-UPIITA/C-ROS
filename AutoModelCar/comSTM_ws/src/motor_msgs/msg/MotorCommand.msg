# Comando para motores DC y servo
# dir_dc: 0=Stop, 1=Adelante, 2=Atrás
# speed_dc: 0-100
# dir_servo: 1110-1740 (1500=Centro)

int8 dir_dc
int8 speed_dc
int16 dir_servo

# Comando para Luces
# stop_lights: 0=Apagado, 1=Encendido
# turn_signals: 0=Apagado, 1=Derecha, 2=Izquierda, 3=Intermitentes (Ambas)
int8 stop_lights
int8 turn_signals
