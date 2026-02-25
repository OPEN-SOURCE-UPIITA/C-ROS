import serial
import curses
import time

PUERTO = '/dev/ttyACM0'
BAUDIOS = 115200

try:
    ser = serial.Serial(PUERTO, BAUDIOS, timeout=0.1)
except Exception as e:
    print(f"Error abriendo el puerto {PUERTO}: {e}")
    exit()

def send_command(dir_dc, speed_dc, dir_servo):
    packet = bytearray([0xAA, 0x55, 0x01, dir_dc, speed_dc, dir_servo, 0xFF])
    ser.write(packet)

def main(stdscr):
    stdscr.nodelay(1) 
    curses.noecho()   
    
    # Estados Iniciales Persistentes (El coche empieza quieto y centrado)
    dir_val = 0
    speed_val = 0
    servo_val = 0 

    while True:
        stdscr.clear()
        stdscr.addstr(0, 0, "=== Control Motores 1.0 ===")
        stdscr.addstr(1, 0, "W: Avanzar | S: Reversa | ESPACIO: Frenar")
        stdscr.addstr(2, 0, "A: Izquierda | D: Derecha | C: Centrar Dirección")
        stdscr.addstr(3, 0, "Presiona 'Q' para salir.")
        
        # Mostrar el estado actual
        stdscr.addstr(5, 0, f"Motor DC: {dir_val}  |  Servo: {servo_val}")

        key = stdscr.getch()

        if key != -1:
            # Control de Tracción
            if key == ord('s') or key == ord('S'):
                dir_val = 1
                speed_val = 100
            elif key == ord('w') or key == ord('W'):
                dir_val = 2
                speed_val = 100
            elif key == ord(' '): # Barra espaciadora para frenar
                dir_val = 0
                speed_val = 0
                
            # Control de Dirección
            elif key == ord('d') or key == ord('D'):
                servo_val = 1
            elif key == ord('a') or key == ord('A'):
                servo_val = 2
            elif key == ord('c') or key == ord('C'):
                servo_val = 0
                
            # Salir
            elif key == ord('q') or key == ord('Q'):
                break

        # Enviar el estado SIEMPRE, para mantener el Watchdog del STM32 feliz
        send_command(dir_val, speed_val, servo_val)
        
        stdscr.refresh()
        time.sleep(0.05) # Loop a 20Hz (Súper estable)

# Iniciar el programa
curses.wrapper(main)
send_command(0, 0, 0) # Frenar todo al salir
ser.close()
