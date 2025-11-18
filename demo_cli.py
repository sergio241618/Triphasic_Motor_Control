import serial
import time
import sys
from mcu_comms import connect_to_esp32, send_command

def read_esp32_logs(ser, duration_s):
    """
    Reads and displays logs from the ESP32 for 'duration_s' seconds.
    Uses the same non-blocking logic as test_sequences.py.
    """
    start_time = time.time()
    end_time = start_time + duration_s
    line_buffer = b'' # Buffer for partial data
    
    try:
        while time.time() < end_time:
            bytes_waiting = ser.in_waiting
            if bytes_waiting > 0:
                data = ser.read(bytes_waiting)
                line_buffer += data
            
            while b'\n' in line_buffer:
                newline_pos = line_buffer.find(b'\n')
                line = line_buffer[:newline_pos]
                line_buffer = line_buffer[newline_pos+1:]
                
                if line: 
                    print(f"ESP32 -> {line.decode('utf-8', 'ignore').strip()}")
            
            time.sleep(0.05) 
    
    except serial.SerialException as e:
        print(f"Error leyendo del puerto: {e}")
        raise # Raise the exception so the main loop stops

def run_interactive_demo(port=None):
    ser = connect_to_esp32(port)
    if not ser:
        return

    print("\n--- Demo Interactiva (Control Frec/Amp/RPM) ---")
    
    try:
        while True:
            # 1. Show menu
            print("\nElige un canal para controlar:")
            print("  1. CH1 (Frecuencia)")
            print("  2. CH2 (Amplitud)")
            print("  3. CH3 (RPM de Referencia)")
            print("  q. Salir")
            
            choice = input("Opción: ").strip()

            # 2. Process choice
            if choice == 'q' or choice == 'Q':
                print("Saliendo...")
                break
            
            if choice == '1':
                channel = "CH1"
                prompt_label = "Frecuencia (Hz)"
                prompt_ex = "Ej: 60 (para 60 Hz) o 10,20,30"
            elif choice == '2':
                channel = "CH2"
                prompt_label = "Amplitud (0-100)"
                prompt_ex = "Ej: 100 (para 100%) o 100,50,0"
            elif choice == '3':
                channel = "CH3"
                prompt_label = "RPM de Referencia"
                prompt_ex = "Ej: 3000 (para 3000 RPM)"
            else:
                print(f"'{choice}' no es una opción válida.")
                continue

            # 3. Request values
            print(f"\nCanal seleccionado: {channel} ({prompt_label})")
            print(prompt_ex)
            values_str = input(f"Valores para {channel}: ").strip()

            if not values_str:
                print("No se ingresaron valores. Cancelando.")
                continue

            # 4. Send command
            send_command(ser, channel, values_str)

            # 5. Listen for the ESP32's response for 0.5 seconds
            read_esp32_logs(ser, 0.5)
            # If a long sequence was sent, listen for longer
            if ',' in values_str:
                read_esp32_logs(ser, T_PULSE_MS * values_str.count(',') / 1000.0)


    except KeyboardInterrupt:
        print("\nSaliendo por Ctrl+C.")
    except serial.SerialException:
        print("\nError: Se perdió la conexión con el ESP32.")
    finally:
        if ser and ser.is_open:
            ser.close()
            print("Puerto serie cerrado.")

if __name__ == "__main__":
    # Get T_PULSE_MS from the ESP32 code (for the interactive demo)
    # This is a bit 'hacky' but avoids defining it in two places
    T_PULSE_MS = 250 
    
    if len(sys.argv) > 1:
        run_interactive_demo(port=sys.argv[1])
    else:
        run_interactive_demo()