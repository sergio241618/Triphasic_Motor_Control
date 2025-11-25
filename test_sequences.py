import serial
import time
import sys
from mcu_comms import connect_to_esp32, send_command

# --- Sequence Configuration ---
# CH1 = Frequency (Hz)
SEQ_CH1 = "50,100,150,200,250,200,150,100,50,50,100,150,200,250,200,150,100,50"

# CH2 = Amplitude (0-100)
SEQ_CH2 = "100,80,60,40,20,0,20,40,60,80,100"

# CH3 = RPM Reference (RPM)
SEQ_CH3 = "1000,2000,3000,1500"

# Wait time (in seconds) between sending each complete sequence
WAIT_BETWEEN_SEQUENCES = 5 

def run_automatic_test(port=None):
    ser = connect_to_esp32(port)
    if not ser:
        return

    print("\n--- Iniciando Prueba Automática (Frec/Amp/RPM) ---")
    print("Presiona Ctrl+C para detener.")

    # --- (NON-BLOCKING) READ FUNCTION ---
    def read_from_esp(duration_s):
        start_time = time.time()
        end_time = start_time + duration_s
        print(f"[Monitoreando ESP32 por {duration_s}s... (Debug Habilitado)]")
        
        line_buffer = b''
        
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
                
                print(".", end='', flush=True)
                time.sleep(0.1) 
        
        except serial.SerialException as e:
            print(f"Error leyendo del puerto: {e}")
            raise
        
        print("\n[Monitoreo terminado]") 

    try:
        # --- Main loop ---
        while True:
            print("\n--- Enviando Secuencia de FRECUENCIA (CH1) ---")
            send_command(ser, "CH1", SEQ_CH1)
            read_from_esp(WAIT_BETWEEN_SEQUENCES)

            print("\n--- Enviando Secuencia de AMPLITUD (CH2) ---")
            send_command(ser, "CH2", SEQ_CH2)
            read_from_esp(WAIT_BETWEEN_SEQUENCES)
            
            print("\n--- Enviando Secuencia de RPM REF (CH3) ---")
            send_command(ser, "CH3", SEQ_CH3)
            read_from_esp(WAIT_BETWEEN_SEQUENCES)


    except KeyboardInterrupt:
        print("\nPrueba detenida por el usuario.")
    except serial.SerialException:
        print("\nError: Se perdió la conexión con el ESP32.")
    finally:
        if ser and ser.is_open:
            ser.close()
            print("Puerto serie cerrado.")

if __name__ == "__main__":
    # You can pass the port as an argument, e.g.: python test_sequences.py /dev/ttyUSB1
    if len(sys.argv) > 1:
        run_automatic_test(port=sys.argv[1])
    else:
        run_automatic_test()