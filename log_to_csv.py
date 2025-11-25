import re
import sys
import argparse

def extract_csv_from_log(log_file, output_csv):
    """
    Extracts CSV data from ESP32 log file
    Searches for lines with format: Time(s),Frequency(Hz),RPM
    Handles ANSI codes and escape characters
    """
    # More robust regex that ignores ANSI characters at the beginning
    csv_pattern = re.compile(r'.*?(\d+\.\d+),(\d+\.\d+),(\d+\.\d+)\s*$')
    
    csv_lines = []
    header_written = False
    
    with open(log_file, 'r', encoding='utf-8', errors='ignore') as f:
        for line in f:
            # Remove ANSI codes (escape sequences)
            # Pattern: \x1b[...m or non-printable characters
            clean_line = re.sub(r'\x1b\[[0-9;]*m', '', line)
            clean_line = re.sub(r'[\x00-\x1f\x7f-\x9f]', '', clean_line)
            clean_line = clean_line.strip()
            
            if not clean_line:
                continue
            
            # Search for CSV data lines
            match = csv_pattern.match(clean_line)
            if match:
                time_val, freq_val, rpm_val = match.groups()
                csv_line = f"{time_val},{freq_val},{rpm_val}"
                
                # Write header if it's the first line
                if not header_written:
                    csv_lines.append('Time(s),Frequency(Hz),RPM')
                    header_written = True
                
                csv_lines.append(csv_line)
    
    if not csv_lines:
        print("❌ No se encontraron datos CSV en el archivo de log")
        print("Asegúrate de que PLANT_MODELING esté en 1 y que el ESP32 esté enviando datos")
        return False
    
    # Write CSV file
    with open(output_csv, 'w') as f:
        for line in csv_lines:
            f.write(line + '\n')
    
    print(f"✅ Datos extraídos exitosamente:")
    print(f"   - Líneas de datos: {len(csv_lines) - 1}")
    print(f"   - Archivo guardado: {output_csv}")
    
    return True

def main():
    parser = argparse.ArgumentParser(
        description='Convertir logs de ESP32 a CSV para modelado de planta',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Ejemplos de uso:
  # Convertir archivo de log existente
  python3 log_to_csv.py esp32_log.txt plant_data.csv
  
  # Capturar desde monitor serial y guardar
  idf.py monitor | tee esp32_log.txt
  # Luego presiona Ctrl+C y ejecuta:
  python3 log_to_csv.py esp32_log.txt plant_data.csv
        """
    )
    
    parser.add_argument('log_file', help='Archivo de log de la ESP32')
    parser.add_argument('output_csv', help='Archivo CSV de salida')
    
    args = parser.parse_args()
    
    print(f"📊 Procesando log: {args.log_file}")
    print(f"📁 Salida CSV: {args.output_csv}")
    print("-" * 50)
    
    success = extract_csv_from_log(args.log_file, args.output_csv)
    
    if success:
        print("\n🎉 ¡Conversión completada!")
        print(f"\nPuedes importar el archivo '{args.output_csv}' en MATLAB/Python para modelar la planta")
    else:
        sys.exit(1)

if __name__ == '__main__':
    main()
