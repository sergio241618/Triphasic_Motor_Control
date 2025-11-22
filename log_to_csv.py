#!/usr/bin/env python3
"""
Script para convertir logs de la ESP32 a archivo CSV para modelado de planta
Uso: python3 log_to_csv.py <archivo_log.txt> <salida.csv>
O capturar directamente desde el monitor serial
"""

import re
import sys
import argparse

def extract_csv_from_log(log_file, output_csv):
    """
    Extrae datos CSV del archivo de log de la ESP32
    Busca líneas con formato: Time(s),Frequency(Hz),RPM
    Maneja códigos ANSI y caracteres de escape
    """
    # Regex más robusto que ignora caracteres ANSI al inicio
    csv_pattern = re.compile(r'.*?(\d+\.\d+),(\d+\.\d+),(\d+\.\d+)\s*$')
    
    csv_lines = []
    header_written = False
    
    with open(log_file, 'r', encoding='utf-8', errors='ignore') as f:
        for line in f:
            # Remover códigos ANSI (escape sequences)
            # Patrón: \x1b[...m o caracteres no imprimibles
            clean_line = re.sub(r'\x1b\[[0-9;]*m', '', line)
            clean_line = re.sub(r'[\x00-\x1f\x7f-\x9f]', '', clean_line)
            clean_line = clean_line.strip()
            
            if not clean_line:
                continue
            
            # Buscar líneas de datos CSV
            match = csv_pattern.match(clean_line)
            if match:
                time_val, freq_val, rpm_val = match.groups()
                csv_line = f"{time_val},{freq_val},{rpm_val}"
                
                # Escribir header si es la primera línea
                if not header_written:
                    csv_lines.append('Time(s),Frequency(Hz),RPM')
                    header_written = True
                
                csv_lines.append(csv_line)
    
    if not csv_lines:
        print("❌ No se encontraron datos CSV en el archivo de log")
        print("Asegúrate de que PLANT_MODELING esté en 1 y que el ESP32 esté enviando datos")
        return False
    
    # Escribir archivo CSV
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
