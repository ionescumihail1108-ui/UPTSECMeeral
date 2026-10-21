import serial
import time
import math

# --- CONFIGURARE PORT SERIAL ---
# Încearcă '/dev/ttyAMA0' sau '/dev/serial0'
SERIAL_PORT = '/dev/ttyAMA0' 
BAUD_RATE = 115200

def test_lidar():
    print("--------------------------------------------------")
    print(f" Începere testare LiDAR pe portul: {SERIAL_PORT}")
    print(" Se încearcă deschiderea conexiunii seriale...")
    print("--------------------------------------------------\n")

    try:
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
        print("✅ Portul serial a fost deschis cu succes!")
        print("Așteptăm date de la LiDAR... (Apasă Ctrl+C pentru a opri)\n")
    except Exception as e:
        print(f"❌ EROARE la deschiderea portului serial: {e}")
        print("\nSfat: Rulează comanda folosind 'sudo python3 test_lidar.py'")
        return

    buffer = bytearray()
    last_print_time = time.time()
    read_count = 0
    valid_points = []

    try:
        while True:
            # Check dacă sunt octeți disponibili pe serial
            if ser.in_waiting > 0:
                raw_bytes = ser.read(ser.in_waiting)
                buffer.extend(raw_bytes)
                read_count += len(raw_bytes)

                # Parsare pachete M1C1 (Header: 0xAA 0x55)
                while len(buffer) >= 10:
                    if buffer[0] == 0xAA and buffer[1] == 0x55:
                        sample_count = buffer[3]
                        pkt_len = 10 + sample_count * 2

                        if len(buffer) < pkt_len:
                            break # Așteptăm tot pachetul

                        packet = buffer[:pkt_len]
                        buffer = buffer[pkt_len:]

                        # Extrage unghiurile și distanțele
                        start_angle = ((packet[4] | (packet[5] << 8)) >> 1) / 64.0
                        end_angle = ((packet[6] | (packet[7] << 8)) >> 1) / 64.0
                        
                        diff = end_angle - start_angle
                        if diff < 0:
                            diff += 360.0
                        step = diff / (sample_count - 1) if sample_count > 1 else 0

                        for i in range(sample_count):
                            idx = 10 + i * 2
                            dist_mm = (packet[idx] | (packet[idx+1] << 8)) / 4.0
                            angle_deg = (start_angle + step * i) % 360.0

                            if dist_mm > 0:
                                valid_points.append((round(angle_deg, 1), round(dist_mm, 1)))
                    else:
                        buffer.pop(0)

            # Afișare stare o dată la o secundă (1.0 sec)
            current_time = time.time()
            if current_time - last_print_time >= 1.0:
                if read_count == 0:
                    print("⚠️ [STATUS]: Nu se primesc date! (0 octeți cititi). Verifică alimentarea și conexiunea TX/RX.")
                else:
                    total_points = len(valid_points)
                    print(f"✅ [STATUS OK]: S-au primit {read_count} octeți brute.")
                    print(f"   Puncte valide citite în ultima secundă: {total_points}")
                    
                    if total_points > 0:
                        # Afișăm câteva mostre de citiri
                        sample_str = ", ".join([f"{a}°: {d}mm" for a, d in valid_points[:5]])
                        print(f"   Mostre date (Unghi: Distanță): {sample_str} ...")
                    else:
                        print("   ⚠️ Pachete găsite, dar nicio distanță validă filtrată.")

                print("-" * 50)
                
                # Resetăm contoarele pentru următoarea secundă
                read_count = 0
                valid_points.clear()
                last_print_time = current_time

            time.sleep(0.01)

    except KeyboardInterrupt:
        print("\nTest oprit de utilizator.")
    finally:
        if ser.is_open:
            ser.close()
            print("Port serial închis.")

if __name__ == '__main__':
    test_lidar()