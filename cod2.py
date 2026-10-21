import os
import sys

# --- FIX PENTRU EROAREA WAYLAND / QWINDOW ---
# Forțăm Matplotlib să folosească backend-ul TkAgg pentru a evita problemele de Qt/Wayland
os.environ["QT_QPA_PLATFORM"] = "xcb" 
import matplotlib
matplotlib.use('TkAgg')

import serial
import math
import time
import threading
import matplotlib.pyplot as plt

# --- CONFIGURARE PORT SERIAL ---
SERIAL_PORT = '/dev/ttyAMA0' # Sau '/dev/serial0'
BAUD_RATE = 115200

# --- DICIȚIONAR CODURI DE EROARE ---
ERROR_CODES = {
    "ERR_01": "[ERR_01] Nu s-a putut deschide portul serial. Verifică drepturile (sudo) sau calea /dev/ttyAMA0.",
    "ERR_02": "[ERR_02] Pierdere conexiune serială / Time-out la citirea datelor.",
    "ERR_03": "[ERR_03] Header incorect de pachet (Așteptat: 0xAA 0x55). Verifica legătura pinului RX/TX.",
    "ERR_04": "[ERR_04] Eroare de Checksum sau pachet corupt de la LiDAR.",
    "ERR_05": "[ERR_05] Eroare necunoscută la parsare pachete."
}

class M1C1Lidar:
    def __init__(self, port, baudrate):
        self.port = port
        self.baudrate = baudrate
        self.ser = None
        self.is_running = False
        self.angles_rad = []     # Unghiuri în RADIANI (pt. matplotlib)
        self.distances_mm = []   # Distanțe în MILIMETRI (mm)
        self.lock = threading.Lock()
        self.thread = None

    def connect(self):
        try:
            self.ser = serial.Serial(self.port, self.baudrate, timeout=1)
            print(f"[STATUS] Conectat pe {self.port} la {self.baudrate} baud.")
            return True
        except serial.SerialException as e:
            print(f"\n❌ {ERROR_CODES['ERR_01']}")
            print(f"   Detaliu tehnic: {e}\n")
            return False

    def start(self):
        if not self.is_running:
            if self.ser is None or not self.ser.is_open:
                if not self.connect():
                    return
            self.is_running = True
            self.thread = threading.Thread(target=self._read_loop, daemon=True)
            self.thread.start()
            print("[STATUS] SCANARE PORNITĂ (Tasta L)")

    def stop(self):
        if self.is_running:
            self.is_running = False
            if self.thread and self.thread.is_alive():
                self.thread.join(timeout=1.0)
            print("[STATUS] SCANARE OPRITĂ (Tasta P)")

    def _read_loop(self):
        buffer = bytearray()
        consecutive_errors = 0
        
        while self.is_running and self.ser and self.ser.is_open:
            try:
                if self.ser.in_waiting > 0:
                    buffer.extend(self.ser.read(self.ser.in_waiting))
                    consecutive_errors = 0 # Resetăm contorul de erori
                    
                    while len(buffer) >= 10:
                        # Verificare antet pachet M1C1
                        if buffer[0] == 0xAA and buffer[1] == 0x55:
                            sample_count = buffer[3]
                            pkt_len = 10 + sample_count * 2
                            
                            if len(buffer) < pkt_len:
                                break # Așteptăm mai mulți octeți
                            
                            packet = buffer[:pkt_len]
                            buffer = buffer[pkt_len:]
                            
                            self._parse_packet(packet, sample_count)
                        else:
                            buffer.pop(0)
                else:
                    time.sleep(0.005)
            except Exception as e:
                consecutive_errors += 1
                if consecutive_errors > 10:
                    print(f"\n❌ {ERROR_CODES['ERR_02']}")
                    print(f"   Detaliu tehnic: {e}\n")
                    self.stop()
                    break

    def _parse_packet(self, packet, sample_count):
        try:
            start_angle = ((packet[4] | (packet[5] << 8)) >> 1) / 64.0
            end_angle = ((packet[6] | (packet[7] << 8)) >> 1) / 64.0
            
            diff = end_angle - start_angle
            if diff < 0:
                diff += 360.0
                
            step = diff / (sample_count - 1) if sample_count > 1 else 0
            
            current_angles = []
            current_distances = []
            
            for i in range(sample_count):
                idx = 10 + i * 2
                # Unitatea de măsură returnată de LiDAR: MILIMETRI (mm)
                dist_mm = (packet[idx] | (packet[idx+1] << 8)) / 4.0 
                angle_deg = (start_angle + step * i) % 360.0
                
                # Filtrare valori invalide/zgomot
                if dist_mm > 10 and dist_mm < 6000: 
                    current_angles.append(math.radians(angle_deg))
                    current_distances.append(dist_mm)

            with self.lock:
                self.angles_rad.extend(current_angles)
                self.distances_mm.extend(current_distances)
                
                if len(self.angles_rad) > 600:
                    self.angles_rad = self.angles_rad[-500:]
                    self.distances_mm = self.distances_mm[-500:]
        except Exception:
            print(f"⚠️ {ERROR_CODES['ERR_05']}")

    def get_data(self):
        with self.lock:
            return list(self.angles_rad), list(self.distances_mm)

# --- INTERFAȚĂ GRAFICĂ ---
def main():
    lidar = M1C1Lidar(SERIAL_PORT, BAUD_RATE)
    
    try:
        plt.ion()
        fig = plt.figure(figsize=(7, 7))
        ax = fig.add_subplot(111, projection='polar')
        
        # Indicație clară pentru unitatea de măsură
        ax.set_title("LiDAR M1C1 - Vizualizare\n[Unitate de măsură: mm (Milimetri)]\nTasta 'L' = Start | Tasta 'P' = Stop", fontsize=10)
        ax.set_theta_zero_location('N')
        ax.set_theta_direction(-1)
        ax.set_rmax(4000) # Raza maximă: 4000 mm (4 Metri)
        
        scatter = ax.scatter([], [], s=6, c='red', alpha=0.8)

        def on_key(event):
            if event.key:
                k = event.key.lower()
                if k == 'l':
                    lidar.start()
                elif k == 'p':
                    lidar.stop()

        fig.canvas.mpl_connect('key_press_event', on_key)

        print("\n=========================================")
        print(" Apasă tasta 'L' în fereastra grafică -> START")
        print(" Apasă tasta 'P' în fereastra grafică -> STOP")
        print(" Unitate de măsură distanță: MILIMETRI (mm)")
        print("=========================================\n")

        while plt.fignum_exists(fig.number):
            if lidar.is_running:
                angles, distances = lidar.get_data()
                if angles and distances:
                    offsets = list(zip(angles, distances))
                    scatter.set_offsets(offsets)
            plt.pause(0.05)
            
    except Exception as e:
        print(f"\n❌ Eroare Interfață Grafică: {e}")
        print("Sfat: Dacă ești conectat prin SSH, asigură-te că folosești conexiune cu X11 Forwarding (`ssh -X pi@ip`) sau că rulați din interfața desktop a Raspberry Pi.")
    finally:
        lidar.stop()
        if lidar.ser and lidar.ser.is_open:
            lidar.ser.close()

if __name__ == '__main__':
    main()