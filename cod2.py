import os
import sys

# --- FIX PENTRU WAYLAND & RASPBERRY PI OS ---
# Forțăm Matplotlib să folosească backend-ul TkAgg pentru a evita problemele Wayland/QWindow
os.environ["QT_QPA_PLATFORM"] = "xcb" 
import matplotlib
matplotlib.use('TkAgg')

import serial
import math
import time
import threading
import matplotlib.pyplot as plt

# --- CONFIGURARE PORT SERIAL ---
SERIAL_PORT = '/dev/serial0'
BAUD_RATE = 115200

class M1C1LidarGUI:
    def __init__(self, port, baudrate):
        self.port = port
        self.baudrate = baudrate
        self.ser = None
        self.is_running = False
        self.angles_rad = []     # Radiani pentru plot polar
        self.distances_mm = []   # Milimetri (mm)
        self.lock = threading.Lock()
        self.thread = None

    def connect(self):
        try:
            self.ser = serial.Serial(self.port, self.baudrate, timeout=0.1)
            self.ser.flushInput()
            print(f"[LiDAR] Conectat cu succes pe {self.port}")
            return True
        except Exception as e:
            print(f"❌ Eroare la deschiderea portului serial: {e}")
            return False

    def start(self):
        if not self.is_running:
            if self.ser is None or not self.ser.is_open:
                if not self.connect():
                    return
            self.is_running = True
            self.thread = threading.Thread(target=self._read_loop, daemon=True)
            self.thread.start()
            print("[STATUS] Scanare PORNIŢĂ (Tasta L)")

    def stop(self):
        if self.is_running:
            self.is_running = False
            if self.thread and self.thread.is_alive():
                self.thread.join(timeout=1.0)
            print("[STATUS] Scanare OPRITĂ (Tasta P)")

    def _read_loop(self):
        """Loop optimizat de citire și parsare pachete M1C1"""
        while self.is_running and self.ser and self.ser.is_open:
            try:
                if self.ser.in_waiting > 0:
                    header = self.ser.read(2)
                    if len(header) < 2:
                        continue
                    
                    # Verificare antet 0xAA 0x55
                    if header[0] == 0xAA and header[1] == 0x55:
                        info = self.ser.read(2)
                        if len(info) < 2:
                            continue
                        
                        sample_count = info[1]
                        if sample_count == 0 or sample_count > 80:
                            self.ser.flushInput()
                            continue

                        payload_len = 4 + (sample_count * 2)
                        payload = self.ser.read(payload_len)
                        
                        if len(payload) < payload_len:
                            continue

                        start_angle = ((payload[0] | (payload[1] << 8)) >> 1) / 64.0
                        end_angle = ((payload[2] | (payload[3] << 8)) >> 1) / 64.0

                        diff = end_angle - start_angle
                        if diff < 0:
                            diff += 360.0
                        step = diff / (sample_count - 1) if sample_count > 1 else 0

                        curr_angles = []
                        curr_dists = []

                        for i in range(sample_count):
                            idx = 4 + i * 2
                            dist_mm = (payload[idx] | (payload[idx+1] << 8)) / 4.0
                            angle_deg = (start_angle + step * i) % 360.0

                            # Filtrare zgomot (doar distante între 5 cm și 6 m)
                            if 50 <= dist_mm <= 6000:
                                curr_angles.append(math.radians(angle_deg))
                                curr_dists.append(dist_mm)

                        with self.lock:
                            self.angles_rad.extend(curr_angles)
                            self.distances_mm.extend(curr_dists)
                            
                            # Păstrăm ultimele ~500 de puncte (o rotație completă)
                            if len(self.angles_rad) > 600:
                                self.angles_rad = self.angles_rad[-500:]
                                self.distances_mm = self.distances_mm[-500:]
                    else:
                        self.ser.read(1)
                else:
                    time.sleep(0.002)
            except Exception:
                pass

    def get_data(self):
        with self.lock:
            return list(self.angles_rad), list(self.distances_mm)

def main():
    lidar = M1C1LidarGUI(SERIAL_PORT, BAUD_RATE)
    
    # Configurare grafic polar Matplotlib
    plt.ion()
    fig = plt.figure(figsize=(7, 7))
    ax = fig.add_subplot(111, projection='polar')
    
    ax.set_title("M1C1 LiDAR 360° - Interfață Grafică\n[Unitate măsură: mm]\n'L' = Start  |  'P' = Stop", fontsize=11, fontweight='bold')
    ax.set_theta_zero_location('N') # Nordul (0°) este sus
    ax.set_theta_direction(-1)      # Sens orar
    ax.set_rmax(4000)                # Scală până la 4000 mm (4 metri)
    
    scatter = ax.scatter([], [], s=8, c='red', alpha=0.75)

    def on_key(event):
        if event.key:
            k = event.key.lower()
            if k == 'l':
                lidar.start()
            elif k == 'p':
                lidar.stop()

    fig.canvas.mpl_connect('key_press_event', on_key)

    print("\n=========================================")
    print(" INTERFAȚĂ GRAFICĂ M1C1 LIDAR")
    print(" Click pe fereastră și apasă:")
    print("   • Tasta 'L' -> PORNIRE scanare")
    print("   • Tasta 'P' -> OPRIRE scanare")
    print(" Unitate de măsură: MILIMETRI (mm)")
    print("=========================================\n")

    try:
        while plt.fignum_exists(fig.number):
            if lidar.is_running:
                angles, distances = lidar.get_data()
                if angles and distances:
                    offsets = list(zip(angles, distances))
                    scatter.set_offsets(offsets)
            
            plt.pause(0.03) # ~30 FPS refresh rate
            
    except KeyboardInterrupt:
        print("\nÎnchidere...")
    finally:
        lidar.stop()
        if lidar.ser and lidar.ser.is_open:
            lidar.ser.close()

if __name__ == '__main__':
    main()