import serial
import math
import time
import threading
import matplotlib.pyplot as plt

# --- CONFIGURARE PORT SERIAL ---
# Dacă ai activat dtoverlay=miniuart-bt folosește '/dev/ttyAMA0', altfel '/dev/serial0'
SERIAL_PORT = '/dev/ttyAMA0'
BAUD_RATE = 115200

class M1C1Lidar:
    def __init__(self, port, baudrate):
        self.port = port
        self.baudrate = baudrate
        self.ser = None
        self.is_running = False
        self.angles = []
        self.distances = []
        self.lock = threading.Lock()
        self.thread = None

    def connect(self):
        try:
            self.ser = serial.Serial(self.port, self.baudrate, timeout=1)
            print(f"[LiDAR] Conectat cu succes pe {self.port}")
        except Exception as e:
            print(f"[Eroare] Nu s-a putut deschide portul serial: {e}")

    def start(self):
        if not self.is_running:
            if self.ser is None or not self.ser.is_open:
                self.connect()
            self.is_running = True
            self.thread = threading.Thread(target=self._read_loop, daemon=True)
            self.thread.start()
            print("[LiDAR] SCANARE PORNITĂ (Tasta L)")

    def stop(self):
        if self.is_running:
            self.is_running = False
            if self.thread and self.thread.is_alive():
                self.thread.join(timeout=1.0)
            print("[LiDAR] SCANARE OPRITĂ (Tasta P)")

    def _read_loop(self):
        """Loop de citire și parsare pachete brute M1C1"""
        buffer = bytearray()
        
        while self.is_running and self.ser and self.ser.is_open:
            try:
                if self.ser.in_waiting > 0:
                    buffer.extend(self.ser.read(self.ser.in_waiting))
                    
                    # Căutăm antetul standard M1C1 (0xAA 0x55)
                    while len(buffer) >= 10:
                        if buffer[0] == 0xAA and buffer[1] == 0x55:
                            # Exemplu simplificat de extragere cadru/pachet
                            pkt_type = buffer[2]
                            sample_count = buffer[3]
                            
                            pkt_len = 10 + sample_count * 2
                            if len(buffer) < pkt_len:
                                break # Așteptăm mai mulți octeți
                            
                            packet = buffer[:pkt_len]
                            buffer = buffer[pkt_len:]
                            
                            self._parse_packet(packet, sample_count)
                        else:
                            buffer.pop(0) # Trecem la următorul octet dacă nu e header
            except Exception as e:
                print(f"[Eroare Citire Serial]: {e}")
                break

    def _parse_packet(self, packet, sample_count):
        """Parsează un pachet și extrage unghiul și distanța (mm)"""
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
            dist = (packet[idx] | (packet[idx+1] << 8)) / 4.0 # Distanța în mm
            angle = (start_angle + step * i) % 360.0
            
            if dist > 0: # Filtrare citiri invalide
                # Convertim unghiul în radiani pentru graficul polar
                current_angles.append(math.radians(angle))
                current_distances.append(dist)

        # Actualizăm datele globale în mod thread-safe
        with self.lock:
            self.angles.extend(current_angles)
            self.distances.extend(current_distances)
            
            # Păstrăm doar o rotație completă (~360-500 de puncte)
            if len(self.angles) > 600:
                self.angles = self.angles[-500:]
                self.distances = self.distances[-500:]

    def get_data(self):
        with self.lock:
            return list(self.angles), list(self.distances)

# --- INTERFAȚĂ GRAFICĂ ȘI CONTROL DE LA TASTATURĂ ---
def main():
    lidar = M1C1Lidar(SERIAL_PORT, BAUD_RATE)
    
    # Setare grafic polar Matplotlib
    plt.ion()
    fig = plt.figure(figsize=(7, 7))
    ax = fig.add_subplot(111, projection='polar')
    ax.set_title("M1C1 LiDAR - Vizualizare 360°\nApasa 'L' pt Start | Apasa 'P' pt Stop", fontsize=10)
    ax.set_theta_zero_location('N') # Nordul (0 grade) sus
    ax.set_theta_direction(-1)      # Sens orar
    ax.set_rmax(4000)                # Scală maximă 4 metri (4000 mm)
    
    scatter = ax.scatter([], [], s=5, c='red', alpha=0.75)

    def on_key(event):
        key = event.key.lower()
        if key == 'l':
            lidar.start()
        elif key == 'p':
            lidar.stop()

    fig.canvas.mpl_connect('key_press_event', on_key)

    print("\n=========================================")
    print(" Apasă tasta 'L' pe fereastra grafică pentru PORNIRE")
    print(" Apasă tasta 'P' pe fereastra grafică pentru OPRIRE")
    print("=========================================\n")

    try:
        while plt.fignum_exists(fig.number):
            if lidar.is_running:
                angles, distances = lidar.get_data()
                if angles and distances:
                    # Actualizare puncte grafice
                    offsets = list(zip(angles, distances))
                    scatter.set_offsets(offsets)
            
            plt.pause(0.05) # Refresh ~20 FPS pentru grafic
            
    except KeyboardInterrupt:
        print("\nÎnchidere aplicație...")
    finally:
        lidar.stop()
        if lidar.ser and lidar.ser.is_open:
            lidar.ser.close()

if __name__ == '__main__':
    main()