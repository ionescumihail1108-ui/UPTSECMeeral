import sys
import time
import math
import serial
import matplotlib.pyplot as plt

# Raspberry Pi 3 hardware UART
PORT = "/dev/serial0"

# The M1C1 may use either rate, depending on version.
BAUD_RATES = [115200, 150000]

MAX_RANGE_M = 8.0
ANGLE_OFFSET_DEG = 0.0


def read_packet(ser, buffer):
    """Read and validate one M1C1 AA 55 data packet."""
    buffer.extend(ser.read(ser.in_waiting or 1))

    while True:
        start = buffer.find(b"\xAA\x55")

        if start < 0:
            # Preserve a possible partial header.
            if buffer and buffer[-1] == 0xAA:
                buffer[:] = b"\xAA"
            else:
                buffer.clear()
            return None

        if start:
            del buffer[:start]

        if len(buffer) < 10:
            return None

        # Packet layout:
        # AA 55 CT LSN FSA(2) LSA(2) CS(2) samples...
        count = buffer[3]

        if count == 0 or count > 40:
            del buffer[0]
            continue

        packet_len = 10 + count * 2
        if len(buffer) < packet_len:
            return None

        packet = bytes(buffer[:packet_len])

        ct = packet[2]
        fsa = int.from_bytes(packet[4:6], "little")
        lsa = int.from_bytes(packet[6:8], "little")
        checksum = int.from_bytes(packet[8:10], "little")

        samples = [
            int.from_bytes(packet[10 + 2*i:12 + 2*i], "little")
            for i in range(count)
        ]

        expected = (
            0x55AA ^ (ct | (count << 8)) ^ fsa ^ lsa
        )
        for sample in samples:
            expected ^= sample

        del buffer[:packet_len]

        if checksum != (expected & 0xFFFF):
            continue

        # Angles are encoded in units of 1/64 degree.
        start_deg = (fsa >> 1) / 64.0
        end_deg = (lsa >> 1) / 64.0

        # Handle angle wrap at 360 degrees.
        span = (end_deg - start_deg) % 360.0

        points = []
        for i, raw_distance in enumerate(samples):
            distance_m = (raw_distance >> 2) / 1000.0

            if not (0.05 <= distance_m <= MAX_RANGE_M):
                continue

            angle = (
                start_deg + span * i / max(1, count - 1)
                + ANGLE_OFFSET_DEG
            ) % 360.0

            # Convert polar coordinates to Cartesian coordinates.
            # Zero degrees is displayed upwards.
            theta = math.radians(angle)
            x = distance_m * math.sin(theta)
            y = distance_m * math.cos(theta)
            points.append((x, y))

        return points


def main():
    global PORT

    if len(sys.argv) > 1:
        PORT = sys.argv[1]

    ser = None
    buffer = bytearray()

    # Try each documented baud rate until valid packets arrive.
    for baud in BAUD_RATES:
        try:
            test = serial.Serial(PORT, baud, timeout=0.05)
            test.reset_input_buffer()
            test_buffer = bytearray()
            deadline = time.monotonic() + 3.0
            valid = None

            print(f"Testing {PORT} at {baud} baud...")

            while time.monotonic() < deadline:
                valid = read_packet(test, test_buffer)
                if valid is not None:
                    break

            if valid is not None:
                ser = test
                buffer = test_buffer
                print(f"Valid LiDAR packets at {baud} baud!")
                break

            test.close()

        except serial.SerialException as exc:
            print(f"Could not open {PORT}: {exc}")
            return

    if ser is None:
        print("No valid packets detected.")
        print("Check power, TX-to-RX wiring, UART settings and baud rate.")
        return

    plt.ion()
    fig, ax = plt.subplots(figsize=(8, 8))
    ax.set_title("SigmaNortec M1C1 Live 2D LiDAR")
    ax.set_xlabel("X (metres)")
    ax.set_ylabel("Y (metres)")
    ax.set_aspect("equal", adjustable="box")
    ax.set_xlim(-MAX_RANGE_M, MAX_RANGE_M)
    ax.set_ylim(-MAX_RANGE_M, MAX_RANGE_M)
    ax.grid(True, alpha=0.35)

    # Draw range rings.
    for radius in [1, 2, 3, 4, 5, 6, 7, 8]:
        ring = plt.Circle(
            (0, 0), radius, fill=False, alpha=0.2
        )
        ax.add_patch(ring)

    ax.plot(0, 0, "r+", markersize=12)
    scatter = ax.scatter([], [], s=5)
    status = ax.text(
        0.02, 0.98, "Waiting for scan...",
        transform=ax.transAxes, va="top"
    )

    xs, ys = [], []
    last_rotation = time.monotonic()
    last_update = 0.0
    packet_count = 0

    try:
        while plt.fignum_exists(fig.number):
            points = read_packet(ser, buffer)

            if points is not None:
                xs.extend(x for x, y in points)
                ys.extend(y for x, y in points)
                packet_count += 1

            # Refresh the plot about 10 times per second.
            now = time.monotonic()
            if now - last_update >= 0.1:
                scatter.set_offsets(
                    list(zip(xs, ys)) if xs else [[0, 0]]
                )
                status.set_text(
                    f"Baud: {ser.baudrate}\n"
                    f"Points: {len(xs)}\n"
                    f"Packets: {packet_count}"
                )
                fig.canvas.draw_idle()
                fig.canvas.flush_events()
                last_update = now

            # Clear accumulated points roughly once per rotation.
            # M1C1 rotates at approximately 10 Hz.
            if now - last_rotation >= 0.1:
                xs.clear()
                ys.clear()
                last_rotation = now

    except KeyboardInterrupt:
        print("\nStopping viewer.")

    finally:
        ser.close()
        plt.close("all")


if __name__ == "__main__":
    main()