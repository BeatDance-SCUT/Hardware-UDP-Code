"""Receive BeatDance ESP32-S3 IMU JSON over UDP and plot 6-axis data live.

Install: python -m pip install matplotlib
Run:     python imu_udp_plot.py --port 5005
Optional: python imu_udp_plot.py --port 5005 --window 10 --csv imu.csv
"""

import argparse
import csv
import json
import math
import socket
import time
from collections import deque

import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation


FIELDS = ("ax", "ay", "az", "gx", "gy", "gz")


def main():
    parser = argparse.ArgumentParser(description="Live ESP32 IMU UDP plotting")
    parser.add_argument("--bind", default="0.0.0.0", help="local listening IP")
    parser.add_argument("--port", type=int, default=5005, help="UDP port")
    parser.add_argument("--window", type=float, default=10.0, help="visible seconds")
    parser.add_argument("--csv", help="optional file to save received samples")
    args = parser.parse_args()
    if not (0 < args.port < 65536 and args.window > 0):
        parser.error("port must be 1..65535 and window must be > 0")

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((args.bind, args.port))
    sock.setblocking(False)
    print(f"Listening for UDP JSON on {args.bind}:{args.port}")
    print("Close the plot window to stop. Use Ctrl+C if necessary.")

    csv_file = None
    csv_writer = None
    if args.csv:
        csv_file = open(args.csv, "w", encoding="utf-8", newline="")
        csv_writer = csv.writer(csv_file)
        csv_writer.writerow(["pc_time", "device", "sender_ip", "seq", "t_ms",
                             "ax_g", "ay_g", "az_g", "gx_dps", "gy_dps", "gz_dps", "temp_c"])
        print(f"Saving CSV to {args.csv}")

    ts = deque()
    values = {key: deque() for key in FIELDS}
    arrivals = deque()
    state = {"packets": 0, "gaps": 0, "last_seq": None,
             "last_receive": None, "sender": "-", "temp": None}

    fig, (acc_ax, gyro_ax) = plt.subplots(2, 1, figsize=(11, 7), sharex=True)
    fig.subplots_adjust(hspace=0.25, top=0.88)
    acc_ax.set_title("Acceleration (g)")
    gyro_ax.set_title("Angular velocity (deg/s)")
    gyro_ax.set_xlabel("Time relative to now (s)")
    acc_ax.set_ylabel("g")
    gyro_ax.set_ylabel("deg/s")
    acc_ax.grid(True, alpha=0.3)
    gyro_ax.grid(True, alpha=0.3)
    acc_ax.set_ylim(-4.1, 4.1)
    gyro_ax.set_ylim(-50, 50)
    gyro_ax.set_xlim(-args.window, 0)

    acc_lines = {k: acc_ax.plot([], [], label=k.upper())[0]
                 for k in ("ax", "ay", "az")}
    gyro_lines = {k: gyro_ax.plot([], [], label=k.upper())[0]
                  for k in ("gx", "gy", "gz")}
    acc_ax.legend(loc="upper right", ncol=3)
    gyro_ax.legend(loc="upper right", ncol=3)

    def update(_frame):
        received_this_frame = 0
        # Drain pending packets without blocking the GUI.
        for _ in range(400):
            try:
                packet, addr = sock.recvfrom(4096)
            except BlockingIOError:
                break
            except OSError as exc:
                print("Socket error:", exc)
                break

            try:
                obj = json.loads(packet.decode("utf-8"))
                if obj.get("v") != 1:
                    continue
                acc = obj["acc_g"]
                gyro = obj["gyro_dps"]
                if len(acc) != 3 or len(gyro) != 3:
                    continue
                six = [float(v) for v in (*acc, *gyro)]
                if not all(math.isfinite(v) for v in six):
                    continue
            except (ValueError, TypeError, KeyError, UnicodeDecodeError, AttributeError):
                continue

            now = time.monotonic()
            ts.append(now)
            for k, value in zip(FIELDS, six):
                values[k].append(value)
            arrivals.append(now)
            state["packets"] += 1
            state["last_receive"] = now
            state["sender"] = addr[0]
            state["temp"] = obj.get("temp_c")
            seq = obj.get("seq")
            if type(seq) is int:
                prev = state["last_seq"]
                if prev is not None and seq > prev + 1:
                    state["gaps"] += seq - prev - 1
                state["last_seq"] = seq

            if csv_writer:
                csv_writer.writerow([time.time(), obj.get("device", ""), addr[0],
                                     seq, obj.get("t_ms"), *six, obj.get("temp_c")])
            received_this_frame += 1

        if csv_file and received_this_frame:
            csv_file.flush()

        now = time.monotonic()
        while ts and now - ts[0] > args.window:
            ts.popleft()
            for series in values.values():
                series.popleft()
        while arrivals and now - arrivals[0] > 1.0:
            arrivals.popleft()

        xs = [t - now for t in ts]
        for k, line in {**acc_lines, **gyro_lines}.items():
            line.set_data(xs, list(values[k]))

        if ts:
            recent_max_gyro = max(abs(v) for k in ("gx", "gy", "gz")
                                  for v in values[k])
            gyro_limit = min(550.0, max(10.0, recent_max_gyro * 1.2))
            gyro_ax.set_ylim(-gyro_limit, gyro_limit)

        status = ("receiving" if state["last_receive"] is not None and
                  now - state["last_receive"] < 2 else "WAITING FOR UDP")
        temp = state["temp"]
        temp_text = f" | T={temp} C" if temp is not None else ""
        fig.suptitle(
            f"ESP32 IMU | {status} | from {state['sender']} | "
            f"{len(arrivals)} pkt/s | total {state['packets']} | "
            f"seq gaps {state['gaps']}{temp_text}", fontsize=11
        )
        return list(acc_lines.values()) + list(gyro_lines.values())

    animation = FuncAnimation(fig, update, interval=50, blit=False,
                              cache_frame_data=False)
    try:
        plt.show()
    finally:
        # Keep animation referenced during plt.show(); close cleanly afterward.
        del animation
        sock.close()
        if csv_file:
            csv_file.close()


if __name__ == "__main__":
    main()
