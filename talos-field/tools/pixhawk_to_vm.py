#!/usr/bin/env python3
"""Forward MAVLink between a Pixhawk on the Mac's USB and Mission Planner in the Ubuntu VM.

Standard library only (no pyserial, no MAVProxy). Bytes are relayed untouched in both
directions, so the ground station sees the flight controller exactly as if it were local.

Usage:
    python3 tools/pixhawk_to_vm.py                       # auto-pick /dev/cu.usbmodem*
    python3 tools/pixhawk_to_vm.py /dev/cu.usbmodem1234  # explicit port
    python3 tools/pixhawk_to_vm.py --dest 192.168.64.2:14550

Mission Planner side: UDP, port 14550, Connect. It only needs packets to arrive.
"""
import argparse
import glob
import os
import select
import socket
import sys
import termios
import time


def open_serial(path: str) -> int:
    fd = os.open(path, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
    attrs = termios.tcgetattr(fd)
    # raw mode: no echo, no line discipline, no flow control, 8N1
    attrs[0] = 0                                   # iflag
    attrs[1] = 0                                   # oflag
    attrs[2] = termios.CS8 | termios.CREAD | termios.CLOCAL  # cflag
    attrs[3] = 0                                   # lflag
    attrs[4] = termios.B115200                     # ispeed (ignored by USB CDC, set anyway)
    attrs[5] = termios.B115200                     # ospeed
    attrs[6][termios.VMIN] = 0
    attrs[6][termios.VTIME] = 0
    termios.tcsetattr(fd, termios.TCSANOW, attrs)
    termios.tcflush(fd, termios.TCIOFLUSH)
    return fd


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("port", nargs="?", help="serial device, default: first /dev/cu.usbmodem*")
    ap.add_argument("--dest", default="192.168.64.2:14550", help="VM ip:port (default 192.168.64.2:14550)")
    ap.add_argument("--listen", type=int, default=14555, help="local UDP port for replies from the GCS")
    args = ap.parse_args()

    port = args.port or next(iter(sorted(glob.glob("/dev/cu.usbmodem*"))), None)
    if not port:
        print("no /dev/cu.usbmodem* found: plug the Pixhawk's USB-C into the Mac", file=sys.stderr)
        return 1
    host, _, p = args.dest.partition(":")
    dest = (host, int(p or 14550))

    fd = open_serial(port)
    udp = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    udp.bind(("0.0.0.0", args.listen))
    udp.setblocking(False)
    print(f"{port}  <->  udp {dest[0]}:{dest[1]}   (replies on local udp {args.listen})")
    print("Ctrl-C to stop")

    up = down = 0
    last = time.time()
    gcs_addr = None
    try:
        while True:
            r, _, _ = select.select([fd, udp], [], [], 1.0)
            if fd in r:
                data = os.read(fd, 4096)
                if data:
                    udp.sendto(data, dest)
                    up += len(data)
            if udp in r:
                data, addr = udp.recvfrom(4096)
                gcs_addr = addr
                os.write(fd, data)
                down += len(data)
            if time.time() - last >= 5:
                print(f"fc->gcs {up:8d} B   gcs->fc {down:8d} B   gcs {gcs_addr}")
                last = time.time()
    except KeyboardInterrupt:
        pass
    finally:
        os.close(fd)
        udp.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
