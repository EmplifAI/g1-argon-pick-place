"""Left hand only. Register protocol from installed dfx_inspire_service."""
import fcntl
import os
import select
import struct
import termios
import time


class InspireLeft:
    def __init__(self, port="/dev/ttyUSB2"):
        if port != "/dev/ttyUSB2":
            raise ValueError("Left-only port policy")
        self.fd = os.open(port, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
        fcntl.flock(self.fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
        a = termios.tcgetattr(self.fd)
        a[0] = a[1] = a[3] = 0
        a[2] = termios.CS8 | termios.CREAD | termios.CLOCAL
        a[4] = a[5] = termios.B115200
        a[6][termios.VMIN] = a[6][termios.VTIME] = 0
        termios.tcsetattr(self.fd, termios.TCSANOW, a)

    def request(self, op, addr, payload, size):
        termios.tcflush(self.fd, termios.TCIFLUSH)
        packet = bytearray([235, 144, 1, len(payload) + 3, op, addr & 255, addr >> 8]) + payload
        packet.append(sum(packet[2:]) & 255)
        if os.write(self.fd, packet) != len(packet):
            raise RuntimeError("Hand serial short write")
        end = time.monotonic() + 0.06
        response = b""
        while len(response) < size and time.monotonic() < end:
            if select.select([self.fd], [], [], max(0, end - time.monotonic()))[0]:
                response += os.read(self.fd, size - len(response))
        if (len(response) != size or response[:2] != b"\x90\xeb"
                or response[2] != 1 or response[4:7] != bytes([op, addr & 255, addr >> 8])
                or (sum(response[2:-1]) & 255) != response[-1]):
            raise RuntimeError("Invalid/late hand serial response")
        return response[7:-1]

    def read(self, addr, length):
        return self.request(17, addr, bytes([length]), length + 8)

    def positions(self):
        return [x / 1000 for x in struct.unpack("<6h", self.read(1546, 12))]

    def telemetry(self):
        return {"q": self.positions(),
                "force_n": [x * 0.0098 for x in struct.unpack("<6h", self.read(1582, 12))],
                "current_ma": list(struct.unpack("<6h", self.read(1594, 12))),
                "errors": list(self.read(1606, 6)),
                "temperature_c": list(self.read(1618, 6))}

    def set_positions(self, q):
        payload = struct.pack("<6h", *[round(x * 1000) for x in q])
        if self.request(18, 1486, payload, 9) != b"\x01":
            raise RuntimeError("Hand position write rejected")

    def close(self):
        os.close(self.fd)
