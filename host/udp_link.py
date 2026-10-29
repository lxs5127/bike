"""Wi-Fi UDP client; one state/command packet per datagram, no serial access."""
import select
import socket
import time
from collections import deque

from protocol import decode_datagram, encode_hello


class UDPLink:
    def __init__(self, address="192.168.4.1", port=3333):
        # Numeric IPv4 only: fail early rather than silently selecting IPv6.
        socket.inet_pton(socket.AF_INET, address)
        if not 1 <= port <= 65535:
            raise ValueError("UDP port must be in [1,65535]")
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket.setblocking(False)
        self.socket.connect((address, port))  # filters datagrams from other peers
        self.last_hello = float("-inf")
        self.last_timestamp = None
        self.boot_id = None
        self.retired_boot_ids = deque(maxlen=8)
        self.bad_packets = self.old_packets = self.gaps = 0
        self.coalesced = 0  # fresh states replaced by a newer state in the same read
        self.last_sequence = None

    def write(self, data):
        if self.socket.send(data) != len(data):
            raise OSError("Incomplete UDP datagram send")

    def read_state(self, timeout=.1):
        now = time.monotonic()
        if now - self.last_hello >= .25:
            self.write(encode_hello())  # registration only, never an actuator heartbeat
            self.last_hello = now
        if not select.select([self.socket], [], [], timeout)[0]:
            return None
        latest = None
        # Bounded drain of a burst; newer ESP32 time wins even if UDP reorders.
        for _ in range(256):
            try:
                data = self.socket.recv(2048)
            except BlockingIOError:
                break
            try:
                state = decode_datagram(data)
            except ValueError:
                self.bad_packets += 1
                continue
            if state.boot_id != self.boot_id:
                if state.boot_id in self.retired_boot_ids:
                    self.old_packets += 1
                    continue
                if self.boot_id is not None:
                    self.retired_boot_ids.append(self.boot_id)
                self.boot_id = state.boot_id
                self.last_timestamp = self.last_sequence = None
            if self.last_timestamp is not None and state.timestamp_us <= self.last_timestamp:
                self.old_packets += 1
                continue
            if self.last_sequence is not None:
                delta = (state.sequence - self.last_sequence) & 0xffffffff
                if 1 < delta < 0x80000000:
                    self.gaps += delta - 1
            self.last_timestamp = state.timestamp_us
            self.last_sequence = state.sequence
            if latest is not None and latest.boot_id == state.boot_id:
                self.coalesced += 1
            latest = state
        return latest

    def close(self):
        self.socket.close()
