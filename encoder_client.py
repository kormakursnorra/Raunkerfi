"""
Client for the encoderd daemon (UNIX datagram socket at /tmp/encoder.sock).

Every request returns the cumulative step counts "left,right" since the
daemon started. Motor A (GPIO25/GPIO20) is left, Motor B (GPIO26/GPIO5) is right.
"""

import os
import socket

SOCKET_PATH = "/tmp/encoder.sock"


class EncoderClient:
    def __init__(self, path=SOCKET_PATH, timeout=0.1):
        self.sock = socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM)
        # The client MUST be bound, otherwise the daemon has no address to
        # reply to, its sendto() fails and the daemon exits. A leading \0
        # makes it an abstract address (Linux only), so no file is left behind.
        self.sock.bind(f"\0encoder-client-{os.getpid()}-{id(self)}")
        try:
            self.sock.connect(path)
        except (FileNotFoundError, ConnectionRefusedError) as e:
            self.sock.close()
            raise RuntimeError(
                f"Encoder daemon not reachable at {path}. "
                "Check it with: systemctl status encoderd"
            ) from e
        self.sock.settimeout(timeout)

    def _drain(self):
        """Throw away late replies from earlier timed-out requests."""
        timeout = self.sock.gettimeout()
        self.sock.setblocking(False)
        try:
            while True:
                self.sock.recv(64)
        except BlockingIOError:
            pass
        finally:
            self.sock.settimeout(timeout)

    def read(self, retries=2):
        """Return (left_steps, right_steps), raw counts with no sign correction."""
        for _ in range(retries + 1):
            self._drain()
            self.sock.send(b"?")
            try:
                data = self.sock.recv(64)
            except socket.timeout:
                continue
            left, right = data.decode().split(",")
            return int(left), int(right)
        raise RuntimeError("Encoder daemon did not reply")

    def close(self):
        self.sock.close()

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.close()


if __name__ == "__main__":
    # Quick manual test: spin the wheels by hand and watch the numbers.
    from time import sleep
    with EncoderClient() as enc:
        while True:
            print(enc.read())
            sleep(0.2)