import socket


class EncoderClient:
    def __init__(self, path="/tmp/encoder.sock", timeout=0.5):
        self.sock = socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM)
        self.sock.bind("")          # autobind, so the server has somewhere to reply
        self.sock.connect(path)
        self.sock.settimeout(timeout)

    def read(self):
        """Return (motor_a_steps, motor_b_steps) since the server started."""
        self.sock.send(b"?")        # content is ignored, any datagram works
        data = self.sock.recv(64)
        a, b = data.decode().split(",")
        return int(a), int(b)

    def close(self):
        self.sock.close()

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.close()


if __name__ == "__main__":
    import time
    with EncoderClient() as enc:
        last = enc.read()
        while True:
            time.sleep(0.1)
            now = enc.read()
            print(now, "delta:", (now[0] - last[0], now[1] - last[1]))
            last = now