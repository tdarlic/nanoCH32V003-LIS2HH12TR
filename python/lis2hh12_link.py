"""Serial link to the nanoCH32V003 LIS2HH12 test firmware.

Runs a background reader thread so incoming lines (ACC stream, command
responses, spontaneous INTEVENT lines) are never missed while the UI is
doing something else, and command responses can be waited on explicitly.
"""
import queue
import threading
import time

import serial


class Link:
    def __init__(self, port="/dev/ttyACM0", baud=115200):
        self.ser = serial.Serial(port, baud, timeout=0.2)
        time.sleep(0.3)
        self.ser.reset_input_buffer()
        self._lines = queue.Queue()
        self._stop = False
        self._thread = threading.Thread(target=self._reader, daemon=True)
        self._thread.start()

    def _reader(self):
        while not self._stop:
            try:
                line = self.ser.readline()
            except serial.SerialException:
                break
            if line:
                text = line.decode(errors="replace").rstrip()
                if text:
                    self._lines.put(text)

    def send(self, cmd):
        self.ser.write((cmd + "\n").encode())

    def poll_lines(self):
        """Drain and return all currently queued lines (non-blocking)."""
        out = []
        while True:
            try:
                out.append(self._lines.get_nowait())
            except queue.Empty:
                break
        return out

    def wait_for(self, prefixes, timeout=2.0):
        """Collect lines until one starts with any of `prefixes`, or timeout.

        Returns the list of lines collected, including the terminal one
        (if found). `prefixes` is a tuple of strings, per str.startswith.
        """
        end = time.time() + timeout
        collected = []
        while time.time() < end:
            try:
                line = self._lines.get(timeout=0.1)
            except queue.Empty:
                continue
            collected.append(line)
            if line.startswith(prefixes):
                return collected
        return collected

    def close(self):
        self._stop = True
        try:
            self.ser.close()
        except serial.SerialException:
            pass


def parse_acc(line):
    """Return (x, y, z) mg ints if `line` is an ACC line, else None."""
    if not line.startswith("ACC,"):
        return None
    try:
        x, y, z = (int(v) for v in line[4:].split(","))
    except ValueError:
        return None
    return x, y, z
