#!/usr/bin/env python3
"""Terminal dashboard for testing the LIS2HH12 accelerometer.

Covers: self-test, changing the output data rate, FIFO / stream mode,
interrupt generation, and a live bar-graph visualization. Raw X/Y/Z stays
pinned at the bottom of the screen at all times.

Usage: python3 dashboard.py [port]   (default /dev/ttyACM0)
"""
import curses
import sys
import time

from lis2hh12_link import Link, parse_acc

BAR_WIDTH = 40
BAR_RANGE_MG = 2000  # +-2g full scale

MENU = [
    ("s", "Self-test"),
    ("o", "Change ODR"),
    ("f", "FIFO / stream mode"),
    ("i", "Interrupt test"),
    ("r", "Resume/pause live stream"),
    ("q", "Quit"),
]

ODR_NAMES = ["off", "10Hz", "50Hz", "100Hz", "200Hz", "400Hz", "800Hz"]

FIFO_MODES = [
    ("1", "BYPASS"),
    ("2", "FIFO"),
    ("3", "STREAM"),
    ("4", "STREAM2FIFO"),
    ("5", "BYPASS2STREAM"),
    ("6", "BYPASS2FIFO"),
]


class Dashboard:
    def __init__(self, stdscr, link):
        self.stdscr = stdscr
        self.link = link
        self.x = self.y = self.z = 0
        self.log = []  # newest last
        self.odr_name = "100Hz"
        curses.curs_set(0)
        stdscr.nodelay(True)
        stdscr.timeout(50)
        try:
            curses.start_color()
            curses.use_default_colors()
            curses.init_pair(1, curses.COLOR_GREEN, -1)
            curses.init_pair(2, curses.COLOR_YELLOW, -1)
            curses.init_pair(3, curses.COLOR_RED, -1)
            self.has_color = True
        except curses.error:
            self.has_color = False

    def add_log(self, line):
        self.log.append(line)
        self.log = self.log[-200:]

    def consume(self, lines):
        """Route ACC lines into the live readout, log everything else."""
        for line in lines:
            acc = parse_acc(line)
            if acc:
                self.x, self.y, self.z = acc
            else:
                self.add_log(line)

    def drain(self):
        self.consume(self.link.poll_lines())

    # ---- rendering ----------------------------------------------------
    def safe_addstr(self, win, y, x, text, attr=0):
        h, w = win.getmaxyx()
        if y < 0 or y >= h or x >= w - 1:
            return
        try:
            win.addstr(y, x, text[: max(0, w - 1 - x)], attr)
        except curses.error:
            pass  # bottom-right-corner cursor-advance quirk; harmless

    def bar(self, win, row, label, value):
        center = 4 + BAR_WIDTH // 2
        self.safe_addstr(win, row, 0, f"{label}:")
        self.safe_addstr(win, row, 4, "|" + " " * BAR_WIDTH + "|")
        pos = int(center + (value / BAR_RANGE_MG) * (BAR_WIDTH // 2))
        pos = max(5, min(4 + BAR_WIDTH, pos))
        lo, hi = (center, pos) if pos >= center else (pos, center)
        attr = 0
        if self.has_color:
            mag = abs(value)
            color = 1 if mag < 1200 else (2 if mag < 1800 else 3)
            attr = curses.color_pair(color)
        self.safe_addstr(win, row, lo, "#" * max(1, hi - lo), attr)
        self.safe_addstr(win, row, 4 + BAR_WIDTH + 3, f"{value:6d} mg")

    def draw(self):
        stdscr = self.stdscr
        stdscr.erase()
        h, w = stdscr.getmaxyx()

        self.safe_addstr(stdscr, 0, 0, "LIS2HH12 test dashboard".ljust(w - 1), curses.A_BOLD)
        menu_line = f"ODR: {self.odr_name}   |   " + "  ".join(f"[{k}]{label}" for k, label in MENU)
        self.safe_addstr(stdscr, 1, 0, menu_line)

        self.bar(stdscr, 3, "X", self.x)
        self.bar(stdscr, 4, "Y", self.y)
        self.bar(stdscr, 5, "Z", self.z)

        mag = (self.x ** 2 + self.y ** 2 + self.z ** 2) ** 0.5
        self.safe_addstr(stdscr, 6, 0, f"|g| = {mag / 1000:.3f} g")

        self.safe_addstr(stdscr, 8, 0, "Log:", curses.A_UNDERLINE)
        log_h = max(1, h - 12)
        for i, line in enumerate(self.log[-log_h:]):
            self.safe_addstr(stdscr, 9 + i, 0, line)

        # pinned raw readout at the very bottom
        self.safe_addstr(stdscr, h - 2, 0, "-" * (w - 1))
        self.safe_addstr(stdscr, h - 1, 0,
            f"RAW  x={self.x:6d} mg  y={self.y:6d} mg  z={self.z:6d} mg",
            curses.A_BOLD)

        stdscr.refresh()

    # ---- blocking helpers for menu prompts -----------------------------
    def prompt(self, label, default=""):
        h, w = self.stdscr.getmaxyx()
        self.stdscr.addstr(h - 4, 0, " " * (w - 1))
        self.stdscr.addstr(h - 4, 0, label)
        curses.curs_set(1)
        curses.echo()
        self.stdscr.nodelay(False)
        self.stdscr.timeout(-1)
        win = curses.newwin(1, 30, h - 4, len(label) + 1)
        win.addstr(0, 0, default)
        curses.curs_set(1)
        s = win.getstr(0, 0, 20).decode(errors="replace") or default
        curses.noecho()
        curses.curs_set(0)
        self.stdscr.nodelay(True)
        self.stdscr.timeout(50)
        return s.strip()

    def wait_key(self, prompt_text):
        h, w = self.stdscr.getmaxyx()
        self.stdscr.addstr(h - 4, 0, prompt_text[: w - 1])
        self.stdscr.refresh()
        self.stdscr.nodelay(False)
        self.stdscr.timeout(-1)
        c = self.stdscr.getch()
        self.stdscr.nodelay(True)
        self.stdscr.timeout(50)
        return chr(c) if 0 < c < 256 else ""

    # ---- actions --------------------------------------------------------
    def run_selftest(self):
        self.add_log(">>> SELFTEST (do not move the board)")
        self.link.send("SELFTEST")
        self.consume(self.link.wait_for(("SELFTEST,DONE",), timeout=3.0))

    def change_odr(self):
        s = self.prompt(f"ODR index 0-6 {ODR_NAMES}: ")
        try:
            n = int(s)
        except ValueError:
            self.add_log(f"invalid ODR index: {s!r}")
            return
        if not 0 <= n <= 6:
            self.add_log("ODR index must be 0-6")
            return
        self.link.send(f"ODR {n}")
        self.consume(self.link.wait_for(("OK,ODR", "ERR"), timeout=1.0))
        self.odr_name = ODR_NAMES[n]

    def fifo_menu(self):
        opts = "  ".join(f"[{k}]{name}" for k, name in FIFO_MODES)
        c = self.wait_key(f"FIFO mode: {opts}  [r]ead  [t]status  or Esc: ")
        mode = dict(FIFO_MODES).get(c)
        if mode:
            thresh = self.prompt("FIFO threshold (0-31, default 0): ", "0")
            self.link.send(f"FIFO MODE {mode} {thresh or '0'}")
            self.consume(self.link.wait_for(("OK,FIFO", "ERR"), timeout=1.0))
        elif c == "r":
            self.add_log(">>> FIFO READ")
            self.link.send("FIFO READ")
            self.consume(self.link.wait_for(("FIFOREAD,DONE",), timeout=3.0))
        elif c == "t":
            self.link.send("FIFO STATUS")
            self.consume(self.link.wait_for(("FIFOSTATUS",), timeout=1.0))

    def int_menu(self):
        c = self.wait_key("Interrupt: [o]n  [f]off  [t]status  or Esc: ")
        if c == "o":
            axis = self.prompt("Axis (X/Y/Z/ANY): ", "ANY").upper() or "ANY"
            direction = self.prompt("Direction (HIGH/LOW): ", "HIGH").upper() or "HIGH"
            thresh = self.prompt("Threshold mg (default 500): ", "500") or "500"
            dur = self.prompt("Duration (ODR cycles, default 0): ", "0") or "0"
            self.link.send(f"INT ON {axis} {direction} {thresh} {dur}")
            self.consume(self.link.wait_for(("OK,INT", "ERR"), timeout=1.0))
            self.add_log("armed - move/shake the board to trigger it")
        elif c == "f":
            self.link.send("INT OFF")
            self.consume(self.link.wait_for(("OK,INT",), timeout=1.0))
        elif c == "t":
            self.link.send("INT STATUS")
            self.consume(self.link.wait_for(("INTSTATUS",), timeout=1.0))

    def toggle_stream(self):
        self._streaming = not getattr(self, "_streaming", True)
        self.link.send(f"STREAM {'ON' if self._streaming else 'OFF'}")
        self.consume(self.link.wait_for(("OK",), timeout=1.0))

    def run(self):
        while True:
            self.drain()
            self.draw()
            c = self.stdscr.getch()
            if c == -1:
                continue
            ch = chr(c) if 0 < c < 256 else ""
            if ch == "q":
                return
            elif ch == "s":
                self.run_selftest()
            elif ch == "o":
                self.change_odr()
            elif ch == "f":
                self.fifo_menu()
            elif ch == "i":
                self.int_menu()
            elif ch == "r":
                self.toggle_stream()


def main(stdscr):
    port = sys.argv[1] if len(sys.argv) > 1 else "/dev/ttyACM0"
    link = Link(port)
    try:
        Dashboard(stdscr, link).run()
    finally:
        link.close()


if __name__ == "__main__":
    curses.wrapper(main)
