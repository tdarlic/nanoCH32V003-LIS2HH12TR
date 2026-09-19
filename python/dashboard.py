#!/usr/bin/env python3
"""Terminal dashboard for testing the LIS2HH12 accelerometer.

Covers: self-test, changing the output data rate, changing the full-scale
range (+-2g/4g/8g), FIFO / stream mode, interrupt generation (both the
I2C-polled status and the real electrical INT1 pin on PD4/EXTI4),
peak-hold (highest |g| since reset), and a live bar-graph visualization.
Raw X/Y/Z stays pinned at the bottom of the screen at all times.

Usage: python3 dashboard.py [port]   (default /dev/ttyACM0)
"""
import curses
import sys
import time

from lis2hh12_link import Link, parse_acc

BAR_WIDTH = 40
PROMPT_ROW = 3  # fixed row, directly under the menu - always the same place

MENU = [
    ("s", "Self-test"),
    ("o", "Change ODR"),
    ("g", "Change scale"),
    ("f", "FIFO / stream mode"),
    ("i", "Interrupt test"),
    ("p", "Reset peak"),
    ("r", "Resume/pause live stream"),
    ("q", "Quit"),
]

ODR_NAMES = ["off", "10Hz", "50Hz", "100Hz", "200Hz", "400Hz", "800Hz"]
FS_OPTIONS = ["2", "4", "8"]

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
        self.x = self.y = self.z = self.temp_mc = 0
        self.log = []  # newest last
        self.odr_name = "100Hz"
        self.fs_g = 2
        self.bar_range_mg = 2000
        self.peak_mg = 0
        self._last_peak_poll = 0.0
        self.int_pin_level = 0
        self.int_pin_edges = 0
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
        """Route ACC/PEAK lines into the live readout, log everything else."""
        for line in lines:
            acc = parse_acc(line)
            if acc:
                self.x, self.y, self.z, self.temp_mc = acc
            elif line.startswith("PEAK,"):
                try:
                    self.peak_mg = int(line[len("PEAK,"):])
                except ValueError:
                    self.add_log(line)
            elif line.startswith("INTPIN"):
                # covers both the spontaneous "INTPIN,level=..,edges=.."
                # and the on-demand "INTPINSTATUS,level=..,edges=.." reply -
                # both share the same "level=..,edges=.." payload.
                try:
                    kv = dict(p.split("=") for p in line.split(",", 1)[1].split(","))
                    self.int_pin_level = int(kv["level"])
                    self.int_pin_edges = int(kv["edges"])
                except (IndexError, ValueError, KeyError):
                    self.add_log(line)
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
        pos = int(center + (value / self.bar_range_mg) * (BAR_WIDTH // 2))
        pos = max(5, min(4 + BAR_WIDTH, pos))
        lo, hi = (center, pos) if pos >= center else (pos, center)
        attr = 0
        if self.has_color:
            frac = abs(value) / self.bar_range_mg
            color = 1 if frac < 0.6 else (2 if frac < 0.9 else 3)
            attr = curses.color_pair(color)
        self.safe_addstr(win, row, lo, "#" * max(1, hi - lo), attr)
        self.safe_addstr(win, row, 4 + BAR_WIDTH + 3, f"{value:6d} mg")

    def draw(self):
        stdscr = self.stdscr
        stdscr.erase()
        h, w = stdscr.getmaxyx()

        self.safe_addstr(stdscr, 0, 0, "LIS2HH12 test dashboard".ljust(w - 1), curses.A_BOLD)
        status_line = f"ODR: {self.odr_name}   Scale: +-{self.fs_g}g"
        self.safe_addstr(stdscr, 1, 0, status_line)
        menu_line = "  ".join(f"[{k}]{label}" for k, label in MENU)
        self.safe_addstr(stdscr, 2, 0, menu_line)

        self.bar(stdscr, 4, "X", self.x)
        self.bar(stdscr, 5, "Y", self.y)
        self.bar(stdscr, 6, "Z", self.z)

        mag = (self.x ** 2 + self.y ** 2 + self.z ** 2) ** 0.5
        self.safe_addstr(stdscr, 7, 0,
            f"|g| = {mag / 1000:.3f} g   peak = {self.peak_mg / 1000:.3f} g   "
            f"temp = {self.temp_mc / 1000:.1f} C")

        pin_attr = curses.color_pair(1) if (self.has_color and self.int_pin_level) else 0
        self.safe_addstr(stdscr, 8, 0,
            f"INT1 pin (PD4, real electrical, EXTI4): level={self.int_pin_level}  "
            f"edges={self.int_pin_edges}", pin_attr)

        self.safe_addstr(stdscr, 10, 0, "Log:", curses.A_UNDERLINE)
        log_h = max(1, h - 14)
        for i, line in enumerate(self.log[-log_h:]):
            self.safe_addstr(stdscr, 11 + i, 0, line)

        # pinned raw readout at the very bottom
        self.safe_addstr(stdscr, h - 2, 0, "-" * (w - 1))
        self.safe_addstr(stdscr, h - 1, 0,
            f"RAW  x={self.x:6d} mg  y={self.y:6d} mg  z={self.z:6d} mg  "
            f"temp={self.temp_mc / 1000:5.1f} C",
            curses.A_BOLD)

        stdscr.refresh()

    # ---- blocking helpers for menu prompts -----------------------------
    # Both always draw on the same fixed row, right under the menu, with a
    # reverse-video bar so it's unmistakable that input is expected and
    # exactly where to look - rather than a bare cursor floating somewhere
    # near the bottom of the screen with no visible label.
    def clear_prompt_row(self):
        h, w = self.stdscr.getmaxyx()
        self.stdscr.addstr(PROMPT_ROW, 0, " " * (w - 1))

    def prompt(self, label, default=""):
        h, w = self.stdscr.getmaxyx()
        self.clear_prompt_row()
        header = f"> {label}"[: w - 1]
        self.stdscr.addstr(PROMPT_ROW, 0, header.ljust(w - 1), curses.A_REVERSE)
        self.stdscr.refresh()

        curses.curs_set(1)
        curses.echo()
        self.stdscr.nodelay(False)
        self.stdscr.timeout(-1)

        input_col = min(len(header) + 1, max(0, w - 22))
        win = curses.newwin(1, 20, PROMPT_ROW, input_col)
        win.addstr(0, 0, default)
        win.refresh()
        try:
            s = win.getstr(0, 0, 19).decode(errors="replace") or default
        except curses.error:
            s = default

        curses.noecho()
        curses.curs_set(0)
        self.stdscr.nodelay(True)
        self.stdscr.timeout(50)
        self.clear_prompt_row()
        self.stdscr.refresh()
        return s.strip()

    def wait_key(self, prompt_text):
        h, w = self.stdscr.getmaxyx()
        self.clear_prompt_row()
        header = f"> {prompt_text}"[: w - 1]
        self.stdscr.addstr(PROMPT_ROW, 0, header.ljust(w - 1), curses.A_REVERSE)
        self.stdscr.refresh()
        self.stdscr.nodelay(False)
        self.stdscr.timeout(-1)
        c = self.stdscr.getch()
        self.stdscr.nodelay(True)
        self.stdscr.timeout(50)
        self.clear_prompt_row()
        self.stdscr.refresh()
        return chr(c) if 0 < c < 256 else ""

    # ---- actions --------------------------------------------------------
    def run_selftest(self):
        self.add_log(">>> SELFTEST (do not move the board)")
        self.link.send("SELFTEST")
        self.consume(self.link.wait_for(("SELFTEST,DONE",), timeout=3.0))

    def change_odr(self):
        odr_hint = " ".join(f"{i}={name}" for i, name in enumerate(ODR_NAMES))
        s = self.prompt(f"ODR ({odr_hint}): ")
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

    def change_scale(self):
        s = self.prompt(f"Full scale {FS_OPTIONS} g: ")
        if s not in FS_OPTIONS:
            self.add_log(f"invalid scale: {s!r} (must be 2, 4 or 8)")
            return
        g = int(s)
        self.link.send(f"FS {g}")
        lines = self.link.wait_for(("OK,FS", "ERR"), timeout=1.0)
        self.consume(lines)
        if any(l.startswith("OK,FS") for l in lines):
            self.fs_g = g
            self.bar_range_mg = g * 1000

    def reset_peak(self):
        self.peak_mg = 0
        self.link.send("PEAK RESET")
        self.consume(self.link.wait_for(("OK,PEAK",), timeout=1.0))

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
        c = self.wait_key("Interrupt: [o]n  [f]off  [t]status (I2C-polled + real pin)  or Esc: ")
        if c == "o":
            axis = self.prompt("Axis (X/Y/Z/ANY): ", "ANY").upper() or "ANY"
            direction = self.prompt("Direction (HIGH/LOW): ", "HIGH").upper() or "HIGH"
            thresh = self.prompt("Threshold mg (default 500): ", "500") or "500"
            dur = self.prompt("Duration (ODR cycles, default 0): ", "0") or "0"
            self.link.send(f"INT ON {axis} {direction} {thresh} {dur}")
            self.consume(self.link.wait_for(("OK,INT", "ERR"), timeout=1.0))
            self.add_log("armed - move/shake the board to trigger it "
                          "(watch the INT1 pin line above for the real edge)")
        elif c == "f":
            self.link.send("INT OFF")
            self.consume(self.link.wait_for(("OK,INT",), timeout=1.0))
        elif c == "t":
            self.link.send("INT STATUS")
            self.consume(self.link.wait_for(("INTSTATUS",), timeout=1.0))
            self.link.send("INT PINSTATUS")
            self.consume(self.link.wait_for(("INTPINSTATUS",), timeout=1.0))

    def toggle_stream(self):
        self._streaming = not getattr(self, "_streaming", True)
        self.link.send(f"STREAM {'ON' if self._streaming else 'OFF'}")
        self.consume(self.link.wait_for(("OK",), timeout=1.0))

    def run(self):
        # Sync the real INT1 pin's current level/edge count before the
        # first draw - after this, spontaneous INTPIN lines keep it live.
        self.link.send("INT PINSTATUS")
        self.consume(self.link.wait_for(("INTPINSTATUS",), timeout=1.0))

        while True:
            self.drain()
            now = time.time()
            if now - self._last_peak_poll > 1.0:
                self._last_peak_poll = now
                self.link.send("PEAK")
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
            elif ch == "g":
                self.change_scale()
            elif ch == "f":
                self.fifo_menu()
            elif ch == "i":
                self.int_menu()
            elif ch == "p":
                self.reset_peak()
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
