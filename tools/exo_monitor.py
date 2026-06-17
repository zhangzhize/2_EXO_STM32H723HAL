"""Exoskeleton Motion Monitor — reads VOFA JustFloat telemetry over TCP and displays current action mode + terrain slope + torque.

Usage:
    python exo_monitor.py [--host HOST] [--port PORT]

Requires only Python stdlib (tkinter + socket + struct + threading). No pip install needed.
"""

import argparse
import socket
import struct
import threading
import tkinter as tk

# ── mode mapping ──────────────────────────────────────────────────
MODE_NAMES = {
    0: ("SITTING",    "坐  稳",     "#e94560"),
    1: ("SIT → STAND", "坐 → 站",    "#f5a623"),
    2: ("STANDING",    "站  直",     "#0f3460"),
    3: ("STAND → SIT", "站 → 坐",    "#f5a623"),
    4: ("WALKING",     "步  行",     "#00b894"),
    5: ("RAMP ASCENT", "上  坡",     "#00b894"),
    6: ("RAMP DESCENT","下  坡",     "#00b894"),
    7: ("STAIR ASCENT","上 楼 梯",    "#a29bfe"),
    8: ("STAIR DESCENT","下 楼 梯",   "#a29bfe"),
}

# VOFA JustFloat frame: N floats (4B LE) + 4B tail (0x00 0x00 0x80 0x7F = +Inf)
NUM_CHANNELS = 4
FLOAT_BYTES = NUM_CHANNELS * 4
TAIL = b"\x00\x00\x80\x7f"
FRAME_SIZE = FLOAT_BYTES + 4


# ── telemetry reader (shared between windows) ─────────────────────
class TelemetryReader(threading.Thread):
    def __init__(self, host, port):
        super().__init__(daemon=True)
        self.host = host
        self.port = port
        self.current_mode = 0
        self.slope_deg = 0.0
        self.ref_torque = 0.0
        self.out_torque = 0.0
        self._lock = threading.Lock()

    def snapshot(self):
        with self._lock:
            return self.current_mode, self.slope_deg, self.ref_torque, self.out_torque

    def run(self):
        buf = bytearray()
        while True:
            try:
                with socket.create_connection((self.host, self.port), timeout=3) as s:
                    s.settimeout(1)
                    while True:
                        chunk = s.recv(4096)
                        if not chunk:
                            break
                        buf.extend(chunk)
                        while len(buf) >= FRAME_SIZE:
                            if buf[FLOAT_BYTES:FLOAT_BYTES+4] == TAIL:
                                f = struct.unpack_from("<4f", buf, 0)
                                buf = buf[FRAME_SIZE:]
                                with self._lock:
                                    self.current_mode = int(f[0])
                                    self.slope_deg = f[1]
                                    self.ref_torque = f[2]
                                    self.out_torque = f[3]
                            else:
                                buf.pop(0)
            except (socket.timeout, ConnectionRefusedError, OSError):
                pass


# ── mode window ───────────────────────────────────────────────────
BG = "#0d0d0d"

class ModeOverlay(tk.Toplevel):
    TICK_MS = 66

    def __init__(self, reader: TelemetryReader):
        super().__init__()
        self.reader = reader
        self.title("Mode")
        self.geometry("500x400")
        self.minsize(300, 250)
        self.configure(bg=BG)
        self.protocol("WM_DELETE_WINDOW", self.withdraw)  # can't close, just hide

        self.grid_rowconfigure(0, weight=1)
        self.grid_rowconfigure(1, weight=1)
        self.grid_rowconfigure(2, weight=1)
        self.grid_columnconfigure(0, weight=1)

        self.label_en = tk.Label(self, text="", fg="#fff", bg=BG, anchor=tk.CENTER)
        self.label_en.grid(row=0, column=0, sticky="s", pady=(10, 0))

        self.label_cn = tk.Label(self, text="", fg="#bbb", bg=BG, anchor=tk.CENTER)
        self.label_cn.grid(row=1, column=0, sticky="n", pady=(0, 0))

        # slope (row 2, shown only when walking)
        self.slope_label = tk.Label(self, text="", bg=BG, anchor=tk.CENTER)
        self.slope_label.grid(row=2, column=0, sticky="n", pady=(0, 10))

        self.bind("<Configure>", self._on_resize)
        self._mode_sz = 0
        self._tick()

    def _on_resize(self, event):
        if event.widget != self:
            return
        h = event.height
        new_sz = max(18, h * 14 // 100)
        if new_sz != self._mode_sz:
            self._mode_sz = new_sz
            self._font_en = ("Consolas", new_sz, "bold")
            self._font_cn = ("Microsoft YaHei", int(new_sz * 0.75), "bold")
            self._font_slope = ("Consolas", int(new_sz * 1.6), "bold")

    def _tick(self):
        mode, slope, _, _ = self.reader.snapshot()
        fe = getattr(self, "_font_en", ("Consolas", 40, "bold"))
        fc = getattr(self, "_font_cn", ("Microsoft YaHei", 30, "bold"))
        fs = getattr(self, "_font_slope", ("Consolas", 64, "bold"))

        en, cn, color = MODE_NAMES.get(mode, (f"MODE {mode}", "", "#fff"))
        self.label_en.config(text=en, fg=color, font=fe)
        self.label_cn.config(text=cn, fg=color, font=fc)

        if mode in (4, 5, 6):
            abs_s = abs(slope)
            if abs_s < 0.5:
                text, tg = "≈ 0°", "#bbb"
            else:
                text, tg = f"{slope:+.1f}°", "#e17055" if slope > 0 else "#0984e3"
            self.slope_label.config(text=text, fg=tg, font=fs)
        else:
            self.slope_label.config(text="", font=fs)

        self.after(self.TICK_MS, self._tick)


# ── torque window ─────────────────────────────────────────────────
class TorqueOverlay(tk.Toplevel):
    TICK_MS = 66

    def __init__(self, reader: TelemetryReader):
        super().__init__()
        self.reader = reader
        self.title("Torque")
        self.geometry("360x280")
        self.minsize(250, 180)
        self.configure(bg=BG)
        self.protocol("WM_DELETE_WINDOW", self.withdraw)

        self.grid_rowconfigure(0, weight=1)
        self.grid_rowconfigure(1, weight=3)
        self.grid_columnconfigure(0, weight=1)

        # numeric display
        self.value_label = tk.Label(
            self, text="0.00 Nm", fg="#fff", bg=BG, anchor=tk.CENTER)
        self.value_label.grid(row=0, column=0, sticky="s", pady=(10, 5))

        # bar canvas
        self.canvas = tk.Canvas(self, bg=BG, highlightthickness=0)
        self.canvas.grid(row=1, column=0, sticky="nsew", padx=30, pady=(0, 20))

        self.bind("<Configure>", self._on_resize)
        self._sz = 0
        self._tick()

    def _on_resize(self, event):
        if event.widget != self:
            return
        h = event.height
        new_sz = max(16, h * 12 // 100)
        if new_sz != self._sz:
            self._sz = new_sz
            self._font = ("Consolas", new_sz, "bold")

    def _tick(self):
        _, _, ref, _ = self.reader.snapshot()
        ft = getattr(self, "_font", ("Consolas", 32, "bold"))

        disp = ref * 10.0
        self.value_label.config(
            text=f"Assisting {disp:+.2f} Nm",
            fg="#e17055" if ref > 0 else "#0984e3" if ref < 0 else "#bbb",
            font=ft,
        )

        c = self.canvas
        cw = max(c.winfo_width(), 10)
        ch = max(c.winfo_height(), 10)
        c.delete("all")

        # torque gauge
        mid_x, mid_y = cw / 2, ch / 2
        bar_h = ch * 0.15
        bar_w = cw * 0.8
        left = mid_x - bar_w / 2
        right = mid_x + bar_w / 2
        top = mid_y - bar_h / 2
        bot = mid_y + bar_h / 2

        # background track
        c.create_rectangle(left, top, right, bot, fill="#1a1a1a", outline="")
        c.create_line(mid_x, top - 10, mid_x, bot + 10, fill="#444")

        # filled bar
        frac = max(-1.0, min(1.0, ref / 3.0))
        if frac >= 0:
            c.create_rectangle(mid_x, top, mid_x + frac * bar_w / 2, bot,
                               fill="#e17055", outline="")
        else:
            c.create_rectangle(mid_x + frac * bar_w / 2, top, mid_x, bot,
                               fill="#0984e3", outline="")

        # scale labels
        sf = ("Consolas", max(7, int(ch * 0.06)))
        c.create_text(left, bot + 12, text="-3", fill="#444", anchor=tk.N, font=sf)
        c.create_text(right, bot + 12, text="+3", fill="#444", anchor=tk.N, font=sf)
        c.create_text(mid_x, bot + 12, text="0", fill="#bbb", anchor=tk.N, font=sf)

        self.after(self.TICK_MS, self._tick)


# ── root window (just shows both sub-windows) ────────────────────
class ExoMonitor(tk.Tk):
    def __init__(self, reader: TelemetryReader):
        super().__init__()
        self.withdraw()  # hide root window
        ModeOverlay(reader)
        TorqueOverlay(reader)


def main():
    parser = argparse.ArgumentParser(description="Exoskeleton Motion Monitor")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=4002)
    args = parser.parse_args()

    reader = TelemetryReader(args.host, args.port)
    reader.start()

    app = ExoMonitor(reader)
    app.mainloop()


if __name__ == "__main__":
    main()
