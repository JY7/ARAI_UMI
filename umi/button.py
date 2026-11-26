"""Record buttons: BT buttons (evdev, no window focus needed) + terminal keys. Queue items: (cmd, src)."""
import os
import queue
import select
import sys
import threading
import time
from contextlib import suppress

KEYMAP = {" ": "toggle", "d": "discard", "t": "task", "o": "origin", "q": "quit"}


class PressClassifier:
    """Key down/up -> 'toggle' (1 click) | 'origin' (2 quick) | 'discard' (3 quick)."""

    def __init__(self, double_click_s=0.5, debounce_s=0.12, recording=lambda: False):
        self.dbl = float(double_click_s)
        self.debounce = float(debounce_s)  # some remotes send 2 keycodes per press
        self.recording = recording
        self.down_t = None
        self.last_press = -1e9
        self.count = 0
        self.last_up = -1e9
        self.instant = False
        self.swallow_until = -1e9   # so a 4th click after discard doesn't start recording

    def _finish(self):
        c, inst = self.count, self.instant
        self.count, self.instant = 0, False
        if inst:
            return None
        return {1: "toggle", 2: "origin"}.get(c)

    def on_key(self, value, t):
        """value: 1=down, 0=up, 2=autorepeat"""
        if value == 1:
            if self.down_t is not None or t - self.last_press < self.debounce:
                return None
            self.down_t = self.last_press = t
            if self.count and t - self.last_up > self.dbl:   # next press came before poll() -> resolve previous group
                return self._finish()
            return None
        if value == 0:
            if self.down_t is None:
                return None
            self.down_t = None
            if t <= self.swallow_until:
                self.swallow_until = t + self.dbl
                return None
            self.count, self.last_up = self.count + 1, t
            if self.count >= 3:
                self.count, self.instant, self.swallow_until = 0, False, t + self.dbl
                return "discard"
            if self.count == 1 and self.recording():  # stop at once so no trailing motion is recorded
                self.instant = True
                return "toggle"
            return None
        return self.poll(t)

    def poll(self, t):
        if self.count and self.down_t is None and t - self.last_up > self.dbl:
            return self._finish()
        return None


def _match(dev, pattern):
    """pattern: device name substring, MAC (uniq) or /dev/input path"""
    if pattern.startswith("/dev/"):
        return os.path.realpath(pattern) == os.path.realpath(dev.path)
    p = pattern.lower()
    return p in (dev.name or "").lower() or p == (dev.uniq or "").lower()


class BluetoothButton(threading.Thread):
    def __init__(self, hand, pattern, out_q, double_click_s=0.5, recording=lambda: False):
        super().__init__(daemon=True)
        self.hand = hand
        self.pattern = pattern
        self.q = out_q
        self.dbl = double_click_s
        self.recording = recording
        self.cls = PressClassifier(double_click_s, recording=recording)
        self.connected = False
        self.error = None
        self._halt = threading.Event()

    def _open(self):
        import evdev
        devs = []
        for p in evdev.list_devices():
            try:
                d = evdev.InputDevice(p)
            except OSError as e:
                self.error = "%s: %s (check input group permission)" % (p, e)
                continue
            if _match(d, self.pattern) and evdev.ecodes.EV_KEY in d.capabilities():
                with suppress(OSError):
                    d.grab()   # exclusive: key presses don't reach other apps
                devs.append(d)
            else:
                d.close()
        return devs

    def run(self):
        import evdev
        while not self._halt.is_set():
            devs = self._open()
            if not devs:
                self.connected = False
                self._halt.wait(1.0)
                continue
            self.connected, self.error = True, None
            # reset per connection: a press cut off by a disconnect would block the next one
            self.cls = PressClassifier(self.dbl, recording=self.recording)
            fds = {d.fd: d for d in devs}
            try:
                while not self._halt.is_set():
                    r, _, _ = select.select(list(fds), [], [], 0.05)
                    now = time.monotonic()
                    cmds = [self.cls.on_key(ev.value, now) for fd in r for ev in fds[fd].read()
                            if ev.type == evdev.ecodes.EV_KEY]
                    for cmd in cmds + [self.cls.poll(now)]:
                        if cmd:
                            self.q.put((cmd, "bt_" + self.hand))
            except OSError:
                pass
            finally:
                for d in devs:
                    with suppress(Exception):
                        d.close()
                self.connected = False

    def stop(self):
        self._halt.set()


class Buttons:
    def __init__(self, cfg, recording=lambda: False):
        self.q = queue.Queue()
        self._halt = threading.Event()
        self._paused = threading.Event()
        self._kb_idle = threading.Event()
        self._old_term = None
        self.bt = []
        bcfg = cfg.get("bluetooth") or {}
        # realpath by-id links before lowercasing (paths are case-sensitive)
        pats = [os.path.realpath(p) if p.startswith("/dev/") else p.lower()
                for p in (str(p) for p in (bcfg.get("devices") or {}).values() if p)]
        if any(a in b for i, a in enumerate(pats) for j, b in enumerate(pats) if i != j):
            # substring match: both hands would grab the same device
            raise ValueError("button.bluetooth.devices: both hands match the same device %s (names match partially). Use MAC for identical buttons" % pats)
        for hand, pattern in (bcfg.get("devices") or {}).items():
            if pattern is not None and not isinstance(pattern, str):
                # YAML reads an all-digit MAC (11:22:33:...) as a base-60 int
                raise ValueError("button.bluetooth.devices.%s is not a string (%r). Quote MACs: \"11:22:33:44:55:66\""
                                 % (hand, pattern))
            if pattern:
                b = BluetoothButton(hand, str(pattern), self.q, float(bcfg.get("double_click_s", 0.5)), recording)
                b.start()
                self.bt.append(b)
        if sys.stdin.isatty():
            import termios
            import tty
            self._old_term = termios.tcgetattr(sys.stdin)
            tty.setcbreak(sys.stdin.fileno())
            threading.Thread(target=self._kb_loop, daemon=True).start()

    def _kb_loop(self):
        fd = sys.stdin.fileno()
        while not self._halt.is_set():
            if self._paused.is_set():
                self._kb_idle.set()
                self._halt.wait(0.05)
                continue
            r, _, _ = select.select([fd], [], [], 0.1)
            if r:
                # os.read: sys.stdin.read(1) buffers leftover chars where select can't see them
                for c in os.read(fd, 64).decode("utf-8", "ignore"):
                    if c in KEYMAP:
                        self.q.put((KEYMAP[c], "keyboard"))

    def get(self, timeout=0.1):
        try:
            return self.q.get(timeout=timeout)
        except queue.Empty:
            return None, None

    def read_line(self, prompt):
        if self._old_term is None:
            return None
        import termios
        import tty
        self._kb_idle.clear()
        self._paused.set()
        self._kb_idle.wait(0.5)   # else the 1st typed char is taken as a command
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self._old_term)
        try:
            return input(prompt)
        finally:
            tty.setcbreak(sys.stdin.fileno())
            self._paused.clear()

    def close(self):
        self._halt.set()
        for b in self.bt:
            b.stop()
        if self._old_term is not None:
            import termios
            with suppress(termios.error, OSError):
                termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self._old_term)


def test_buttons():
    import evdev
    devs = []
    for p in sorted(evdev.list_devices()):
        try:
            d = evdev.InputDevice(p)
        except OSError:
            continue
        print("%-20s %-35s uniq=%s" % (p, d.name, d.uniq or ""))
        if evdev.ecodes.EV_KEY in d.capabilities():
            devs.append(d)
        else:
            d.close()
    if not devs:
        print("No readable input devices. Run `sudo usermod -aG input $USER` and log in again")
        return
    print("\nWaiting for button presses. Put the device name (or uniq=MAC) into button.bluetooth.devices in config.")
    print("Quit: q or Ctrl+C (read from the keyboard device, works even if the terminal is not focused)")
    e = evdev.ecodes
    ctrl = {e.KEY_LEFTCTRL, e.KEY_RIGHTCTRL}
    held = set()   # Ctrl read from the device so Ctrl+C works without terminal focus
    t0, down_t = time.time(), {}
    fds = {d.fd: d for d in devs}
    try:
        while True:
            r, _, _ = select.select(list(fds), [], [], 0.5)
            for fd in r:
                d = fds[fd]
                try:
                    for ev in d.read():
                        if ev.type == e.EV_KEY:
                            key = e.KEY.get(ev.code, ev.code)
                            t = ev.timestamp() - t0
                            if ev.value == 1:
                                down_t[(fd, ev.code)] = t
                            dt = t - down_t.pop((fd, ev.code), t) if ev.value == 0 else None
                            print("%8.3fs  %-20s %-30s uniq=%-17s %s %s%s" % (
                                t, d.path, d.name, d.uniq, key, {1: "down", 0: "up", 2: "repeat"}[ev.value],
                                "  (held %.3fs)" % dt if dt is not None else ""))
                            if ev.code in ctrl:
                                (held.add if ev.value else held.discard)((fd, ev.code))
                            elif ev.value == 1 and (ev.code == e.KEY_Q or (ev.code == e.KEY_C and any(f == fd for f, _ in held))):
                                return
                except OSError:
                    del fds[fd]
    except KeyboardInterrupt:
        pass
    finally:
        for d in devs:
            with suppress(Exception):
                d.close()
