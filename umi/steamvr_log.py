"""SteamVR frame changes (tilt re-estimate ~4-5 min after start, restart) shift the table by cm -> origin invalid."""
import datetime
import os
import re

LOG_PATH = os.path.expanduser("~/.steam/steam/logs/vrserver.txt")
CHANGE_RE = re.compile(rb"^\w{3} (\w{3}) (\d{2}) (\d{4}) (\d{2}):(\d{2}):(\d{2})\.\d+ .*(Setting universe tilt|Selected \w+ universe)")
MONTHS = [m.encode() for m in "Jan Feb Mar Apr May Jun Jul Aug Sep Oct Nov Dec".split()]   # locale-independent (instead of %b)


class FrameWatch:
    """Tail vrserver log -> frame changes [(epoch s, line)]; reopens the new log after a SteamVR restart."""

    def __init__(self, path=LOG_PATH):
        self.path, self.f, self.ino, self.buf = path, None, None, b""
        self.last = None
        self.poll()        # last change in existing log (startup origin check)

    def _reopen(self):
        try:
            st = os.stat(self.path)
        except OSError:
            return False
        if self.f is None or st.st_ino != self.ino or st.st_size < self.f.tell():
            if self.f is not None:
                self.f.close()
            self.f, self.ino, self.buf = open(self.path, "rb"), st.st_ino, b""
        return True

    def poll(self):
        if not self._reopen():
            return []
        lines = (self.buf + self.f.read()).split(b"\n")
        self.buf = lines.pop()   # last line may still be partial
        out = []
        for line in lines:
            m = CHANGE_RE.match(line)
            if m and m.group(1) in MONTHS:
                mon, (d, y, H, M, S) = MONTHS.index(m.group(1)) + 1, map(int, m.group(2, 3, 4, 5, 6))
                t = datetime.datetime(y, mon, d, H, M, S).timestamp()   # log is in local time
                out.append((t, line.decode("utf-8", "replace").split(" - ", 1)[-1].strip()))
        if out:
            self.last = out[-1]
        return out
