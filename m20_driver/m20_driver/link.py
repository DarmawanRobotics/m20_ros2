"""UDP link to the M20: requests and heartbeats out, reports and call status responses in."""

import itertools
import select
import socket
import threading
import time
from typing import Callable, Dict, List, Optional, Tuple

from m20_driver import protocol as p


class Pending:
    """A request waiting for its call status response."""

    def __init__(self, message_id: int, key: Tuple[int, int], sent_at: float):
        """Remember the request's ID, (Type, Command) and send time."""
        self.message_id = message_id
        self.key = key
        self.sent_at = sent_at
        self.event = threading.Event()
        self.response: Optional[p.Response] = None


class RobotLink:
    """
    One UDP socket: requests go to the robot, which reports back to this socket's port.

    A receive thread hands every frame on; with heartbeat_hz > 0 a heartbeat thread keeps the
    robot reporting to this socket (1.2.1: at least 1 Hz). heartbeat_ok, when given, is asked
    before every heartbeat: while it returns False (the caller is hung) heartbeats pause, so the
    robot notices the lost controller; on_heartbeat_pause(paused) reports the changes.
    """

    def __init__(
        self,
        host: str,
        port: int,
        local_ip: str,
        local_port: int,
        on_frame: Callable[[p.Frame], None],
        poll_sec: float = 0.2,
        heartbeat_hz: float = 0.0,
        heartbeat_ok: Optional[Callable[[], bool]] = None,
        on_heartbeat_pause: Optional[Callable[[bool], None]] = None,
    ):
        """Store the addresses; nothing is opened until start()."""
        self.remote = (host, int(port))
        self.local = (local_ip, int(local_port))
        self._on_frame = on_frame
        self._poll_sec = poll_sec
        self._heartbeat_hz = float(heartbeat_hz)
        self._heartbeat_ok = heartbeat_ok
        self._on_heartbeat_pause = on_heartbeat_pause
        self.heartbeats_paused = False
        self._sock: Optional[socket.socket] = None
        self._threads: List[threading.Thread] = []
        self._running = threading.Event()
        self._stopping = threading.Event()
        self._lock = threading.Lock()
        self._ids = itertools.count()
        self._pending: Dict[int, Pending] = {}
        self.heartbeats = 0
        self.rx_count = 0
        self.tx_count = 0
        self.tx_errors = 0
        self.rx_errors = 0
        self.bad_frames = 0
        self.last_rx: Optional[float] = None
        self.last_error = ''
        self.last_failure: Optional[p.Response] = None

    def start(self):
        """Bind the socket and start the receive (and heartbeat) thread."""
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.bind(self.local)       # no SO_REUSEADDR: a second driver on the port must fail
        sock.setblocking(False)
        self._sock = sock
        self.local = sock.getsockname()
        self._stopping.clear()
        self._running.set()
        self._threads = [threading.Thread(target=self._receive_loop, name='m20_rx', daemon=True)]
        if self._heartbeat_hz > 0.0:
            self._threads.append(threading.Thread(target=self._heartbeat_loop,
                                                  name='m20_heartbeat', daemon=True))
        for thread in self._threads:
            thread.start()

    def stop(self):
        """Stop the threads and close the socket."""
        self._running.clear()
        self._stopping.set()
        for thread in self._threads:
            thread.join(timeout=2.0)
        self._threads = []
        if self._sock is not None:
            self._sock.close()
            self._sock = None

    def next_id(self) -> int:
        """Return the next message ID: 0, 1, ... 65535, 0 (1.1.5)."""
        with self._lock:
            return next(self._ids) & 0xFFFF

    def send(self, message: Tuple[int, int], items: Optional[dict] = None) -> Optional[int]:
        """Send one request; return its message ID, None when it could not be sent."""
        sock = self._sock
        if sock is None:
            return None
        message_id = self.next_id()
        try:
            data = p.encode(message, items, message_id)
            with self._lock:
                sock.sendto(data, self.remote)
        except (OSError, ValueError) as error:
            self.tx_errors += 1
            self.last_error = str(error)
            return None
        self.tx_count += 1
        return message_id

    def submit(self, message: Tuple[int, int], items: Optional[dict] = None) -> Optional[Pending]:
        """Send a request and register it for its call status response; None when not sent."""
        sock = self._sock
        if sock is None:
            return None
        message_id = self.next_id()
        pending = Pending(message_id, (int(message[0]), int(message[1])), time.monotonic())
        with self._lock:
            self._pending[message_id] = pending
        try:
            data = p.encode(message, items, message_id)
            with self._lock:
                sock.sendto(data, self.remote)
        except (OSError, ValueError) as error:
            self.tx_errors += 1
            self.last_error = str(error)
            self.forget(pending)
            return None
        self.tx_count += 1
        return pending

    def forget(self, pending: Pending):
        """Stop waiting for a request's response."""
        with self._lock:
            if self._pending.get(pending.message_id) is pending:
                del self._pending[pending.message_id]

    def wait(self, pending: Pending, timeout: float,
             abort: Optional[Callable[[], bool]] = None) -> Optional[p.Response]:
        """Wait for a submitted request's response; None on timeout or when abort() turns True."""
        end = time.monotonic() + timeout
        try:
            while True:
                left = end - time.monotonic()
                if left <= 0.0 or (abort is not None and abort()):
                    return pending.response
                if pending.event.wait(min(left, 0.05)):
                    return pending.response
        finally:
            self.forget(pending)

    def request(self, message: Tuple[int, int], items: Optional[dict] = None,
                timeout: float = 1.0,
                abort: Optional[Callable[[], bool]] = None) -> Tuple[bool, Optional[p.Response]]:
        """
        Send a request and wait for its call status response.

        Return (sent, response); response is None when none arrived within the timeout.
        """
        pending = self.submit(message, items)
        if pending is None:
            return False, None
        return True, self.wait(pending, timeout, abort)

    def age(self, now: Optional[float] = None) -> float:
        """Return seconds since the last frame from the robot, infinity before the first."""
        if self.last_rx is None:
            return float('inf')
        return (time.monotonic() if now is None else now) - self.last_rx

    def handle(self, data: bytes):
        """Process one datagram from the robot (the receive thread, or a test)."""
        frame = p.parse_frame(data)
        if frame is None:
            self.bad_frames += 1
            return
        self.rx_count += 1
        self.last_rx = time.monotonic()
        response = p.decode_response(frame)
        if response is not None:
            if not response.ok:
                self.last_failure = response
            key = (response.kind, response.command)
            with self._lock:
                pending = self._pending.get(response.message_id)
                if pending is None or pending.key != key or pending.response is not None:
                    # an ID that matches nothing waiting: the oldest request of the same kind
                    waiting = [q for q in self._pending.values()
                               if q.key == key and q.response is None]
                    pending = min(waiting, key=lambda q: q.sent_at) if waiting else None
                if pending is not None:
                    pending.response = response
                    pending.event.set()
        self._on_frame(frame)

    def _heartbeat_loop(self):
        """Send the heartbeat at heartbeat_hz until stopped."""
        period = 1.0 / self._heartbeat_hz
        due = time.monotonic()
        while self._running.is_set():
            ok = self._heartbeat_ok is None or self._heartbeat_ok()
            if ok == self.heartbeats_paused:
                self.heartbeats_paused = not ok
                if self._on_heartbeat_pause is not None:
                    self._on_heartbeat_pause(self.heartbeats_paused)
            if ok and self.send(p.HEARTBEAT) is not None:
                self.heartbeats += 1
            due += period
            delay = due - time.monotonic()
            if delay < 0.0:          # fell behind (suspended): do not burst to catch up
                due, delay = time.monotonic(), 0.0
            if self._stopping.wait(delay):
                return

    def _receive_loop(self):
        """Read datagrams until stopped; a failing callback never kills the loop."""
        while self._running.is_set():
            sock = self._sock
            if sock is None:
                return
            try:
                readable, _, _ = select.select([sock], [], [], self._poll_sec)
            except (OSError, ValueError):
                return
            if not readable:
                continue
            try:
                data, _address = sock.recvfrom(65535)
            except (BlockingIOError, InterruptedError):
                continue
            except OSError as error:
                self.rx_errors += 1
                self.last_error = str(error)
                continue
            try:
                self.handle(data)
            except Exception as error:  # noqa: B902  a bad datagram must not stop the link
                self.rx_errors += 1
                self.last_error = f'{type(error).__name__}: {error}'
