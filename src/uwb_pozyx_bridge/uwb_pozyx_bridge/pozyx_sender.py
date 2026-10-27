"""TCP transport for the Pozyx wire protocol.

The ArduPilot SITL serial port is exposed as a TCP server, so the node acts as
a TCP client.  The transport is deliberately tolerant: if the connection drops
it reconnects in the background rather than raising, because the SITL process
may restart while the ROS node keeps running.

Frames are small (about 40 bytes at 20 Hz), so writes go straight to the
socket.  A non-blocking write that cannot complete is dropped rather than
queued, because stale ranging data is worse than no data for the EKF.
"""

from __future__ import annotations

import socket
import threading
import time

__all__ = ["PozyxSender"]


class PozyxSender:
    """Sends Pozyx frames to an ArduPilot SITL serial port over TCP."""

    def __init__(
        self,
        host: str = "127.0.0.1",
        port: int = 5761,
        reconnect_delay: float = 1.0,
        connect_timeout: float = 2.0,
    ) -> None:
        self.host = host
        self.port = port
        self.reconnect_delay = reconnect_delay
        self.connect_timeout = connect_timeout

        self._sock: socket.socket | None = None
        self._lock = threading.Lock()
        self._last_attempt = 0.0

        #: Number of frames handed to the transport.
        self.frames_sent = 0
        #: Number of frames dropped because the socket was not writable.
        self.frames_dropped = 0
        #: Number of successful connections.
        self.connections = 0

    # -- connection -----------------------------------------------------

    @property
    def connected(self) -> bool:
        """True when a socket is currently open."""
        with self._lock:
            return self._sock is not None

    def _open(self) -> None:
        """Try to open the socket.  Returns True on success."""
        try:
            sock = socket.create_connection(
                (self.host, self.port), timeout=self.connect_timeout
            )
            sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            # Do not let a stalled peer block the ROS executor.
            sock.settimeout(0.5)
        except OSError:
            return False

        with self._lock:
            old = self._sock
            self._sock = sock
        if old is not None:
            try:
                old.close()
            except OSError:
                pass
        self.connections += 1
        return True

    def connect(self, timeout: float = 30.0) -> bool:
        """Block until the connection is established or the timeout expires.

        Args:
            timeout: Maximum time to wait, in seconds.

        Returns:
            True if the connection was established.
        """
        deadline = time.time() + timeout
        while time.time() < deadline:
            if self._open():
                return True
            time.sleep(0.5)
        return False

    def close(self) -> None:
        """Close the socket."""
        with self._lock:
            sock = self._sock
            self._sock = None
        if sock is not None:
            try:
                sock.close()
            except OSError:
                pass

    # -- sending --------------------------------------------------------

    def send(self, data: bytes) -> bool:
        """Send a frame, reconnecting if necessary.

        Args:
            data: The encoded frame.

        Returns:
            True if the frame was written to the socket.
        """
        if not data:
            return False

        if not self.connected:
            # Rate limit reconnection attempts.
            now = time.time()
            if now - self._last_attempt < self.reconnect_delay:
                self.frames_dropped += 1
                return False
            self._last_attempt = now
            if not self._open():
                self.frames_dropped += 1
                return False

        with self._lock:
            sock = self._sock

        if sock is None:
            self.frames_dropped += 1
            return False

        try:
            sock.sendall(data)
        except (BrokenPipeError, ConnectionResetError, socket.timeout, OSError):
            # The peer went away; drop the socket and try again next time.
            with self._lock:
                if self._sock is sock:
                    self._sock = None
            try:
                sock.close()
            except OSError:
                pass
            self.frames_dropped += 1
            return False

        self.frames_sent += 1
        return True
