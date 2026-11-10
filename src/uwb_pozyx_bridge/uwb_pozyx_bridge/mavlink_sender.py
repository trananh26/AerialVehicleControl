"""TCP transport for MAVLink messages to an ArduPilot SITL serial port.

The ArduPilot SITL serial port is exposed as a TCP server, so the node acts
as a TCP client.  The transport is deliberately tolerant: if the connection
drops it reconnects in the background rather than raising, because the SITL
process may restart while the ROS node keeps running.

MAVLink framing is produced by ``pymavlink``: the message is packed into a
MAVLink v2 frame with the standard heartbeat stream id, so that ArduPilot
accepts it on a serial port that is configured for MAVLink.

Only the outgoing direction is implemented.  ArduPilot's ODOMETRY handling
does not need a reply, but the autopilot does send its own telemetry back
over the same port, so the socket is drained by a background thread.  Without
that drain the receive buffer fills up and the autopilot's writes stall.
"""

from __future__ import annotations

import socket
import threading
import time

__all__ = ["MavlinkSender", "SYSTEM_ID"]

#: MAVLink system id used by the companion node.  It has to differ from
#: ArduPilot's own system id (1) so that the message is not mistaken for
#: the autopilot's own component.
#:
#: It does have to match ``SYSID_MYGCS`` (255 by default), because
#: ``GCS_MAVLINK::accept_packet()`` only accepts packets from the GCS system
#: id.  Without that, commands such as ``SET_GPS_GLOBAL_ORIGIN`` are
#: silently dropped.
SYSTEM_ID = 255

#: MAVLink component id of the companion node.  It matches the ESP32 tag
#: (``MAV_COMP_ID_ONBOARD_COMPUTER``) so that both ends of the link are
#: indistinguishable at the framing level.
COMPONENT_ID = 191


class MavlinkSender:
    """Sends MAVLink messages to an ArduPilot SITL serial port over TCP."""

    def __init__(
        self,
        host: str = "127.0.0.1",
        port: int = 5761,
        reconnect_delay: float = 1.0,
        connect_timeout: float = 2.0,
        on_message=None,
    ) -> None:
        self.host = host
        self.port = port
        self.reconnect_delay = reconnect_delay
        self.connect_timeout = connect_timeout
        #: Optional callback invoked with every MAVLink message the
        #: autopilot sends back over this link.
        self._on_message = on_message

        self._sock: socket.socket | None = None
        self._lock = threading.Lock()
        self._last_attempt = 0.0
        self._mav = None
        self._parser = None
        self._reader: threading.Thread | None = None
        self._stop = threading.Event()

        #: Number of messages handed to the transport.
        self.messages_sent = 0
        #: Number of messages dropped because the socket was not writable.
        self.messages_dropped = 0
        #: Number of successful connections.
        self.connections = 0

    # -- connection -----------------------------------------------------

    @property
    def connected(self) -> bool:
        """True when a socket is currently open."""
        with self._lock:
            return self._sock is not None

    def _open(self) -> bool:
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
        self._start_reader(sock)
        return True

    def _start_reader(self, sock: socket.socket) -> None:
        """Drain the socket in the background.

        ArduPilot sends its own telemetry over the same link.  Nothing reads
        it here, so the kernel receive buffer would fill up and the
        autopilot's writes would eventually block.
        """
        self._stop.clear()
        self._reader = threading.Thread(
            target=self._read_loop,
            args=(sock,),
            name="mavlink-drain",
            daemon=True,
        )
        self._reader.start()

    def _read_loop(self, sock: socket.socket) -> None:
        """Read and discard whatever the autopilot sends."""
        while not self._stop.is_set():
            try:
                data = sock.recv(4096)
            except socket.timeout:
                continue
            except OSError:
                break
            if not data:
                break
            self._dispatch_incoming(data)

    def _dispatch_incoming(self, data: bytes) -> None:
        """Parse incoming MAVLink frames and hand them to the callback.

        The autopilot answers the commands the companion sends with
        ``COMMAND_ACK`` messages, and it sends ``HOME_POSITION`` once the
        home position is known.  Both are needed to confirm that the origin
        and home declarations were accepted, so the frames are parsed
        instead of being discarded.
        """
        if self._on_message is None:
            return

        from pymavlink.dialects.v20 import common as mavlink

        if self._parser is None:
            self._parser = mavlink.MAVLink(None, srcSystem=1, srcComponent=1)

        try:
            messages = self._parser.parse_char(data) or []
        except Exception:  # noqa: BLE001 - a malformed frame must not kill the link
            return

        for message in messages:
            try:
                self._on_message(message)
            except Exception:  # noqa: BLE001 - a bad callback must not kill the link
                continue

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
        self._stop.set()
        with self._lock:
            sock = self._sock
            self._sock = None
        if sock is not None:
            try:
                sock.close()
            except OSError:
                pass
        reader = self._reader
        self._reader = None
        if reader is not None and reader.is_alive():
            reader.join(timeout=1.0)

    # -- sending --------------------------------------------------------

    def send(self, message) -> bool:
        """Send a MAVLink message, reconnecting if necessary.

        Args:
            message: A ``pymavlink`` message instance.

        Returns:
            True if the message was written to the socket.
        """
        if message is None:
            return False

        if not self.connected:
            # Rate limit reconnection attempts.
            now = time.time()
            if now - self._last_attempt < self.reconnect_delay:
                self.messages_dropped += 1
                return False
            self._last_attempt = now
            if not self._open():
                self.messages_dropped += 1
                return False

        with self._lock:
            sock = self._sock

        if sock is None:
            self.messages_dropped += 1
            return False

        try:
            data = self._encode(message)
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
            self.messages_dropped += 1
            return False

        self.messages_sent += 1
        return True

    def _encode(self, message) -> bytes:
        """Pack a MAVLink message into a wire frame.

        ``pymavlink``'s ``MAVLink.send()`` writes straight to a file object,
        which does not fit a plain socket, so the message is packed by hand
        instead.  The sequence number lives on the ``MAVLink`` object and has
        to be advanced here.
        """
        if self._mav is None:
            from pymavlink.dialects.v20 import common as mavlink

            self._mav = mavlink.MAVLink(
                None,
                srcSystem=SYSTEM_ID,
                srcComponent=COMPONENT_ID,
            )

        data = message.pack(self._mav)
        self._mav.seq = (self._mav.seq + 1) % 256
        return bytes(data)

    # -- housekeeping ---------------------------------------------------

    def send_heartbeat(self) -> bool:
        """Send a GCS heartbeat to the autopilot.

        ArduPilot uses the heartbeat to associate a system id with a link.
        Sending one before any command makes sure that the commands are
        accepted as coming from the GCS system id.
        """
        from pymavlink.dialects.v20 import common as mavlink

        message = mavlink.MAVLink_heartbeat_message(
            mavlink.MAV_TYPE_GCS,
            mavlink.MAV_AUTOPILOT_INVALID,
            0,
            0,
            mavlink.MAV_STATE_ACTIVE,
            3,
        )
        return self.send(message)
