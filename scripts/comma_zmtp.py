"""A minimal ZeroMQ publisher and subscriber in plain Python, for the Comma.

The ROS 2 bridge talks to the PC the way openpilot's cereal bridge did: one ZeroMQ PUB socket per openpilot service,
on a TCP port derived from the service name, carrying each message as a single frame. Newer openpilot has no such
bridge, and dropped pyzmq from its dependencies, so the Comma side speaks the protocol itself: ZMTP 3.1 with the NULL mechanism
(https://rfc.zeromq.org/spec/37/), which is all a libzmq peer needs. It never sends ERROR, and nothing but the
greeting and READY before the peer's READY: libzmq stops reconnecting for good after either, while a closed
connection just makes it connect again.

- Publisher is what zmq_bind() of a PUB socket on tcp://*:<port> does. libzmq SUB sockets connect to it and get
  every message: their subscriptions are read and ignored (a libzmq SUB filters by itself as well). A peer that
  falls behind loses whole messages, never part of one, like a libzmq PUB at its high-water mark. Its PINGs let
  peers notice when it's gone.
- Subscriber is what zmq_connect() of a SUB socket subscribed to everything does. It connects again when the
  connection drops, or when nothing arrives for a while from a peer that sends all the time.

Only the standard library is used, so this runs with any Python 3.8 or newer.
"""

import errno
import select
import socket
import struct
import time
from collections import deque

MORE = 0x01
LONG = 0x02
COMMAND = 0x04

# ZMTP 3.1 like libzmq 4.3.3 and later. The signature's padding carries libzmq's (routing id size + 1) for old ZMTP 1.0
# peers, which libzmq sets to 1.
GREETING = b'\xff' + (1).to_bytes(8, 'big') + b'\x7f' + bytes([3, 1]) + b'NULL'.ljust(20, b'\x00') + b'\x00' * 32
GREETING_SIZE = len(GREETING)  # 64
MAX_MESSAGE = 64 << 20  # a larger incoming frame or message closes the connection
PEER_MESSAGE = 64 << 10  # the same for what a publisher's peers send: subscriptions, nothing big
PING_INTERVAL = 1.0  # s between the publisher's PINGs
# The PINGs ask the peer to drop the connection after this long without hearing from us (in 0.1 s; libzmq computes
# ms in 16 bits, so at most 655). Long enough for the largest frame to arrive over a slow link.
PING_TTL = 100


OUT_HANDSHAKE, OUT_COMMAND, OUT_MESSAGE = range(3)  # kinds of outgoing frames


class ProtocolError(Exception):
  pass


def frame_header(size, flags=0):
  if size < 256:
    return bytes((flags, size))
  return bytes((flags | LONG,)) + size.to_bytes(8, 'big')


def command(name, body=b''):
  payload = bytes((len(name),)) + name + body
  return frame_header(len(payload), COMMAND) + payload


def ready_command(socket_type):
  name = b'Socket-Type'
  return command(b'READY', bytes((len(name),)) + name + struct.pack('>I', len(socket_type)) + socket_type)


def parse_properties(data):
  props = {}
  i = 0
  while i < len(data):
    n = data[i]
    name = bytes(data[i + 1:i + 1 + n])
    i += 1 + n
    if i + 4 > len(data):
      raise ProtocolError('truncated property')
    size = int.from_bytes(data[i:i + 4], 'big')
    props[name.lower()] = bytes(data[i + 4:i + 4 + size])
    i += 4 + size
  if i != len(data):
    raise ProtocolError('truncated property')
  return props


class Connection:
  """One ZMTP connection over a connected TCP socket: handshake, framing and an outgoing queue."""

  def __init__(self, sock, socket_type, peer_types, max_messages=1000, max_bytes=8 << 20, max_message=MAX_MESSAGE):
    self.sock = sock
    sock.setblocking(False)
    self.socket_type = socket_type
    self.peer_types = peer_types
    self.max_messages = max_messages
    self.max_bytes = max_bytes
    self.max_message = max_message
    self.inbuf = bytearray()
    self.out = deque()  # frames to send: [list of memoryviews, size, kind], kind one of OUT_*
    self.head_sent = 0  # bytes of the first frame already sent
    self.out_bytes = 0
    self.out_messages = 0
    self.greeted = False  # the peer's greeting arrived
    self.ready = False  # handshake done, messages may flow
    self.closed = False
    self.error = None
    self.parts = []
    self.parts_size = 0
    self.created = time.monotonic()
    self.last_received = self.created
    self._queue([GREETING], OUT_HANDSHAKE)
    self.write()

  def fileno(self):
    return self.sock.fileno()

  @property
  def wants_write(self):
    return bool(self.out) and not self.closed

  def close(self, error=None):
    if not self.closed:
      self.closed = True
      self.error = error
      self.out.clear()
      try:
        self.sock.close()
      except OSError:
        pass

  def _queue(self, parts, kind):
    parts = [memoryview(part) for part in parts if len(part)]
    size = sum(len(part) for part in parts)
    entry = [parts, size, kind]
    if kind == OUT_COMMAND:
      # Ahead of the messages still waiting, as libzmq does, but after the handshake and never into a frame that has
      # started going out
      i = 1 if self.head_sent else 0
      while i < len(self.out) and self.out[i][2] != OUT_MESSAGE:
        i += 1
      self.out.insert(i, entry)
    else:
      self.out.append(entry)
    self.out_bytes += size
    if kind == OUT_MESSAGE:
      self.out_messages += 1

  def send_command(self, data):
    if not self.closed:
      was_idle = not self.out
      self._queue([data], OUT_COMMAND)
      if was_idle:
        self.write()

  def send_message(self, data):
    """Queues one single-frame message. False if it was dropped because the peer isn't keeping up."""
    if self.closed or not self.ready:
      return False
    size = len(data)
    if self.out_messages >= self.max_messages or (self.out_messages and self.out_bytes + size > self.max_bytes):
      return False
    was_idle = not self.out
    self._queue([frame_header(size), data], OUT_MESSAGE)
    if was_idle:
      self.write()
    return True

  def write(self):
    """Sends what the socket takes now. Returns False once the connection is closed."""
    while self.out and not self.closed:
      parts, size, kind = self.out[0]
      views, skip = [], self.head_sent
      for part in parts:
        if skip >= len(part):
          skip -= len(part)
        else:
          views.append(part[skip:] if skip else part)
          skip = 0
      try:
        sent = self.sock.sendmsg(views) if len(views) > 1 else self.sock.send(views[0])
      except (BlockingIOError, InterruptedError):
        return True
      except OSError as exc:
        self.close(exc)
        return False
      self.out_bytes -= sent
      self.head_sent += sent
      if self.head_sent < size:
        return True
      self.out.popleft()
      self.head_sent = 0
      if kind == OUT_MESSAGE:
        self.out_messages -= 1
    return not self.closed

  def read(self):
    """Reads what arrived and returns the complete messages in it. Commands are handled here."""
    messages = []
    while not self.closed:
      try:
        data = self.sock.recv(1 << 16)
      except (BlockingIOError, InterruptedError):
        break
      except OSError as exc:
        self.close(exc)
        break
      if not data:
        self.close('closed by the peer')
        break
      self.last_received = time.monotonic()
      self.inbuf += data
      try:
        self._parse(messages)
      except ProtocolError as exc:
        self.close(exc)
      if len(data) < (1 << 16):
        break
    return messages

  def _parse(self, messages):
    buf = self.inbuf
    if not self.greeted:
      if buf[:1] not in (b'', b'\xff'):
        raise ProtocolError('not a ZMTP peer')
      if len(buf) < GREETING_SIZE:
        return
      if buf[9] != 0x7f or buf[10] < 3:
        raise ProtocolError(f'unsupported ZMTP version {buf[10]}.{buf[11]}')
      if bytes(buf[12:32]).rstrip(b'\x00') != b'NULL':
        raise ProtocolError('the peer wants a security mechanism other than NULL')
      del buf[:GREETING_SIZE]
      self.greeted = True
      self._queue([ready_command(self.socket_type)], OUT_HANDSHAKE)
      self.write()
    start = 0
    try:
      while True:
        if len(buf) - start < 2:
          break
        flags = buf[start]
        if flags & ~(MORE | LONG | COMMAND):
          raise ProtocolError('bad frame flags')
        if flags & LONG:
          if len(buf) - start < 9:
            break
          size = int.from_bytes(buf[start + 1:start + 9], 'big')
          head = 9
        else:
          size = buf[start + 1]
          head = 2
        if size > self.max_message:
          raise ProtocolError(f'frame of {size} bytes')
        end = start + head + size
        if len(buf) < end:
          break
        body = bytes(buf[start + head:end])
        start = end
        if flags & COMMAND:
          self._command(body)
        elif not self.ready:
          raise ProtocolError('message before the handshake')
        else:
          self.parts.append(body)
          self.parts_size += len(body)
          if self.parts_size > self.max_message:
            raise ProtocolError(f'message of more than {self.max_message} bytes')
          if not flags & MORE:
            messages.append(self.parts[0] if len(self.parts) == 1 else b''.join(self.parts))
            self.parts = []
            self.parts_size = 0
    finally:
      del buf[:start]

  def _command(self, body):
    if not body or len(body) < 1 + body[0]:
      raise ProtocolError('bad command')
    name = body[1:1 + body[0]]
    data = body[1 + body[0]:]
    if name == b'READY':
      peer_type = parse_properties(data).get(b'socket-type', b'')
      if peer_type not in self.peer_types:
        raise ProtocolError(f'a {peer_type.decode(errors="replace") or "?"} socket can not talk to a '
                            f'{self.socket_type.decode()} socket')
      self.ready = True
      self.on_ready()
    elif name == b'PING':
      # TTL (2 bytes), then up to 16 bytes of context that the PONG returns
      self.send_command(command(b'PONG', data[2:18]))
    elif name == b'ERROR':
      reason = data[1:1 + data[0]].decode(errors='replace') if data else ''
      raise ProtocolError(f'the peer reported an error: {reason}')
    # SUBSCRIBE, CANCEL (ZMTP 3.1) and PONG need nothing

  def on_ready(self):
    pass


class Publisher:
  """The server side of a ZeroMQ PUB socket bound to a TCP port, for event loops built on select or selectors.

  Register fileno() for reading and call accept() when it's readable; register each connection in `peers` for
  reading (and for writing while its wants_write is set) and call its read() and write(). send() queues a message
  for every peer that finished its handshake.
  """

  def __init__(self, port, host='', max_messages=1000, max_bytes=8 << 20, handshake_timeout=10.0):
    self.port = port
    self.max_messages = max_messages
    self.max_bytes = max_bytes
    self.handshake_timeout = handshake_timeout
    self.listener = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    try:
      self.listener.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
      self.listener.bind((host, port))
      self.listener.listen(100)  # libzmq's default ZMQ_BACKLOG
      self.listener.setblocking(False)
    except OSError:
      self.listener.close()
      raise
    self.peers = []
    self.dropped = 0
    self.last_ping = 0.0

  def fileno(self):
    return self.listener.fileno()

  def heartbeat(self, now):
    """PINGs the peers about once a second. The PING's TTL makes a libzmq SUB drop the connection when it stops hearing
    from us, also one without heartbeats of its own (openpilot's C++ bridge), so it connects again once we're back."""
    if now - self.last_ping < PING_INTERVAL:
      return
    self.last_ping = now
    ping = command(b'PING', struct.pack('>H', PING_TTL))
    for peer in self.peers:
      if peer.ready and not peer.closed:
        peer.send_command(ping)

  def accept(self):
    """Accepts the waiting connections and returns them."""
    new = []
    while True:
      try:
        sock, _ = self.listener.accept()
      except (BlockingIOError, InterruptedError):
        break
      except OSError as exc:
        if exc.errno in (errno.EMFILE, errno.ENFILE, errno.ENOBUFS, errno.ENOMEM):
          break
        continue
      try:
        configure_tcp(sock)
      except OSError:
        sock.close()
        continue
      conn = Connection(sock, b'PUB', (b'SUB', b'XSUB'), self.max_messages, self.max_bytes, PEER_MESSAGE)
      # Sending the greeting fails at once on a connection the client already reset: never hand that one out
      if not conn.closed:
        new.append(conn)
    self.peers.extend(new)
    return new

  def ready_peers(self):
    return [peer for peer in self.peers if peer.ready and not peer.closed]

  def send(self, data):
    """Queues one message for every peer past its handshake; returns how many took it."""
    taken = 0
    for peer in self.peers:
      if peer.ready and not peer.closed:
        if peer.send_message(data):
          taken += 1
        else:
          self.dropped += 1
    return taken

  def sweep(self, now=None):
    """Drops closed connections and the ones that never finished their handshake; returns the dropped ones."""
    now = time.monotonic() if now is None else now
    gone = []
    for peer in self.peers:
      if not peer.closed and not peer.ready and now - peer.created > self.handshake_timeout:
        peer.close('no handshake')
      if peer.closed:
        gone.append(peer)
    if gone:
      self.peers = [peer for peer in self.peers if not peer.closed]
    return gone

  def close(self):
    for peer in self.peers:
      peer.close()
    self.peers = []
    self.listener.close()


def configure_tcp(sock):
  sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
  # Notice a peer that went away without closing the connection (a PC that left the Wi-Fi): unanswered data for
  # 10 s, or about 10 s of silence, ends it
  sock.setsockopt(socket.SOL_SOCKET, socket.SO_KEEPALIVE, 1)
  for name, value in (('TCP_USER_TIMEOUT', 10000), ('TCP_KEEPIDLE', 4), ('TCP_KEEPINTVL', 2), ('TCP_KEEPCNT', 3)):
    if hasattr(socket, name):
      try:
        sock.setsockopt(socket.IPPROTO_TCP, getattr(socket, name), value)
      except OSError:
        pass


class Subscriber:
  """The client side of a ZeroMQ SUB socket subscribed to everything, connected to tcp://<host>:<port>.

  It connects again `reconnect` seconds after a connection fails or closes, and drops a connection that has been
  silent for `silence` seconds (for peers that send all the time, as the bridge's joystick sender does).
  With conflate, recv() returns only the newest message, like ZMQ_CONFLATE.
  """

  def __init__(self, host, port, conflate=False, reconnect=0.5, silence=None, max_queued=1000):
    self.address = (host, port)
    self.conflate = conflate
    self.reconnect = reconnect
    self.silence = silence
    self.messages = deque(maxlen=1 if conflate else max_queued)
    self.sock = None
    self.conn = None
    self.next_attempt = 0.0

  def _drop(self, now):
    if self.conn is not None:
      self.conn.close()
    elif self.sock is not None:
      self.sock.close()
    self.sock = self.conn = None
    self.next_attempt = now + self.reconnect

  def _connect(self, now):
    sock = None
    try:
      sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
      sock.setblocking(False)
      err = sock.connect_ex(self.address)
    except OSError:  # a host name that doesn't resolve (socket.gaierror), or no file descriptors left: try again later
      err = -1
    if err not in (0, errno.EINPROGRESS, errno.EWOULDBLOCK, errno.EAGAIN):
      if sock is not None:
        sock.close()
      self.next_attempt = now + self.reconnect
      return
    self.sock = sock

  def _connected(self):
    configure_tcp(self.sock)
    conn = Connection(self.sock, b'SUB', (b'PUB', b'XPUB'))

    def subscribe_all():
      # A message of 0x01 and the prefix, empty for everything: every libzmq PUB takes this form, whatever version
      # the peers speak
      conn._queue([frame_header(1), b'\x01'], OUT_MESSAGE)
      conn.write()
    conn.on_ready = subscribe_all
    self.conn = conn

  def recv(self, timeout):
    """The next message, or None after `timeout` seconds without one."""
    deadline = time.monotonic() + timeout
    while True:
      if self.messages:
        return self.messages.popleft()
      now = time.monotonic()
      if self.sock is None and now >= self.next_attempt:
        self._connect(now)
      if self.conn is not None and self.silence is not None and now - self.conn.last_received > self.silence:
        self._drop(now)
        continue
      if now >= deadline:
        return None
      wait = deadline - now
      if self.sock is None:
        time.sleep(min(wait, max(0.0, self.next_attempt - now)))
        continue
      if self.silence is not None and self.conn is not None:
        wait = min(wait, max(0.0, self.conn.last_received + self.silence - now) + 0.001)
      connecting = self.conn is None
      want_write = connecting or self.conn.wants_write
      try:
        readable, writable, _ = select.select([self.sock], [self.sock] if want_write else [], [], wait)
      except InterruptedError:
        continue
      now = time.monotonic()
      if connecting:
        if not writable and not readable:
          continue
        err = self.sock.getsockopt(socket.SOL_SOCKET, socket.SO_ERROR)
        if err:
          self._drop(now)
          continue
        try:
          self._connected()
        except OSError:
          self._drop(now)
          continue
        if self.conn.closed:  # the greeting couldn't be sent
          self._drop(now)
        continue
      if writable:
        self.conn.write()
      if readable:
        self.messages.extend(self.conn.read())
      if self.conn.closed:
        self._drop(now)

  def close(self):
    self._drop(time.monotonic())
