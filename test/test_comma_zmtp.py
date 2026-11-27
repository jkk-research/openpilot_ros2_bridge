#!/usr/bin/env python3
"""Interop tests for scripts/comma_zmtp.py against libzmq, through pyzmq.

comma_zmtp.py speaks ZeroMQ on the Comma, where newer openpilot has no pyzmq. The PC side is libzmq (pyzmq), so this
checks the two against each other. It needs only pyzmq, which the bridge venv has:
  .venv/bin/python3 test/test_comma_zmtp.py
"""

import os
import selectors
import socket
import struct
import sys
import threading
import time
import unittest

import zmq
from zmq.utils.monitor import recv_monitor_message

sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'scripts'))
import comma_zmtp  # noqa: E402


def free_port():
  with socket.socket() as s:
    s.bind(('127.0.0.1', 0))
    return s.getsockname()[1]


def wait_for(cond, timeout):
  end = time.monotonic() + timeout
  while time.monotonic() < end:
    if cond():
      return True
    time.sleep(0.01)
  return cond()


def recv_all(sock, n, timeout):
  got = []
  end = time.monotonic() + timeout
  while len(got) < n and time.monotonic() < end:
    if sock.poll(50):
      got.append(sock.recv())
  return got


def monitor_events(mon):
  events = []
  while mon.poll(10):
    events.append(recv_monitor_message(mon)['event'])
  return events


class PubLoop:
  """A comma_zmtp.Publisher in a thread with a selectors loop, as comma_bridge.py runs it. `frozen` stops it cold."""

  def __init__(self, port, **kwargs):
    self.pub = comma_zmtp.Publisher(port, host='127.0.0.1', **kwargs)
    self.lock = threading.Lock()
    self.outbox = []
    self.stop = False
    self.frozen = False
    self.thread = threading.Thread(target=self.run, daemon=True)
    self.thread.start()

  def send(self, data):
    with self.lock:
      self.outbox.append(data)

  def ready_count(self):
    return len(self.pub.ready_peers())

  def run(self):
    sel = selectors.DefaultSelector()
    sel.register(self.pub, selectors.EVENT_READ, None)
    registered = {}
    while not self.stop:
      if self.frozen:
        time.sleep(0.01)
        continue
      for key, events in sel.select(0.005):
        if key.data is None:
          self.pub.accept()
        else:
          if events & selectors.EVENT_READ:
            key.data.read()
          if events & selectors.EVENT_WRITE and not key.data.closed:
            key.data.write()
      with self.lock:
        box, self.outbox = self.outbox, []
      for data in box:
        self.pub.send(data)
      self.pub.heartbeat(time.monotonic())
      for peer in list(registered):
        if peer.closed:
          try:
            sel.unregister(registered.pop(peer))
          except (KeyError, ValueError):
            pass
      self.pub.sweep()
      for peer in self.pub.peers:
        mask = selectors.EVENT_READ | (selectors.EVENT_WRITE if peer.wants_write else 0)
        if peer not in registered:
          registered[peer] = peer.sock
          sel.register(peer.sock, mask, peer)
        else:
          sel.modify(peer.sock, mask, peer)
    self.pub.close()

  def close(self):
    self.stop = True
    self.thread.join()


class PublisherTest(unittest.TestCase):
  """comma_zmtp.Publisher with libzmq SUB sockets, as on the PC."""

  def setUp(self):
    self.ctx = zmq.Context()
    self.port = free_port()
    self.loops = []
    self.socks = []

  def tearDown(self):
    for loop in self.loops:
      loop.close()
    for sock in self.socks:
      sock.close(0)
    self.ctx.destroy(linger=0)

  def loop(self, **kwargs):
    loop = PubLoop(self.port, **kwargs)
    self.loops.append(loop)
    return loop

  def sub(self, **options):
    sock = self.ctx.socket(zmq.SUB)
    sock.setsockopt(zmq.SUBSCRIBE, b'')
    for name, value in options.items():
      sock.setsockopt(getattr(zmq, name), value)
    sock.connect(f'tcp://127.0.0.1:{self.port}')
    self.socks.append(sock)
    return sock

  def test_sizes_and_order(self):
    loop = self.loop()
    sub = self.sub()
    self.assertTrue(wait_for(lambda: loop.ready_count() == 1, 5))
    payloads = [os.urandom(n) for n in (0, 1, 254, 255, 256, 257, 65535, 65536, 65537, 1 << 20, 5 << 20, 3)]
    for p in payloads:
      loop.send(p)
    self.assertEqual(recv_all(sub, len(payloads), 10), payloads)
    small = [i.to_bytes(4, 'little') * 50 for i in range(3000)]
    for m in small:
      loop.send(m)
    self.assertEqual(recv_all(sub, len(small), 10), small)
    sub.close(0)
    self.assertTrue(wait_for(lambda: not loop.pub.peers, 5), 'the closed SUB is noticed')

  def test_stalled_subscriber_loses_whole_messages_only(self):
    loop = self.loop(max_messages=100, max_bytes=1 << 20)
    fast = self.sub()
    slow = self.sub(RCVHWM=1, RCVBUF=4096)
    self.assertTrue(wait_for(lambda: loop.ready_count() == 2, 5))
    payload = os.urandom(200 << 10)
    n = 400
    got = {}
    reader = threading.Thread(target=lambda: got.update(fast=recv_all(fast, n, 30)))
    reader.start()
    for i in range(n):
      loop.send(i.to_bytes(4, 'little') + payload)
      time.sleep(0.002)
    reader.join()
    self.assertEqual([int.from_bytes(m[:4], 'little') for m in got['fast']], list(range(n)), 'the fast SUB gets all')
    self.assertGreater(loop.pub.dropped, 0, 'messages for the stalled SUB are dropped, not queued without bound')
    late = recv_all(slow, n, 3)
    self.assertTrue(late and all(m[4:] == payload for m in late), 'the stalled SUB gets whole messages only')

  def test_heartbeating_sub_stays_through_silence(self):
    loop = self.loop()
    sub = self.sub(HEARTBEAT_IVL=200, HEARTBEAT_TIMEOUT=1000)
    mon = sub.get_monitor_socket(zmq.EVENT_DISCONNECTED)
    self.assertTrue(wait_for(lambda: loop.ready_count() == 1, 5))
    time.sleep(3)
    self.assertNotIn(zmq.EVENT_DISCONNECTED, monitor_events(mon))
    loop.send(b'after silence')
    self.assertEqual(recv_all(sub, 1, 2), [b'after silence'])

  def test_heartbeating_sub_stays_through_backlog(self):
    loop = self.loop(max_messages=1000, max_bytes=64 << 20)
    sub = self.sub(RCVHWM=2, RCVBUF=65536, HEARTBEAT_IVL=300, HEARTBEAT_TIMEOUT=1500)
    mon = sub.get_monitor_socket(zmq.EVENT_DISCONNECTED)
    self.assertTrue(wait_for(lambda: loop.ready_count() == 1, 5))
    payload = os.urandom(256 << 10)
    for i in range(100):  # 25 MB queued for a SUB that reads slowly: PONGs have to get ahead of it
      loop.send(i.to_bytes(4, 'little') + payload)
    end = time.monotonic() + 6
    while time.monotonic() < end:
      if sub.poll(50):
        sub.recv()
        time.sleep(0.05)
    self.assertNotIn(zmq.EVENT_DISCONNECTED, monitor_events(mon))

  def test_plain_sub_drops_frozen_publisher_and_reconnects(self):
    # Like openpilot's C++ bridge: no heartbeats of its own, so only the publisher's PING TTL can tell it
    old_ttl = comma_zmtp.PING_TTL
    comma_zmtp.PING_TTL = 20  # 2 s
    try:
      loop = self.loop()
      sub = self.sub(RECONNECT_IVL_MAX=500)
      mon = sub.get_monitor_socket(zmq.EVENT_DISCONNECTED)
      self.assertTrue(wait_for(lambda: loop.ready_count() == 1, 5))
      time.sleep(3)
      self.assertNotIn(zmq.EVENT_DISCONNECTED, monitor_events(mon), 'stays connected while the publisher lives')
      loop.frozen = True
      time.sleep(4)
      loop.frozen = False
      self.assertIn(zmq.EVENT_DISCONNECTED, monitor_events(mon), 'drops the frozen publisher')
      self.assertTrue(wait_for(lambda: loop.ready_count() >= 1, 5), 'and connects again')
      loop.send(b'back')
      self.assertEqual(recv_all(sub, 1, 3), [b'back'])
    finally:
      comma_zmtp.PING_TTL = old_ttl

  def test_conflate_connect_first_and_publisher_restart(self):
    sub = self.sub(CONFLATE=1, RECONNECT_IVL_MAX=500)  # connects before the publisher exists
    time.sleep(0.5)
    loop = self.loop()
    self.assertTrue(wait_for(lambda: loop.ready_count() == 1, 5))
    for i in range(50):
      loop.send(b'msg%d' % i)
    time.sleep(0.3)
    self.assertEqual(recv_all(sub, 5, 0.5), [b'msg49'])
    loop.close()
    self.loops.remove(loop)
    time.sleep(0.3)
    loop = self.loop()  # the Comma side restarted
    self.assertTrue(wait_for(lambda: loop.ready_count() == 1, 5))
    loop.send(b'again')
    self.assertEqual(recv_all(sub, 1, 2), [b'again'])

  def test_connections_reset_before_accept(self):
    # A client that resets its connection (RST) before the publisher accepts it must not break the publisher
    loop = self.loop()
    loop.frozen = True  # so the resets happen before accept() runs
    for _ in range(20):
      s = socket.create_connection(('127.0.0.1', self.port))
      s.setsockopt(socket.SOL_SOCKET, socket.SO_LINGER, struct.pack('ii', 1, 0))
      s.close()
    loop.frozen = False
    sub = self.sub()
    self.assertTrue(wait_for(lambda: loop.ready_count() == 1, 5))
    loop.send(b'still here')
    self.assertEqual(recv_all(sub, 1, 2), [b'still here'])
    self.assertTrue(loop.thread.is_alive())

  def test_non_zmtp_and_silent_clients_are_dropped(self):
    loop = self.loop(handshake_timeout=1.0)
    with socket.create_connection(('127.0.0.1', self.port)) as s:
      s.sendall(b'GET / HTTP/1.1\r\n\r\n')
      self.assertTrue(wait_for(lambda: not loop.pub.peers, 3))
    with socket.create_connection(('127.0.0.1', self.port)):
      self.assertTrue(wait_for(lambda: len(loop.pub.peers) == 1, 2))
      self.assertTrue(wait_for(lambda: not loop.pub.peers, 3))

  def test_big_messages_from_peers_close_the_connection(self):
    # A SUB peer sends only subscriptions; whatever else it sends must not pile up on the Comma, in one frame or many
    loop = self.loop()
    for frames in ([comma_zmtp.PEER_MESSAGE + 1], [comma_zmtp.PEER_MESSAGE // 4] * 5):
      with socket.create_connection(('127.0.0.1', self.port)) as s:
        s.sendall(comma_zmtp.GREETING + comma_zmtp.ready_command(b'SUB'))
        self.assertTrue(wait_for(lambda: loop.ready_count() == 1, 2))
        for i, size in enumerate(frames):
          more = comma_zmtp.MORE if i < len(frames) - 1 else 0
          s.sendall(comma_zmtp.frame_header(size, more) + bytes(size))
        self.assertTrue(wait_for(lambda: not loop.pub.peers, 3), f'{len(frames)} frame(s) of {frames[0]} bytes')
    sub = self.sub()
    self.assertTrue(wait_for(lambda: loop.ready_count() == 1, 5))
    loop.send(b'still here')
    self.assertEqual(recv_all(sub, 1, 2), [b'still here'])


class SubscriberTest(unittest.TestCase):
  """comma_zmtp.Subscriber with a libzmq PUB socket, as the joystick relay uses it."""

  def setUp(self):
    self.ctx = zmq.Context()
    self.port = free_port()
    self.pub = self.bind()

  def tearDown(self):
    self.pub.close(0)
    self.ctx.destroy(linger=0)

  def bind(self):
    pub = self.ctx.socket(zmq.PUB)
    pub.setsockopt(zmq.LINGER, 0)
    pub.bind(f'tcp://127.0.0.1:{self.port}')
    return pub

  def connected(self, sub):
    return wait_for(lambda: (sub.recv(0.05), sub.conn is not None and sub.conn.ready)[1], 3)

  def test_sizes_and_publisher_restart(self):
    sub = comma_zmtp.Subscriber('127.0.0.1', self.port, reconnect=0.2, silence=1.0)
    got = []
    stop = threading.Event()

    def reader():
      while not stop.is_set():
        m = sub.recv(0.1)
        if m is not None:
          got.append(m)
    t = threading.Thread(target=reader, daemon=True)
    t.start()
    try:
      time.sleep(0.5)  # a libzmq PUB sends only once the subscription has arrived
      payloads = [os.urandom(n) for n in (0, 1, 255, 256, 70000, 2 << 20)]
      for p in payloads:
        self.pub.send(p)
      self.assertTrue(wait_for(lambda: len(got) >= len(payloads), 5))
      self.assertEqual(got[:len(payloads)], payloads)
      self.pub.close(0)  # the PC side restarts
      time.sleep(0.3)
      self.pub = self.bind()
      got.clear()
      end = time.monotonic() + 5
      while time.monotonic() < end and not got:
        self.pub.send(b'back')
        time.sleep(0.05)
      self.assertTrue(got, 'it connects again')
    finally:
      stop.set()
      t.join()
      sub.close()

  def test_conflate(self):
    sub = comma_zmtp.Subscriber('127.0.0.1', self.port, conflate=True, reconnect=0.2)
    self.assertTrue(self.connected(sub))
    time.sleep(0.2)
    for i in range(100):
      self.pub.send(b'c%d' % i)
    time.sleep(0.2)
    self.assertEqual(sub.recv(0.5), b'c99')
    self.assertIsNone(sub.recv(0.2))
    sub.close()

  def test_silent_connection_is_made_again(self):
    sub = comma_zmtp.Subscriber('127.0.0.1', self.port, conflate=True, reconnect=0.2, silence=0.5)
    self.assertTrue(self.connected(sub))
    before = sub.conn
    sub.recv(1.0)
    self.assertIsNot(sub.conn, before)
    sub.close()

  def test_unresolvable_host_is_retried(self):
    # libzmq keeps trying a host name that doesn't resolve; so must the relay's receiver, rather than raise
    sub = comma_zmtp.Subscriber('pc-name.invalid', self.port, reconnect=0.1)
    self.assertIsNone(sub.recv(0.5))
    self.assertIsNone(sub.sock)
    sub.close()


if __name__ == '__main__':
  unittest.main(verbosity=2)
