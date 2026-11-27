#!/usr/bin/env python3
"""Tests of scripts/comma_bridge.py's checks of openpilot's msgq queues, on queue files written here. Needs no openpilot.

Run: python3 test/test_comma_bridge.py
"""
import os
import sys
import threading
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'scripts'))
import comma_bridge  # noqa: E402

QUEUE_SIZE = 4096


class Service:
  def __init__(self, name, queue_size=QUEUE_SIZE):
    self.name = name
    self.queue_size = queue_size
    self.uid = None
    self.readers = None
    self.cleared = []


class ReaderSlotsTest(unittest.TestCase):
  """reader_slots() and why_not_read() on msgq queue files with the header layout of openpilot's msgq."""

  def setUp(self):
    self.name = f'cbtest_{os.getpid()}'
    self.path = f'/dev/shm/msgq_{self.name}'
    self.stop = threading.Event()
    self.threads = []

  def tearDown(self):
    self.stop.set()
    for t in self.threads:
      t.join()
    try:
      os.remove(self.path)
    except OSError:
      pass

  def live_tid(self):
    """The thread id of a thread that keeps running until the test ends: a live reader."""
    tids = []
    ready = threading.Event()

    def run():
      tids.append(threading.get_native_id())
      ready.set()
      self.stop.wait()
    t = threading.Thread(target=run, daemon=True)
    t.start()
    ready.wait()
    self.threads.append(t)
    return tids[0]

  def write_queue(self, total, uids, segment=QUEUE_SIZE, extra=0, readers=None):
    readers = len(uids) if readers is None else readers
    header = readers.to_bytes(8, 'little') + bytes(16) + bytes(16 * total)
    header += b''.join(uid.to_bytes(8, 'little') for uid in uids) + bytes(8 * (total - len(uids)) + extra)
    with open(self.path, 'wb') as fh:
      fh.write(header + bytes(segment))

  def test_no_queue_yet(self):
    self.assertIsNone(comma_bridge.reader_slots(self.name, QUEUE_SIZE))
    self.assertIsNone(comma_bridge.why_not_read(Service(self.name), None))

  def test_free_slots(self):
    for total in (15, 25):
      self.write_queue(total, [(7 << 32) | self.live_tid()])
      slots = comma_bridge.reader_slots(self.name, QUEUE_SIZE)
      self.assertEqual(slots, comma_bridge.Slots(readers=1, total=total, used=1, live=None, ours=None))
      self.assertIsNone(comma_bridge.why_not_read(Service(self.name), slots))

  def test_all_slots_live_holds_back(self):
    for total in (15, 25):
      self.write_queue(total, [(i << 32) | self.live_tid() for i in range(total)])
      slots = comma_bridge.reader_slots(self.name, QUEUE_SIZE)
      self.assertEqual(slots, comma_bridge.Slots(readers=total, total=total, used=total, live=total, ours=None))
      self.assertIn('evicting', comma_bridge.why_not_read(Service(self.name), slots))

  def test_stale_slots_dont_hold_back(self):
    # Readers that are gone keep their slots; one more reader evicts once and clears them out
    gone = 2 ** 22 + 12345  # above any pid this test machine hands out
    self.write_queue(15, [(1 << 32) | gone] + [(i << 32) | self.live_tid() for i in range(14)])
    slots = comma_bridge.reader_slots(self.name, QUEUE_SIZE)
    self.assertEqual((slots.used, slots.total, slots.live), (15, 15, 14))
    self.assertIsNone(comma_bridge.why_not_read(Service(self.name), slots))

  def test_own_slot(self):
    # It reads in its main thread, whose thread id is the pid; the uid's high bits change with every subscription
    uid = (0xABCDEF << 32) | os.getpid()
    self.write_queue(15, [(i << 32) | self.live_tid() for i in range(14)] + [uid])
    slots = comma_bridge.reader_slots(self.name, QUEUE_SIZE)
    self.assertEqual(slots.ours, uid)
    self.assertIsNone(comma_bridge.why_not_read(Service(self.name), slots))

  def test_size_mismatch_refused(self):
    # A queue made with another size than this checkout's: subscribing would resize it under its publisher
    self.write_queue(15, [], segment=QUEUE_SIZE * 2)
    slots = comma_bridge.reader_slots(self.name, QUEUE_SIZE)
    self.assertEqual(slots, comma_bridge.MISMATCH)
    self.assertIn('size', comma_bridge.why_not_read(Service(self.name), slots))
    self.assertEqual(os.path.getsize(self.path), 24 + 24 * 15 + QUEUE_SIZE * 2, 'the file is left as it is')

  def test_unknown_layout(self):
    # A header layout it doesn't know isn't a size mismatch, and no reason to hold back: msgq's pending remove-signals
    # change adds 32 words after the slots, and openpilot before 0.11.2 gives no queue sizes
    self.write_queue(25, [], extra=8 * 32, readers=3)
    slots = comma_bridge.reader_slots(self.name, QUEUE_SIZE)
    self.assertEqual(slots, comma_bridge.Slots(readers=3, total=None, used=None, live=None, ours=None))
    self.assertIsNone(comma_bridge.why_not_read(Service(self.name), slots))
    self.write_queue(15, [], segment=12345)
    self.assertEqual(comma_bridge.reader_slots(self.name, 0).total, None)


class EvictionsTest(unittest.TestCase):
  """evictions_seen(): msgq clearing a queue's slots again and again means its readers keep evicting each other."""

  def slots(self, readers, ours=None):
    return comma_bridge.Slots(readers=readers, total=15, used=min(readers, 15), live=None, ours=ours)

  def test_one_clearing_is_no_storm(self):
    # A publisher that restarts clears the slots once; this reader then subscribes again with a new uid
    s = Service('x')
    self.assertFalse(comma_bridge.evictions_seen(s, self.slots(15, ours=1), 0.0))
    self.assertFalse(comma_bridge.evictions_seen(s, self.slots(2), 0.1))  # cleared, not subscribed again yet
    self.assertFalse(comma_bridge.evictions_seen(s, self.slots(14, ours=2), 0.2))  # the same clearing
    self.assertFalse(comma_bridge.evictions_seen(s, self.slots(15, ours=2), 0.3))
    self.assertEqual(len(s.cleared), 1)

  def test_clearings_in_a_row_are_a_storm(self):
    s = Service('x')
    comma_bridge.evictions_seen(s, self.slots(15, ours=1), 0.0)
    self.assertFalse(comma_bridge.evictions_seen(s, self.slots(15, ours=2), 0.1))  # num_readers back up already
    self.assertTrue(comma_bridge.evictions_seen(s, self.slots(9, ours=3), 0.2))

  def test_slow_storm(self):
    s = Service('x')
    comma_bridge.evictions_seen(s, self.slots(15), 0.0)
    self.assertFalse(comma_bridge.evictions_seen(s, self.slots(4), 1.0))
    self.assertFalse(comma_bridge.evictions_seen(s, self.slots(2), 2.5))
    self.assertTrue(comma_bridge.evictions_seen(s, self.slots(1), 4.0))

  def test_clearings_far_apart_are_no_storm(self):
    s = Service('x')
    comma_bridge.evictions_seen(s, self.slots(15), 0.0)
    for t in range(1, 30, 3):
      self.assertFalse(comma_bridge.evictions_seen(s, self.slots(3), float(t)))
      self.assertFalse(comma_bridge.evictions_seen(s, self.slots(15), t + 0.5))

  def test_unknown_layout_still_watched(self):
    s = Service('x')
    unknown = [comma_bridge.Slots(readers=r, total=None, used=None, live=None, ours=None) for r in (25, 3, 1)]
    self.assertFalse(comma_bridge.evictions_seen(s, unknown[0], 0.0))
    self.assertFalse(comma_bridge.evictions_seen(s, unknown[1], 0.1))
    self.assertTrue(comma_bridge.evictions_seen(s, unknown[2], 0.2))


class PortTest(unittest.TestCase):
  def test_ports_match_the_cereal_bridge(self):
    # testJoystick's port is documented in the README; the others are what openpilot's bridge_zmq.cc gives
    self.assertEqual(comma_bridge.bridge_port('testJoystick'), 57078)
    self.assertEqual(comma_bridge.bridge_port('carState'), 9041)


if __name__ == '__main__':
  unittest.main(verbosity=2)
