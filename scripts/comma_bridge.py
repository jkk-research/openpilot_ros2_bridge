#!/usr/bin/env python3
"""Runs on the Comma: serves openpilot's services to the ROS 2 bridge on the PC, in place of openpilot's cereal bridge.

openpilot removed its cereal bridge (cereal/messaging/bridge) in October 2026; 0.11.3 is the first release without it.
This takes its place on every openpilot version and speaks the same protocol, so the PC side works with either: one
ZeroMQ PUB socket per openpilot service, on TCP port 8023 + FNV-1a(name) % 57512, each message as one frame. It
reads the services with openpilot's own messaging library and Python, so it always matches the Comma's msgq version,
and nothing has to be built or installed. ZeroMQ comes from comma_zmtp.py next to it: openpilot dropped pyzmq from
its dependencies in October 2026, so the Comma's Python may not have it.

The bridge launch copies both files to the Comma and starts this over SSH (start_remote_bridge:=true). To start it
by hand: copy scripts/comma_bridge.py and scripts/comma_zmtp.py to the Comma, then run
  ssh -t comma@<comma ip> /usr/local/venv/bin/python3 /tmp/openpilot_ros2_bridge/comma_bridge.py
On a terminal, closing the session (SIGHUP) stops it. If one already runs (another PC's launch started it), a new one
shares it instead: it waits, and takes over once that one ends.

A service is read only while a PC is connected to its port.
"""

import argparse
import collections
import fcntl
import os
import selectors
import signal
import sys
import time
import traceback

# One set per user, so files left by a run as another user (root) can't get in the way
FILES = f'/tmp/openpilot_ros2_comma_bridge_{os.getuid()}'
PIDFILE = FILES + '.pid'
LOCKFILE = FILES + '.lock'  # held while one that starts decides whether to run or to share
# Its pid, how many PCs it serves and when it wrote this, rewritten in place every second (so a full /tmp can't stop
# it), so the next one started can tell a working instance (share it) from one that hangs (replace it)
STATUSFILE = FILES + '.status'
STALE_AFTER = 5.0  # s without a change of its status or CPU time after which the running instance counts as hung
POLL_TIMEOUT = 10  # ms the loop waits for openpilot's messages before it looks at the network anyway
WRITE_WAIT = 0.002  # s the loop waits for sockets, while one has data to send, before it reads the services anyway
REPORT_PERIOD = 1.0  # s over which connects and disconnects are gathered into one line
SLOT_CHECK_PERIOD = 0.1  # s between looks at the msgq queue of each service it reads
# Readers keep evicting each other when they're evicted this often. One eviction clears out the slots of readers that
# are gone (at every drive start, once a drive has left its readers' slots behind), and the next one is far off, unless
# a reader finds every slot in use by running readers again. openpilot's readers poll far more often than that.
STORM_EVICTIONS = 2
STORM_WINDOW = 10.0
IMPORT_RETRY = 10.0  # s between tries to load openpilot's messaging, with --wait-on-error
HOLD_BACK = 10.0  # s before it tries again to read a service it held back
RETRY_PORTS = 5.0  # s between tries to listen on ports that were in use


def bridge_port(endpoint):
  # Same as get_port() in openpilot's cereal/messaging/bridge_zmq.cc
  value = 0xcbf29ce484222325
  for byte in endpoint.encode():
    value ^= byte
    value = (value * 0x100000001b3) & 0xFFFFFFFFFFFFFFFF
  return 8023 + value % (65535 - 8023)


def say(text):
  try:
    print(f"[comma bridge] {text}", flush=True)
  except OSError:
    pass  # the SSH session's terminal is gone; keep going so the cleanup still runs


class CantStart(Exception):
  """An error that starting again won't fix."""


def give_up(message, wait):
  """Reports an error that starting again won't fix. With wait (the launch starts this again whenever it ends) it
  then waits to be stopped, so the Comma doesn't get a new SSH session and Python start every few seconds."""
  say(message)
  if not wait:
    sys.exit(3)
  say("Starting it again wouldn't help; fix that and restart the launch")
  for sig in (signal.SIGHUP, signal.SIGTERM, signal.SIGINT):
    signal.signal(sig, signal.SIG_DFL)
  while True:
    signal.pause()


def running_instance():
  """The pid of another comma_bridge.py that is running, or None."""
  try:
    with open(PIDFILE) as fh:
      pid = int(fh.read())
    with open(f'/proc/{pid}/cmdline', 'rb') as fh:
      running = os.path.basename(__file__).encode() in fh.read()
  except (OSError, ValueError):
    return None
  return pid if running and pid != os.getpid() else None


def instance_status(pid):
  """(PCs served, time written) from the running instance pid's status file, or None when the file isn't its."""
  try:
    with open(STATUSFILE) as fh:
      owner, pcs, stamp = fh.read().split()
    if int(owner) == pid:
      return int(pcs), stamp
  except (OSError, ValueError):
    pass
  return None


def cpu_time(pid):
  """The process's CPU time so far, in clock ticks, or None."""
  try:
    with open(f'/proc/{pid}/stat') as fh:
      fields = fh.read().rsplit(')', 1)[1].split()  # after the command name, which may hold anything
    return int(fields[11]) + int(fields[12])  # utime and stime
  except (OSError, ValueError, IndexError):
    return None


class Status:
  """This instance's status file. Written in place with the same length every time, so it needs no more room on /tmp
  once it exists."""

  def __init__(self):
    self.fd = None
    try:
      self.fd = os.open(STATUSFILE, os.O_RDWR | os.O_CREAT, 0o644)
      os.ftruncate(self.fd, 0)
    except OSError:
      pass
    self.write(0)

  def write(self, pcs):
    if self.fd is not None:
      try:
        os.pwrite(self.fd, f'{os.getpid():>10} {pcs:>5} {time.monotonic():>16.3f}\n'.encode(), 0)
      except OSError:
        pass


def take_over(stopping):
  """Makes this the running instance and returns its Status. One that already runs is shared instead, whether another
  PC's launch started it or it's left from an SSH session that died without the Comma noticing: this waits while it
  runs, which keeps the session that started this open, and takes over once it's gone. Only one that hangs is
  replaced. The lock keeps two that start at once from both taking over."""
  shared = None
  seen = None  # the running instance, its status and CPU time as last read, and since when they're unchanged
  lock = os.open(LOCKFILE, os.O_RDONLY | os.O_CREAT, 0o644)  # flock needs no write access
  try:
    while not stopping:
      fcntl.flock(lock, fcntl.LOCK_EX)
      try:
        pid = running_instance()
        status = instance_status(pid) if pid is not None else None
        now = time.monotonic()
        state = (pid, status, cpu_time(pid) if pid is not None else None)
        if seen is None or seen[0] != state:
          seen = (state, now)
        # Hung when neither its status nor its CPU time change while this watches it (not by comparing times: a Comma
        # frozen and resumed would look hung for a moment)
        if pid is None or now - seen[1] >= STALE_AFTER:
          if pid is not None:
            say(f"Stopping comma_bridge.py (pid {pid}): it stopped working")
            try:
              os.kill(pid, signal.SIGKILL)
            except OSError:
              pass
            for _ in range(50):
              if not os.path.exists(f'/proc/{pid}'):
                break
              time.sleep(0.1)
          try:
            with open(PIDFILE, 'w') as fh:
              fh.write(str(os.getpid()))
          except OSError as exc:
            say(f"Can't write {PIDFILE} ({exc.strerror}): another one started later won't find this one")
          return Status()
      finally:
        fcntl.flock(lock, fcntl.LOCK_UN)
      if pid != shared and status is not None:
        shared = pid
        say(f"comma_bridge.py is already running on the Comma (pid {pid}, serving {status[0]} "
            f"PC{'s' if status[0] != 1 else ''}); using it")
      time.sleep(0.5)
    sys.exit(0)
  finally:
    os.close(lock)


def import_messaging(openpilot_dir):
  # As openpilot's own processes import it (launch_chffrplus.sh puts the checkout on PYTHONPATH, with msgq and opendbc
  # linked in from their submodules), so this uses the Comma's own msgq build, ahead of any msgq package in the
  # Python environment. The submodule paths are for checkouts without those links.
  paths = [openpilot_dir, os.path.join(openpilot_dir, 'msgq_repo'), os.path.join(openpilot_dir, 'opendbc_repo')]
  sys.path[:0] = [path for path in paths if path not in sys.path]
  try:
    from openpilot.cereal import messaging
  except ModuleNotFoundError as exc:
    # openpilot before 0.11.2 keeps cereal at the top; anything else missing is a real error worth seeing as it is
    if exc.name not in ('openpilot', 'openpilot.cereal'):
      raise
    import cereal.messaging as messaging
  # A msgq built for another openpilot version would crash openpilot's own readers, so only the Comma's own will do
  msgq = sys.modules.get('msgq')  # openpilot before 0.9.7 has no msgq package
  if msgq is not None and not os.path.realpath(msgq.__file__).startswith(os.path.realpath(openpilot_dir) + os.sep):
    raise CantStart(f"msgq comes from {msgq.__file__}, not from {openpilot_dir}; not starting")
  return messaging


MISMATCH = 'mismatch'
# readers: the queue's num_readers, which only goes down when msgq clears all slots. writer: the publisher's uid, new
# with every publisher start, which clears the slots too. total: how many slots there are,
# None for a header layout this doesn't know (then used, live and ours are None too). ours: this process's uid when one
# of the used slots is its (it subscribes and reads in its main thread, whose thread id is the pid). live: how many of
# them belong to running threads, only counted when none is ours and all are used.
Slots = collections.namedtuple('Slots', 'readers writer total used live ours')


def reader_slots(name, queue_size):
  """The reader slots of the service's msgq queue as Slots; None when there's no queue yet; MISMATCH when the queue
  was made with another size than this openpilot checkout gives the service. Subscribing to that would resize it
  under its publisher, which then crashes.

  msgq gives every subscriber a slot and never takes it back, so slots of readers that are gone stay used. A
  subscriber that finds all slots used evicts every reader of the queue, openpilot's own included, which then
  subscribe again. That clears out the stale slots, but if every slot belongs to a live reader, it repeats without end.
  The header holds num_readers, the write pointer and the write uid (one uint64 each), then three arrays of one uint64
  per slot: read pointers, valid flags and uids, whose low 32 bits are the reader's thread id. msgq before September
  2026 has 15 slots (a 384 byte header), later 25 (624 bytes).
  """
  prefix = os.environ.get('OPENPILOT_PREFIX', '')
  folder = f'{prefix}/' if prefix else ''
  # msgq since December 2025 names its files msgq_<endpoint>; older versions just <endpoint>
  for path in (f'/dev/shm/msgq_{folder}{name}', f'/dev/shm/{folder}{name}'):
    try:
      size = os.stat(path).st_size
      with open(path, 'rb') as fh:
        header = fh.read(24 + 24 * 25)
    except OSError:
      continue
    if len(header) < 24:
      return None
    readers = int.from_bytes(header[:8], 'little')
    writer = int.from_bytes(header[16:24], 'little')
    # The header is small, while queue sizes differ by far more, so this holds whatever the header's layout
    if queue_size and not 0 < size - queue_size <= 4096:
      return MISMATCH
    # openpilot before 0.11.2 has no queue sizes: then it's msgq's default segment, 10 MiB in older versions, 1 MiB later,
    # the same for every process with this msgq build
    for segment in (queue_size,) if queue_size else (10 << 20, 1 << 20):
      total = (size - segment - 24) // 24
      if (size - segment - 24) % 24 == 0 and total in (15, 25) and len(header) >= 24 + 24 * total:
        break
    else:
      return Slots(readers, writer, None, None, None, None)
    used = min(readers, total)
    uids = [int.from_bytes(header[24 + 16 * total + 8 * i:24 + 16 * total + 8 * i + 8], 'little') for i in range(used)]
    ours = next((uid for uid in uids if uid & 0xFFFFFFFF == os.getpid()), None)
    live = sum(1 for uid in uids if uid & 0xFFFFFFFF and os.path.exists(f'/proc/{uid & 0xFFFFFFFF}')) \
        if ours is None and used >= total else None
    return Slots(readers, writer, total, used, live, ours)
  return None


def why_not_read(service, slots):
  """Why reading the service now would harm openpilot, or None. Taking a reader slot while every slot belongs to a
  running reader evicts openpilot's own readers, again and again. That matters for the first subscription, and
  whenever msgq has cleared the slots since (a publisher restarted, or all readers were evicted): the next read then
  subscribes again by itself."""
  if slots == MISMATCH:
    return ("its msgq queue has another size than this openpilot checkout gives it (was openpilot updated without a "
            "restart?), and subscribing would crash its publisher")
  if isinstance(slots, Slots) and slots.total is not None and slots.ours is None and slots.used >= slots.total and \
      slots.live >= slots.total:
    return f"all {slots.total} of its msgq reader slots belong to running readers, and one more would keep evicting them"
  return None


def evictions_seen(service, slots, now):
  """Notes when the service's readers were evicted since the last look, and returns whether that keeps happening.
  msgq then clears all slots, and each read and poll subscribes this reader again by itself, before anything here can
  look: so an eviction shows as num_readers going down, or as this reader's new uid. A publisher that starts clears
  the slots as well, but gets a new uid of its own; that's no eviction."""
  if not isinstance(slots, Slots):
    return False
  evicted = False
  if service.writer is not None and slots.writer != service.writer:
    service.uid = None  # this reader subscribes again by itself after that
  else:
    if service.readers is not None and slots.readers < service.readers:
      evicted = True
      service.uid = None  # this reader subscribes again by itself: the same eviction
    if slots.ours is not None and service.uid is not None and slots.ours != service.uid:
      evicted = True
  service.writer = slots.writer
  service.readers = slots.readers
  if slots.ours is not None:
    service.uid = slots.ours
  if evicted:
    service.evictions = [t for t in service.evictions if now - t < STORM_WINDOW] + [now]
  return len(service.evictions) >= STORM_EVICTIONS


class Service:
  def __init__(self, name, publisher, queue_size):
    self.name = name
    self.publisher = publisher
    self.queue_size = queue_size
    self.sock = None
    self.active = False
    self.next_try = 0.0  # when to try again to read it after holding back
    self.next_check = 0.0  # when to check its reader slots again while reading it
    self.held_back = False
    self.uid = None  # its msgq reader's uid when last checked; msgq gives it a new one with each subscription
    self.readers = None  # its queue's num_readers when last checked
    self.writer = None  # its publisher's uid when last checked
    self.evictions = []  # when its readers were seen evicted lately


class Reporter:
  """Gathers connects and disconnects per PC address, so a PC connecting to a dozen services gives one line."""

  def __init__(self):
    self.events = {}
    self.since = None

  def add(self, what, address, service):
    self.events.setdefault((what, address), []).append(service)
    if self.since is None:
      self.since = time.monotonic()

  def flush(self, now, force=False):
    if self.since is None or (not force and now - self.since < REPORT_PERIOD):
      return
    for (what, address), services in sorted(self.events.items()):
      say(f"{address} {what} {len(services)} service{'s' if len(services) != 1 else ''}: {', '.join(sorted(services))}")
    self.events = {}
    self.since = None


def peer_address(conn):
  try:
    return conn.sock.getpeername()[0]
  except OSError:
    return 'a PC'


def main():
  parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
  parser.add_argument('--openpilot-dir', default='/data/openpilot')
  parser.add_argument('--services', default='', help='comma-separated services to serve (default: all of them)')
  parser.add_argument('--host', default='', help='address to listen on (default: all)')
  parser.add_argument('--wait-on-error', action='store_true',
                      help="after an error that starting again won't fix, wait to be stopped instead of exiting")
  args = parser.parse_args()

  failed = False
  while True:
    try:
      messaging = import_messaging(args.openpilot_dir)
      break
    except CantStart as exc:
      give_up(str(exc), args.wait_on_error)
    except Exception as exc:
      if not failed and not isinstance(exc, ModuleNotFoundError):
        traceback.print_exc()
      message = f"Can't load openpilot's messaging from {args.openpilot_dir} ({exc}): is openpilot there, and built?"
      if not args.wait_on_error:
        say(message)
        sys.exit(3)
      # It may still be getting built (right after a boot on a source branch): try again by itself rather than make the
      # launch start a new SSH session and Python every few seconds
      if not failed:
        say(f"{message} Trying again every {IMPORT_RETRY:.0f} s")
        failed = True
      time.sleep(IMPORT_RETRY)
  if failed:
    say("Loaded openpilot's messaging")
  sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
  import comma_zmtp

  stopping = []

  def stop(signum, frame):
    stopping.append(signum)
  # SIGHUP: the SSH session closed (Ctrl+C on the launch)
  for sig in (signal.SIGHUP, signal.SIGTERM, signal.SIGINT):
    signal.signal(sig, stop)

  status = take_over(stopping)

  names = [n for n in args.services.split(',') if n] if args.services else list(messaging.SERVICE_LIST)
  services = []
  ports = {}
  busy = []
  # Services whose port was in use, tried again every RETRY_PORTS s. Even with SO_REUSEADDR a port can't be listened on
  # while it's the local end of an outgoing connection, and many of them lie in the range those are taken from.
  waiting = []
  for name in names:
    if name not in messaging.SERVICE_LIST:
      say(f"This openpilot has no {name} service")
      continue
    port = bridge_port(name)
    if port in ports:
      say(f"{name} would use port {port} like {ports[port]}, so it's left out")
      continue
    ports[port] = name
    queue_size = getattr(messaging.SERVICE_LIST[name], 'queue_size', 0)
    try:
      services.append(Service(name, comma_zmtp.Publisher(port, host=args.host), queue_size))
    except OSError as exc:
      waiting.append((name, port, queue_size))
      busy.append(f"{name} ({port}: {exc.strerror})")
  if busy:
    hint = " (is openpilot's cereal bridge running?)" if not services else ""
    say(f"Can't listen for {len(busy)} service{'s' if len(busy) != 1 else ''} yet, "
        f"{'their ports are' if len(busy) != 1 else 'its port is'} in use{hint}; trying again every {RETRY_PORTS:.0f} s: "
        f"{', '.join(busy)}")
  next_port_retry = time.monotonic() + RETRY_PORTS

  sel = selectors.DefaultSelector()
  for service in services:
    sel.register(service.publisher.listener, selectors.EVENT_READ, (service, None))
  registered = {}  # connection -> its selector mask
  poller = None  # msgq poller over the services being read, rebuilt when that set changes
  polled = ()
  connected = set()  # the services that have connections; only these need looking at in the loop
  reporter = Reporter()
  last_drop_report = {}
  unknown_layout = []  # said once that the queues' layout is unknown
  say(f"Serving {len(services)} openpilot services on their cereal bridge ports (pid {os.getpid()})")
  status.write(0)
  last_status = time.monotonic()

  def forget(conn):
    try:
      sel.unregister(conn.sock)
    except (KeyError, ValueError):
      pass
    registered.pop(conn, None)

  try:
    while not stopping:
      reading = tuple(service for service in connected if service.active)
      if not reading:
        poller, polled = None, ()
        events = sel.select(0.25)
      elif any(mask & selectors.EVENT_WRITE for mask in registered.values()):
        # A frame is still going out: wake as soon as a socket takes more, and read the services meanwhile
        events = sel.select(WRITE_WAIT)
      else:
        # Woken by msgq as soon as one of them has a message (only as a wakeup: on some msgq versions poll() returns
        # new socket objects); never poll(-1), which can spin
        if reading != polled:
          poller = messaging.Poller()
          for service in reading:
            poller.registerSocket(service.sock)
          polled = reading
        poller.poll(POLL_TIMEOUT)
        events = sel.select(0)
      now = time.monotonic()
      if waiting and now >= next_port_retry:
        next_port_retry = now + RETRY_PORTS
        still_waiting = []
        for name, port, queue_size in waiting:
          try:
            service = Service(name, comma_zmtp.Publisher(port, host=args.host), queue_size)
          except OSError:
            still_waiting.append((name, port, queue_size))
            continue
          services.append(service)
          sel.register(service.publisher.listener, selectors.EVENT_READ, (service, None))
          say(f"Listening for {name} now")
        waiting = still_waiting
      accepting = []
      # Connections first, so a socket closed here is unregistered before an accept can reuse its number
      for key, mask in events:
        service, conn = key.data
        if conn is None:
          accepting.append(service)
          continue
        if mask & selectors.EVENT_READ:
          conn.read()  # subscriptions and heartbeats; the messages are ignored
        if mask & selectors.EVENT_WRITE and not conn.closed:
          conn.write()
        if conn.closed:
          forget(conn)
      for service in accepting:
        for conn in service.publisher.accept():
          conn.address = peer_address(conn)
          conn.reported = False
          try:
            sel.register(conn.sock, selectors.EVENT_READ, (service, conn))
          except (ValueError, OSError):
            conn.close('gone before it was registered')
            continue
          registered[conn] = selectors.EVENT_READ
          connected.add(service)

      for service in list(connected):
        publisher = service.publisher
        for conn in publisher.sweep(now):
          forget(conn)
          if conn.reported:
            reporter.add('disconnected from', conn.address, service.name)
        if not publisher.peers:
          connected.discard(service)
          service.active = False
          continue
        publisher.heartbeat(now)
        ready = publisher.ready_peers()
        for conn in ready:
          if not conn.reported:
            conn.reported = True
            reporter.add('connected to', conn.address, service.name)
        if ready and not service.active and now >= service.next_try:
          slots = reader_slots(service.name, service.queue_size)
          if isinstance(slots, Slots) and slots.total is None and not unknown_layout:
            unknown_layout.append(service.name)
            say("openpilot's msgq queues have a layout this doesn't know (a newer msgq?), so free reader slots aren't "
                "checked before subscribing; readers that keep evicting each other are still noticed")
          reason = why_not_read(service, slots)
          if reason:
            if not service.held_back:
              service.held_back = True
              say(f"Not reading {service.name} for now: {reason}. Trying again every {HOLD_BACK:.0f} s")
            service.next_try = now + HOLD_BACK
            continue
          if service.held_back:
            service.held_back = False
            say(f"Reading {service.name} now")
          if service.sock is None:
            # Subscribed once and kept: msgq hands out reader slots without taking them back, so subscribing on
            # every connect would use them up and evict openpilot's own readers. cereal's sub_sock passes the
            # service's queue size, which must match the publisher's.
            service.sock = messaging.sub_sock(service.name, conflate=False)
          else:
            messaging.drain_sock_raw(service.sock)  # what queued up while no PC was connected is stale
          service.active = True
          service.uid = service.readers = service.writer = None
          service.evictions = []
          evictions_seen(service, reader_slots(service.name, service.queue_size), now)
          service.next_check = now + SLOT_CHECK_PERIOD
        elif not ready and service.active:
          service.active = False
        elif service.active and now >= service.next_check:
          service.next_check = now + SLOT_CHECK_PERIOD
          slots = reader_slots(service.name, service.queue_size)
          reason = why_not_read(service, slots)
          if evictions_seen(service, slots, now) and reason is None:
            reason = "its msgq readers keep evicting each other: there are more of them than reader slots"
          if reason:
            service.active = False
            service.held_back = True
            service.next_try = now + HOLD_BACK
            say(f"Stopped reading {service.name}: {reason}. Trying again every {HOLD_BACK:.0f} s")
            continue
        if service.active:
          for data in messaging.drain_sock_raw(service.sock):
            publisher.send(data)
          for conn in publisher.peers:
            if conn.closed:
              forget(conn)
        if publisher.dropped and now - last_drop_report.get(service.name, -1e9) > 5.0:
          last_drop_report[service.name] = now
          say(f"Dropped {publisher.dropped} {service.name} messages for a PC that isn't keeping up")
          publisher.dropped = 0

      for conn, mask in list(registered.items()):
        if conn.closed:
          forget(conn)
          continue
        want = selectors.EVENT_READ | (selectors.EVENT_WRITE if conn.wants_write else 0)
        if want != mask:
          sel.modify(conn.sock, want, sel.get_key(conn.sock).data)
          registered[conn] = want
      reporter.flush(now)
      if now - last_status >= 1.0:
        last_status = now
        status.write(len({conn.address for service in connected for conn in service.publisher.ready_peers()}))
  finally:
    for sig in (signal.SIGHUP, signal.SIGTERM, signal.SIGINT):
      signal.signal(sig, signal.SIG_IGN)
    reporter.flush(time.monotonic(), force=True)
    for service in services:
      service.publisher.close()
    for path in (PIDFILE, STATUSFILE):
      try:
        with open(path) as fh:
          if int(fh.read().split()[0]) == os.getpid():
            os.remove(path)
      except (OSError, ValueError, IndexError):
        pass
    say("Stopped")


if __name__ == '__main__':
  main()
