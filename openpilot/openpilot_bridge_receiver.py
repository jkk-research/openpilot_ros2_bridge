#!/usr/bin/env python3
"""PC side of the bridge: receives openpilot's services from the Comma and passes them on to the ROS 2 nodes here.

Connects to the Comma's side of the bridge, scripts/comma_bridge.py, or openpilot's own cereal bridge where that runs
instead: one ZeroMQ SUB socket per service, on the port the cereal bridge uses for it. Each service
is published again on a local ZeroMQ endpoint (see bridge_messaging.py), so the nodes that read the same service
share one stream from the Comma. This replaces openpilot's cereal bridge in ZMQ-to-msgq mode on the PC, so nothing
from openpilot has to be built here.

Usage: openpilot_bridge_receiver.py <comma ip> <service>[,<service>...] [--endpoint-dir DIR]
"""

import argparse
import errno
import fcntl
import os
import signal
import sys
import time

import zmq

from bridge_messaging import endpoint, endpoint_dir, make_endpoint_dir

SILENT_AFTER = 5.0  # s without any message before the log says so
MAX_IPC_PATH = 107  # what fits in a Unix socket address


def bridge_port(endpoint_name):
  # Same as get_port() in openpilot's cereal/messaging/bridge_zmq.cc
  value = 0xcbf29ce484222325
  for byte in endpoint_name.encode():
    value ^= byte
    value = (value * 0x100000001b3) & 0xFFFFFFFFFFFFFFFF
  return 8023 + value % (65535 - 8023)


def say(text):
  print(f"[bridge receiver] {text}", flush=True)


def main():
  parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
  parser.add_argument('comma_ip')
  parser.add_argument('services')
  parser.add_argument('--endpoint-dir', default='')
  args = parser.parse_args()

  try:
    directory = make_endpoint_dir(args.endpoint_dir or endpoint_dir())
  except PermissionError as exc:
    sys.exit(str(exc))
  # One receiver per directory: another one, from a second launch with the same namespace, would take the endpoints over
  lock = os.fdopen(os.open(os.path.join(directory, '.lock'), os.O_WRONLY | os.O_CREAT | os.O_NOFOLLOW, 0o600), 'w')
  try:
    fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
  except OSError:
    sys.exit(f"Another bridge receiver is using {directory}: is a bridge with the same namespace already running?")
  # Each name once, in order; a service this openpilot version doesn't have just never sends anything
  services = list(dict.fromkeys(name.strip() for name in args.services.split(',') if name.strip()))
  if not services:
    sys.exit("No services to receive")
  longest = max(services, key=len)
  if len(endpoint(longest, directory)) - len('ipc://') > MAX_IPC_PATH:
    sys.exit(f"The local endpoint for {longest} would be longer than {MAX_IPC_PATH} characters: use a shorter namespace")

  stopping = []
  for sig in (signal.SIGINT, signal.SIGTERM):
    signal.signal(sig, lambda signum, frame: stopping.append(signum))

  ctx = zmq.Context()
  poller = zmq.Poller()
  pairs = {}
  for name in services:
    sub = ctx.socket(zmq.SUB)
    sub.setsockopt(zmq.LINGER, 0)
    sub.setsockopt(zmq.SUBSCRIBE, b'')
    sub.setsockopt(zmq.RECONNECT_IVL_MAX, 500)
    # Heartbeats notice a connection that died without closing (the Comma rebooted or left the Wi-Fi), so it's
    # made again; both the cereal bridge and comma_bridge.py answer them
    sub.setsockopt(zmq.HEARTBEAT_IVL, 1000)
    sub.setsockopt(zmq.HEARTBEAT_TIMEOUT, 5000)
    sub.connect(f'tcp://{args.comma_ip}:{bridge_port(name)}')
    pub = ctx.socket(zmq.PUB)
    pub.setsockopt(zmq.LINGER, 0)
    pub.bind(endpoint(name, directory))
    poller.register(sub, zmq.POLLIN)
    pairs[sub] = (name, pub)
  say(f"Receiving {len(services)} services from {args.comma_ip}: {', '.join(services)}")

  receiving = set()
  new = []
  last_message = None
  last_report = time.monotonic()
  try:
    while not stopping:
      try:
        events = poller.poll(100)
      except zmq.ZMQError as exc:
        if exc.errno == errno.EINTR:
          continue
        raise
      now = time.monotonic()
      for sub, _ in events:
        name, pub = pairs[sub]
        while True:
          try:
            frame = sub.recv(zmq.NOBLOCK, copy=False)
          except zmq.Again:
            break
          pub.send(frame, copy=False)
        last_message = now
        if name not in receiving:
          receiving.add(name)
          new.append(name)
      if new and now - last_report >= 1.0:
        say(f"Data from the Comma: {', '.join(new)}")
        new = []
        last_report = now
      if receiving and last_message is not None and now - last_message > SILENT_AFTER:
        say(f"Nothing from the Comma for {SILENT_AFTER:.0f} s; waiting for it")
        receiving.clear()
        new = []
  finally:
    for sub, (name, pub) in pairs.items():
      sub.close()
      pub.close()
    ctx.term()
    for name in services:
      try:
        os.unlink(os.path.join(directory, name))
      except OSError:
        pass


if __name__ == '__main__':
  main()
