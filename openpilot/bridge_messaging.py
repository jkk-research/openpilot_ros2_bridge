"""openpilot's services for the ROS 2 nodes, as openpilot_bridge_receiver.py passes them on from the Comma.

Takes the place of the parts of openpilot's cereal.messaging the nodes use: sub_sock(), recv_one() and
SERVICE_LIST. Messages are decoded with the openpilot checkout's own schema (the checkout is on PYTHONPATH and
matches the Comma's openpilot version), but nothing in the checkout has to be built: the services come over local
ZeroMQ endpoints instead of openpilot's msgq.
"""

import hashlib
import os
import stat

import zmq

NO_TRAVERSAL_LIMIT = 2**64 - 1
ENDPOINT_DIR_VARIABLE = 'OPENPILOT_ROS2_BRIDGE_IPC'


def __getattr__(name):
  # The checkout's cereal is imported on first use, so the receiver, which only needs the endpoints, works without it
  if name == 'SERVICE_LIST':
    from cereal.services import SERVICE_LIST
    return SERVICE_LIST
  raise AttributeError(f"module {__name__!r} has no attribute {name!r}")


def endpoint_dir(namespace='openpilot'):
  # One per user and ROS namespace, so two bridges on one machine don't mix. The launch sets the variable the same way;
  # the namespace is hashed because a Unix socket's path is at most 107 characters.
  digest = hashlib.sha1(namespace.encode()).hexdigest()[:12]
  return os.environ.get(ENDPOINT_DIR_VARIABLE) or f'/tmp/openpilot_ros2_bridge_{os.getuid()}_{digest}'


def endpoint(service, directory=None):
  return f'ipc://{directory or endpoint_dir()}/{service}'


def make_endpoint_dir(directory=None):
  """Creates the endpoint directory, or checks the one that exists: in a shared /tmp another user could have made it
  first, to put endpoints of their own there. The receiver and the nodes both call this, whichever starts first."""
  directory = directory or endpoint_dir()
  try:
    os.makedirs(directory, mode=0o700, exist_ok=True)
  except OSError:
    pass  # something else is in the way: the checks below tell what
  try:
    info = os.lstat(directory)
  except OSError as exc:
    raise PermissionError(f"Can't create {directory}: {exc.strerror}") from exc
  if not stat.S_ISDIR(info.st_mode) or info.st_uid != os.getuid() or info.st_mode & 0o022:
    raise PermissionError(f"{directory} isn't a directory of this user that only this user can write to; use another "
                          f"namespace, or set {ENDPOINT_DIR_VARIABLE} to a directory of your own")
  return directory


def sub_sock(service):
  """A socket that receives every message of the service from openpilot_bridge_receiver.py, once it runs."""
  directory = make_endpoint_dir()
  sock = zmq.Context.instance().socket(zmq.SUB)
  sock.setsockopt(zmq.LINGER, 0)
  sock.setsockopt(zmq.SUBSCRIBE, b'')
  sock.connect(endpoint(service, directory))
  return sock


def log_from_bytes(data):
  from cereal import log
  with log.Event.from_bytes(data, traversal_limit_in_words=NO_TRAVERSAL_LIMIT) as msg:
    return msg


def recv_one(sock, timeout=100):
  """The next message as a log.Event, or None when none arrives within timeout ms."""
  if sock.poll(timeout):
    return log_from_bytes(sock.recv())
  return None
