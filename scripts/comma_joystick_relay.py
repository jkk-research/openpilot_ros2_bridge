#!/usr/bin/env python3
"""Runs on the Comma: passes the ROS 2 bridge's drive commands to openpilot's joystick mode.

The bridge launch copies it to the Comma and runs it there over SSH with openpilot's own Python
(start_control:=true; scripts/start_comma_joystick_relay.sh). While it runs, it:
- receives testJoystick from the PC over ZMQ, on the port openpilot's cereal bridge uses for it, checks it and
  publishes it to joystickd. Commands that arrive late, such as a backlog after a Wi-Fi stall, are dropped.
- turns on openpilot's joystick mode (the JoystickDebugMode param) for the next drive, only while the car has been
  off for a few seconds and the PC's sender is running. The sender transmits at 100 Hz even without commands, so
  that stream shows it's alive. Without it for 5 s while the car is off, joystick mode is turned off again.
- turns joystick mode off when it exits with the car off. openpilot also turns it off at the end of every drive,
  and this turns it on again for the next one. If it's turned off on the Comma (Developer settings), or a
  maneuver mode is on, it stays off.
- keeps the testJoystick queue: in joystick mode openpilot also starts its own joystick_control, which sends zeros
  when no gamepad is plugged in. The publisher that starts last owns the queue, so this takes it back whenever
  joystick_control takes it.
"""

import argparse
import math
import os
import signal
import sys
import time

SERVICE = 'testJoystick'
PARAM = 'JoystickDebugMode'
PIDFILE = '/tmp/openpilot_ros2_joystick_relay.pid'
# Present while a relay has joystick mode on and hasn't turned it off yet, so the next relay cleans up after one
# that died without doing so
MARKER = '/tmp/openpilot_ros2_joystick_relay.armed'
CAR_OFF_SETTLE = 3.0  # s with the car off before turning joystick mode on; a manager restart reads offroad briefly
PC_ALIVE = 1.0  # s since the PC's last message for joystick mode to be turned on
PC_GONE = 5.0  # s without the PC, with the car off, before joystick mode is turned off again
MAX_LATE = 0.1  # s later than the fastest delivery seen before a command is dropped
CLOCK_DRIFT = 1e-3  # s/s the reference may drift, far more than real clocks do, far less than a backlog


def bridge_port(endpoint):
    # Same as get_port() in openpilot's cereal/messaging/bridge_zmq.cc
    value = 0xcbf29ce484222325
    for byte in endpoint.encode():
        value ^= byte
        value = (value * 0x100000001b3) & 0xFFFFFFFFFFFFFFFF
    return 8023 + value % (65535 - 8023)


def say(text):
    try:
        print(f"[comma joystick relay] {text}", flush=True)
    except OSError:
        pass  # the SSH session's terminal is gone; keep going so the cleanup still runs


def remove(path):
    try:
        os.remove(path)
    except OSError:
        pass


def stop_previous_relay():
    # A relay whose SSH session died without the Comma noticing would fight this one for the queue
    try:
        with open(PIDFILE) as fh:
            pid = int(fh.read())
        with open(f'/proc/{pid}/cmdline', 'rb') as fh:
            running = os.path.basename(__file__).encode() in fh.read()
    except (OSError, ValueError):
        running = False
    if running and pid != os.getpid():
        say(f"Stopping the relay left from an earlier session (pid {pid})")
        os.kill(pid, signal.SIGTERM)
        for _ in range(50):
            if not os.path.exists(f'/proc/{pid}'):
                break
            time.sleep(0.1)
    with open(PIDFILE, 'w') as fh:
        fh.write(str(os.getpid()))


class Freshness:
    """Tells commands that arrive late from fresh ones.

    The PC's and the Comma's clocks differ, so the reference is the fastest delivery seen, allowed to drift slowly.
    """

    def __init__(self):
        self.offset = None
        self.offset_time = 0.0
        self.late_since = None

    def fresh(self, delay_ns, now):
        if self.offset is None or delay_ns < self.offset + (now - self.offset_time) * CLOCK_DRIFT * 1e9:
            self.offset, self.offset_time = delay_ns, now
        if delay_ns - self.offset <= MAX_LATE * 1e9:
            self.late_since = None
            return True
        if self.late_since is None:
            self.late_since = now
        elif now - self.late_since > 3.0:
            # Late for 3 s in a row isn't a backlog: the PC's clock jumped (it restarted); start over from here
            self.offset, self.offset_time, self.late_since = delay_ns, now, None
            return True
        return False


class JoystickMode:
    """Turns openpilot's joystick mode on for drives while the PC is there, and off again."""

    def __init__(self, params):
        self.params = params
        self.ours = os.path.exists(MARKER)
        self.car_off_since = None
        self.drive = None
        self.seen_on = False  # on during this stretch with the car off
        self.turned_off_on_comma = False
        self.maneuver_reported = False

    def get(self, key):
        try:
            return self.params.get_bool(key)
        except Exception:
            return False  # a key this openpilot version doesn't have

    def set(self, on):
        # Blocking, so the write is on disk before this process can exit
        self.params.put_bool(PARAM, on, block=True)
        self.ours = on
        if on:
            open(MARKER, 'w').close()
        else:
            remove(MARKER)

    def update(self, now, pc_silent_for):
        on = self.get(PARAM)
        if not self.get('IsOffroad'):
            self.car_off_since = None
            self.seen_on = False
            if self.drive is None:
                self.drive = 'joystick' if on else 'normal'
                if on:
                    say("Driving in joystick mode: the ROS 2 commands act while openpilot is engaged")
                else:
                    say("This drive uses openpilot's normal controls; joystick mode can only be turned on while the "
                        "car is off")
            return

        if self.car_off_since is None:
            self.car_off_since = now
            self.drive = None
        if on:
            if self.ours and pc_silent_for > PC_GONE:
                self.set(False)
                self.seen_on = False
                say(f"Nothing from the PC for {PC_GONE:.0f} s; joystick mode is off until it's back")
            else:
                self.seen_on = True
            return
        if self.seen_on:
            # On earlier with the car off, and nobody drove: turned off on the Comma, or openpilot restarted
            self.seen_on = False
            self.turned_off_on_comma = True
            self.ours = False
            remove(MARKER)
            say("Joystick mode was turned off on the Comma; it stays off until the launch is restarted")
            return
        if self.turned_off_on_comma or pc_silent_for > PC_ALIVE or now - self.car_off_since < CAR_OFF_SETTLE:
            return
        if self.get('LongitudinalManeuverMode') or self.get('LateralManeuverMode'):
            if not self.maneuver_reported:
                self.maneuver_reported = True
                say("A maneuver mode is on on the Comma, so joystick mode stays off")
            return
        self.maneuver_reported = False
        if self.get('IsOffroad'):
            self.set(True)
            say("Joystick mode is on for the next drive: start the car and engage openpilot with the cruise control "
                "to let the ROS 2 commands drive")

    def release(self):
        if not self.ours:
            return
        if not self.get('IsOffroad'):
            # The marker stays: openpilot turns joystick mode off when the drive ends
            say("Stopping during a drive: the car gets accel 0 and steer 0, and openpilot turns joystick mode off "
                "when the drive ends")
        elif self.get(PARAM):
            self.set(False)
            say("Joystick mode is off again")
        else:
            remove(MARKER)


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--openpilot-dir', default='/data/openpilot')
    parser.add_argument('--host', default='', help="the PC's address (default: where this SSH session comes from)")
    parser.add_argument('--port', type=int, default=0, help=f"default: the cereal bridge's port for {SERVICE}")
    args = parser.parse_args()

    host = args.host or os.environ.get('SSH_CLIENT', '').split(' ')[0]
    if not host:
        sys.exit("No --host given and not started over SSH: pass the PC's address with --host")
    port = args.port or bridge_port(SERVICE)

    # Newer openpilot keeps its packages under <dir>/openpilot; older versions import openpilot.* through a symlink
    sys.path[:0] = [args.openpilot_dir, os.path.join(args.openpilot_dir, 'msgq_repo')]
    import zmq
    try:
        from openpilot.cereal import messaging
    except ImportError:
        import cereal.messaging as messaging
    from openpilot.common.params import Params

    stopping = []

    def stop(signum, frame):
        stopping.append(signum)
    # SIGHUP: the SSH session closed (Ctrl+C on the launch). The loop checks the flag at least every 100 ms.
    for sig in (signal.SIGHUP, signal.SIGTERM, signal.SIGINT):
        signal.signal(sig, stop)

    stop_previous_relay()
    mode = JoystickMode(Params())
    freshness = Freshness()
    context = zmq.Context()
    sub = context.socket(zmq.SUB)
    sub.setsockopt(zmq.SUBSCRIBE, b'')
    sub.setsockopt(zmq.CONFLATE, 1)
    sub.setsockopt(zmq.RCVBUF, 4096)
    sub.setsockopt(zmq.RCVTIMEO, 100)
    # Notice a PC that went away without closing the connection, and reconnect when it's back
    sub.setsockopt(zmq.HEARTBEAT_IVL, 1000)
    sub.setsockopt(zmq.HEARTBEAT_TIMEOUT, 3000)
    sub.connect(f'tcp://{host}:{port}')
    pub = messaging.pub_sock(SERVICE)
    say(f"Waiting for joystick commands from {host}:{port}")

    last_received = -math.inf
    receiving = False
    next_check = 0.0
    last_drop_report = -math.inf
    dropped = 0
    try:
        while not stopping:
            now = time.monotonic()
            if now >= next_check:
                next_check = now + 1.0
                mode.update(now, now - last_received)

            try:
                data = sub.recv()
            except zmq.Again:
                if receiving and now - last_received > 1.0:
                    receiving = False
                    say("Nothing from the PC for 1 s")
                continue

            try:
                event = messaging.log_from_bytes(data)
                axes = list(event.testJoystick.axes) if event.which() == SERVICE else []
                delay = time.monotonic_ns() - event.logMonoTime
            except Exception:
                axes, delay = [], 0
            # joystickd reads two axes and doesn't check them
            valid = len(axes) == 2 and all(math.isfinite(a) and abs(a) <= 1.0 for a in axes)
            now = time.monotonic()
            if not valid or not freshness.fresh(delay, now):
                dropped += 1
                # The first one right away, then a count at most every 5 s
                if now - last_drop_report > 5.0:
                    say(f"Dropped commands that arrived late or weren't valid ({dropped} since the last message)")
                    last_drop_report, dropped = now, 0
                continue

            last_received = now
            if not receiving:
                receiving = True
                say("Receiving joystick commands from the PC")
            try:
                pub.send(data)
            except messaging.MultiplePublishersError:
                say(f"openpilot's joystick_control took the {SERVICE} queue; taking it back")
                pub = messaging.pub_sock(SERVICE)
                pub.send(data)
    finally:
        # Nothing may interrupt the cleanup
        for sig in (signal.SIGHUP, signal.SIGTERM, signal.SIGINT):
            signal.signal(sig, signal.SIG_IGN)
        mode.release()
        try:
            with open(PIDFILE) as fh:
                if int(fh.read()) == os.getpid():
                    os.remove(PIDFILE)
        except (OSError, ValueError):
            pass


if __name__ == '__main__':
    main()
