"""The UDP link against the simulated M20 on the loopback interface (no ROS)."""

import time

from fake_robot import FakeM20, FakeServer
from m20_driver import protocol as p
from m20_driver.link import RobotLink
import pytest


def wait_for(predicate, timeout: float = 3.0) -> bool:
    """Poll until the predicate holds."""
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        if predicate():
            return True
        time.sleep(0.02)
    return False


@pytest.fixture
def rig():
    """Run a fake robot and a started link that keeps its received frames."""
    server = FakeServer(FakeM20(), rate_scale=5.0)
    frames = []
    link = RobotLink('127.0.0.1', server.port, '127.0.0.1', 0, frames.append)
    link.start()
    yield server, link, frames
    link.stop()
    server.close()


def test_reports_follow_the_heartbeat(rig):
    """No reports before a heartbeat; all four status reports after."""
    server, link, frames = rig
    time.sleep(0.2)
    assert not [f for f in frames if f.key in p.STATUS_REPORTS]
    link.send(p.HEARTBEAT)
    assert wait_for(lambda: {f.key for f in frames} >= set(p.STATUS_REPORTS))
    assert link.age() < 1.0 and server.heartbeats == 1


def test_heartbeat_thread_keeps_the_reports_coming():
    """With heartbeat_hz the link sends heartbeats by itself, and stops with the link."""
    server = FakeServer(FakeM20(), rate_scale=5.0)
    frames = []
    link = RobotLink('127.0.0.1', server.port, '127.0.0.1', 0, frames.append, heartbeat_hz=20.0)
    link.start()
    try:
        assert wait_for(lambda: server.heartbeats >= 5)
        assert wait_for(lambda: {f.key for f in frames} >= set(p.STATUS_REPORTS))
    finally:
        link.stop()
    count = server.heartbeats
    time.sleep(0.2)
    assert server.heartbeats == count and link.heartbeats >= count
    server.close()


def test_heartbeat_pauses_while_the_caller_is_hung():
    """heartbeat_ok False (the ROS side is hung): no heartbeats, reported once each way."""
    server = FakeServer(FakeM20(), rate_scale=5.0)
    alive = [True]
    changes = []
    link = RobotLink('127.0.0.1', server.port, '127.0.0.1', 0, lambda frame: None,
                     heartbeat_hz=50.0, heartbeat_ok=lambda: alive[0],
                     on_heartbeat_pause=changes.append)
    link.start()
    try:
        assert wait_for(lambda: server.heartbeats >= 3)
        alive[0] = False
        assert wait_for(lambda: link.heartbeats_paused)
        count = server.heartbeats
        time.sleep(0.3)
        assert server.heartbeats <= count + 1
        alive[0] = True
        assert wait_for(lambda: server.heartbeats >= count + 3)
        assert changes == [True, False] and not link.heartbeats_paused
    finally:
        link.stop()
        server.close()


def test_a_second_driver_on_the_same_port_fails():
    """No SO_REUSEADDR: two links cannot share a fixed local port and split the reports."""
    first = RobotLink('127.0.0.1', 1, '127.0.0.1', 0, lambda frame: None)
    first.start()
    try:
        second = RobotLink('127.0.0.1', 1, '127.0.0.1', first.local[1], lambda frame: None)
        with pytest.raises(OSError):
            second.start()
    finally:
        first.stop()


def test_reply_with_another_id_reaches_the_request_of_its_kind(rig):
    """A refusal answered with message ID 0 still fails the request it belongs to."""
    server, link, _ = rig
    server.reply_id = 0
    server.robot.reject[p.CHARGE] = 0xE00A
    link.next_id()                                  # our request will not have ID 0
    sent, response = link.request(p.CHARGE, p.charge_items(p.CHARGE_START), timeout=1.0)
    assert sent and response is not None and response.error_code == 0xE00A


def test_request_wait_can_be_aborted(rig):
    """abort() turning True ends the wait at once, without a response."""
    server, link, _ = rig
    server.respond = False
    start = time.monotonic()
    sent, response = link.request(p.LIGHTS, p.lights_items(True, True), timeout=2.0,
                                  abort=lambda: time.monotonic() - start > 0.2)
    assert sent and response is None and time.monotonic() - start < 1.0


def test_request_gets_its_response(rig):
    """The call status response is matched by message ID."""
    server, link, _ = rig
    sent, response = link.request(p.GAIT, p.gait_items(p.GAIT_FLAT_AGILE), timeout=1.0)
    assert sent and response is not None and response.ok
    assert (response.kind, response.command) == p.GAIT
    assert server.robot.gait == p.GAIT_FLAT_AGILE


def test_refusal_is_reported(rig):
    """An ErrorCode other than 0 comes back as a failed response."""
    server, link, _ = rig
    server.robot.reject[p.LIGHTS] = 0xE007
    sent, response = link.request(p.LIGHTS, p.lights_items(True, True), timeout=1.0)
    assert sent and not response.ok and response.error_code == 0xE007
    assert link.last_failure is response


def test_no_response_times_out():
    """A robot that does not answer: sent, but no response."""
    server = FakeServer(FakeM20(), respond=False)
    link = RobotLink('127.0.0.1', server.port, '127.0.0.1', 0, lambda frame: None)
    link.start()
    try:
        start = time.monotonic()
        sent, response = link.request(p.LIGHTS, p.lights_items(False, False), timeout=0.3)
        assert sent and response is None and time.monotonic() - start >= 0.3
    finally:
        link.stop()
        server.close()


def test_message_ids_wrap():
    """0, 1, ... 65535, then 0 again."""
    link = RobotLink('127.0.0.1', 1, '127.0.0.1', 0, lambda frame: None)
    ids = [link.next_id() for _ in range(65538)]
    assert ids[:2] == [0, 1] and ids[65535] == 65535 and ids[65536:] == [0, 1]


def test_bad_datagrams_are_counted():
    """Garbage is counted, never handed on."""
    frames = []
    link = RobotLink('127.0.0.1', 1, '127.0.0.1', 0, frames.append)
    link.handle(b'garbage')
    link.handle(p.encode_report(p.BASIC_STATUS, {'BasicStatus': {'MotionState': 1}}))
    assert link.bad_frames == 1 and len(frames) == 1 and link.rx_count == 1
