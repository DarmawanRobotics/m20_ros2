"""The velocity policy: ratios, limits, watchdog and the regular usage mode."""

from m20_driver import protocol as p
from m20_driver.motion import ratio, SpeedEstimator, VelocityConfig, VelocityPolicy
import pytest


def axis_of(requests):
    """Return the Items of the axis command among the requests."""
    found = [items for message, items in requests if message == p.AXIS]
    assert len(found) == 1
    return found[0]


def modes_of(requests):
    """Return the usage modes requested."""
    return [items['Mode'] for message, items in requests if message == p.USAGE_MODE]


def test_ratio_of_the_full_scale():
    """Speed / full scale, 2 decimals, clamped; a bad scale gives 0."""
    assert ratio(1.0, 5.0) == 0.2
    assert ratio(-0.25, 2.0) == -0.12
    assert ratio(9.0, 2.0) == 1.0
    assert ratio(1.0, 0.0) == 0.0


def test_rep103_signs_pass_through():
    """X forward, Y left, Yaw counter-clockwise: the official client's signs, no negation."""
    policy = VelocityPolicy(VelocityConfig(full_scale_vx=2.0, full_scale_vy=1.0,
                                           full_scale_wz=2.0, max_wz=2.0))
    policy.command(0.5, 0.25, 1.0, now=0.0)
    items = axis_of(policy.tick(0.0, p.USAGE_REGULAR))
    assert (items['X'], items['Y'], items['Yaw']) == (0.25, 0.25, 0.5)
    assert (items['Z'], items['Roll'], items['Pitch']) == (0.0, 0.0, 0.0)


def test_inversions():
    """invert_vy / invert_wz flip the lateral and yaw signs."""
    policy = VelocityPolicy(VelocityConfig(full_scale_vy=1.0, full_scale_wz=1.0, invert_vy=True,
                                           invert_wz=True))
    policy.command(0.0, 0.2, 0.5, now=0.0)
    items = axis_of(policy.tick(0.0, p.USAGE_REGULAR))
    assert (items['Y'], items['Yaw']) == (-0.2, -0.5)


def test_direction_back_negates_x_and_y():
    """Forward direction Back: X and Y flip to stay in base_link, the yaw rate does not."""
    policy = VelocityPolicy(VelocityConfig(full_scale_vx=2.0, full_scale_vy=1.0,
                                           full_scale_wz=2.0))
    policy.command(0.5, 0.25, 0.5, now=0.0)
    items = axis_of(policy.tick(0.0, p.USAGE_REGULAR, reverse=True))
    assert (items['X'], items['Y'], items['Yaw']) == (-0.25, -0.25, 0.25)


def test_clamped_to_the_configured_limits():
    """cmd_vel is clamped to max_* before scaling; NaN is zero."""
    policy = VelocityPolicy(VelocityConfig(max_vx=1.0, full_scale_vx=2.0))
    policy.command(3.0, float('nan'), 0.0, now=0.0)
    assert policy.current == (1.0, 0.0, 0.0)
    assert axis_of(policy.tick(0.0, p.USAGE_REGULAR))['X'] == 0.5


def test_watchdog_zeros_then_silence():
    """Fresh: the command; after the timeout: zeros for the burst; then nothing."""
    policy = VelocityPolicy(VelocityConfig(cmd_timeout_sec=0.5, stop_burst_sec=0.5))
    assert policy.tick(0.0, p.USAGE_REGULAR) == []          # nothing before the first command
    policy.command(1.0, 0.0, 0.0, now=0.0)
    assert axis_of(policy.tick(0.4, p.USAGE_REGULAR))['X'] > 0.0
    assert axis_of(policy.tick(0.8, p.USAGE_REGULAR)) == p.axis_items()
    assert policy.tick(1.2, p.USAGE_REGULAR) == []
    assert not policy.moving(1.2) and policy.recently_moved(1.2, 2.0)


def test_regular_mode_requested_when_motion_starts_elsewhere():
    """Navigation (or unknown) mode: regular mode first, then again only every refresh."""
    policy = VelocityPolicy(VelocityConfig(mode_refresh_sec=2.0))
    policy.command(0.5, 0.0, 0.0, now=0.0)
    first = policy.tick(0.0, p.USAGE_NAVIGATION)
    assert modes_of(first) == [p.USAGE_REGULAR]
    assert first[0][0] == p.USAGE_MODE and first[-1][0] == p.AXIS
    policy.command(0.5, 0.0, 0.0, now=1.0)
    assert modes_of(policy.tick(1.0, p.USAGE_NAVIGATION)) == []
    policy.command(0.5, 0.0, 0.0, now=2.1)
    assert modes_of(policy.tick(2.1, None)) == [p.USAGE_REGULAR]
    policy.command(0.5, 0.0, 0.0, now=2.2)
    assert modes_of(policy.tick(2.2, p.USAGE_REGULAR)) == []


def test_no_mode_switch_for_zero_commands_or_when_disabled():
    """Zeros never switch the mode; auto_regular_mode false never does."""
    policy = VelocityPolicy(VelocityConfig())
    policy.command(0.0, 0.0, 0.0, now=0.0)
    assert modes_of(policy.tick(0.0, p.USAGE_NAVIGATION)) == []
    off = VelocityPolicy(VelocityConfig(auto_regular_mode=False))
    off.command(0.5, 0.0, 0.0, now=0.0)
    assert modes_of(off.tick(0.0, p.USAGE_NAVIGATION)) == []


def test_stop_streams_zeros():
    """stop() replaces the command with zeros."""
    policy = VelocityPolicy(VelocityConfig())
    policy.command(0.5, 0.2, 0.3, now=0.0)
    policy.stop(now=0.1)
    assert axis_of(policy.tick(0.1, p.USAGE_REGULAR)) == p.axis_items()


def test_speed_estimator():
    """Measured speed / sent ratio, smoothed; small ratios and opposite signs are ignored."""
    est = SpeedEstimator(min_ratio=0.2, smoothing=0.5)
    est.update({'X': 0.5, 'Y': 0.1, 'Yaw': -0.4}, (1.0, 0.05, -0.6))
    assert est.estimates['vx'] == pytest.approx(2.0) and est.estimates['vy'] is None
    assert est.estimates['wz'] == pytest.approx(1.5)
    est.update({'X': 0.5, 'Y': 0.0, 'Yaw': 0.4}, (1.5, None, -0.6))
    assert est.estimates['vx'] == pytest.approx(2.5)
    assert est.estimates['wz'] == pytest.approx(1.5)
