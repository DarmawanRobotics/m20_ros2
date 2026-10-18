"""The velocity gate: ratios, holds, arming, takeover latch, E006, watchdog and zero burst."""

from m20_driver import protocol as p
from m20_driver.motion import (
    is_motion,
    NAV_HOLDS,
    ratio,
    RobotView,
    SpeedEstimator,
    VelocityConfig,
    VelocityGate,
)
import pytest

RATE = 20.0
READY = {'usage_mode': p.USAGE_REGULAR, 'motion_state': p.MOTION_RL_CONTROL, 'hes': 0,
         'charge': 0, 'direction': 0, 'gait': p.GAIT_BASIC}


def view(**changes) -> RobotView:
    """Return a robot in RL control and regular mode, with changes."""
    values = dict(READY)
    values.update(changes)
    return RobotView(**values)


def gate(**config) -> VelocityGate:
    """Return a gate that has seen the ready robot once (armed, nothing commanded)."""
    g = VelocityGate(VelocityConfig(**config))
    assert g.tick(0.0, view()) == []
    return g


def axes(requests) -> list:
    """Return the axis Items among the requests."""
    return [items for message, items in requests if message == p.AXIS]


def modes(requests) -> list:
    """Return the usage modes requested."""
    return [items['Mode'] for message, items in requests if message == p.USAGE_MODE]


def run(g, start, end, robot=None, cmd=None):
    """Tick at RATE from start to end, commanding cmd before every tick; return the requests."""
    out = []
    steps = int(round((end - start) * RATE))
    for i in range(steps):
        now = start + i / RATE
        if cmd is not None:
            g.command(*cmd, now)
        out += g.tick(now, robot or view())
    return out


def xs(requests) -> list:
    """Return the X values of the axis commands."""
    return [items['X'] for items in axes(requests)]


def test_ratio_of_the_full_scale():
    """Speed / full scale, 2 decimals, clamped; a bad scale gives 0."""
    assert ratio(1.0, 5.0) == 0.2
    assert ratio(-0.25, 2.0) == -0.12
    assert ratio(9.0, 2.0) == 1.0
    assert ratio(1.0, 0.0) == 0.0


def test_rep103_signs_pass_through():
    """X forward, Y left, Yaw counter-clockwise: the official client's signs, no negation."""
    g = gate(full_scale={'basic': (2.0, 1.0, 2.0)}, max_wz=2.0)
    g.command(0.5, 0.25, 1.0, 0.1)
    items = axes(g.tick(0.1, view()))[0]
    assert (items['X'], items['Y'], items['Yaw']) == (0.25, 0.25, 0.5)
    assert (items['Z'], items['Roll'], items['Pitch']) == (0.0, 0.0, 0.0)
    assert g.status() == ('driving', False)


def test_inversions_and_limits():
    """invert_vy / invert_wz flip the signs; cmd_vel is clamped to max_*."""
    g = gate(full_scale={'basic': (1.0, 1.0, 1.0)}, invert_vy=True, invert_wz=True,
             max_vx=0.5)
    g.command(9.0, 0.5, 0.5, 0.1)
    items = axes(g.tick(0.1, view()))[0]
    assert (items['X'], items['Y'], items['Yaw']) == (0.5, -0.5, -0.5)


def test_direction_back_negates_x_and_y():
    """Forward direction Back: X and Y flip to stay in base_link, the yaw rate does not."""
    g = gate(full_scale={'basic': (2.0, 1.0, 2.0)})
    g.command(0.5, 0.25, 0.5, 0.1)
    items = axes(g.tick(0.1, view(direction=1)))[0]
    assert (items['X'], items['Y'], items['Yaw']) == (-0.25, -0.25, 0.25)


def test_full_scale_per_gait_and_the_largest_for_an_unknown_gait():
    """The reported gait picks the full scale; an unknown gait takes the largest of each axis."""
    scales = {'basic': (2.0, 1.0, 1.0), 'stair': (1.0, 0.5, 0.5), 'flat_agile': (4.0, 2.0, 3.0),
              'stair_agile': (1.0, 1.0, 1.0)}
    g = gate(full_scale=scales)
    g.command(0.4, 0.0, 0.0, 0.1)
    assert xs(g.tick(0.1, view(gait=p.GAIT_STAIR))) == [0.4]
    assert xs(g.tick(0.15, view(gait=p.GAIT_FLAT_AGILE))) == [0.1]
    assert xs(g.tick(0.2, view(gait=p.GAIT_HIGH_OBSTACLES))) == [0.1]   # 4.0: the largest
    assert g.full_scale() == (4.0, 2.0, 3.0)


def test_zero_burst_then_silence_while_zero_twists_keep_coming():
    """After motion: zeros for stop_burst_sec, then nothing although zero Twists keep arriving."""
    g = gate()
    out = run(g, 0.05, 1.0, cmd=(0.5, 0.0, 0.0))
    assert set(xs(out)) == {0.1}
    out = run(g, 1.0, 3.0, cmd=(0.0, 0.0, 0.0))
    assert set(xs(out)) == {0.0}
    assert len(axes(out)) == pytest.approx(0.5 * RATE, abs=1)          # the burst only
    assert axes(run(g, 3.0, 5.0, cmd=(0.0, 0.0, 0.0))) == []


def test_zero_twists_alone_send_nothing():
    """Zero Twists before any motion keep the axis channel free (the official client's dedupe)."""
    g = gate()
    assert run(g, 0.05, 4.0, cmd=(0.0, 0.0, 0.0)) == []


def test_watchdog_burst_then_silence_when_cmd_vel_stops():
    """cmd_vel stops: the command goes stale after cmd_timeout_sec, zeros, then nothing."""
    g = gate()
    run(g, 0.05, 1.0, cmd=(0.5, 0.0, 0.0))
    out = run(g, 1.0, 3.0)
    moving = [x for x in xs(out) if x != 0.0]
    zeros = [x for x in xs(out) if x == 0.0]
    assert len(moving) == pytest.approx(0.5 * RATE, abs=1)             # until stale
    assert len(zeros) == pytest.approx(0.5 * RATE, abs=1)
    assert axes(run(g, 3.0, 4.0)) == []


def test_starved_timer_still_sends_the_burst():
    """The timer resumes long after the last command: the zero burst still goes out first."""
    g = gate()
    run(g, 0.05, 1.0, cmd=(0.5, 0.0, 0.0))
    out = g.tick(5.0, view())
    assert xs(out) == [0.0]
    assert set(xs(run(g, 5.05, 5.5))) == {0.0}
    assert axes(run(g, 5.5, 6.5)) == []


@pytest.mark.parametrize('reason', ['navigation_mode', 'assist_mode', 'charge_start', 'sleep',
                                    'sit_down'])
def test_hold_then_a_late_period_still_sends_a_zero(reason):
    """The burst starts at the next period: a zero goes out even 0.64 s after the hold."""
    g = gate()
    g.command(0.5, 0.0, 0.0, 0.0)
    assert is_motion(axes(g.tick(0.05, view()))[0])
    g.hold(reason, 0.06)                                     # the services thread
    later = []
    for t in (0.7, 0.75, 0.8, 2.0):                          # 0.64 s late, then on time
        later += axes(g.tick(t, view()))
    assert later and not is_motion(later[0])
    assert all(not is_motion(items) for items in later)


def test_hold_with_a_stop_burst_shorter_than_a_period_still_sends_a_zero():
    """stop_burst_sec 0.02 at 20 Hz: still one zero after the hold."""
    g = VelocityGate(VelocityConfig(stop_burst_sec=0.02))
    g.tick(0.0, view())
    g.command(0.5, 0.0, 0.0, 0.0)
    assert is_motion(axes(g.tick(0.05, view()))[0])
    g.hold('charge_start', 0.06)
    later = []
    for i in range(2, 40):
        later += axes(g.tick(round(0.05 * i, 2), view()))
    assert len(later) == 1 and not is_motion(later[0])


def test_overspeed_then_a_late_period_still_sends_a_zero():
    """The overspeed disarm comes from the receive thread: the next period sends the zero."""
    g = gate()
    t = 0.0
    while t < 0.6:
        g.command(0.5, 0.0, 0.0, t)
        assert is_motion(axes(g.tick(t, view()))[0])
        g.observe_speed(t, 3.0, 0.0, 0.0)
        t = round(t + 0.05, 2)
    g.observe_speed(0.62, 3.0, 0.0, 0.0)
    later = []
    for t in (1.2, 1.25, 2.0):                               # 0.6 s late
        later += axes(g.tick(t, view()))
    assert later and not any(is_motion(items) for items in later)


def test_e006_then_a_late_period_still_sends_a_zero():
    """E006 arrives on the receive thread: the next period still sends a zero."""
    g = gate()
    g.command(0.5, 0.0, 0.0, 0.0)
    assert is_motion(axes(g.tick(0.05, view()))[0])
    g.axis_refused(0xE006, 0.06)
    later = axes(g.tick(0.9, view()))                        # 0.84 s late
    assert later and not is_motion(later[0])


def test_a_zero_burst_follows_every_motion_whatever_the_timing():
    """After any motion at least one zero goes out, for any lateness of the next period."""
    for late in (0.01, 0.05, 0.3, 0.6, 2.0, 30.0):
        for stop in ('stale', 'zero', 'hold', 'takeover', 'e006'):
            if stop == 'stale' and late <= 0.5:
                continue                                     # still fresh: still driving
            g = gate()
            g.command(0.5, 0.0, 0.0, 0.0)
            g.tick(0.05, view())
            robot = view()
            if stop == 'zero':
                g.command(0.0, 0.0, 0.0, 0.06)
            elif stop == 'hold':
                g.hold('sleep', 0.06)
            elif stop == 'takeover':
                robot = view(usage_mode=p.USAGE_NAVIGATION)
            elif stop == 'e006':
                g.axis_refused(0xE006, 0.06)
            out = axes(g.tick(0.05 + late, robot))
            assert out and not is_motion(out[0]), (late, stop)


def test_record_sent_keeps_the_burst_due():
    """emergency_stop's own zero is recorded; the timer's burst still follows."""
    g = gate()
    g.command(0.5, 0.0, 0.0, 0.0)
    g.tick(0.05, view())
    g.hold('emergency_stop', 0.06)
    g.record_sent(p.axis_items(), 0.06)
    assert not is_motion(g.last_axis)
    assert axes(g.tick(0.1, view())) == [p.axis_items()]


def test_release_since_keeps_a_hold_that_came_in_later():
    """A releasing service lifts nothing when a hold came in after it started."""
    g = gate()
    g.hold('sit_down', 0.0)
    started = g.hold_seq                                     # stand_up starts
    g.hold('emergency_stop', 0.1)                            # e-stop while it runs
    assert not g.release(since=started)
    assert g.holds == ('emergency_stop', 'sit_down')
    assert g.release(since=g.hold_seq)
    assert g.holds == ()


def test_nonfinite_twist_is_a_stop():
    """A NaN in any field: zeros (the burst) and a throttled warning, not the other fields."""
    g = gate()
    run(g, 0.05, 0.5, cmd=(0.5, 0.0, 0.0))
    g.command(0.5, float('nan'), 0.0, 0.5, finite=True)
    out = g.tick(0.5, view())
    assert xs(out) == [0.0]
    g.command(0.5, 0.0, 0.0, 0.55, finite=False)          # e.g. angular.x was infinite
    assert xs(g.tick(0.55, view())) == [0.0]
    assert [level for level, _ in g.drain_events()] == ['warn']


def test_noise_level_cmd_vel_never_requests_a_mode():
    """0.002 m/s rounds to 0.00: no motion, so no regular mode request and no axis."""
    g = VelocityGate(VelocityConfig())
    nav = view(usage_mode=p.USAGE_NAVIGATION)
    assert run(g, 0.0, 2.0, robot=nav, cmd=(0.002, 0.0, 0.0)) == []


def test_hold_stops_a_stream_and_release_needs_zero_or_absent():
    """emergency_stop holds: only the burst; stand_up's release does not drive the stale stream."""
    g = gate()
    run(g, 0.05, 1.0, cmd=(0.5, 0.0, 0.0))
    g.hold('emergency_stop', 1.0)
    out = run(g, 1.0, 3.0, cmd=(0.5, 0.0, 0.0))
    assert set(xs(out)) == {0.0} and len(xs(out)) == pytest.approx(0.5 * RATE, abs=1)
    assert g.status() == ('hold:emergency_stop', True)
    assert g.release()
    assert xs(run(g, 3.0, 5.0, cmd=(0.5, 0.0, 0.0))) == []      # still the old stream
    g.command(0.0, 0.0, 0.0, 5.0)
    assert set(xs(run(g, 5.05, 5.5, cmd=(0.5, 0.0, 0.0)))) == {0.1}


def test_release_with_cmd_vel_absent_drives_the_next_stream():
    """No cmd_vel when the hold is lifted: armed at once, the next stream drives."""
    g = gate()
    g.hold('sit_down', 0.0)
    run(g, 0.0, 1.0)
    g.release()
    run(g, 1.0, 1.5)
    assert set(xs(run(g, 1.5, 2.0, cmd=(0.5, 0.0, 0.0)))) == {0.1}


def test_regular_mode_lifts_only_the_navigation_holds():
    """release(NAV_HOLDS) keeps an emergency_stop hold."""
    g = gate()
    g.hold('emergency_stop', 0.0)
    g.hold('navigation_mode', 0.0)
    assert g.release(NAV_HOLDS)
    assert g.holds == ('emergency_stop',)
    assert not g.release(NAV_HOLDS)


def test_takeover_while_driving_latches_without_mode_requests():
    """The app switches to navigation mid-stream: burst, then nothing and no mode request."""
    g = gate()
    run(g, 0.05, 1.0, cmd=(0.5, 0.0, 0.0))
    nav = view(usage_mode=p.USAGE_NAVIGATION)
    out = run(g, 1.0, 4.0, robot=nav, cmd=(0.5, 0.0, 0.0))
    assert set(xs(out)) == {0.0} and modes(out) == []
    state, warn = g.status()
    assert warn and state.startswith('takeover (navigation mode)')
    assert any(level == 'warn' and 'takeover' in text for level, text in g.drain_events())


def test_takeover_while_cmd_vel_is_zero_latches():
    """Leaving regular mode latches even when nothing is commanded."""
    g = gate()
    run(g, 0.05, 1.0, cmd=(0.0, 0.0, 0.0))
    g.tick(1.0, view(usage_mode=p.USAGE_ASSIST))
    assert g.status()[0].startswith('takeover (assist mode)')


def test_takeover_latch_survives_zeros_and_silence():
    """5 s of zeros and 5 s of silence do not end it; a new episode asks for no mode either."""
    g = gate()
    run(g, 0.05, 1.0, cmd=(0.5, 0.0, 0.0))
    nav = view(usage_mode=p.USAGE_NAVIGATION)
    run(g, 1.0, 6.0, robot=nav, cmd=(0.0, 0.0, 0.0))
    run(g, 6.0, 11.0, robot=nav)
    out = run(g, 11.0, 14.0, robot=nav, cmd=(0.5, 0.0, 0.0))    # a new episode
    assert axes(out) == [] and modes(out) == []
    assert g.status()[0].startswith('takeover')


def test_takeover_ends_on_regular_mode_then_needs_zero_or_absent():
    """Handed back: regular mode reported; the running stream drives only after a zero."""
    g = gate()
    run(g, 0.05, 1.0, cmd=(0.5, 0.0, 0.0))
    run(g, 1.0, 2.0, robot=view(usage_mode=p.USAGE_NAVIGATION), cmd=(0.5, 0.0, 0.0))
    out = run(g, 2.0, 3.0, cmd=(0.5, 0.0, 0.0))                  # regular again
    assert [x for x in xs(out) if x != 0.0] == []
    assert not g.status()[0].startswith('takeover')
    assert any(level == 'info' for level, _ in g.drain_events())
    g.command(0.0, 0.0, 0.0, 3.0)
    assert set(xs(run(g, 3.05, 3.5, cmd=(0.5, 0.0, 0.0)))) == {0.1}


def test_navigation_mode_service_latches_like_a_takeover():
    """Our own navigation_mode: a hold, and once reported a latch that stand_up does not end."""
    g = gate()
    g.hold('navigation_mode', 0.0)
    g.tick(0.05, view(usage_mode=p.USAGE_NAVIGATION))
    assert g.status()[0].startswith('takeover (navigation mode)')
    g.release()                                                  # stand_up / rl_control
    out = run(g, 0.1, 2.0, robot=view(usage_mode=p.USAGE_NAVIGATION), cmd=(0.5, 0.0, 0.0))
    assert axes(out) == [] and modes(out) == []
    assert g.status()[0].startswith('takeover')


def test_regular_mode_requested_at_the_start_only_while_never_seen():
    """Navigation since boot: up to 3 requests 0.5 s apart, then drive at once once regular."""
    g = VelocityGate(VelocityConfig())
    nav = view(usage_mode=p.USAGE_NAVIGATION)
    g.tick(0.0, nav)
    out = run(g, 0.05, 2.5, robot=nav, cmd=(0.5, 0.0, 0.0))
    assert modes(out) == [p.USAGE_REGULAR] * 3 and axes(out) == []
    assert any('did not switch' in text for _, text in g.drain_events())
    g2 = VelocityGate(VelocityConfig())
    g2.tick(0.0, nav)
    out = run(g2, 0.05, 0.3, robot=nav, cmd=(0.5, 0.0, 0.0))
    assert modes(out) == [p.USAGE_REGULAR]
    assert set(xs(run(g2, 0.3, 0.6, cmd=(0.5, 0.0, 0.0)))) == {0.1}   # no zero needed


def test_regular_mode_tries_are_bounded_in_time():
    """Tries only within MODE_WINDOW_SEC of the first: none 60 s later in the same episode."""
    g = VelocityGate(VelocityConfig())
    nav = view(usage_mode=p.USAGE_NAVIGATION)
    g.tick(0.0, nav)
    g.command(0.5, 0.0, 0.0, 0.05)
    assert modes(g.tick(0.05, nav)) == [p.USAGE_REGULAR]          # try 1
    late = []
    t = 0.05
    while t < 60.0:                                                # zeros keep the episode
        t = round(t + 0.1, 2)
        g.command(0.0, 0.0, 0.0, t)
        late += modes(g.tick(t, nav))
    g.command(0.5, 0.0, 0.0, 60.1)                                 # it moves again
    late += modes(g.tick(60.1, nav))
    assert late == []
    assert any('did not switch after 1 requests' in text for _, text in g.drain_events())


def test_no_mode_request_from_assist_unknown_held_or_charging():
    """Assist or unknown mode, a hold, charging: never ask for regular mode."""
    for robot in (view(usage_mode=p.USAGE_ASSIST), view(usage_mode=None),
                  view(usage_mode=p.USAGE_NAVIGATION, charge=2)):
        g = VelocityGate(VelocityConfig())
        g.tick(0.0, robot)
        assert modes(run(g, 0.05, 2.0, robot=robot, cmd=(0.5, 0.0, 0.0))) == []
    g = VelocityGate(VelocityConfig())
    nav = view(usage_mode=p.USAGE_NAVIGATION)
    g.tick(0.0, nav)
    g.hold('navigation_mode', 0.0)
    assert modes(run(g, 0.05, 2.0, robot=nav, cmd=(0.5, 0.0, 0.0))) == []
    g = VelocityGate(VelocityConfig(auto_regular_mode=False))
    g.tick(0.0, nav)
    assert modes(run(g, 0.05, 2.0, robot=nav, cmd=(0.5, 0.0, 0.0))) == []


def test_e006_yields_until_cmd_vel_was_silent():
    """Another client holds the axis channel: quiet until silence; the next episode tries again."""
    g = gate()
    run(g, 0.05, 1.0, cmd=(0.5, 0.0, 0.0))
    g.axis_refused(0xE006, 1.0)
    out = run(g, 1.0, 4.0, cmd=(0.5, 0.0, 0.0))
    assert set(xs(out)) == {0.0} and len(xs(out)) == pytest.approx(0.5 * RATE, abs=1)
    assert g.status() == ('axis_owner_other', True)
    run(g, 4.0, 5.0, cmd=(0.0, 0.0, 0.0))                       # zeros do not end it
    assert g.status() == ('axis_owner_other', True)
    run(g, 5.0, 6.5)                                             # silence does
    assert set(xs(run(g, 6.5, 7.0, cmd=(0.5, 0.0, 0.0)))) == {0.1}
    g.axis_refused(0xE008, 7.0)                                  # other refusals: no yield
    assert g.status() == ('driving', False)


@pytest.mark.parametrize('robot,reason', [
    (view(motion_state=p.MOTION_SIT), 'not_rl_control'),
    (view(hes=1), 'hard_estop'),
    (view(charge=1), 'charging'),
    (view(direction=None), 'direction_unknown'),
])
def test_disarm_conditions_send_nothing_but_zeros(robot, reason):
    """The robot is not drivable: a stream gives only the zero burst; re-arming needs a zero."""
    g = gate()
    run(g, 0.05, 1.0, cmd=(0.5, 0.0, 0.0))
    out = run(g, 1.0, 3.0, robot=robot, cmd=(0.5, 0.0, 0.0))
    assert set(xs(out)) == {0.0}
    assert g.status() == (f'disarmed:{reason}', True)
    assert xs(run(g, 3.0, 4.0, cmd=(0.5, 0.0, 0.0))) == []      # condition gone, stale stream
    g.command(0.0, 0.0, 0.0, 4.0)
    assert set(xs(run(g, 4.05, 4.5, cmd=(0.5, 0.0, 0.0)))) == {0.1}


def test_overspeed_disarms_with_an_error():
    """Reported speed far above max_* for 0.5 s while driving: a full scale is wrong."""
    g = gate()
    run(g, 0.05, 1.0, cmd=(0.5, 0.0, 0.0))
    for i in range(12):
        g.observe_speed(1.0 + i * 0.1, 2.0, 0.0, 0.0)          # 2.0 > 1.3 * 1.0 + 0.1
        g.command(0.5, 0.0, 0.0, 1.0 + i * 0.1)
        g.tick(1.0 + i * 0.1, view())
    assert g.status() == ('disarmed:overspeed', True)
    assert any(level == 'error' and 'full_scale' in text for level, text in g.drain_events())
    g.observe_speed(2.3, 0.1, 0.0, 0.0)
    assert xs(run(g, 2.3, 3.0, cmd=(0.5, 0.0, 0.0))) == []      # needs a zero first


def test_speed_estimator_per_gait():
    """Steady commands give reported speed / ratio per gait; small ratios, reversals skipped."""
    estimator = SpeedEstimator()
    for _ in range(50):
        estimator.update('basic', {'X': 0.25, 'Y': 0.0, 'Yaw': -0.5}, (0.5, 0.0, -1.5))
    estimator.update('basic', {'X': 0.1, 'Y': 0.0, 'Yaw': 0.0}, (5.0, 0.0, 0.0))
    estimator.update('stair', {'X': 0.5, 'Y': 0.0, 'Yaw': 0.0}, (-0.5, 0.0, 0.0))
    estimator.update(None, {'X': 0.5}, (0.5, 0.0, 0.0))
    assert estimator.estimates['basic']['vx'] == pytest.approx(2.0)
    assert estimator.estimates['basic']['wz'] == pytest.approx(3.0)
    assert estimator.estimates['basic']['vy'] is None
    assert estimator.estimates['stair']['vx'] is None
