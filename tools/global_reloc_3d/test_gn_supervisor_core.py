import math
import unittest

import gn_supervisor_core as sc


class Supervisor(sc.Supervisor):
    """The node feeds the supervisor two streams from gn_loc: the 1 Hz status (heartbeat, alarms) and one fit sample per scan (confirmation). The tests that only
    describe 1 Hz statuses mean both: every status that carries a new scan is also one sample."""
    def on_status(self, t, state, inliers, tight, scans=None):
        super().on_status(t, state, inliers, tight, scans)
        if self.stall == 0:
            self.on_scan_fit(t, state, inliers, tight)


def run(s, t, *events):
    """Feed (method, args...) events at time t, then tick; return the actions emitted."""
    for e in events:
        getattr(s, e[0])(t, *e[1:])
    s.tick(t)
    return s.drain()


def kinds(actions):
    return [a[0] for a in actions]


class Fixture(unittest.TestCase):
    def tracking(self, s, t0, n=3, pose=(1., 2., 0.)):
        """GN healthy for n status periods: the supervisor leaves its start-up confirmation."""
        for k in range(n):
            run(s, t0 + k, ('on_status', 'TRACKING', .9, .8), ('on_gn_pose', pose))
        return t0 + n

    def new(self, **kw):
        s = Supervisor(sc.Params(**kw))
        t = self.tracking(s, 0.)
        self.assertEqual(s.state, 'MONITOR')
        return s, t


class StartupTest(Fixture):
    def test_navigation_is_held_until_gn_confirms_a_pose(self):
        s = Supervisor(sc.Params(auto_init_s=0.))                      # a pose from elsewhere (the operator): the start-up localization is not what is tested
        self.assertTrue(s.hold)
        run(s, 0., ('on_status', 'WAITING_POSE', 0., 0.))
        run(s, 1., ('on_status', 'TRACKING', .9, .3))       # a pose that does not fit the map yet
        self.assertTrue(s.hold)
        self.tracking(s, 2.)
        self.assertFalse(s.hold)


class AutoInitTest(Fixture):
    """At start-up nobody has given gn_loc a pose: after auto_init_s the supervisor localizes by itself and only asks the operator when that fails."""
    def waiting(self, s, t0, t1):
        for k in range(int(t0), int(t1) + 1):
            a = run(s, float(k), ('on_status', 'WAITING_POSE', 0., 0.))
            if 'reloc_start' in kinds(a):
                return k, a
        return None, []

    def hold(self, s, t0, t1):
        """gn_loc keeps publishing its 1 Hz status while it waits (silence would be gn_status_stale)."""
        for k in range(int(t0), int(t1) + 1):
            run(s, float(k), ('on_status', 'WAITING_POSE', 0., 0.))

    def test_it_localizes_by_itself_at_once_when_gn_loc_waits_for_a_pose(self):
        s = Supervisor(sc.Params(auto_init_s=8.))
        t, a = self.waiting(s, 0, 20)
        self.assertEqual(t, 0)                                            # the first status: no wait for an operator that may click
        self.assertIn(('reloc_start', 'startup_auto'), a)
        self.assertEqual((s.state, s.reason), ('RELOCALIZING', 'startup_auto'))

    def test_it_can_be_switched_off(self):
        s = Supervisor(sc.Params(auto_init_s=0.))
        t, _ = self.waiting(s, 0, 60)
        self.assertIsNone(t)
        self.assertEqual((s.state, s.reason), ('CONFIRMING', 'startup'))

    def test_an_operator_pose_given_first_wins_over_the_start_up_localization(self):
        s = Supervisor(sc.Params(auto_init_s=8.))
        self.waiting(s, 0, 3)
        run(s, 3.5, ('on_operator_pose', (1., 2., 0.)))
        t, _ = self.waiting(s, 4, 30)
        self.assertIsNone(t)

    def test_an_operator_pose_before_any_status_means_no_start_up_localization(self):
        s = Supervisor(sc.Params(auto_init_s=8.))
        run(s, 0., ('on_operator_pose', (1., 2., 0.)))
        t, _ = self.waiting(s, 1, 5)
        self.assertIsNone(t)
        self.assertEqual((s.state, s.reason), ('CONFIRMING', 'operator_pose'))

    def test_a_stale_waiting_status_does_not_start_it(self):
        s = Supervisor(sc.Params(auto_init_s=8.))
        s.gn_state, s.t_status = 'WAITING_POSE', 0.                 # gn_loc said it waits, then went silent for longer than the stale limit
        self.assertNotIn('reloc_start', kinds(run(s, 5.)))

    def test_a_silent_gn_loc_is_waited_for_before_localizing(self):
        s = Supervisor(sc.Params(auto_init_s=8.))
        seen = [k for k in range(7) if 'reloc_start' in kinds(run(s, float(k)))]
        self.assertEqual(seen, [])
        self.assertIn('reloc_start', kinds(run(s, 8.)))

    def test_an_operator_pose_during_the_auto_init_aborts_it_and_wins(self):
        s = Supervisor(sc.Params(auto_init_s=8.))
        self.waiting(s, 0, 8)
        self.hold(s, 1, 11)
        a = run(s, 12., ('on_operator_pose', (1., 2., 0.)))
        self.assertIn('reloc_abort', kinds(a))
        self.assertEqual((s.state, s.reason), ('CONFIRMING', 'operator_pose'))

    def test_the_accepted_pose_is_published_and_confirmed_without_the_operator(self):
        s = Supervisor(sc.Params(auto_init_s=8.))
        self.waiting(s, 0, 8)
        self.hold(s, 1, 19)
        a = run(s, 20., ('on_reloc_result', True, (1., 2., .3), 'accepted'))
        self.assertIn(('initialpose', (1., 2., .3)), a)
        self.assertEqual(s.state, 'CONFIRMING')
        self.tracking(s, 21., pose=(1., 2., .3))
        self.assertEqual((s.state, s.reason), ('MONITOR', 'confirmed'))

    def test_two_rejections_ask_the_operator(self):
        s = Supervisor(sc.Params(auto_init_s=8.))
        self.waiting(s, 0, 8)
        self.hold(s, 1, 19)
        run(s, 20., ('on_reloc_result', False, None, 'ambiguous'))
        self.assertEqual(s.state, 'RELOCALIZING')                         # the second attempt
        self.hold(s, 21, 39)
        run(s, 40., ('on_reloc_result', False, None, 'ambiguous'))
        self.assertEqual(s.state, 'FAILED')
        self.assertTrue(s.reason.startswith('needs_operator:'), s.reason)
        a = run(s, 41., ('on_operator_pose', (1., 2., 0.)))                # the operator always wins
        self.assertEqual((s.state, s.reason), ('CONFIRMING', 'operator_pose'))

    def test_a_pose_gn_does_not_confirm_asks_the_operator(self):
        s = Supervisor(sc.Params(auto_init_s=8.))
        self.waiting(s, 0, 8)
        self.hold(s, 1, 19)
        run(s, 20., ('on_reloc_result', True, (1., 2., .3), 'accepted'))
        for k in range(21, 40):                                            # GN sits on another pose with a poor fit
            run(s, float(k), ('on_status', 'TRACKING', .9, .3), ('on_gn_pose', (5., 5., 0.)))
        self.assertEqual(s.state, 'FAILED')
        self.assertEqual(s.reason, 'needs_operator:gn_not_confirmed')

    def test_gn_settling_by_itself_is_not_overridden(self):
        s = Supervisor(sc.Params(auto_init_s=8.))
        seen = []
        for k in range(15):                                               # TRACKING since the start, the fit still poor: gn_loc is settling, not waiting
            seen += kinds(run(s, float(k), ('on_status', 'TRACKING', .9, .3), ('on_gn_pose', (1., 2., 0.))))
        self.assertNotIn('reloc_start', seen)
        self.assertEqual((s.state, s.reason), ('CONFIRMING', 'startup'))

    def test_a_tracking_that_never_confirms_is_localized_after_a_bounded_wait(self):
        s = Supervisor(sc.Params(auto_init_s=8.))
        t = None
        for k in range(40):                                               # TRACKING with a poor fit for ever: the start-up has no timeout of its own
            if 'reloc_start' in kinds(run(s, float(k), ('on_status', 'TRACKING', .9, .3), ('on_gn_pose', (1., 2., 0.)))):
                t = k; break
        self.assertEqual(t, 16)                                           # twice the wait: long enough for a gn_loc that settles by itself
        self.assertEqual((s.state, s.reason), ('RELOCALIZING', 'startup_auto'))

    def test_leaving_the_auto_init_through_a_verification_forgets_it(self):
        s = Supervisor(sc.Params(auto_init_s=8.))
        self.waiting(s, 0, 8)
        run(s, 9., ('on_status', 'TRACKING', .9, .8), ('on_gn_pose', (1., 2., 0.)))   # gn_loc got there by itself meanwhile
        run(s, 10., ('on_reloc_result', True, (1., 2., 0.), 'accepted'))
        self.assertEqual((s.state, s.reason), ('MONITOR', 'verified_gn_agrees'))
        for k in range(11, 21):                                            # an unrelated loss later, long enough to stop
            run(s, float(k), ('on_status', 'LOST', 0., 0.))
        self.assertEqual(s.state, 'RELOCALIZING')
        run(s, 21., ('on_reloc_result', False, None, 'x'))
        run(s, 22., ('on_reloc_result', False, None, 'x'))
        self.assertEqual(s.state, 'FAILED')
        self.assertFalse(s.reason.startswith('needs_operator'), s.reason)

    def test_a_failure_after_a_stop_keeps_its_own_name(self):
        s, t = self.new()
        run(s, t, ('on_status', 'LOST', 0., 0.))
        run(s, t + 5, ('on_reloc_result', False, None, 'x'))
        run(s, t + 9, ('on_reloc_result', False, None, 'x'))
        self.assertEqual(s.state, 'FAILED')
        self.assertFalse(s.reason.startswith('needs_operator'), s.reason)


class LostTest(Fixture):
    def test_lost_stops_and_starts_the_relocalizer_once(self):
        s, t = self.new()
        a = run(s, t, ('on_status', 'LOST', .1, .1))
        self.assertEqual(kinds(a)[:2], ['cancel', 'reloc_start'])
        self.assertTrue(s.hold)
        self.assertEqual(s.state, 'RELOCALIZING')
        self.assertNotIn('reloc_start', kinds(run(s, t + 1, ('on_status', 'LOST', .1, .1))))

    def test_gn_recovering_by_itself_is_still_verified(self):
        # 131206: GN's own LOST recovery locked onto an alias 1.5 m along a corridor: its TRACKING after a LOST is not trusted alone
        s, t = self.new()
        run(s, t, ('on_status', 'LOST', .1, .1))
        run(s, t + 1, ('on_status', 'TRACKING', .9, .8), ('on_gn_pose', (1., 2., 0.)))
        self.assertTrue(s.hold)
        self.assertEqual(s.state, 'RELOCALIZING')

    def test_relocalizer_agreeing_with_a_healthy_gn_resumes_without_a_new_pose(self):
        s, t = self.new()
        run(s, t, ('on_status', 'LOST', .1, .1))
        run(s, t + 1, ('on_status', 'TRACKING', .9, .8), ('on_gn_pose', (1., 2., 0.)))
        a = run(s, t + 2, ('on_reloc_result', True, (1.1, 2.05, math.radians(2)), 'accepted'))
        self.assertNotIn('initialpose', kinds(a))
        self.assertEqual(s.state, 'MONITOR')
        self.assertFalse(s.hold)

    def test_relocalizer_disagreeing_publishes_its_pose_and_waits_for_gn(self):
        s, t = self.new()
        run(s, t, ('on_status', 'LOST', .1, .1))
        run(s, t + 1, ('on_status', 'TRACKING', .9, .8), ('on_gn_pose', (1., 2., 0.)))   # GN locked on an alias
        a = run(s, t + 2, ('on_reloc_result', True, (2.5, 2., 0.), 'accepted'))
        self.assertIn(('initialpose', (2.5, 2., 0.)), a)
        self.assertEqual(s.state, 'CONFIRMING')
        self.assertTrue(s.hold)
        for k in range(3):
            run(s, t + 3 + k, ('on_status', 'TRACKING', .9, .7), ('on_gn_pose', (2.52, 2.01, 0.)))
        self.assertEqual(s.state, 'MONITOR')
        self.assertFalse(s.hold)

    def test_gn_must_reach_the_acceptance_consistency_after_the_pose(self):
        s, t = self.new(confirm_timeout_s=10.)
        run(s, t, ('on_status', 'LOST', .1, .1))
        run(s, t + 1, ('on_reloc_result', True, (2.5, 2., 0.), 'accepted'))
        for k in range(12):      # GN takes the pose but the scan does not fit well enough (0.55 < accept 0.6): never resumes
            run(s, t + 2 + k, ('on_status', 'TRACKING', .9, .55), ('on_gn_pose', (2.5, 2., 0.)))
        self.assertEqual(s.state, 'FAILED')
        self.assertTrue(s.hold)

    def test_gn_settling_away_from_the_relocalized_pose_is_a_failure(self):
        s, t = self.new()
        run(s, t, ('on_status', 'LOST', .1, .1))
        run(s, t + 1, ('on_reloc_result', True, (2.5, 2., 0.), 'accepted'))
        for k in range(16):      # GN's own search moved 0.5 m away from the relocalized pose: two methods disagree
            run(s, t + 2 + k, ('on_status', 'TRACKING', .9, .8), ('on_gn_pose', (3.0, 2., 0.)))
        self.assertEqual(s.state, 'FAILED')

    def test_status_older_than_the_pose_does_not_confirm(self):
        s, t = self.new()
        run(s, t, ('on_status', 'LOST', .1, .1))
        run(s, t + 1, ('on_reloc_result', True, (2.5, 2., 0.), 'accepted'))
        run(s, t + 1.2, ('on_status', 'TRACKING', .9, .8), ('on_gn_pose', (2.5, 2., 0.)))   # within confirm_lag_s of the pose
        run(s, t + 2, ('on_status', 'TRACKING', .9, .8), ('on_gn_pose', (2.5, 2., 0.)))
        run(s, t + 3, ('on_status', 'TRACKING', .9, .8), ('on_gn_pose', (2.5, 2., 0.)))
        self.assertEqual(s.state, 'CONFIRMING')
        run(s, t + 4, ('on_status', 'TRACKING', .9, .8), ('on_gn_pose', (2.5, 2., 0.)))
        self.assertEqual(s.state, 'MONITOR')


class FailSafeTest(Fixture):
    def test_relocalizer_failures_end_stopped_after_the_attempts(self):
        s, t = self.new(max_attempts=2)
        run(s, t, ('on_status', 'LOST', .1, .1))
        a = run(s, t + 2, ('on_status', 'LOST', .1, .1), ('on_reloc_result', False, None, 'ambiguous_no_alternative_margin'))
        self.assertIn('reloc_start', kinds(a))
        a = run(s, t + 4, ('on_status', 'LOST', .1, .1), ('on_reloc_result', False, None, 'ambiguous_no_alternative_margin'))
        self.assertEqual(s.state, 'FAILED')
        self.assertTrue(s.hold)
        self.assertIn('reloc_failed', s.reason)

    def test_relocalizer_timeout_aborts_it_and_fails(self):
        s, t = self.new(reloc_timeout_s=40., max_attempts=1)
        run(s, t, ('on_status', 'LOST', .1, .1))
        a = run(s, t + 41, ('on_status', 'LOST', .1, .1))
        self.assertIn('reloc_abort', kinds(a))
        self.assertEqual(s.state, 'FAILED')

    def test_failed_waits_for_the_operator_pose(self):
        s, t = self.new(max_attempts=1)
        run(s, t, ('on_status', 'LOST', .1, .1))
        run(s, t + 1, ('on_reloc_result', False, None, 'reject_geometry'))
        for k in range(5):                         # GN recovering by itself does not lift FAILED
            run(s, t + 2 + k, ('on_status', 'TRACKING', .9, .8), ('on_gn_pose', (1., 2., 0.)))
        self.assertEqual(s.state, 'FAILED')
        run(s, t + 8, ('on_operator_pose', (1., 2., 0.)))
        for k in range(3):
            run(s, t + 9 + k, ('on_status', 'TRACKING', .9, .8), ('on_gn_pose', (1., 2., 0.)))
        self.assertEqual(s.state, 'MONITOR')

    def test_gn_status_going_silent_stops_and_fails(self):
        s, t = self.new(status_stale_s=3.)
        a = run(s, t + 3.5)
        self.assertIn('cancel', kinds(a))
        self.assertEqual(s.state, 'FAILED')
        self.assertIn('gn_status_stale', s.reason)

    def test_held_navigation_is_cancelled_again(self):
        s, t = self.new(max_attempts=1)
        run(s, t, ('on_status', 'LOST', .1, .1))
        run(s, t + 1, ('on_reloc_result', False, None, 'reject_geometry'))
        self.assertIn('cancel', kinds(run(s, t + 2, ('on_goal_status', 'navigate_to_pose', True))))      # an operator goal while FAILED
        s2, t2 = self.new()
        self.assertNotIn('cancel', kinds(run(s2, t2, ('on_goal_status', 'navigate_to_pose', True))))    # never while MONITOR

    def test_robot_not_stopping_is_reported(self):
        s, t = self.new(stop_check_s=3.)
        run(s, t, ('on_status', 'LOST', .1, .1), ('on_wheel', .4, 0.))
        run(s, t + 2, ('on_status', 'LOST', .1, .1), ('on_wheel', .3, 0.))
        self.assertNotIn('not_stopped', s.reason)
        run(s, t + 3.5, ('on_status', 'LOST', .1, .1), ('on_wheel', .3, 0.))
        self.assertIn('not_stopped', s.reason)


class ParkedDriftTest(Fixture):
    def park(self, s, t, n, pose_fn):
        for k in range(n):
            run(s, t + k, ('on_wheel', 0., 0.), ('on_status', 'TRACKING', .9, .8), ('on_gn_pose', pose_fn(k)))
        return t + n

    def test_gn_pose_moving_while_the_wheels_are_still_triggers(self):
        s, t = self.new(park_arm_s=3., park_drift_xy=.25)
        t = self.park(s, t, 6, lambda k: (1. + .01 * k, 2., 0.))       # FAST-LIO drift level: a few cm
        self.assertEqual(s.state, 'MONITOR')
        run(s, t, ('on_wheel', 0., 0.), ('on_status', 'TRACKING', .9, .8), ('on_gn_pose', (1.4, 2., 0.)))
        self.assertEqual(s.state, 'MONITOR')                             # must persist park_drift_hold_s (1 s)
        a = run(s, t + 1, ('on_wheel', 0., 0.), ('on_status', 'TRACKING', .9, .8), ('on_gn_pose', (1.4, 2., 0.)))
        self.assertIn('reloc_start', kinds(a))
        self.assertIn('parked_drift', s.reason)

    def test_yaw_drift_also_triggers_and_motion_disarms(self):
        s, t = self.new(park_arm_s=3., park_drift_yaw_deg=5.)
        t = self.park(s, t, 4, lambda k: (1., 2., 0.))
        run(s, t, ('on_wheel', .3, 0.), ('on_gn_pose', (1.5, 2., 0.)))   # driving: the pose moves with the wheels, no alarm
        self.assertEqual(s.state, 'MONITOR')
        t = self.park(s, t + 1, 4, lambda k: (1.5, 2., 0.))
        for k in range(2):
            run(s, t + k, ('on_wheel', 0., 0.), ('on_status', 'TRACKING', .9, .8), ('on_gn_pose', (1.5, 2., math.radians(6))))
        self.assertEqual(s.state, 'RELOCALIZING')

    def test_turn_onset_seen_by_gn_before_the_wheels_is_not_drift(self):
        # 140131 t 1886: the scan pose turns ~0.1-1 s before the wheel speed reports it; the drift must persist while the wheels stay still
        s, t = self.new(park_arm_s=3., park_drift_yaw_deg=5., park_drift_hold_s=1.)
        t = self.park(s, t, 5, lambda k: (1., 2., 0.))
        run(s, t, ('on_wheel', 0., 0.), ('on_status', 'TRACKING', .9, .8), ('on_gn_pose', (1., 2., math.radians(20))))
        run(s, t + .5, ('on_wheel', 0., .6), ('on_gn_pose', (1., 2., math.radians(40))))
        run(s, t + 1.5, ('on_wheel', 0., .6), ('on_status', 'TRACKING', .9, .8), ('on_gn_pose', (1., 2., math.radians(60))))
        self.assertEqual(s.state, 'MONITOR')

    def test_a_gap_in_the_wheel_stream_disarms(self):
        # 131206 t 145-151: no /odom for 7 s, then the odometry glitched; stillness is unknown during a gap and must be re-established
        s, t = self.new(park_arm_s=3., park_drift_hold_s=1., wheel_stale_s=.5)
        t = self.park(s, t, 5, lambda k: (1., 2., 0.))
        for k in range(6):
            run(s, t + k, ('on_status', 'TRACKING', .9, .8), ('on_gn_pose', (1., 2., 0.)))          # no wheel messages
        for k in range(2):
            run(s, t + 6 + k, ('on_wheel', 0., 0.), ('on_status', 'TRACKING', .9, .8), ('on_gn_pose', (1.4, 2., 0.)))
        self.assertEqual(s.state, 'MONITOR')

    def test_no_wheel_source_never_arms(self):
        s, t = self.new(park_arm_s=3.)
        for k in range(8):
            run(s, t + k, ('on_status', 'TRACKING', .9, .8), ('on_gn_pose', (1. + .2 * k, 2., 0.)))
        self.assertEqual(s.state, 'MONITOR')

    def test_periodic_parked_check_does_not_hold_and_resumes_quietly(self):
        s, t = self.new(park_arm_s=3., verify_period_s=60.)
        t = self.park(s, t, 3, lambda k: (1., 2., 0.))
        a = run(s, t, ('on_wheel', 0., 0.))
        self.assertIn('reloc_start', kinds(a))                          # parked for park_arm_s
        self.assertEqual(s.state, 'VERIFYING')
        self.assertFalse(s.hold)                                         # the robot may leave: a check, not a stop
        run(s, t + 1, ('on_reloc_result', True, (1.02, 2.01, 0.), 'accepted'))
        self.assertEqual(s.state, 'MONITOR')
        a = run(s, t + 5, ('on_wheel', 0., 0.))
        self.assertNotIn('reloc_start', kinds(a))                       # not again before verify_period_s

    def test_periodic_check_that_disagrees_becomes_a_stop(self):
        s, t = self.new(park_arm_s=3., verify_period_s=60.)
        t = self.park(s, t, 4, lambda k: (1., 2., 0.))
        run(s, t, ('on_wheel', 0., 0.))
        a = run(s, t + 1, ('on_reloc_result', True, (2.5, 2., 0.), 'accepted'))
        self.assertEqual(kinds(a)[:2], ['cancel', 'initialpose'])
        self.assertEqual(s.state, 'CONFIRMING')
        self.assertTrue(s.hold)

    def test_lost_during_a_parked_check_still_stops(self):
        s, t = self.new(park_arm_s=3., verify_period_s=60.)
        t = self.park(s, t, 3, lambda k: (1., 2., 0.))
        run(s, t, ('on_wheel', 0., 0.))
        self.assertEqual(s.state, 'VERIFYING')
        a = run(s, t + 1, ('on_wheel', .3, 0.), ('on_status', 'LOST', .1, .1))     # the robot drove off and GN lost the map
        self.assertIn('cancel', kinds(a))
        self.assertEqual(s.state, 'RELOCALIZING')
        self.assertTrue(s.hold)

    def test_periodic_check_without_a_pose_is_not_an_alarm(self):
        s, t = self.new(park_arm_s=3., verify_period_s=60.)
        t = self.park(s, t, 4, lambda k: (1., 2., 0.))
        run(s, t, ('on_wheel', 0., 0.))
        a = run(s, t + 1, ('on_reloc_result', False, None, 'imu_not_stationary'))
        self.assertNotIn('cancel', kinds(a))
        self.assertEqual(s.state, 'MONITOR')


class StarvedScanTest(Fixture):
    """gn_core.hpp update(): a scan with < 20 valid beams (rmin 0.5 m: a covered lidar) returns before counting as bad, so GN stays TRACKING with
    inliers = tight = 0 and never goes LOST; and with no /scan at all (FAST-LIO publishes nothing) its 1 Hz status repeats the last values."""
    def statuses(self, s, t, n, inl, tight, scans0=None, step=10):
        for k in range(n):
            run(s, t + k, ('on_status', 'TRACKING', inl, tight, None if scans0 is None else scans0 + step * k), ('on_gn_pose', (1., 2., 0.)))
        return t + n

    def test_tracking_without_valid_beams_stops(self):
        s, t = self.new(bad_stop_s=4.)
        self.statuses(s, t, 4, 0., 0.)                       # statuses t .. t+3: 3 s
        self.assertEqual(s.state, 'MONITOR')
        a = run(s, t + 4, ('on_status', 'TRACKING', 0., 0.))
        self.assertEqual(kinds(a)[:2], ['cancel', 'reloc_start'])
        self.assertEqual(s.reason, 'gn_bad_scans')

    def test_default_stops_a_covered_lidar_after_3_s(self):
        # Narcis, 2026-11-01: bad_stop_s 3 s (blind travel ~0.6 m shorter than 4 s); the longest clean-run bad span is 2.2 s
        s, t = self.new()
        self.statuses(s, t, 3, 0., 0.)                       # statuses t .. t+2: 2 s
        self.assertEqual(s.state, 'MONITOR')
        a = run(s, t + 3, ('on_status', 'TRACKING', 0., 0.))
        self.assertEqual(kinds(a)[:2], ['cancel', 'reloc_start'])
        self.assertEqual(s.reason, 'gn_bad_scans')

    def test_a_short_bad_span_is_not_an_alarm(self):
        # 28 clean runs (a08, 1 Hz): the longest span of TRACKING statuses below GN's track thresholds is 2.2 s
        s, t = self.new(bad_stop_s=4.)
        t = self.statuses(s, t, 3, .9, .45)
        self.statuses(s, t, 5, .9, .8)
        self.assertEqual(s.state, 'MONITOR')

    def test_a_frozen_scan_counter_stops(self):
        s, t = self.new()
        t = self.statuses(s, t, 2, .9, .8, scans0=100)
        self.statuses(s, t, 2, .9, .8, scans0=200, step=0)   # 200 at t, t+1: the second status unchanged
        self.assertEqual(s.state, 'MONITOR')
        run(s, t + 2, ('on_status', 'TRACKING', .9, .8, 200))
        self.assertEqual(s.state, 'RELOCALIZING')
        self.assertEqual(s.reason, 'gn_no_scans')

    def test_confirmation_needs_new_scans(self):
        s = Supervisor(sc.Params())
        self.statuses(s, 0., 6, .9, .8, scans0=50, step=0)
        self.assertEqual(s.state, 'CONFIRMING')
        self.statuses(s, 6., 4, .9, .8, scans0=60)
        self.assertEqual(s.state, 'MONITOR')

    def test_bad_scans_during_a_parked_check_stop(self):
        s, t = self.new(park_arm_s=3., verify_period_s=60., bad_stop_s=4.)
        for k in range(3):
            run(s, t + k, ('on_wheel', 0., 0.), ('on_status', 'TRACKING', .9, .8), ('on_gn_pose', (1., 2., 0.)))
        t += 3
        run(s, t, ('on_wheel', 0., 0.))
        self.assertEqual(s.state, 'VERIFYING')
        seen = []
        for k in range(6):
            seen += kinds(run(s, t + k, ('on_wheel', 0., 0.), ('on_status', 'TRACKING', 0., 0.)))
        self.assertEqual(s.state, 'RELOCALIZING')
        self.assertTrue(s.hold)
        self.assertIn('reloc_abort', seen)                               # the parked check is aborted by the core, not only by the node's start()
        self.assertIn('reloc_start', seen)                               # the parked check did not restart FAST-LIO2; the stop that holds Nav2 must


class ReviewFixesTest(Fixture):
    """Found by the code review of the supervisor PR."""
    def test_an_operator_pose_while_parked_is_not_read_as_drift(self):
        s, t = self.new(park_arm_s=3., park_drift_xy=.25, park_drift_hold_s=1.)
        for k in range(6):
            run(s, t + k, ('on_wheel', 0., 0.), ('on_status', 'TRACKING', .9, .8), ('on_gn_pose', (1., 2., 0.)))   # parked: the anchor is taken
        t += 6
        seen = kinds(run(s, t, ('on_wheel', 0., 0.), ('on_operator_pose', (5., 2., 0.))))
        for k in range(1, 9):                                            # GN settles on the new pose and the robot stays parked
            seen += kinds(run(s, t + k, ('on_wheel', 0., 0.), ('on_status', 'TRACKING', .9, .8), ('on_gn_pose', (5., 2., 0.))))
        self.assertEqual(s.state, 'MONITOR')
        self.assertNotIn('reloc_start', seen)                            # the old anchor (1, 2) must not make the re-pose look like drift
        self.assertNotIn('cancel', seen)

    def test_a_gn_that_is_neither_tracking_nor_lost_is_not_trusted(self):
        s, t = self.new()
        a = run(s, t, ('on_status', 'WAITING_POSE', 0., 0.))             # a restarted gn_loc waits for a pose and publishes no map->odom
        self.assertEqual(kinds(a)[:2], ['cancel', 'reloc_start'])
        self.assertEqual(s.reason, 'gn_not_tracking')

    def test_need_reloc_during_a_parked_verification_is_not_lost(self):
        s, t = self.new(verify_period_s=10., park_arm_s=3.)
        for k in range(14):
            run(s, t + k, ('on_wheel', 0., 0.), ('on_status', 'TRACKING', .9, .8), ('on_gn_pose', (1., 2., 0.)))
        self.assertEqual(s.state, 'VERIFYING')
        a = run(s, t + 14, ('on_wheel', 0., 0.), ('on_need_reloc', True))      # latched edge: it is not delivered again
        self.assertIn('cancel', kinds(a))
        self.assertEqual((s.state, s.reason), ('RELOCALIZING', 'gn_need_reloc'))

    def test_the_not_stopped_note_does_not_leak_into_the_retry(self):
        s, t = self.new(stop_check_s=3.)
        run(s, t, ('on_status', 'LOST', 0., 0.))
        run(s, t + 5, ('on_wheel', .5, 0.), ('on_status', 'LOST', 0., 0.))
        self.assertIn('+not_stopped', s.reason)
        s.on_reloc_result(t + 6, False, None, 'no_pose')
        starts = [a for a in s.drain() if a[0] == 'reloc_start']
        self.assertEqual(starts, [('reloc_start', 'gn_lost')], starts)

    def test_our_own_initialpose_comes_back_as_the_same_pose_also_across_the_yaw_wrap(self):
        self.assertTrue(sc.same_pose((1., 2., 3.14159), (1., 2., -3.14159)))       # 2 pi apart after the quaternion round trip
        self.assertFalse(sc.same_pose((1., 2., 0.), (1.5, 2., 0.)))
        self.assertFalse(sc.same_pose((1., 2., 0.), (1., 2., .5)))


class MovedWhileConfirmingTest(Fixture):
    """A robot that is moved right after the relocalizer's pose (a push, a kidnapping) while GN is tracking well is not a failed confirmation."""
    def relocalized(self, pose=(1., 2., .3)):
        s, t = self.new()
        run(s, t, ('on_status', 'LOST', 0., 0.))                            # lost_stop_s = 0: the stop and the relocalization start at once
        for k in range(1, 5):
            run(s, t + k, ('on_status', 'LOST', 0., 0.))                    # gn_loc keeps publishing its 1 Hz status while the relocalizer works
        run(s, t + 5, ('on_reloc_result', True, pose, 'accepted'))
        self.assertEqual((s.state, s.reason), ('CONFIRMING', 'relocalized_confirming'))
        return s, t + 5

    def good(self, s, t, pose):
        return run(s, t, ('on_status', 'TRACKING', .9, .8), ('on_gn_pose', pose))

    def test_gn_close_to_the_pose_and_then_carried_along_with_a_good_fit_is_confirmed(self):
        s, t = self.relocalized()
        self.good(s, t + 1, (1., 2., .3))                                    # GN took the pose: the lock is the relocalizer's
        self.good(s, t + 2, (1.6, 2.4, .3))                                  # the robot is pushed ~0.7 m per status (1 Hz): GN follows it
        self.good(s, t + 3, (2.2, 2.8, .3))
        self.assertEqual((s.state, s.reason), ('MONITOR', 'confirmed'))

    def test_gn_that_jumps_elsewhere_with_a_good_fit_after_the_close_sample_is_not_confirmed(self):
        s, t = self.relocalized()
        self.good(s, t + 1, (1., 2., .3))
        for k in range(2, 20):
            self.good(s, t + k, (6., 2., .3))                                # a snap to another well-fitting place (symmetric corridor): not the pose it was given
        self.assertEqual((s.state, s.reason), ('FAILED', 'gn_not_confirmed'))

    def test_gn_that_was_never_close_to_the_pose_is_still_not_confirmed(self):
        s, t = self.relocalized()
        for k in range(1, 20):
            self.good(s, t + k, (5., 5., 0.))                                # a good fit somewhere else: a lock on a pose nobody gave
        self.assertEqual((s.state, s.reason), ('FAILED', 'gn_not_confirmed'))

    def test_a_poor_fit_after_the_close_sample_does_not_confirm(self):
        s, t = self.relocalized()
        self.good(s, t + 1, (1., 2., .3))
        for k in range(2, 6):
            run(s, t + k, ('on_status', 'TRACKING', .9, .3), ('on_gn_pose', (3., 4., 0.)))      # moved and GN lost its quality
        self.assertEqual(s.state, 'CONFIRMING')


class LockGateTest(Fixture):
    def test_the_lock_follows_a_speed_not_a_distance_per_sample(self):
        # samples at 10 Hz: GN may follow a robot pushed at ~1 m/s (0.1 m per sample) but not slide 0.5 m between two samples
        s, t = self.new()
        run(s, t, ('on_status', 'LOST', .1, .1))
        run(s, t + 1, ('on_reloc_result', True, (2.5, 2., 0.), 'accepted'))
        t0 = t + 1.6
        confirmed = False
        for k in range(40):
            x = 2.5 + (0.05 * k if k < 3 else 0.15 + 0.5 * (k - 2))           # 0.5 m/s for 0.3 s, then 5 m/s
            run(s, t0 + .1 * k, ('on_scan_fit', 'TRACKING', .9, .8), ('on_gn_pose', (x, 2., 0.)))
            confirmed = confirmed or (s.state, s.reason) == ('MONITOR', 'confirmed')
        self.assertFalse(confirmed)                                          # the 5 m/s slide broke the lock: the run of good samples started again every time


class LockAcrossBadSamplesTest(Fixture):
    def test_a_robot_that_goes_on_moving_through_a_few_bad_samples_is_still_followed(self):
        s, t = self.new()
        run(s, t, ('on_status', 'LOST', .1, .1))
        run(s, t + 1, ('on_reloc_result', True, (2.5, 2., 0.), 'accepted'))
        t0, confirmed = t + 1.6, False
        for k in range(60):
            tight = .3 if 4 <= k < 7 else .8                                  # a momentary dip of the fit while the robot is pushed at 0.3 m/s
            if k % 10 == 0:
                sc.Supervisor.on_status(s, t0 + .1 * k, 'TRACKING', .9, tight, k)       # the 1 Hz heartbeat
            run(s, t0 + .1 * k, ('on_scan_fit', 'TRACKING', .9, tight), ('on_gn_pose', (2.5 + .03 * k, 2., 0.)))
            confirmed = confirmed or (s.state, s.reason) == ('MONITOR', 'confirmed')
        self.assertTrue(confirmed)


class ScanRateConfirmTest(Fixture):
    """Confirmation counts one sample per GN scan (~10 Hz): a run of good samples at least confirm_s long, so the relocalized pose is released ~1 s after GN takes it, not after three 1 Hz statuses."""
    def relocalized(self, pose=(2.5, 2., 0.)):
        s, t = self.new()
        run(s, t, ('on_status', 'LOST', .1, .1))
        run(s, t + 1, ('on_reloc_result', True, pose, 'accepted'))
        self.assertEqual(s.state, 'CONFIRMING')
        return s, t + 1

    def scan(self, s, t, pose=(2.5, 2., 0.), tight=.8, state='TRACKING'):
        self.n = getattr(self, 'n', 0) + 1
        if self.n % 10 == 1:                                                 # gn_loc keeps publishing its 1 Hz status (silence would be gn_status_stale)
            sc.Supervisor.on_status(s, t, state, .9, tight, self.n)
        return run(s, t, ('on_scan_fit', state, .9, tight), ('on_gn_pose', pose))

    def test_a_second_of_good_scans_confirms(self):
        s, t = self.relocalized()
        t0 = t + .6                                                          # confirm_lag_s = 0.5
        for k in range(10):
            self.scan(s, t0 + .1 * k)
        self.assertEqual(s.state, 'CONFIRMING')                              # 0.9 s of samples: not yet
        self.scan(s, t0 + 1.0)
        self.assertEqual((s.state, s.reason), ('MONITOR', 'confirmed'))

    def test_confirmation_takes_about_a_second_not_three(self):
        s, t = self.relocalized()
        k = 0
        while s.state == 'CONFIRMING' and k < 100:
            self.scan(s, t + .1 * k)
            k += 1
        self.assertEqual(s.state, 'MONITOR')
        self.assertLess(.1 * k, 1.8)

    def test_one_bad_scan_restarts_the_run(self):
        s, t = self.relocalized()
        t0 = t + .6
        for k in range(8):
            self.scan(s, t0 + .1 * k)
        self.scan(s, t0 + .8, tight=.3)                                      # a poor fit
        for k in range(9, 19):
            self.scan(s, t0 + .1 * k)
        self.assertEqual(s.state, 'CONFIRMING')                              # 1.0 s since the bad one has not passed
        self.scan(s, t0 + 1.9)
        self.assertEqual(s.state, 'MONITOR')

    def test_samples_within_the_lag_of_the_pose_do_not_count(self):
        s, t = self.relocalized()
        for k in range(1, 5):                                                # 0.1 ... 0.4 s after the pose: they may describe a scan before it
            self.scan(s, t + .1 * k)
        self.assertEqual(s.state, 'CONFIRMING')
        self.assertEqual(s.good, 0)

    def test_a_jump_to_another_pose_restarts_the_run_and_never_confirms(self):
        s, t = self.relocalized()
        for k in range(6, 40):
            self.scan(s, t + .1 * k, pose=(6., 2., 0.))                      # a good fit far from the relocalizer's pose
        self.assertNotEqual(s.state, 'MONITOR')

    def test_the_1_hz_status_alone_does_not_confirm(self):
        s, t = self.relocalized()
        for k in range(1, 10):
            sc.Supervisor.on_status(s, t + k, 'TRACKING', .9, .8, k)         # the heartbeat, without the scan stream
            run(s, t + k, ('on_gn_pose', (2.5, 2., 0.)))
        self.assertNotEqual(s.state, 'MONITOR')

    def test_a_bad_status_alarm_is_still_the_status_stream_s(self):
        s, t = self.new()
        for k in range(5):
            run(s, t + k, ('on_status', 'TRACKING', .9, .2, 100 + k), ('on_gn_pose', (1., 2., 0.)))
        self.assertEqual(s.state, 'RELOCALIZING')                            # bad_stop_s alarm, unchanged by the scan stream


class FailedRetryTest(Fixture):
    """FAILED after a relocalization that did not work retries by itself: the lidar may have been uncovered, the robot parked, since."""
    def lost(self, s, t0, t1):
        for k in range(int(t0), int(t1)):
            run(s, float(k), ('on_status', 'LOST', 0., 0.))                    # gn_loc keeps publishing its 1 Hz status

    def failed(self, **kw):
        s, t = self.new(**kw)
        run(s, t, ('on_status', 'LOST', 0., 0.))
        self.lost(s, t + 1, t + 5)
        run(s, t + 5, ('on_reloc_result', False, None, 'x'))
        self.lost(s, t + 6, t + 9)
        run(s, t + 9, ('on_reloc_result', False, None, 'x'))
        self.assertEqual((s.state, s.reason), ('FAILED', 'reloc_failed_x'))
        return s, t + 9

    def test_it_starts_a_new_relocalization_30_s_after_the_failure(self):
        s, t = self.failed()
        self.lost(s, t + 1, t + 30)
        self.assertEqual(s.state, 'FAILED')
        a = run(s, t + 30., ('on_status', 'LOST', 0., 0.))
        self.assertIn(('reloc_start', 'failed_retry'), a)
        self.assertEqual((s.state, s.reason, s.hold), ('RELOCALIZING', 'failed_retry', True))

    def test_an_accepted_retry_is_confirmed_and_navigation_is_released(self):
        s, t = self.failed()
        self.lost(s, t + 1, t + 30)
        run(s, t + 30., ('on_status', 'LOST', 0., 0.))
        self.lost(s, t + 31, t + 42)
        a = run(s, t + 42., ('on_reloc_result', True, (1., 2., .3), 'accepted'))
        self.assertIn(('initialpose', (1., 2., .3)), a)
        for k in range(1, 5):
            run(s, t + 42. + k, ('on_status', 'TRACKING', .9, .8), ('on_gn_pose', (1., 2., .3)))
        self.assertEqual((s.state, s.reason, s.hold), ('MONITOR', 'confirmed', False))

    def test_a_retry_that_fails_twice_waits_another_30_s(self):
        s, t = self.failed()
        self.lost(s, t + 1, t + 30)
        run(s, t + 30., ('on_status', 'LOST', 0., 0.))
        self.lost(s, t + 31, t + 35)
        run(s, t + 35., ('on_reloc_result', False, None, 'x'))
        self.assertEqual(s.state, 'RELOCALIZING')                           # its second attempt
        self.lost(s, t + 36, t + 40)
        run(s, t + 40., ('on_reloc_result', False, None, 'x'))
        self.assertEqual(s.state, 'FAILED')
        self.lost(s, t + 41, t + 69)
        self.assertEqual(s.state, 'FAILED')
        run(s, t + 70., ('on_status', 'LOST', 0., 0.))
        self.assertEqual(s.reason, 'failed_retry')

    def failed_by(self, reason_setup):
        s, t = self.new()
        run(s, t, ('on_status', 'LOST', 0., 0.))
        self.lost(s, t + 1, t + 5)
        run(s, t + 5, ('on_reloc_result', False, None, reason_setup))
        self.lost(s, t + 6, t + 9)
        run(s, t + 9, ('on_reloc_result', False, None, reason_setup))
        return s, t + 9

    def test_a_relocalizer_exception_is_not_retried(self):
        s, t = self.failed_by('exception_in_start')
        self.assertEqual(s.state, 'FAILED')
        a = []
        for k in range(1, 130):
            a += run(s, t + float(k), ('on_status', 'LOST', 0., 0.))
        self.assertNotIn('reloc_start', kinds(a))                            # a bug would loop: each cycle restarts FAST-LIO2

    def test_a_deterministic_failure_is_not_retried_either(self):
        for why in ('search_worker_exit_1', 'exception_in_poll'):
            s, t = self.failed_by(why)
            a = []
            for k in range(1, 130):
                a += run(s, t + float(k), ('on_status', 'LOST', 0., 0.))
            self.assertNotIn('reloc_start', kinds(a), why)                    # a wrong parameter or a missing map would restart FAST-LIO2 every 30 s for ever

    def test_a_transient_failure_is_retried(self):
        for why in ('moved_since_exposure', 'ambiguous_no_alternative_margin', 'lio_restart_timeout', 'search_worker_exit_-9'):
            s, t = self.failed_by(why)
            a = []
            for k in range(1, 40):
                a += run(s, t + float(k), ('on_status', 'LOST', 0., 0.))
            self.assertIn('reloc_start', kinds(a), why)

    def test_the_retry_cycles_are_capped(self):
        s, t = self.failed(failed_retry_max=2)
        starts = 0
        for k in range(1, 400):
            tt = t + float(k)
            a = run(s, tt, ('on_status', 'LOST', 0., 0.))
            if s.state == 'RELOCALIZING':
                a += run(s, tt, ('on_reloc_result', False, None, 'x'))        # every attempt of every cycle fails
            starts += kinds(a).count('reloc_start')
        self.assertEqual(starts, 2 * 2)                                       # 2 cycles x 2 attempts, then the operator
        self.assertEqual(s.state, 'FAILED')

    def test_an_operator_pose_gives_the_retries_back(self):
        s, t = self.failed(failed_retry_max=1)
        self.lost(s, t + 1, t + 30)
        run(s, t + 30., ('on_status', 'LOST', 0., 0.))
        self.assertEqual(s.reason, 'failed_retry')
        run(s, t + 31., ('on_operator_pose', (1., 2., 0.)))
        for k in range(32, 36):
            run(s, t + float(k), ('on_status', 'TRACKING', .9, .8), ('on_gn_pose', (1., 2., 0.)))
        self.assertEqual(s.state, 'MONITOR')
        self.assertEqual(s.retries, 0)

    def test_a_failure_that_is_not_a_relocalization_one_is_never_retried(self):
        s, t = self.failed()
        self.lost(s, t + 1, t + 30)
        run(s, t + 30., ('on_status', 'LOST', 0., 0.))                        # the retry starts ...
        self.assertEqual(s.reason, 'failed_retry')
        for k in range(1, 40):
            run(s, t + 30. + k)                                                # ... and gn_loc goes silent: gn_status_stale
        self.assertEqual((s.state, s.reason), ('FAILED', 'gn_status_stale'))
        a = []
        for k in range(40, 200):
            a += run(s, t + 30. + k)
        self.assertNotIn('reloc_start', kinds(a))
        s2, t2 = self.failed()                                                # the same for the node's internal_error hold
        s2.go(t2 + 1., 'FAILED', 'internal_error'); s2.drain()
        a = []
        for k in range(2, 200):
            a += run(s2, t2 + float(k), ('on_status', 'LOST', 0., 0.))
        self.assertNotIn('reloc_start', kinds(a))

    def test_a_pose_that_gn_did_not_confirm_is_not_searched_again(self):
        s, t = self.new()
        run(s, t, ('on_status', 'LOST', 0., 0.))
        self.lost(s, t + 1, t + 5)
        run(s, t + 5, ('on_reloc_result', True, (1., 2., .3), 'accepted'))
        for k in range(1, 20):
            run(s, t + 5 + k, ('on_status', 'TRACKING', .9, .3), ('on_gn_pose', (5., 5., 0.)))
        self.assertEqual(s.reason, 'gn_not_confirmed')
        a = []
        for k in range(20, 90):
            a += run(s, t + 5 + k, ('on_status', 'TRACKING', .9, .3), ('on_gn_pose', (5., 5., 0.)))
        self.assertNotIn('reloc_start', kinds(a))                            # the search would give the same pose: the operator decides

    def test_it_can_be_switched_off(self):
        s, t = self.failed(failed_retry_s=0.)
        self.lost(s, t + 1, t + 100)
        self.assertEqual(s.state, 'FAILED')

    def test_the_operator_pose_during_the_wait_wins(self):
        s, t = self.failed()
        self.lost(s, t + 1, t + 20)
        a = run(s, t + 20., ('on_operator_pose', (1., 2., 0.)))
        self.assertEqual((s.state, s.reason), ('CONFIRMING', 'operator_pose'))
        self.assertNotIn('reloc_start', kinds(a))


class GoalHoldTest(Fixture):
    def test_a_goal_active_at_start_up_is_cancelled(self):
        s = Supervisor(sc.Params())
        self.assertIn('cancel', kinds(run(s, 0., ('on_goal_status', 'navigate_to_pose', True))))

    def test_an_active_goal_is_cancelled_again_until_it_stops(self):
        # a cancel can be lost (service not ready, request dropped): while held and a goal is still active, cancel again every recancel_s
        s, t = self.new(recancel_s=1., max_attempts=1)
        run(s, t, ('on_status', 'LOST', .1, .1), ('on_goal_status', 'navigate_to_pose', True))
        self.assertNotIn('cancel', kinds(run(s, t + .5)))
        self.assertIn('cancel', kinds(run(s, t + 1.1)))
        self.assertIn('cancel', kinds(run(s, t + 2.2)))
        run(s, t + 2.5, ('on_goal_status', 'navigate_to_pose', False), ('on_status', 'LOST', .1, .1))
        self.assertNotIn('cancel', kinds(run(s, t + 4., ('on_status', 'LOST', .1, .1))))

    def test_no_re_cancel_while_monitoring(self):
        s, t = self.new(recancel_s=1.)
        run(s, t, ('on_goal_status', 'navigate_to_pose', True))
        for k in range(1, 4):
            self.assertNotIn('cancel', kinds(run(s, t + k, ('on_status', 'TRACKING', .9, .8))))

    def test_not_stopped_is_still_reported_while_re_cancelling(self):
        s, t = self.new(stop_check_s=3., recancel_s=1.)
        run(s, t, ('on_status', 'LOST', .1, .1), ('on_wheel', .4, 0.), ('on_goal_status', 'navigate_to_pose', True))
        for k in range(1, 5):
            run(s, t + k, ('on_status', 'LOST', .1, .1), ('on_wheel', .3, 0.))
        self.assertIn('not_stopped', s.reason)


if __name__ == '__main__':
    unittest.main()
