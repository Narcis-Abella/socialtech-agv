import math
import unittest

import gn_supervisor_core as sc


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
        s = sc.Supervisor(sc.Params(**kw))
        t = self.tracking(s, 0.)
        self.assertEqual(s.state, 'MONITOR')
        return s, t


class StartupTest(Fixture):
    def test_navigation_is_held_until_gn_confirms_a_pose(self):
        s = sc.Supervisor(sc.Params())
        self.assertTrue(s.hold)
        run(s, 0., ('on_status', 'WAITING_POSE', 0., 0.))
        run(s, 1., ('on_status', 'TRACKING', .9, .3))       # a pose that does not fit the map yet
        self.assertTrue(s.hold)
        self.tracking(s, 2.)
        self.assertFalse(s.hold)


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
        s = sc.Supervisor(sc.Params())
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
        for k in range(6):
            run(s, t + k, ('on_wheel', 0., 0.), ('on_status', 'TRACKING', 0., 0.))
        self.assertEqual(s.state, 'RELOCALIZING')
        self.assertTrue(s.hold)


class GoalHoldTest(Fixture):
    def test_a_goal_active_at_start_up_is_cancelled(self):
        s = sc.Supervisor(sc.Params())
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
