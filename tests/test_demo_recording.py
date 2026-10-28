"""Check the offline evidence used to select the recorded avoidance demo."""

import copy
import importlib.util
from pathlib import Path
import unittest

module_path = Path(__file__).resolve().parents[1] / 'scripts/record_demo.py'
spec = importlib.util.spec_from_file_location('record_demo', module_path)
recorder = importlib.util.module_from_spec(spec)
spec.loader.exec_module(recorder)


class DemoEvidenceTest(unittest.TestCase):
    def setUp(self):
        # x=t crosses a moving box in polynomial piece 2; the flown path is y=2.
        self.data = {
            'recording_start': 0.,
            'odom': [[0., 0., 2., .4], [2., 2., 2., .4]],
            'trajectories': [
                {'stamp': 0., 'duration': [1., 1.],
                 'coef_x': [0., 0., 0., 0., 1., 0., 0., 0., 0., 0., 1., 1.],
                 'coef_y': [0.] * 12,
                 'coef_z': [0., 0., 0., 0., 0., .4] * 2},
                {'stamp': .5}],
            'truth': [[1.5, [[1.5, 0., .4, .3, .3, .3, .1, 0., 0.]]]],
            'goals': [],
        }

    def test_piece_boundary_and_flown_clearance(self):
        events = recorder.avoided_conflicts(self.data)
        self.assertEqual(len(events), 1)
        self.assertEqual(events[0]['previous_plan_clearance_m'], 0.)
        self.assertAlmostEqual(events[0]['flown_clearance_m'], 1.85)
        self.assertEqual(events[0]['object_id'], 0)

    def test_goal_switch_is_not_avoidance(self):
        self.data['goals'] = [[.4, [5., 0., .4]]]
        self.assertEqual(recorder.avoided_conflicts(self.data), [])

    def test_future_goal_switch_ends_comparison(self):
        self.data['goals'] = [[1.4, [5., 2., .4]]]
        self.assertEqual(recorder.avoided_conflicts(self.data), [])

    def test_checks_remaining_plan_beyond_two_seconds(self):
        self.data['trajectories'][0]['duration'] = [1., 3.]
        self.data['odom'] = [[0., 0., 2., .4], [4., 4., 2., .4]]
        self.data['truth'] = [[2.75, [[2.75, 0., .4, .3, .3, .3, .1, 0., 0.]]]]
        events = recorder.avoided_conflicts(self.data)
        self.assertEqual(len(events), 1)
        self.assertEqual(events[0]['previous_plan_clearance_m'], 0.)

    def test_stationary_object_is_not_dynamic_avoidance(self):
        self.data['truth'][0][1][0][6] = 0.
        self.assertEqual(recorder.avoided_conflicts(self.data), [])

    def test_actual_contact_is_not_success(self):
        self.data['odom'] = [[0., 0., 0., .4], [2., 2., 0., .4]]
        self.assertEqual(recorder.avoided_conflicts(self.data), [])

    def test_repeated_samples_do_not_count_as_multiple_obstacles(self):
        self.data['truth'].append(copy.deepcopy(self.data['truth'][0]))
        self.assertEqual(len(recorder.avoided_conflicts(self.data)), 1)


if __name__ == '__main__':
    unittest.main()
