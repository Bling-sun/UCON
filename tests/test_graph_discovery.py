"""Exercise the partial graph discovery that caused the Actions smoke failure."""

from types import SimpleNamespace
import unittest
from unittest.mock import Mock

from test_simulation_smoke import resolved_subscription_names


class GraphDiscoveryTest(unittest.TestCase):
    def test_waits_for_all_endpoint_names(self):
        planner = SimpleNamespace(node_name='fapp_planner_node')
        unknown = SimpleNamespace(node_name='_NODE_NAME_UNKNOWN_')
        node = Mock()
        node.get_subscriptions_info_by_topic.side_effect = [
            [], [unknown], [planner, unknown], [planner],
        ]
        for _ in range(3):
            self.assertIsNone(resolved_subscription_names(node, '/move_base_simple/goal'))
        self.assertEqual(resolved_subscription_names(node, '/move_base_simple/goal'),
                         ['fapp_planner_node'])

    def test_preserves_unexpected_subscribers(self):
        node = Mock()
        node.get_subscriptions_info_by_topic.return_value = [
            SimpleNamespace(node_name='random_map_sensing'),
            SimpleNamespace(node_name='fapp_planner_node'),
        ]
        self.assertEqual(resolved_subscription_names(node, '/move_base_simple/goal'),
                         ['fapp_planner_node', 'random_map_sensing'])

    def test_preserves_duplicate_subscribers(self):
        node = Mock()
        node.get_subscriptions_info_by_topic.return_value = [
            SimpleNamespace(node_name='fapp_planner_node'),
            SimpleNamespace(node_name='fapp_planner_node'),
        ]
        self.assertEqual(resolved_subscription_names(node, '/move_base_simple/goal'),
                         ['fapp_planner_node', 'fapp_planner_node'])


if __name__ == '__main__':
    unittest.main()
