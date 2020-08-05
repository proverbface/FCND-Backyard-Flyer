import unittest
from unittest.mock import patch, MagicMock

from backyard_flyer import BackyardFlyer, States


class TestBackyardFlyer(unittest.TestCase):

    def setUp(self):
        connection_instance = MagicMock()
        connection_instance.execute.return_value = "testing"
        self.flyer = BackyardFlyer(connection_instance)

    def test_calculate_box(self):
        waypoints = self.flyer.calculate_box()
        self.assertEqual([
            [15.0, 0.0, 10.0],
            [15.0, 15.0, 10.0],
            [0.0, 15.0, 10.0],
            [0.0, 0.0, 10.0]
        ], waypoints)

    @patch.object(BackyardFlyer, 'take_control')
    @patch.object(BackyardFlyer, 'arm')
    @patch.object(BackyardFlyer, 'set_home_position')
    def test_arming_transition(self, mock_set_home_position, mock_arm, mock_take_control):
        self.flyer.arming_transition()

        mock_take_control.assert_called_once()
        mock_arm.assert_called_once()
        mock_set_home_position.assert_called_once_with(0, 0, 0)
        self.assertEqual(self.flyer.flight_state, States.ARMING)

    @patch.object(BackyardFlyer, 'takeoff')
    def test_takeoff_transition(self, mock_takeoff):
        self.flyer.takeoff_transition()

        mock_takeoff.assert_called_once_with(3.0)
        self.assertEqual(self.flyer.flight_state, States.TAKEOFF)

    @patch.object(BackyardFlyer, 'land')
    def test_landing_transition(self, mock_land):
        self.flyer.landing_transition()

        mock_land.assert_called_once()
        self.assertEqual(self.flyer.flight_state, States.LANDING)

    @patch.object(BackyardFlyer, 'release_control')
    @patch.object(BackyardFlyer, 'disarm')
    def test_disarming_transition(self, mock_disarm, mock_release_control):
        self.flyer.disarming_transition()

        mock_disarm.assert_called_once()
        mock_release_control.assert_called_once()
        self.assertEqual(self.flyer.flight_state, States.DISARMING)

    @patch.object(BackyardFlyer, 'release_control')
    @patch.object(BackyardFlyer, 'stop')
    def test_manual_transition(self, mock_stop, mock_release_control):
        self.flyer.manual_transition()

        mock_release_control.assert_called_once()
        mock_stop.assert_called_once()
        self.assertEqual(False, self.flyer.in_mission)
        self.assertEqual(self.flyer.flight_state, States.MANUAL)


if __name__ == '__main__':
    unittest.main()
