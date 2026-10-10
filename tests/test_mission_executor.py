# AP_FLAKE8_CLEAN
import unittest
from unittest.mock import Mock, call

from drone_autonomy_lab.missions.executor import MissionExecutor
from drone_autonomy_lab.missions.models import MissionState, MissionStep


class MissionExecutorTest(unittest.TestCase):
    def setUp(self):
        self.vehicle = Mock()
        self.steps = (
            MissionStep('set_mode', {'mode': 'guided'}),
            MissionStep('arm'), MissionStep('takeoff', {'altitude_m': 10}), MissionStep('land'),
        )
        self.executor = MissionExecutor(self.vehicle)

    def test_successful_sequence_and_progress(self):
        self.assertEqual(self.executor.run(self.steps), MissionState.COMPLETED)
        self.assertEqual(self.vehicle.method_calls, [
            call.set_mode(mode='GUIDED'), call.arm(), call.takeoff(altitude_m=10), call.land(),
        ])
        self.assertEqual(self.executor.completed_steps, list(self.steps))
        self.assertIsNone(self.executor.current_step)
        self.assertIsNone(self.executor.error)

    def test_failure_stops_later_actions_and_preserves_original_error(self):
        error = TimeoutError('No altitude confirmation')
        self.vehicle.takeoff.side_effect = error
        with self.assertRaises(TimeoutError) as caught:
            self.executor.run(self.steps)
        self.assertIs(caught.exception, error)
        self.assertEqual(self.executor.state, MissionState.FAILED)
        self.assertEqual(self.executor.completed_steps, list(self.steps[:2]))
        self.assertIs(self.executor.current_step, self.steps[2])
        self.assertIs(self.executor.error, error)
        self.vehicle.land.assert_not_called()

    def test_interruption_is_reported_and_does_not_continue(self):
        self.vehicle.arm.side_effect = KeyboardInterrupt()
        with self.assertRaises(KeyboardInterrupt):
            self.executor.run(self.steps)
        self.assertEqual(self.executor.state, MissionState.INTERRUPTED)
        self.vehicle.takeoff.assert_not_called()

    def test_execution_state_visible_during_action(self):
        def confirm_arm():
            self.assertEqual(self.executor.state, MissionState.RUNNING)
            self.assertIs(self.executor.current_step, self.steps[1])
            self.assertEqual(self.executor.completed_steps, [self.steps[0]])
        self.vehicle.arm.side_effect = confirm_arm
        self.executor.run(self.steps)

    def test_cannot_silently_replay_mission(self):
        self.executor.run(self.steps)
        with self.assertRaises(RuntimeError):
            self.executor.run(self.steps)
        self.vehicle.arm.assert_called_once()

    def test_invalid_plan_sends_no_commands(self):
        for steps in ([], [self.steps[0], 'arm']):
            with self.subTest(steps=steps), self.assertRaises(ValueError):
                self.executor.run(steps)
        self.assertEqual(self.vehicle.method_calls, [])
        self.assertEqual(self.executor.state, MissionState.READY)

    def test_invalid_step_inputs_rejected_before_execution(self):
        for action, parameters in (
            ('close', {}), ('arm', {'force': True}), ('set_mode', {}),
            ('takeoff', {}), ('takeoff', {'altitude_m': float('nan')}),
            ('takeoff', {'altitude_m': 0}), ('arm', {'timeout': float('inf')}),
            ('land', {'timeout': -1}), ('takeoff', {'altitude_m': 10, 'tolerance_m': 10}),
        ):
            with self.subTest(action=action, parameters=parameters), self.assertRaises(ValueError):
                MissionStep(action, parameters)
        self.assertEqual(self.vehicle.method_calls, [])

    def test_step_parameters_are_copied_and_immutable(self):
        parameters = {'altitude_m': 10}
        step = MissionStep('takeoff', parameters)
        parameters['altitude_m'] = -1
        self.assertEqual(step.parameters['altitude_m'], 10)
        with self.assertRaises(TypeError):
            step.parameters['altitude_m'] = 20


if __name__ == '__main__':
    unittest.main()
