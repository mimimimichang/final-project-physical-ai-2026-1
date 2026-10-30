import argparse
import csv
import io
import math
import json
from pathlib import Path
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import patch
import xml.etree.ElementTree as ET

from motion_demo import FIELDS, BYPASS_CLEARANCE, BYPASS_TIMEOUT_NS, BYPASS_SPEED, BYPASS_CLEAR_SCANS
from motion_demo import bypass_decision, choose_turn_direction, command_at, final_forward_decision, finish_time
from motion_demo import front_min_range, main, parse_plan, positive_number, run, run_offline, side_min_range
from motion_demo import AvoidanceController, TURN_DURATION, SCAN_TIMEOUT_NS


class MotionTests(unittest.TestCase):
    def args(self, **changes):
        values = dict(mode="smooth", speed=0.3, duration=4.0, acceleration=0.15, period=0.05)
        values.update(changes)
        return argparse.Namespace(**values)

    def test_step_changes_immediately_and_stops_at_duration(self):
        self.assertEqual(command_at(0, "step", 0.3, 4, 0.15), (0.3, 0.3))
        self.assertEqual(command_at(3.99, "step", 0.3, 4, 0.15), (0.3, 0.3))
        self.assertEqual(command_at(4, "step", 0.3, 4, 0.15), (0, 0))

    def test_smooth_accelerates_holds_and_stops(self):
        for elapsed, expected in ((0, 0), (1, 0.15), (2, 0.3), (4, 0.3), (5, 0.15), (6, 0)):
            with self.subTest(elapsed=elapsed):
                self.assertAlmostEqual(command_at(elapsed, "smooth", 0.3, 4, 0.15)[1], expected)

    def test_short_forward_command_stops_from_reached_speed(self):
        self.assertAlmostEqual(command_at(1, "smooth", 0.3, 1, 0.15)[1], 0.15)
        self.assertAlmostEqual(command_at(1.5, "smooth", 0.3, 1, 0.15)[1], 0.075)
        self.assertEqual(command_at(2, "smooth", 0.3, 1, 0.15), (0, 0))
        self.assertEqual(finish_time(self.args(duration=1)), 2)

    def test_irregular_sampling_respects_acceleration_and_bounds(self):
        times = (0, 0.037, 0.2, 1.95, 2.03, 3.99, 4.01, 4.8, 6.01)
        speeds = [command_at(elapsed, "smooth", 0.3, 4, 0.15)[1] for elapsed in times]
        for previous, current, previous_time, current_time in zip(speeds, speeds[1:], times, times[1:]):
            self.assertLessEqual(abs(current - previous), 0.15 * (current_time - previous_time) + 1e-12)
        self.assertTrue(all(0 <= speed <= 0.3 for speed in speeds))

    def test_csv_modes_share_target_schedule_and_end_at_zero(self):
        targets = []
        for mode in ("step", "smooth"):
            stream = io.StringIO()
            writer = csv.writer(stream)
            writer.writerow(FIELDS)
            run_offline(self.args(mode=mode), writer)
            stream.seek(0)
            rows = list(csv.DictReader(stream))
            self.assertEqual(float(rows[-1]["command_mps"]), 0)
            self.assertEqual(float(rows[-1]["target_mps"]), 0)
            targets.append([(row["time_s"], row["target_mps"]) for row in rows
                            if float(row["time_s"]) < 4])
        self.assertEqual(targets[0], targets[1])

    def test_invalid_cli_numbers(self):
        for value in ("0", "-1", "nan", "inf", "-inf"):
            with self.subTest(value=value), self.assertRaises(argparse.ArgumentTypeError):
                positive_number(value)
        self.assertTrue(math.isfinite(positive_number("0.15")))

    def test_front_sector_uses_nearest_center_ray_and_ignores_side_rays(self):
        ranges = [4.0] * 181
        ranges[90] = 0.72
        ranges[30] = 0.1
        scan = SimpleNamespace(
            angle_min=-math.pi / 2,
            angle_increment=math.pi / 180,
            range_min=0.08,
            range_max=5.0,
            ranges=ranges,
        )
        self.assertAlmostEqual(front_min_range(scan), 0.72)

    def test_front_sector_ignores_nan_infinity_and_out_of_range(self):
        ranges = [float("nan")] * 181
        ranges[90] = float("inf")
        ranges[91] = 0.01
        ranges[89] = 4.9
        scan = SimpleNamespace(
            angle_min=-math.pi / 2,
            angle_increment=math.pi / 180,
            range_min=0.08,
            range_max=5.0,
            ranges=ranges,
        )
        self.assertAlmostEqual(front_min_range(scan), 4.9)

    def test_front_sector_returns_none_when_no_valid_center_measurement(self):
        scan = SimpleNamespace(
            angle_min=-math.pi / 2,
            angle_increment=math.pi / 180,
            range_min=0.08,
            range_max=5.0,
            ranges=[float("nan")] * 181,
        )
        self.assertIsNone(front_min_range(scan))

    def test_front_sector_rejects_zero_angle_increment(self):
        scan = SimpleNamespace(
            angle_min=0,
            angle_increment=0,
            range_min=0.08,
            range_max=5.0,
            ranges=[1.0],
        )
        self.assertIsNone(front_min_range(scan))

    def test_turn_direction_chooses_side_with_more_clearance(self):
        ranges = [5.0] * 181
        ranges[30] = 2.0
        ranges[150] = 0.5
        scan = SimpleNamespace(
            angle_min=-math.pi / 2,
            angle_increment=math.pi / 180,
            range_min=0.08,
            range_max=5.0,
            ranges=ranges,
        )
        self.assertEqual(choose_turn_direction(scan), -1)

    def test_turn_direction_tie_defaults_left(self):
        scan = SimpleNamespace(
            angle_min=-math.pi / 2,
            angle_increment=math.pi / 180,
            range_min=0.08,
            range_max=5.0,
            ranges=[5.0] * 181,
        )
        self.assertEqual(choose_turn_direction(scan), 1)

    def test_turn_direction_stops_when_both_sides_are_blocked(self):
        ranges = [0.2] * 181
        ranges[90] = 0.3
        scan = SimpleNamespace(
            angle_min=-math.pi / 2,
            angle_increment=math.pi / 180,
            range_min=0.08,
            range_max=5.0,
            ranges=ranges,
        )
        self.assertIsNone(choose_turn_direction(scan))

    def test_side_min_range_excludes_forward_and_accepts_clear_side(self):
        ranges = [5.0] * 181
        ranges[90] = 0.12
        ranges[30] = 0.8
        ranges[150] = 0.9
        scan = SimpleNamespace(
            angle_min=-math.pi / 2,
            angle_increment=math.pi / 180,
            range_min=0.08,
            range_max=5.0,
            ranges=ranges,
        )
        self.assertAlmostEqual(side_min_range(scan), 0.8)

    def test_side_min_range_filters_near_robot_self_returns(self):
        ranges = [5.0] * 181
        ranges[30] = 0.08
        ranges[150] = 0.08
        ranges[40] = 0.28
        scan = SimpleNamespace(
            angle_min=-math.pi / 2,
            angle_increment=math.pi / 180,
            range_min=0.08,
            range_max=5.0,
            ranges=ranges,
        )
        self.assertAlmostEqual(side_min_range(scan), 0.28)

    def test_bypass_stops_if_front_obstacle_reappears(self):
        decision = bypass_decision(0.25, 1.0, 1_000_000_000, 0, 0.30, 0.15)
        self.assertEqual(decision.state, "SAFE_STOP_BYPASS_OBSTACLE")
        self.assertEqual(decision.command, 0)
        self.assertEqual(decision.stop_reason, "obstacle_in_bypass_path")

    def test_bypass_stops_on_invalid_side_scan(self):
        decision = bypass_decision(1.0, None, 1_000_000_000, 0, 0.30, 0.15)
        self.assertEqual(decision.state, "SAFE_STOP_SCAN_INVALID")
        self.assertEqual(decision.command, 0)

    def test_bypass_stops_at_timeout(self):
        decision = bypass_decision(1.0, 0.2, BYPASS_TIMEOUT_NS, 0, 0.30, 0.15)
        self.assertEqual(decision.state, "SAFE_STOP_BYPASS_TIMEOUT")
        self.assertEqual(decision.command, 0)

    def test_bypass_requires_consecutive_side_clear_scans(self):
        decisions = []
        clear_scans = 0
        for index in range(BYPASS_CLEAR_SCANS):
            decision = bypass_decision(
                1.0, BYPASS_CLEARANCE, 1_000_000_000 + index * 50_000_000,
                clear_scans, 0.30, 0.15,
            )
            decisions.append(decision)
            clear_scans = decision.clear_scans
        self.assertTrue(all(decision.state == "BYPASS_CLEAR_CHECK" for decision in decisions[:-1]))
        complete = decisions[-1]
        self.assertEqual(complete.state, "BYPASS_COMPLETE")
        self.assertEqual(complete.command, 0)

    def test_bypass_resets_clear_counter_and_ramps_at_limited_speed(self):
        decision = bypass_decision(1.0, BYPASS_CLEARANCE - 0.01, 2_000_000_000, 2, 0.30, 0.08)
        self.assertEqual(decision.state, "BYPASS")
        self.assertEqual(decision.clear_scans, 0)
        self.assertAlmostEqual(decision.command, BYPASS_SPEED)
        self.assertEqual(decision.target, BYPASS_SPEED)

    def test_final_forward_ramps_at_bypass_speed_limit(self):
        decision = final_forward_decision(1.0, 1.0, 2.0, 0.30, 0.15)
        self.assertEqual(decision.state, "FINAL_FORWARD")
        self.assertAlmostEqual(decision.command, BYPASS_SPEED)
        self.assertEqual(decision.target, BYPASS_SPEED)

    def test_final_forward_stops_if_obstacle_is_ahead(self):
        decision = final_forward_decision(0.25, 0.5, 2.0, 0.30, 0.15)
        self.assertEqual(decision.state, "SAFE_STOP_FINAL_OBSTACLE")
        self.assertEqual(decision.command, 0)
        self.assertEqual(decision.stop_reason, "obstacle_in_final_path")

    def test_final_forward_stops_when_duration_expires(self):
        decision = final_forward_decision(1.0, 2.0, 2.0, 0.30, 0.15)
        self.assertEqual(decision.state, "FINAL_FORWARD_COMPLETE")
        self.assertEqual(decision.command, 0)
        self.assertEqual(decision.stop_reason, "final_forward_complete")


class AvoidanceTests(unittest.TestCase):
    def controller(self, **changes):
        settings = dict(
            mode="smooth", speed=0.3, duration=6, acceleration=0.15,
            stop_distance=0.3, turn_after_stop=True, bypass_after_turn=True,
            max_obstacles=2, max_runtime=30, final_forward_duration=2,
        )
        settings.update(changes)
        return AvoidanceController(argparse.Namespace(**settings), 10_000_000_000)

    def advance(self, controller, seconds, front=1.0, side=1.0):
        now_ns = controller.start_ns + round(seconds * 1e9)
        scan = SimpleNamespace(
            angle_min=-math.pi / 2, angle_increment=math.pi / 180,
            range_min=0.08, range_max=5.0, ranges=[5.0] * 181,
        )
        return controller.update(now_ns, front, side, scan, now_ns)

    def clear_obstacle(self, controller, seconds):
        decision = self.advance(controller, seconds, front=0.25)
        self.assertEqual(decision.state, "OBSTACLE_STOP")
        self.assertEqual(self.advance(controller, seconds + 0.5).state, "TURN_LEFT")
        bypass_start = seconds + TURN_DURATION
        self.assertEqual(self.advance(controller, bypass_start).state, "BYPASS_START")
        for index in range(BYPASS_CLEAR_SCANS):
            decision = self.advance(controller, bypass_start + (index + 1) * 0.1)
        self.assertEqual(decision.state, "BYPASS_COMPLETE")
        self.assertEqual(decision.command, 0)
        return bypass_start + BYPASS_CLEAR_SCANS * 0.1

    def test_two_obstacles_reset_forward_timer_then_finish_final_forward(self):
        controller = self.controller()
        self.assertEqual(self.advance(controller, 0).state, "FORWARD")
        first_complete = self.clear_obstacle(controller, 4.0)
        self.assertEqual(controller.obstacles_cleared, 1)
        self.assertAlmostEqual(self.advance(controller, first_complete + 0.5).motion_elapsed, 0.5)
        second_complete = self.clear_obstacle(controller, 8.0)
        self.assertEqual(controller.obstacles_cleared, 2)
        self.assertEqual(controller.phase, "FINAL_FORWARD")
        self.assertIsNone(controller.stop_reason)
        self.assertEqual(self.advance(controller, second_complete + 0.1).state, "FINAL_FORWARD")
        decision = self.advance(controller, second_complete + 2.0)
        self.assertEqual(decision.state, "FINAL_FORWARD_COMPLETE")
        self.assertEqual(decision.command, 0)
        self.assertEqual(controller.stop_reason, "final_forward_complete")

    def test_zero_final_duration_stops_at_target_count(self):
        controller = self.controller(max_obstacles=1, final_forward_duration=0)
        self.clear_obstacle(controller, 4.0)
        self.assertEqual(controller.obstacles_cleared, 1)
        self.assertEqual(controller.stop_reason, "max_obstacles_reached")

    def test_stop_plan_overrides_obstacle_avoidance_and_missing_scan(self):
        for phase in ("FORWARD", "TURN", "BYPASS", "FINAL_FORWARD"):
            for front in (0.25, 1.0, None):
                with self.subTest(phase=phase, front=front):
                    controller = self.controller(speed=0.0, duration=0.0)
                    controller.phase = phase
                    decision = self.advance(controller, 0, front=front)
                    self.assertEqual(decision.state, "STOP_COMMAND")
                    self.assertEqual(decision.command, 0)
                    self.assertEqual(decision.angular, 0)
                    self.assertEqual(controller.stop_reason, "stop_requested")

    def test_bypass_duration_expiry_is_incomplete_before_target_count(self):
        for cleared in (0, 1):
            with self.subTest(cleared=cleared):
                controller = self.controller()
                controller.obstacles_cleared = cleared
                decision = self.advance(controller, 8.0)
                self.assertEqual(decision.state, "TASK_INCOMPLETE")
                self.assertEqual(decision.command, 0)
                self.assertEqual(decision.angular, 0)
                self.assertEqual(controller.stop_reason, "obstacles_not_cleared")

    def test_open_loop_modes_keep_the_original_duration(self):
        for mode, end in (("step", 4.0), ("smooth", 6.0)):
            with self.subTest(mode=mode):
                controller = self.controller(mode=mode, duration=4, stop_distance=None)
                decision = self.advance(controller, end - 0.1)
                self.assertEqual(decision.state, "OPEN_LOOP")
                self.assertIsNone(decision.command)
                self.assertAlmostEqual(decision.motion_elapsed, end - 0.1)
                decision = self.advance(controller, end)
                self.assertEqual(decision.state, "DURATION_COMPLETE")
                self.assertEqual(decision.command, 0)
                self.assertEqual(controller.stop_reason, "duration_complete")

    def test_stop_only_and_turn_only_modes(self):
        for turn_after_stop in (False, True):
            with self.subTest(turn_after_stop=turn_after_stop):
                controller = self.controller(turn_after_stop=turn_after_stop, bypass_after_turn=False)
                self.assertEqual(self.advance(controller, 4.0, front=0.25).state, "OBSTACLE_STOP")
                if turn_after_stop:
                    self.assertIsNone(controller.stop_reason)
                    decision = self.advance(controller, 4.0 + TURN_DURATION)
                    self.assertEqual(decision.state, "TURN_COMPLETE")
                    self.assertEqual(decision.command, 0)
                    self.assertEqual(controller.stop_reason, "turn_complete")
                else:
                    self.assertEqual(controller.stop_reason, "obstacle_detected")
                self.assertEqual(controller.obstacles_cleared, 0)

    def test_repeated_timer_ticks_do_not_count_the_same_scan_again(self):
        controller = self.controller()
        self.advance(controller, 4.0, front=0.25)
        self.advance(controller, 4.0 + TURN_DURATION)
        now_ns = controller.start_ns + 6_000_000_000
        scan = SimpleNamespace()
        for index in range(BYPASS_CLEAR_SCANS + 1):
            decision = controller.update(now_ns + index * 50_000_000, 1.0, 1.0, scan, now_ns)
            self.assertEqual(decision.state, "BYPASS_CLEAR_CHECK")
            self.assertEqual(controller.clear_scans, 1)
            self.assertEqual(controller.obstacles_cleared, 0)

    def test_scan_loss_and_runtime_limit_stop_during_turn(self):
        controller = self.controller()
        self.advance(controller, 4.0, front=0.25)
        now_ns = controller.start_ns + 5_000_000_000
        decision = controller.update(now_ns, 1.0, 1.0, None, now_ns - SCAN_TIMEOUT_NS - 1)
        self.assertEqual(decision.state, "SAFE_STOP_SCAN_LOST")
        self.assertEqual(decision.command, 0)
        controller = self.controller()
        self.advance(controller, 29.5, front=0.25)
        decision = self.advance(controller, 30.0)
        self.assertEqual(decision.state, "TASK_TIMEOUT")
        self.assertEqual(decision.command, 0)


class PlanTests(unittest.TestCase):
    def test_allowed_forward_settings(self):
        for mode in ("slow", "normal"):
            for duration in (2, 4, 6):
                plan = dict(action="forward", speed_mode=mode, duration_s=duration)
                self.assertEqual(parse_plan(json.dumps(plan)), plan)

    def test_stop(self):
        plan = dict(action="stop", speed_mode="none", duration_s=0)
        self.assertEqual(parse_plan(json.dumps(plan)), plan)

    def test_rejects_invalid_structure_and_values(self):
        invalid = [
            None, [], {},
            dict(action="forward", speed_mode="slow"),
            dict(action="forward", speed_mode="slow", duration_s=4, extra=1),
            dict(action="forward", speed_mode="fast", duration_s=4),
            dict(action="forward", speed_mode="none", duration_s=4),
            dict(action="forward", speed_mode="slow", duration_s=30),
            dict(action="forward", speed_mode="slow", duration_s=-2),
            dict(action="forward", speed_mode="slow", duration_s=True),
            dict(action="forward", speed_mode="slow", duration_s=4.0),
            dict(action="forward", speed_mode="slow", duration_s="4"),
            dict(action="forward", speed_mode=[], duration_s=4),
            dict(action=[], speed_mode="slow", duration_s=4),
            dict(action="backward", speed_mode="slow", duration_s=4),
            dict(action="reject", speed_mode="none", duration_s=0),
            dict(action="stop", speed_mode="slow", duration_s=0),
            dict(action="stop", speed_mode="none", duration_s=2),
        ]
        for plan in invalid:
            with self.subTest(plan=plan), self.assertRaises(ValueError):
                parse_plan(json.dumps(plan))

    def test_rejects_malformed_nonfinite_and_duplicate_json(self):
        for text in (
            "not json", '```json\n{}\n```',
            '{"action":"forward","speed_mode":"slow","duration_s":NaN}',
            '{"action":"forward","speed_mode":"slow","duration_s":Infinity}',
            '{"action":"forward","action":"stop","speed_mode":"none","duration_s":0}',
        ):
            with self.subTest(text=text), self.assertRaises(ValueError):
                parse_plan(text)

    def execute(self, folder, plan, **changes):
        path = folder / "plan.json"
        path.write_text(json.dumps(plan), encoding="utf-8")
        settings = dict(plan=path, mode="smooth", speed=None, duration=None,
                        acceleration=0.15, period=0.05, dry_run=True, stop_distance=None,
                        turn_after_stop=False, bypass_after_turn=False,
                        max_obstacles=1, max_runtime=30.0, final_forward_duration=2.0,
                        output=folder / "motion.csv")
        settings.update(changes)
        args = argparse.Namespace(**settings)
        run(args)
        return args

    def test_plan_drives_csv_and_execution_record(self):
        with tempfile.TemporaryDirectory() as directory:
            folder = Path(directory)
            plan = dict(action="forward", speed_mode="slow", duration_s=2)
            args = self.execute(folder, plan)
            with args.output.open() as stream:
                rows = list(csv.DictReader(stream))
            self.assertAlmostEqual(max(float(row["command_mps"]) for row in rows), 0.15)
            self.assertEqual(float(rows[-1]["time_s"]), 3.0)
            self.assertEqual(float(rows[-1]["command_mps"]), 0)
            record = json.loads((folder / "motion.csv.plan.json").read_text())
            self.assertEqual(record["plan"], plan)
            self.assertEqual(record["status"], "completed")
            self.assertTrue(record["dry_run"])

    def test_stop_never_generates_forward_velocity(self):
        with tempfile.TemporaryDirectory() as directory:
            args = self.execute(Path(directory), dict(action="stop", speed_mode="none", duration_s=0))
            with args.output.open() as stream:
                rows = list(csv.DictReader(stream))
            self.assertTrue(rows)
            self.assertTrue(all(float(row["command_mps"]) == 0 for row in rows))

    def test_ros_incomplete_and_safe_stops_abort_execution_record(self):
        for reason in ("obstacles_not_cleared", "duration_complete", "task_timeout", "scan_lost"):
            with self.subTest(reason=reason), tempfile.TemporaryDirectory() as directory:
                folder = Path(directory)
                with patch("motion_demo.run_ros", return_value=reason), self.assertRaises(RuntimeError):
                    self.execute(
                        folder, dict(action="forward", speed_mode="normal", duration_s=6),
                        dry_run=False, stop_distance=0.3, bypass_after_turn=True, max_obstacles=2,
                    )
                record = json.loads((folder / "motion.csv.plan.json").read_text())
                self.assertEqual(record["status"], "aborted")
                self.assertEqual(record["stop_reason"], reason)
                self.assertTrue((folder / "motion.csv").exists())

    def test_ros_completed_tasks_and_stop_plan_complete_execution_record(self):
        cases = (
            ("forward", "final_forward_complete", True),
            ("forward", "max_obstacles_reached", True),
            ("forward", "duration_complete", False),
            ("stop", "stop_requested", True),
        )
        for action, reason, bypass in cases:
            with self.subTest(action=action, reason=reason), tempfile.TemporaryDirectory() as directory:
                plan = dict(action=action, speed_mode="none" if action == "stop" else "normal",
                            duration_s=0 if action == "stop" else 6)
                with patch("motion_demo.run_ros", return_value=reason):
                    self.execute(
                        Path(directory), plan, dry_run=False, stop_distance=0.3,
                        bypass_after_turn=bypass,
                    )
                record = json.loads((Path(directory) / "motion.csv.plan.json").read_text())
                self.assertEqual(record["status"], "completed")
                self.assertEqual(record["stop_reason"], reason)

    def test_cli_returns_nonzero_when_bypass_is_incomplete_without_plan(self):
        with tempfile.TemporaryDirectory() as directory:
            arguments = [
                "motion_demo.py", "run", "--mode", "smooth", "--stop-distance", "0.3",
                "--bypass-after-turn", "--max-obstacles", "2",
                "--output", str(Path(directory) / "motion.csv"),
            ]
            with patch("sys.argv", arguments), patch("motion_demo.run_ros", return_value="obstacles_not_cleared"):
                with patch("sys.stderr", new_callable=io.StringIO) as errors:
                    with self.assertRaises(SystemExit) as failure:
                        main()
                self.assertEqual(failure.exception.code, 1)
                self.assertIn("obstacles_not_cleared", errors.getvalue())

    def test_invalid_plan_does_not_start_ros_or_create_csv(self):
        with tempfile.TemporaryDirectory() as directory, patch("motion_demo.run_ros") as ros:
            folder = Path(directory)
            with self.assertRaises(ValueError):
                self.execute(folder, dict(action="forward", speed_mode="fast", duration_s=4), dry_run=False)
            ros.assert_not_called()
            self.assertFalse((folder / "motion.csv").exists())

    def test_plan_cannot_be_overridden_by_cli_speed_or_duration(self):
        for changes in (dict(speed=0.3), dict(duration=4)):
            with tempfile.TemporaryDirectory() as directory, self.assertRaises(ValueError):
                self.execute(Path(directory), dict(action="forward", speed_mode="slow", duration_s=2), **changes)


class DistributionTests(unittest.TestCase):
    def test_world_uses_only_bundled_meshes(self):
        root = Path(__file__).resolve().parent
        world = ET.parse(root / "models" / "world.sdf")
        uris = [element.text for element in world.findall(".//mesh/uri")]
        self.assertEqual(len(uris), 2)
        for uri in uris:
            self.assertTrue(uri.startswith("model://meshes/"))
            self.assertTrue((root / "models" / uri.removeprefix("model://")).is_file())
        for plugin in world.findall(".//plugin"):
            self.assertTrue(plugin.attrib["filename"].startswith("gz-sim-"))

    def test_license_and_launch_are_bundled(self):
        root = Path(__file__).resolve().parent
        self.assertTrue((root / "simulation.launch.py").is_file())
        license_text = (root / "licenses" / "2DRoboPrac_ws-Apache-2.0.txt").read_text()
        self.assertIn("Apache License", license_text)
        self.assertIn("Version 2.0, January 2004", license_text)


if __name__ == "__main__":
    unittest.main()