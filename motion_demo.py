import argparse
import csv
from dataclasses import dataclass
import hashlib
import json
import math
from pathlib import Path
import time


FIELDS = (
    "time_s", "mode", "target_mps", "command_mps", "angular_z_rps",
    "front_min_m", "side_min_m", "obstacles_cleared", "state",
)
SPEED_MODES = {"slow": 0.15, "normal": 0.30, "none": 0.0}
SCAN_TIMEOUT_NS = 1_000_000_000
SIDE_MIN_CLEARANCE = 0.35
TURN_SPEED = 0.5
TURN_DURATION = 1.5
BYPASS_SPEED = 0.12
BYPASS_CLEARANCE = 0.65
BYPASS_CLEAR_SCANS = 3
BYPASS_TIMEOUT_NS = 8_000_000_000
MAX_OBSTACLE_COUNT = 2
MAX_TASK_RUNTIME = 30.0
FINAL_FORWARD_DURATION = 2.0


@dataclass(frozen=True)
class BypassDecision:
    state: str
    command: float
    clear_scans: int
    stop_reason: str | None = None
    target: float | None = None


@dataclass(frozen=True)
class ControlDecision:
    state: str
    command: float | None = None
    target: float | None = None
    angular: float = 0.0
    motion_elapsed: float | None = None


def unique_fields(pairs):
    result = {}
    for key, value in pairs:
        if key in result:
            raise ValueError(f"Duplicate JSON field: {key}")
        result[key] = value
    return result


def reject_constant(value):
    raise ValueError(f"Non-finite JSON number: {value}")


def parse_plan(text):
    plan = json.loads(text, object_pairs_hook=unique_fields, parse_constant=reject_constant)
    if not isinstance(plan, dict) or set(plan) != {"action", "speed_mode", "duration_s"}:
        raise ValueError("Plan must contain exactly action, speed_mode, duration_s.")
    if not isinstance(plan["action"], str) or not isinstance(plan["speed_mode"], str):
        raise ValueError("Action and speed_mode must be strings.")
    if type(plan["duration_s"]) is not int:
        raise ValueError("Duration must be a JSON integer, not a boolean or decimal.")
    action = plan["action"]
    if action == "reject":
        raise ValueError("The instruction was rejected by the planner; no motion will start.")
    if action == "forward":
        if plan["speed_mode"] not in ("slow", "normal") or plan["duration_s"] not in (2, 4, 6):
            raise ValueError("Forward requires slow/normal speed and 2, 4, or 6 seconds.")
    elif action == "stop":
        if plan["speed_mode"] != "none" or plan["duration_s"] != 0:
            raise ValueError("Stop requires speed_mode=none and duration_s=0.")
    else:
        raise ValueError("Unsupported action; only forward and stop can execute.")
    return plan


def positive_number(value):
    number = float(value)
    if not math.isfinite(number) or number <= 0:
        raise argparse.ArgumentTypeError("Value must be finite and greater than zero.")
    return number


def command_at(elapsed, mode, speed, duration, acceleration):
    if elapsed < 0:
        raise ValueError("Elapsed time cannot be negative.")
    target = speed if elapsed < duration else 0.0
    if mode == "step":
        return target, target
    if mode != "smooth":
        raise ValueError("Unknown control mode.")
    peak = min(speed, acceleration * duration)
    command = min(speed, acceleration * elapsed) if elapsed < duration else max(
        0.0, peak - acceleration * (elapsed - duration)
    )
    return target, command


def sector_min_range(scan, min_angle, max_angle, min_distance=None):
    if not math.isfinite(scan.angle_min) or not math.isfinite(scan.angle_increment):
        return None
    if scan.angle_increment == 0:
        return None
    valid_min = scan.range_min if min_distance is None else max(scan.range_min, min_distance)
    distances = []
    for index, distance in enumerate(scan.ranges):
        angle = scan.angle_min + index * scan.angle_increment
        if (
            min_angle <= angle <= max_angle
            and math.isfinite(distance)
            and valid_min <= distance <= scan.range_max
        ):
            distances.append(distance)
    return min(distances) if distances else None


def front_min_range(scan, half_angle=math.radians(15)):
    return sector_min_range(scan, -half_angle, half_angle)


def choose_turn_direction(scan, min_clearance=SIDE_MIN_CLEARANCE):
    left = sector_min_range(scan, math.radians(25), math.radians(80))
    right = sector_min_range(scan, math.radians(-80), math.radians(-25))
    if left is None and right is None:
        return None
    if left is None:
        return -1 if right >= min_clearance else None
    if right is None:
        return 1 if left >= min_clearance else None
    if max(left, right) < min_clearance:
        return None
    return 1 if left >= right else -1


def side_min_range(scan):
    left = sector_min_range(scan, math.radians(15), math.radians(80), min_distance=0.15)
    right = sector_min_range(scan, math.radians(-80), math.radians(-15), min_distance=0.15)
    values = [value for value in (left, right) if value is not None]
    return min(values) if values else None


def bypass_decision(front_range, side_range, elapsed_ns, clear_scans, stop_distance, acceleration, new_scan=True):
    if front_range <= stop_distance:
        return BypassDecision("SAFE_STOP_BYPASS_OBSTACLE", 0.0, clear_scans, "obstacle_in_bypass_path")
    if side_range is None:
        return BypassDecision("SAFE_STOP_SCAN_INVALID", 0.0, clear_scans, "side_scan_invalid")
    if elapsed_ns >= BYPASS_TIMEOUT_NS:
        return BypassDecision("SAFE_STOP_BYPASS_TIMEOUT", 0.0, clear_scans, "bypass_timeout")
    if side_range >= BYPASS_CLEARANCE:
        if new_scan:
            clear_scans += 1
        if clear_scans >= BYPASS_CLEAR_SCANS:
            return BypassDecision("BYPASS_COMPLETE", 0.0, clear_scans, "bypass_complete")
        return BypassDecision("BYPASS_CLEAR_CHECK", 0.0, clear_scans)
    command = min(BYPASS_SPEED, acceleration * elapsed_ns / 1e9)
    return BypassDecision("BYPASS", command, 0, target=BYPASS_SPEED)


def final_forward_decision(front_range, elapsed, duration, stop_distance, acceleration):
    if front_range <= stop_distance:
        return BypassDecision("SAFE_STOP_FINAL_OBSTACLE", 0.0, 0, "obstacle_in_final_path")
    if elapsed >= duration:
        return BypassDecision("FINAL_FORWARD_COMPLETE", 0.0, 0, "final_forward_complete")
    command = min(BYPASS_SPEED, acceleration * elapsed)
    return BypassDecision("FINAL_FORWARD", command, 0, target=BYPASS_SPEED)


def finish_time(args):
    if args.mode == "step":
        return args.duration
    return args.duration + min(args.speed, args.acceleration * args.duration) / args.acceleration


class AvoidanceController:
    def __init__(self, args, start_ns):
        self.args = args
        self.start_ns = start_ns
        self.phase = "FORWARD"
        self.phase_started_ns = start_ns
        self.obstacles_cleared = 0
        self.turn_direction = 0
        self.clear_scans = 0
        self.last_bypass_scan = None
        self.stop_reason = None

    def stop(self, state, reason):
        self.stop_reason = reason
        return ControlDecision(state, command=0.0)

    def update(self, now_ns, front_range, side_range, scan, last_scan_ns):
        args = self.args
        if args.speed == 0:
            return self.stop("STOP_COMMAND", "stop_requested")
        elapsed = (now_ns - self.start_ns) / 1e9
        phase_elapsed = (now_ns - self.phase_started_ns) / 1e9
        if args.stop_distance is not None:
            if elapsed >= args.max_runtime:
                return self.stop("TASK_TIMEOUT", "task_timeout")
            if front_range is None or last_scan_ns is None or now_ns - last_scan_ns > SCAN_TIMEOUT_NS:
                return self.stop("SAFE_STOP_SCAN_LOST", "scan_lost")
            if self.phase == "TURN":
                if phase_elapsed >= TURN_DURATION:
                    if not args.bypass_after_turn:
                        return self.stop("TURN_COMPLETE", "turn_complete")
                    self.phase = "BYPASS"
                    self.phase_started_ns = now_ns
                    self.clear_scans = 0
                    self.last_bypass_scan = scan
                    return ControlDecision("BYPASS_START", command=0.0)
                state = "TURN_LEFT" if self.turn_direction > 0 else "TURN_RIGHT"
                return ControlDecision(state, command=0.0, angular=self.turn_direction * TURN_SPEED)
            if self.phase == "BYPASS":
                decision = bypass_decision(
                    front_range, side_range, now_ns - self.phase_started_ns,
                    self.clear_scans, args.stop_distance, args.acceleration,
                    new_scan=scan is not self.last_bypass_scan,
                )
                self.last_bypass_scan = scan
                self.clear_scans = decision.clear_scans
                if decision.stop_reason == "bypass_complete":
                    self.obstacles_cleared += 1
                    if self.obstacles_cleared < args.max_obstacles:
                        self.phase = "FORWARD"
                        self.phase_started_ns = now_ns
                        self.clear_scans = 0
                    elif args.final_forward_duration > 0:
                        self.phase = "FINAL_FORWARD"
                        self.phase_started_ns = now_ns
                    else:
                        self.stop_reason = "max_obstacles_reached"
                else:
                    self.stop_reason = decision.stop_reason
                return ControlDecision(
                    decision.state, command=decision.command, target=decision.target,
                    motion_elapsed=0.0 if self.phase == "FORWARD" else elapsed,
                )
            if self.phase == "FINAL_FORWARD":
                decision = final_forward_decision(
                    front_range, phase_elapsed, args.final_forward_duration,
                    args.stop_distance, args.acceleration,
                )
                self.stop_reason = decision.stop_reason
                return ControlDecision(decision.state, command=decision.command, target=decision.target)
            if front_range <= args.stop_distance:
                if not (args.turn_after_stop or args.bypass_after_turn):
                    return self.stop("OBSTACLE_STOP", "obstacle_detected")
                self.turn_direction = choose_turn_direction(scan)
                if self.turn_direction is None:
                    return self.stop("SAFE_STOP_NO_CLEAR_SIDE", "no_clear_turn_direction")
                self.phase = "TURN"
                self.phase_started_ns = now_ns
                return ControlDecision("OBSTACLE_STOP", command=0.0)
        if phase_elapsed >= finish_time(args):
            if args.stop_distance is not None and args.bypass_after_turn:
                return self.stop("TASK_INCOMPLETE", "obstacles_not_cleared")
            return self.stop("DURATION_COMPLETE", "duration_complete")
        state = "FORWARD" if args.stop_distance is not None else "OPEN_LOOP"
        return ControlDecision(state, motion_elapsed=phase_elapsed)


def write_sample(
    writer, elapsed, args, front_min=None, side_min=None, state="OPEN_LOOP",
    command_override=None, angular_command=0.0, target_override=None,
    motion_elapsed=None, obstacles_cleared=0,
):
    if motion_elapsed is None:
        motion_elapsed = elapsed
    target, command = command_at(
        motion_elapsed, args.mode, args.speed, args.duration, args.acceleration
    )
    if command_override is not None:
        command = command_override
    if target_override is not None:
        target = target_override
    front_text = "" if front_min is None else f"{front_min:.9f}"
    side_text = "" if side_min is None else f"{side_min:.9f}"
    writer.writerow((
        f"{elapsed:.9f}", args.mode, f"{target:.9f}", f"{command:.9f}",
        f"{angular_command:.9f}", front_text, side_text,
        str(obstacles_cleared), state,
    ))
    return command


def run_offline(args, writer):
    end = finish_time(args)
    index = 0
    while index * args.period < end:
        write_sample(writer, index * args.period, args)
        index += 1
    write_sample(writer, end, args)


def run_ros(args, writer, stream):
    import rclpy
    from geometry_msgs.msg import Twist
    from rclpy.parameter import Parameter
    from rclpy.qos import qos_profile_sensor_data
    from rclpy.signals import SignalHandlerOptions
    from sensor_msgs.msg import LaserScan

    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = rclpy.create_node(
        "motion_comparison",
        parameter_overrides=[Parameter("use_sim_time", value=True)],
    )
    publisher = node.create_publisher(Twist, "/cmd_vel", 10)
    latest_front = None
    latest_side = None
    latest_scan = None
    last_scan_ns = None
    if args.stop_distance is not None:
        def receive_scan(scan):
            nonlocal latest_front, latest_side, latest_scan, last_scan_ns
            latest_front = front_min_range(scan)
            latest_side = side_min_range(scan)
            latest_scan = scan
            last_scan_ns = node.get_clock().now().nanoseconds

        node.create_subscription(
            LaserScan, "/scan", receive_scan, qos_profile_sensor_data
        )
    start_ns = None
    finished = False
    active = False
    last_elapsed = 0.0
    controller = None
    timer = None

    def publish(command, angular=0.0):
        message = Twist()
        message.linear.x = command
        message.angular.z = angular
        publisher.publish(message)

    def update():
        nonlocal finished, last_elapsed
        now_ns = node.get_clock().now().nanoseconds
        elapsed = (now_ns - start_ns) / 1e9
        if elapsed < last_elapsed:
            raise RuntimeError("Simulation clock moved backwards. Restart the run.")
        last_elapsed = elapsed
        if publisher.get_subscription_count() == 0:
            raise RuntimeError("Velocity subscriber disconnected.")
        decision = controller.update(now_ns, latest_front, latest_side, latest_scan, last_scan_ns)
        if decision.state == "OBSTACLE_STOP" and controller.phase == "TURN":
            node.get_logger().info(
                f"Obstacle detected; turning {'left' if controller.turn_direction > 0 else 'right'}."
            )
        command = write_sample(
            writer,
            elapsed,
            args,
            front_min=latest_front if args.stop_distance is not None else None,
            side_min=latest_side if args.stop_distance is not None else None,
            state=decision.state,
            command_override=decision.command,
            angular_command=decision.angular,
            target_override=decision.target,
            motion_elapsed=decision.motion_elapsed,
            obstacles_cleared=controller.obstacles_cleared,
        )
        stream.flush()
        publish(command, decision.angular)
        finished = controller.stop_reason is not None

    try:
        node.get_logger().info("Waiting for /clock, /cmd_vel, and enabled sensors (10 seconds).")
        deadline = time.monotonic() + 10.0
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
            has_scan = args.stop_distance is None or (
                latest_front is not None
                and last_scan_ns is not None
                and node.get_clock().now().nanoseconds - last_scan_ns <= SCAN_TIMEOUT_NS
            )
            if (
                node.get_clock().now().nanoseconds > 0
                and publisher.get_subscription_count() > 0
                and has_scan
            ):
                break
            if time.monotonic() >= deadline:
                raise RuntimeError("No clock, /cmd_vel subscriber, or valid /scan. Check Gazebo, bridge, and LiDAR.")
        if not rclpy.ok():
            return
        start_ns = node.get_clock().now().nanoseconds
        controller = AvoidanceController(args, start_ns)
        active = True
        publish(0.0)
        update()
        timer = node.create_timer(args.period, update)
        last_clock_ns = start_ns
        last_progress = time.monotonic()
        node.get_logger().info(
            f"Running {args.mode}; CSV uses simulation time. "
            f"Obstacle stop: {args.stop_distance if args.stop_distance is not None else 'off'} m; "
            f"turn after stop: {args.turn_after_stop}; "
            f"bypass after turn: {args.bypass_after_turn}; "
            f"max obstacles: {args.max_obstacles}; max runtime: {args.max_runtime}s."
        )
        while rclpy.ok() and not finished:
            rclpy.spin_once(node, timeout_sec=0.1)
            now_ns = node.get_clock().now().nanoseconds
            if now_ns < last_clock_ns:
                raise RuntimeError("Simulation clock moved backwards. Restart the run.")
            if now_ns != last_clock_ns:
                last_progress = time.monotonic()
                last_clock_ns = now_ns
            if time.monotonic() - last_progress > 5.0:
                raise RuntimeError("Simulation clock stalled for 5 seconds; aborting.")
        if not finished:
            raise RuntimeError("ROS shut down before the run completed.")
        node.get_logger().info(f"Stopped ({controller.stop_reason}); zero velocity commanded.")
    finally:
        if timer is not None:
            timer.cancel()
        if active and node.context.ok():
            publish(0.0)
            front_text = "" if latest_front is None else f"{latest_front:.9f}"
            writer.writerow((
                f"{last_elapsed:.9f}", args.mode, "0.000000000", "0.000000000",
                "0.000000000", front_text,
                "" if latest_side is None else f"{latest_side:.9f}",
                str(controller.obstacles_cleared), "ZERO_COMMAND",
            ))
            stream.flush()
            publisher.wait_for_all_acked(rclpy.duration.Duration(seconds=1.0))
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return controller.stop_reason or "ros_shutdown"


def run(args):
    stop_reason = None
    if args.stop_distance is not None:
        if not 0.2 <= args.stop_distance <= 2.0:
            raise ValueError("Obstacle stop distance must be between 0.20 and 2.0 m.")
        if args.dry_run:
            raise ValueError("Obstacle monitoring requires Gazebo; --dry-run is not supported.")
    if args.turn_after_stop and args.stop_distance is None:
        raise ValueError("--turn-after-stop requires --stop-distance.")
    if args.bypass_after_turn and args.stop_distance is None:
        raise ValueError("--bypass-after-turn requires --stop-distance.")
    if args.bypass_after_turn and not args.turn_after_stop:
        args.turn_after_stop = True
    if not 1 <= args.max_obstacles <= MAX_OBSTACLE_COUNT:
        raise ValueError(f"This demo supports 1 to {MAX_OBSTACLE_COUNT} obstacles.")
    if args.max_obstacles > 1 and not args.bypass_after_turn:
        raise ValueError("Multiple obstacles require --bypass-after-turn.")
    if not 10 <= args.max_runtime <= 60:
        raise ValueError("Maximum task runtime must be between 10 and 60 seconds.")
    if not math.isfinite(args.final_forward_duration) or not 0 <= args.final_forward_duration <= 10:
        raise ValueError("Final forward duration must be between 0 and 10 seconds.")
    record = None
    if args.plan is not None:
        if args.speed is not None or args.duration is not None:
            raise ValueError("Do not combine --plan with --speed or --duration.")
        raw = args.plan.read_text(encoding="utf-8")
        plan = parse_plan(raw)
        args.speed = SPEED_MODES[plan["speed_mode"]]
        args.duration = float(plan["duration_s"])
        record = {
            "plan_file": str(args.plan),
            "plan_sha256": hashlib.sha256(raw.encode("utf-8")).hexdigest(),
            "plan": plan,
            "mode": args.mode,
            "speed_mps": args.speed,
            "duration_s": args.duration,
            "acceleration_mps2": args.acceleration,
            "period_s": args.period,
            "obstacle_stop_distance_m": args.stop_distance,
            "turn_after_stop": args.turn_after_stop,
            "bypass_after_turn": args.bypass_after_turn,
            "max_obstacles": args.max_obstacles,
            "max_runtime_s": args.max_runtime,
            "final_forward_duration_s": args.final_forward_duration,
            "turn_duration_s": TURN_DURATION if args.turn_after_stop else 0.0,
            "dry_run": args.dry_run,
            "status": "started",
        }
        print(f"Validated plan: {json.dumps(plan, ensure_ascii=False)}")
    else:
        args.speed = 0.3 if args.speed is None else args.speed
        args.duration = 4.0 if args.duration is None else args.duration
    if args.speed > 0.3:
        raise ValueError("This demo limits speed to 0.30 m/s.")
    if args.duration > 30:
        raise ValueError("This demo limits the forward command to 30 seconds.")
    if args.acceleration < 0.01 or args.acceleration > 1.0:
        raise ValueError("Acceleration must be between 0.01 and 1.0 m/s^2.")
    if args.period < 0.01 or args.period > 0.1:
        raise ValueError("Period must be between 0.01 and 0.10 seconds.")
    args.output.parent.mkdir(parents=True, exist_ok=True)
    record_path = args.output.with_name(args.output.name + ".plan.json")
    if record is not None and record_path.exists():
        raise FileExistsError(f"Execution record already exists: {record_path}")
    with args.output.open("x", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream)
        writer.writerow(FIELDS)
        if record is not None:
            with record_path.open("x", encoding="utf-8") as record_stream:
                json.dump(record, record_stream, ensure_ascii=False, indent=2)
        try:
            if args.dry_run:
                run_offline(args, writer)
            else:
                stop_reason = run_ros(args, writer, stream)
                if record is not None:
                    record["stop_reason"] = stop_reason
                if args.bypass_after_turn and args.speed > 0:
                    completed_reasons = {"final_forward_complete", "max_obstacles_reached"}
                else:
                    completed_reasons = {
                        "duration_complete", "obstacle_detected", "turn_complete", "stop_requested",
                    }
                if stop_reason not in completed_reasons:
                    raise RuntimeError(f"Motion task did not complete ({stop_reason}). CSV: {args.output}")
        except BaseException:
            if record is not None:
                record["status"] = "aborted"
            raise
        else:
            if record is not None:
                record["status"] = "completed"
        finally:
            if record is not None:
                record_path.write_text(json.dumps(record, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print(f"Saved: {args.output}")


def plot(args):
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    figure, axis = plt.subplots(figsize=(9, 5))
    try:
        for path in args.csv:
            with path.open(newline="", encoding="utf-8") as stream:
                rows = list(csv.DictReader(stream))
            if not rows:
                raise ValueError(f"No samples: {path}")
            times = [float(row["time_s"]) for row in rows]
            target = [float(row["target_mps"]) for row in rows]
            command = [float(row["command_mps"]) for row in rows]
            axis.step(times, target, where="post", linestyle="--", alpha=0.5,
                      label=f"{path.stem}: target")
            axis.plot(times, command, label=f"{path.stem}: command")
        axis.set(xlabel="Elapsed time (s)", ylabel="Command velocity (m/s)",
                 title="Velocity command comparison (not measured robot velocity)")
        axis.grid(True, alpha=0.3)
        axis.legend()
        figure.tight_layout()
        args.output.parent.mkdir(parents=True, exist_ok=True)
        figure.savefig(args.output, dpi=160)
    finally:
        plt.close(figure)
    print(f"Saved: {args.output}")


def main():
    parser = argparse.ArgumentParser(description="Forward/stop comparison for ROS 2 and Gazebo.")
    commands = parser.add_subparsers(dest="action", required=True)
    motion = commands.add_parser("run", help="Run a motion and record velocity commands.")
    motion.add_argument("--mode", choices=("step", "smooth"), required=True)
    motion.add_argument("--plan", type=Path, help="Validated JSON motion plan from a planner.")
    motion.add_argument("--speed", type=positive_number)
    motion.add_argument("--duration", type=positive_number)
    motion.add_argument("--acceleration", type=positive_number, default=0.15)
    motion.add_argument("--period", type=positive_number, default=0.05)
    motion.add_argument(
        "--stop-distance",
        type=positive_number,
        help="Stop when the minimum valid forward LaserScan range reaches this distance in meters.",
    )
    motion.add_argument(
        "--turn-after-stop",
        action="store_true",
        help="After an obstacle stop, turn toward the side with more measured clearance.",
    )
    motion.add_argument(
        "--bypass-after-turn",
        action="store_true",
        help="After turning, move slowly until side clearance confirms the obstacle has passed.",
    )
    motion.add_argument(
        "--max-obstacles", type=int, default=1,
        help="Number of obstacles to bypass before stopping (1-2).",
    )
    motion.add_argument(
        "--max-runtime", type=positive_number, default=MAX_TASK_RUNTIME,
        help="Overall simulation-time limit in seconds for obstacle avoidance.",
    )
    motion.add_argument(
        "--final-forward-duration", type=float, default=FINAL_FORWARD_DURATION,
        help="Low-speed straight segment after the final obstacle bypass (0-10 seconds).",
    )
    motion.add_argument("--output", type=Path, required=True)
    motion.add_argument("--dry-run", action="store_true", help="Generate synthetic samples without ROS.")
    graph = commands.add_parser("plot", help="Plot recorded command CSV files.")
    graph.add_argument("csv", nargs="+", type=Path)
    graph.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    try:
        if args.action == "run":
            run(args)
        else:
            plot(args)
    except KeyboardInterrupt:
        parser.exit(130, "Interrupted. A stop was attempted if ROS remained available.\n")
    except (ValueError, RuntimeError, OSError, ImportError) as error:
        parser.exit(1, f"Error: {error}\n")


if __name__ == "__main__":
    main()