import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Vector3Stamped

from .ardupilot_interface import ArduPilotInterface
from .target_manager import TargetManager
from .trajectory_generator import WaypointGenerator
from .position_controller import PositionController
from .attitude_generator import AttitudeGenerator
from .so3_controller import SO3Controller
from .safety_manager import SafetyManager
from .command_interface import CommandInterface, ControlCommand
from .flight_manager import FlightManager
from .telemetry_logger import TelemetryLogger
from .state_adapter import rotation_matrix_to_quaternion


class ControllerNode(Node):

    def __init__(self):
        super().__init__("so3_controller")

        # Declare ROS 2 parameters
        self.declare_parameter("rate", 50.0)

        self.declare_parameter("pose_topic", "/ap/v1/pose/filtered")
        self.declare_parameter("twist_topic", "/ap/v1/twist/filtered")
        self.declare_parameter("cmd_vel_topic", "/ap/v1/cmd_vel")
        self.declare_parameter("status_topic", "/ap/v1/status")

        self.declare_parameter("target.x", 1.0)
        self.declare_parameter("target.y", 0.0)
        self.declare_parameter("target.z", 1.0)
        self.declare_parameter("target.yaw", 0.0)

        self.declare_parameter("position.kp", [0.5, 0.5, 0.5])
        self.declare_parameter("position.kv", [0.2, 0.2, 0.2])

        self.declare_parameter("limits.max_velocity", 0.5)
        self.declare_parameter("limits.position_tolerance", 0.10)
        self.declare_parameter("limits.velocity_tolerance", 0.05)

        self.declare_parameter("safety.state_timeout", 0.2)
        self.declare_parameter("safety.enable_command", False)

        self.declare_parameter("mass", 1.5)
        self.declare_parameter("gravity", 9.81)
        self.declare_parameter("inertia", [0.02, 0.02, 0.04])
        self.declare_parameter("kR", [4.0, 4.0, 2.0])
        self.declare_parameter("kOmega", [0.8, 0.8, 0.5])

        # Parse parameters
        self.control_rate = float(self.get_parameter("rate").value)

        pose_topic = self.get_parameter("pose_topic").value
        twist_topic = self.get_parameter("twist_topic").value
        cmd_vel_topic = self.get_parameter("cmd_vel_topic").value
        status_topic = self.get_parameter("status_topic").value

        target_x = float(self.get_parameter("target.x").value)
        target_y = float(self.get_parameter("target.y").value)
        target_z = float(self.get_parameter("target.z").value)
        target_yaw = float(self.get_parameter("target.yaw").value)

        kp = self.get_parameter("position.kp").value
        kv = self.get_parameter("position.kv").value

        max_velocity = float(self.get_parameter("limits.max_velocity").value)
        position_tolerance = float(self.get_parameter("limits.position_tolerance").value)
        velocity_tolerance = float(self.get_parameter("limits.velocity_tolerance").value)

        state_timeout = float(self.get_parameter("safety.state_timeout").value)
        self.enable_command = bool(self.get_parameter("safety.enable_command").value)

        mass = float(self.get_parameter("mass").value)
        gravity = float(self.get_parameter("gravity").value)
        inertia = self.get_parameter("inertia").value
        kR = self.get_parameter("kR").value
        kOmega = self.get_parameter("kOmega").value

        # Instantiate components
        self.ardupilot_interface = ArduPilotInterface(
            node=self,
            pose_topic=pose_topic,
            twist_topic=twist_topic,
            cmd_vel_topic=cmd_vel_topic,
            status_topic=status_topic,
            state_timeout=state_timeout
        )

        self.target_manager = TargetManager(
            initial_position=[target_x, target_y, target_z],
            initial_yaw=target_yaw
        )

        self.position_controller = PositionController(
            mass=mass,
            gravity=gravity,
            kp=kp,
            kv=kv,
            max_velocity=max_velocity,
            position_tolerance=position_tolerance,
            velocity_tolerance=velocity_tolerance
        )

        self.attitude_generator = AttitudeGenerator()

        self.so3_controller = SO3Controller(
            inertia=inertia,
            kR=kR,
            kOmega=kOmega
        )

        self.safety_manager = SafetyManager(
            max_velocity=max_velocity,
            state_timeout=state_timeout
        )

        self.command_interface = CommandInterface(
            ardupilot_interface=self.ardupilot_interface
        )

        self.flight_manager = FlightManager(
            position_tolerance=position_tolerance,
            velocity_tolerance=velocity_tolerance
        )
        self.telemetry_logger = TelemetryLogger()

        # Telemetry & SO(3) Debug Publishers
        self.position_error_pub = self.create_publisher(Vector3Stamped, "/drones_controller/position_error", 10)
        self.velocity_error_pub = self.create_publisher(Vector3Stamped, "/drones_controller/velocity_error", 10)
        self.desired_force_pub = self.create_publisher(Vector3Stamped, "/drones_controller/desired_force", 10)
        self.desired_moment_pub = self.create_publisher(Vector3Stamped, "/drones_controller/desired_moment", 10)
        self.attitude_error_pub = self.create_publisher(Vector3Stamped, "/drones_controller/attitude_error", 10)
        self.angular_velocity_error_pub = self.create_publisher(Vector3Stamped, "/drones_controller/angular_velocity_error", 10)
        self.desired_attitude_pub = self.create_publisher(PoseStamped, "/drones_controller/desired_attitude", 10)

        # Timer setup for continuous control loop
        timer_period = 1.0 / self.control_rate
        self.timer = self.create_timer(timer_period, self.control_loop)

        self.get_logger().info(f"Closed-loop Autonomous XYZ Position Controller started at {self.control_rate} Hz")

    def publish_vector(self, publisher, vector: np.ndarray):
        msg = Vector3Stamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "controller"
        msg.vector.x = float(vector[0])
        msg.vector.y = float(vector[1])
        msg.vector.z = float(vector[2])
        publisher.publish(msg)

    def control_loop(self):
        # 1. Receive state feedback from ArduPilot Interface
        state = self.ardupilot_interface.get_state()

        # 2. Validate state safety
        if not self.safety_manager.validate_state(state):
            self.flight_manager.update(is_connected=False, target_reached=False)
            if self.enable_command:
                self.command_interface.send(self.safety_manager._create_zero_command())
            return

        # 3. Read target setpoint
        target = self.target_manager.get_target()

        # 4. Position Controller -> velocity command v_cmd & target reached check
        pos_output = self.position_controller.compute(
            p=state.position,
            v=state.velocity,
            p_d=target.position,
            v_d=target.velocity,
            a_d=target.acceleration
        )

        # 5. SO(3) Attitude Generator & SO(3) Controller (for parallel research debug logging)
        att_target = self.attitude_generator.compute(
            F_d=pos_output.F_d,
            yaw_d=target.yaw
        )
        so3_output = self.so3_controller.compute(
            R=state.rotation,
            omega=state.angular_velocity,
            R_d=att_target.R_d,
            omega_d=target.angular_velocity,
            omega_dot_d=target.angular_acceleration
        )

        # 6. Apply command limits & safety validation
        raw_command = ControlCommand(
            v_cmd=pos_output.v_cmd,
            yaw_rate=0.0,
            e_p=pos_output.e_p,
            e_v=pos_output.e_v,
            F_d=pos_output.F_d,
            R_d=att_target.R_d,
            M_d=so3_output.M_d,
            thrust_d=att_target.thrust_d
        )
        safe_command = self.safety_manager.validate_command(raw_command)

        # 7. Update flight state machine
        flight_state = self.flight_manager.update(
            is_connected=self.ardupilot_interface.is_connected(),
            target_reached=pos_output.target_reached,
            armed=self.ardupilot_interface.armed,
            flying=self.ardupilot_interface.flying,
            failsafe=self.ardupilot_interface.failsafe
        )

        # 8. Continuous command publishing to /ap/v1/cmd_vel if enable_command is True
        enable_cmd = bool(self.get_parameter("safety.enable_command").value)
        if enable_cmd:
            self.command_interface.send(safe_command)

        # 9. Publish debug topics for SO(3) research
        self.publish_vector(self.position_error_pub, pos_output.e_p)
        self.publish_vector(self.velocity_error_pub, pos_output.e_v)
        self.publish_vector(self.desired_force_pub, pos_output.F_d)
        self.publish_vector(self.desired_moment_pub, so3_output.M_d)
        self.publish_vector(self.attitude_error_pub, so3_output.e_R)
        self.publish_vector(self.angular_velocity_error_pub, so3_output.e_omega)

        q_d = rotation_matrix_to_quaternion(att_target.R_d)
        attitude_msg = PoseStamped()
        attitude_msg.header.stamp = self.get_clock().now().to_msg()
        attitude_msg.header.frame_id = "map"
        attitude_msg.pose.orientation.x = float(q_d[0])
        attitude_msg.pose.orientation.y = float(q_d[1])
        attitude_msg.pose.orientation.z = float(q_d[2])
        attitude_msg.pose.orientation.w = float(q_d[3])
        self.desired_attitude_pub.publish(attitude_msg)

        # Log telemetry frame
        self.telemetry_logger.log(
            timestamp=state.timestamp,
            target_pos=target.position,
            actual_pos=state.position,
            pos_error=pos_output.e_p,
            actual_vel=state.velocity,
            vel_error=pos_output.e_v,
            desired_force=pos_output.F_d,
            desired_moment=so3_output.M_d,
            attitude_error=so3_output.e_R,
            angular_vel_error=so3_output.e_omega,
            desired_thrust=att_target.thrust_d,
            flight_state=flight_state.name
        )


def main(args=None):
    rclpy.init(args=args)
    node = ControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
