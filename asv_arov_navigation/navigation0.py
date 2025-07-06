import rclpy
import math
from enum import Enum
from rclpy.node import Node
from geometry_msgs.msg import Twist, Pose, PoseArray
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from asv_arov_navigation.utils import euler_from_quaternion, transform_pose, build_pose, normalize_angle, normalize
from asv_arov_interfaces.srv import SetPump, ControlMode

class ControlState(Enum) :
    NAVIGATING = 0
    CLEANING = 1
    EMPTYING = 2

class PIDController() :
    def __init__(self, kP, kI, kD, saturation, tolerance, dt) :
        self.kP = kP
        self.kI = kI
        self.kD = kD
        self.saturation = saturation
        self.tolerance = tolerance
        self.dt = dt
        self.last_error = None
        self.integral_of_error = 0
        self.target = 0
        self.done = True
        self.continuous = False
        self.min = 0
        self.max = 0
        self.error = 0

    def set_continuous(self, continuous, min, max) :
        self.min = min
        self.max = max
        self.continuous = continuous

    def set_target(self, target) :
        self.target = normalize(target, self.min, self.max) if self.continuous else target
        self.integral_of_error = 0
        self.last_error = None
        self.done = False
    
    def get_target(self) :
        return self.target
    
    def calculate(self, pose) :
        self.error = self.target - pose
        if self.continuous :
            self.error = normalize(self.error, self.min, self.max)
        self.integral_of_error += self.error * self.dt
        d_error = self.error - self.last_error if self.last_error is not None else 0
        self.last_error = self.error
        return max(min(self.kP * self.error + self.kI * self.integral_of_error + self.kD * d_error, self.saturation), -self.saturation)

    def is_done(self) :
        return self.tolerance > abs(self.error)

class Navigation0(Node) :
    def __init__(self) :
        super().__init__('nav0')

        self.declare_parameter('use_sim', False)

        # Primary Params
        self.use_sim: bool = self.get_parameter('use_sim').get_parameter_value().bool_value
        self.dt = 0.1
        self.arov_cmd_vel_topic = "arov/cmd_vel"
        self.asv_cmd_vel_topic = "asv/cmd_vel"
        self.arov_base_link = "arov/base_link"
        self.asv_base_link = "asv/base_link"
        self.asv_home = Pose()
        self.arov_mort_trap_pose = Pose()
        self.surface_z = 0
        self.asv_linear_kP = 0.5
        self.asv_linear_kI = 0
        self.asv_linear_kD = 0
        self.asv_linear_saturation = 1.0
        self.asv_linear_tolerance = 0.1
        self.asv_yaw_kP = 0.5
        self.asv_yaw_kI = 0
        self.asv_yaw_kD = 0
        self.asv_yaw_saturation = 1.0
        self.asv_yaw_tolerance = 0.1
        self.asv_follower_clearance = 1
        self.arov_linear_kP = 0.5
        self.arov_linear_kI = 0
        self.arov_linear_kD = 0
        self.arov_linear_saturation = 1.0
        self.arov_linear_tolerance = 0.1
        self.arov_dive_kP = 0.5
        self.arov_dive_kI = 0
        self.arov_dive_kD = 0
        self.arov_dive_saturation = 1.0
        self.arov_dive_tolerance = 0.1
        self.arov_yaw_kP = 0.5
        self.arov_yaw_kI = 0
        self.arov_yaw_kD = 0
        self.arov_yaw_saturation = 1.0
        self.arov_yaw_tolerance = 0.1
        self.arov_follower_clearance = 1
        self.cleaning_routine_apriltag_x_offset = 0
        self.cleaning_routine_apriltag_y_offset = 0
        self.cleaning_routine_apriltag_clearance = 0
        self.cleaning_routine_depth = -1
        self.cleaning_routine_width = 2
        self.cleaning_routine_strip_width = 0.5
        self.asv_pose_targets = [Pose()]
        self.asv_pose_targets[0].position.x = 6.0
        self.arov_fence_frames = [["fence1", ""]]
        self.cleaning_cycles = 1

        # Secondary Params
        self.arov_cmd_vel_publisher = self.create_publisher(Twist, self.arov_cmd_vel_topic, 1)
        self.asv_cmd_vel_publisher = self.create_publisher(Twist, self.asv_cmd_vel_topic, 1)
        self.asv_x_controller = PIDController(self.asv_linear_kP, self.asv_linear_kI, self.asv_linear_kD, self.asv_linear_saturation, self.asv_linear_tolerance, self.dt)
        self.asv_yaw_controller = PIDController(self.asv_yaw_kP, self.asv_yaw_kI, self.asv_yaw_kD, self.asv_yaw_saturation, self.asv_yaw_tolerance, self.dt)
        self.asv_yaw_controller.set_continuous(True, -math.pi, math.pi)
        self.arov_x_controller = PIDController(self.arov_linear_kP, self.arov_linear_kI, self.arov_linear_kD, self.arov_linear_saturation, self.arov_linear_tolerance, self.dt)
        self.arov_y_controller = PIDController(self.arov_linear_kP, self.arov_linear_kI, self.arov_linear_kD, self.arov_linear_saturation, self.arov_linear_tolerance, self.dt)
        self.arov_z_controller = PIDController(self.arov_dive_kP, self.arov_dive_kI, self.arov_dive_kD, self.arov_dive_saturation, self.arov_dive_tolerance, self.dt)
        self.arov_yaw_controller = PIDController(self.arov_yaw_kP, self.arov_yaw_kI, self.arov_yaw_kD, self.arov_yaw_saturation, self.arov_yaw_tolerance, self.dt)
        self.arov_yaw_controller.set_continuous(True, -math.pi, math.pi)
        self.fence_frame_cleaning_routine_poses = [build_pose([self.cleaning_routine_apriltag_x_offset, self.cleaning_routine_apriltag_y_offset, self.cleaning_routine_apriltag_clearance, 0, math.pi/2, 0])]

        for i in range(0, math.ceil(self.cleaning_routine_width / self.cleaning_routine_strip_width)) :
            previous_pos = self.fence_frame_cleaning_routine_poses[-1].position
            strip_depth = self.cleaning_routine_depth if previous_pos.y == 0 else 0
            self.fence_frame_cleaning_routine_poses.append(build_pose([previous_pos.x, strip_depth, self.cleaning_routine_apriltag_clearance, 0, math.pi/2, 0]))
            self.fence_frame_cleaning_routine_poses.append(build_pose([previous_pos.x + self.cleaning_routine_strip_width, strip_depth, self.cleaning_routine_apriltag_clearance, 0, math.pi/2, 0]))

        # Instance Objects
        self.pose_pub = self.create_publisher(PoseArray, "pose_array", 10)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.pump_client = self.create_client(SetPump, '/set_pump')
        self.control_service = self.create_service(ControlMode, 'control_mode', self.service_callback)
        self.timer = None

        # Instance Variables
        self.asv_goal = None
        self.arov_goal = None
        self.state = ControlState.NAVIGATING
        self.asv_pose_id = 0
        self.arov_routine_pose_id = 0
        self.arov_fence_switch = 0
        self.asv_x = 0
        self.asv_y = 0
        self.asv_yaw = 0
        self.arov_x = 0
        self.arov_y = 0
        self.arov_z = 0
        self.arov_yaw = 0
        self.set_asv_angular_target = True
        self.set_asv_linear_target = True
        self.set_arov_target = True
        self.transformed_fence_poses = []
        self.cleaning_cycles_completed = 0

    def toggle_cleaners(self, active) :
        self.get_logger().info("Activating cleaner pump" if active else "Deactivating cleaner pump")
        if not self.use_sim :
            request = SetPump.Request()
            request.set_pump = active
            self.toggle_cleaners_client.wait_for_service()
            self.toggle_cleaners_client.call_async(request)

    def asv_done(self) :
        return self.asv_x_controller.is_done() and self.asv_yaw_controller.is_done()
    
    def arov_done(self) :
        #self.get_logger().info(f"{self.arov_x_controller.is_done()} {self.arov_y_controller.is_done()} {self.arov_z_controller.is_done()} {self.arov_yaw_controller.is_done()}")
        #self.get_logger().info(f"{self.arov_x_controller.error} {self.arov_y_controller.error} {self.arov_z_controller.error} {self.arov_yaw_controller.error}")
        return self.arov_x_controller.is_done() and self.arov_y_controller.is_done() and self.arov_z_controller.is_done() and self.arov_yaw_controller.is_done()

    def get_asv_linear_distance(self, target) :
        return math.sqrt((target.position.x - self.asv_x)**2 + (target.position.y - self.asv_y)**2)
    
    def arov_follower_pose(self) :
        dx = self.asv_x - self.arov_x
        dy = self.asv_y - self.arov_y
        dz = self.surface_z - self.arov_z
        d = math.sqrt(dx**2 + dy**2 + dz**2)
        if d == 0 :
            return self.arov_x, self.arov_y, self.arov_z
        clear_d = (d - self.arov_follower_clearance) / d
        return self.arov_x + dx * clear_d, self.arov_y + dy * clear_d, self.arov_z + dz * clear_d
    
    def asv_follower_pose(self) :
        dx = self.arov_x - self.asv_x
        dy = self.arov_y - self.asv_y
        d = math.sqrt(dx**2 + dy**2)
        if d == 0 :
            return self.asv_x, self.asv_y, self.asv_yaw
        clear_d = (d - self.asv_follower_clearance) / d
        dtheta = math.atan2(dy, dx)
        return dx * clear_d, dy * clear_d, dtheta
    
    def optimized_asv_target(self, target) :
        asv_target_yaw_towards = math.atan2(target.position.y - self.asv_y, target.position.x - self.asv_x)
        asv_target_yaw_away = normalize_angle(math.atan2(target.position.y - self.asv_y, target.position.x - self.asv_x) + math.pi)
        asv_linear_distance = self.get_asv_linear_distance(target)
        if abs(asv_target_yaw_away - self.asv_yaw) < abs(asv_target_yaw_towards - self.asv_yaw) :
            return -asv_linear_distance, asv_target_yaw_away
        else :
            return asv_linear_distance, asv_target_yaw_towards
    
    def global_to_relative_arov_command(self, twist) :
        new_twist = Twist()
        new_twist.linear.x = twist.linear.x * math.cos(-self.arov_yaw) - twist.linear.y * math.sin(-self.arov_yaw)
        new_twist.linear.y = twist.linear.x * math.sin(-self.arov_yaw) + twist.linear.y * math.cos(-self.arov_yaw)
        new_twist.linear.z = twist.linear.z
        new_twist.angular.z = twist.angular.z
        return new_twist
    
    def stop(self) :
        self.asv_cmd_vel_publisher.publish(Twist())
        self.arov_cmd_vel_publisher.publish(Twist())

    def publish_poses(self, pose_list) :
        msg = PoseArray()
        msg.header.frame_id = 'map'
        msg.poses = pose_list
        self.pose_pub.publish(msg)

    def setup_routine_in_frame(self, frame_id) :
        if len(frame_id) == 0 :
            return [Pose()]
        out = []
        t = None
        while True :
            try :
                t = self.tf_buffer.lookup_transform('map', frame_id, rclpy.time.Time())
                break
            except TransformException as ex :
                self.get_logger().info(f'Could not get AprilTag transform: {ex}')
        for i in self.fence_frame_cleaning_routine_poses :
            out.append(transform_pose(i, t))
            #self.get_logger().info(f'Pose in map frame: {out[-1].position} \n Orientation in map frame: {out[-1].orientation}')
        self.publish_poses(out)
        return out
    
    def get_poses(self) :
        while True :
            try :
                asv_t = self.tf_buffer.lookup_transform('map', self.asv_base_link, rclpy.time.Time())
                self.asv_x = asv_t.transform.translation.x
                self.asv_y = asv_t.transform.translation.y
                self.asv_yaw = euler_from_quaternion(asv_t.transform.rotation)[2]
            except TransformException as ex :
                self.get_logger().info(f'Could not get ASV pose: {ex}')
                continue
            try :
                arov_t = self.tf_buffer.lookup_transform('map', self.arov_base_link, rclpy.time.Time())
                self.arov_x = arov_t.transform.translation.x
                self.arov_y = arov_t.transform.translation.y
                self.arov_z = arov_t.transform.translation.z
                self.arov_yaw = euler_from_quaternion(arov_t.transform.rotation)[2]
                break
            except TransformException as ex :
                self.get_logger().info(f'Could not get AROV pose: {ex}')
    
    def go_home(self) :
        self.get_poses()
        asv_twist = Twist()
        arov_twist = Twist()
        asv_target_distance, asv_target_yaw = self.optimized_asv_target(self.asv_home)
        if self.set_asv_linear_target :
            self.asv_x_controller.set_target(asv_target_distance)
            self.set_asv_linear_target = False
        if self.set_asv_angular_target :
            self.asv_yaw_controller.set_target(normalize_angle(asv_target_yaw))
            self.set_asv_angular_target = False
        asv_twist.angular.z = self.asv_yaw_controller.calculate(self.asv_yaw)
        if self.asv_yaw_controller.is_done() :
            asv_twist.linear.x = self.asv_x_controller.calculate(self.asv_x_controller.get_target() - asv_target_distance)
        arov_target_x, arov_target_y, arov_target_z = self.arov_follower_pose()
        self.arov_x_controller.set_target(arov_target_x)
        self.arov_y_controller.set_target(arov_target_y)
        self.arov_z_controller.set_target(arov_target_z)
        arov_twist.linear.x = self.arov_x_controller.calculate(self.arov_x)
        arov_twist.linear.y = self.arov_y_controller.calculate(self.arov_y)
        arov_twist.linear.z = self.arov_z_controller.calculate(self.arov_z)
        if self.asv_done() and self.arov_done() :
            self.set_asv_angular_target = True
            self.set_asv_linear_target = True
            self.stop()
            self.timer.destroy()

    def run_state_machine(self) :
        self.get_poses()
        asv_twist = Twist()
        arov_twist = Twist()
        if self.state == ControlState.EMPTYING :
            if self.set_arov_target :
                self.arov_x_controller.set_target(self.arov_mort_trap_pose.position.x)
                self.arov_y_controller.set_target(self.arov_mort_trap_pose.position.y)
                self.arov_z_controller.set_target(self.arov_mort_trap_pose.position.z)
                self.arov_yaw_controller.set_target(euler_from_quaternion(self.arov_mort_trap_pose.orientation)[2])
                self.set_arov_target = False
            asv_target_x, asv_target_y, asv_target_theta = self.asv_follower_pose()
            asv_target_distance, asv_target_yaw = self.optimized_asv_target(build_pose([asv_target_x, asv_target_y, 0, 0, 0, asv_target_theta]))
            self.asv_x_controller.set_target(asv_target_distance)
            self.asv_yaw_controller.set_target(normalize_angle(asv_target_yaw))
            asv_twist.angular.z = self.asv_yaw_controller.calculate(self.asv_yaw)
            if self.asv_yaw_controller.is_done() :
                asv_twist.linear.x = self.asv_x_controller.calculate(self.asv_x_controller.get_target() - asv_target_distance)
            if self.arov_done() :
                self.set_arov_target = True
                self.timer.destroy()
        else :
            asv_target_distance, asv_target_yaw = self.optimized_asv_target(self.asv_pose_targets[self.asv_pose_id])
            if self.set_asv_linear_target :
                self.asv_x_controller.set_target(asv_target_distance)
                self.set_asv_linear_target = False
            if self.set_asv_angular_target :
                self.asv_yaw_controller.set_target(normalize_angle(asv_target_yaw))
                self.set_asv_angular_target = False
            asv_twist.angular.z = self.asv_yaw_controller.calculate(self.asv_yaw)
            if self.asv_yaw_controller.is_done() :
                asv_twist.linear.x = self.asv_x_controller.calculate(self.asv_x_controller.get_target() - asv_target_distance)
            if self.state == ControlState.NAVIGATING :
                arov_target_x, arov_target_y, arov_target_z = self.arov_follower_pose()
                self.arov_x_controller.set_target(arov_target_x)
                self.arov_y_controller.set_target(arov_target_y)
                self.arov_z_controller.set_target(arov_target_z)
                arov_twist.linear.x = self.arov_x_controller.calculate(self.arov_x)
                arov_twist.linear.y = self.arov_y_controller.calculate(self.arov_y)
                arov_twist.linear.z = self.arov_z_controller.calculate(self.arov_z)
                if self.asv_done() and self.arov_done() :
                    self.state = ControlState.CLEANING
                    self.transformed_fence_poses.append(self.setup_routine_in_frame(self.arov_fence_frames[self.asv_pose_id][0]))
                    self.transformed_fence_poses.append(self.setup_routine_in_frame(self.arov_fence_frames[self.asv_pose_id][1]))
                    self.set_asv_angular_target = True
                    self.set_asv_linear_target = True
            elif self.state == ControlState.CLEANING :
                if self.set_arov_target :
                    self.arov_x_controller.set_target(self.transformed_fence_poses[self.arov_fence_switch][self.arov_routine_pose_id].position.x)
                    self.arov_y_controller.set_target(self.transformed_fence_poses[self.arov_fence_switch][self.arov_routine_pose_id].position.y)
                    self.arov_z_controller.set_target(self.transformed_fence_poses[self.arov_fence_switch][self.arov_routine_pose_id].position.z)
                    self.arov_yaw_controller.set_target(euler_from_quaternion(self.transformed_fence_poses[self.arov_fence_switch][self.arov_routine_pose_id].orientation)[2])
                    self.set_arov_target = False
                    self.toggle_cleaners(True)
                #self.get_logger().info(f"{self.transformed_fence_poses[self.arov_fence_switch][self.arov_routine_pose_id]}")
                arov_twist.linear.x = self.arov_x_controller.calculate(self.arov_x)
                arov_twist.linear.y = self.arov_y_controller.calculate(self.arov_y)
                arov_twist.linear.z = self.arov_z_controller.calculate(self.arov_z)
                arov_twist.angular.z = self.arov_yaw_controller.calculate(self.arov_yaw)
                if self.arov_done() :
                    self.toggle_cleaners(False)
                    self.set_arov_target = True
                    self.arov_routine_pose_id += 1
                    if self.arov_routine_pose_id >= len(self.transformed_fence_poses[self.arov_fence_switch]) :
                        self.arov_routine_pose_id = 0
                        if self.arov_fence_switch == 1 :
                            self.arov_fence_switch = 0
                            self.cleaning_cycles_completed += 1
                            self.state = ControlState.NAVIGATING if self.cleaning_cycles_completed < self.cleaning_cycles else ControlState.EMPTYING
                        else :
                            self.arov_fence_switch = 1

        self.asv_cmd_vel_publisher.publish(asv_twist)
        self.arov_cmd_vel_publisher.publish(self.global_to_relative_arov_command(arov_twist))
    
    def service_callback(self, request, response) :
        mode_string = "HOME" if request.mode == 2 else "START" if request.mode == 1 else "STOP"
        self.get_logger().info(f"Receiving task with mode {mode_string}")
        if self.timer is not None :
            self.timer.destroy()
        self.stop()
        if request.mode == 1 :
            self.timer = self.create_timer(self.dt, self.run_state_machine)
        elif request.mode == 2 :
            self.timer = self.create_timer(self.dt, self.go_home)
        return response

def main(args=None) :

    rclpy.init()
    node = Navigation0()
    rclpy.spin(node)

if __name__ == '__main__' :
    main()
