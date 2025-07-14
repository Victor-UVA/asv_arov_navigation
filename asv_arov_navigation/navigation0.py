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
        """
        A simple implementation of a PID controller

        :param kP: controller P constant
        :param kI: controller I constant
        :param kD: controller D constant
        :param saturation: system saturation
        :param tolerance: controller minimum error to return success
        :param dt: system delta-time, used for integration
        """
        self.kP = kP
        self.kI = kI
        self.kD = kD
        self.saturation = saturation
        self.tolerance = tolerance
        self.dt = dt
        self.last_error = None
        self.integral_of_error = 0
        self.target = 0
        self.continuous = False
        self.min = 0
        self.max = 0
        self.error = 0

    def set_continuous(self, continuous, min, max) :
        """
        Set if the system this PID controller is operating on has a looping state (like a heading controller)

        :param continuous: True if continuous system, False to disable continuous mode
        :param min: minimum measurement, equal state to maximum for system
        :param max: maximum measurement, equal state to minimum for system
        """
        self.min = min
        self.max = max
        self.continuous = continuous

    def set_target(self, target) :
        """
        Set target state of controller

        :param target: target state of controller
        """
        self.target = normalize(target, self.min, self.max) if self.continuous else target
        self.integral_of_error = 0
        self.last_error = None
    
    def get_target(self) :
        """
        Get target state of controller

        :return: target state of controller
        """
        return self.target
    
    def calculate(self, state) :
        """
        Get output of PID controller

        :param state: current state of system
        :return: output of controller
        """
        self.error = self.target - state
        if self.continuous :
            self.error = normalize(self.error, self.min, self.max)
        self.integral_of_error += self.error * self.dt
        d_error = self.error - self.last_error if self.last_error is not None else 0
        self.last_error = self.error
        return max(min(self.kP * self.error + self.kI * self.integral_of_error + self.kD * d_error, self.saturation), -self.saturation)

    def is_done(self) :
        """
        Is system within tolerance of target state

        :return: True/False
        """
        return self.tolerance > abs(self.error)

class Navigation0(Node) :
    def __init__(self) :
        """
        Navigation node for performing the aquaculture demo routine.
        When the /control_mode service receives a 0 command, it will stop the robots.
        When the /control_mode service receives a 1 command, it will start the demo.
        When the /control_mode service receives a 2 command, it will drive the robots to the home pose
        """
        super().__init__('nav0')

        self.declare_parameter('use_sim', False)

        # Primary Params
        self.use_sim: bool = self.get_parameter('use_sim').get_parameter_value().bool_value # Running in simulated environment
        self.dt = 0.1 # Delta-time in seconds for command generation rate
        self.arov_cmd_vel_topic = "arov/cmd_vel" # Topic to publish AROV commands to
        self.asv_cmd_vel_topic = "asv/cmd_vel" # Topic to publish ASV commands to
        self.arov_base_link = "arov/base_link" # AROV base link frame
        self.asv_base_link = "asv/base_link" # ASV base link frame
        self.home = Pose() # Pose to navigate to when receiving a HOME task
        self.arov_mort_trap_pose = Pose() # Pose to navigate to for mort trap emptying, used at the end of the cleaning routine
        self.arov_follow_z = -1.5 # Depth at which the AROV follows the ASV while in NAVIGATING mode
        self.asv_linear_kP = 0.3 # ASV linear drive controller P
        self.asv_linear_kI = 0 # ASV linear drive controller I
        self.asv_linear_kD = 0 # ASV linear drive controller D
        self.asv_linear_saturation = 0.3 # ASV linear drive system saturation
        self.asv_linear_tolerance = 0.3 # ASV linear drive controller error tolerance
        self.asv_yaw_kP = 0.7 # ASV yaw controller P
        self.asv_yaw_kI = 0 # ASV yaw controller I
        self.asv_yaw_kD = 0 # ASV yaw controller D
        self.asv_yaw_saturation = 0.5 # ASV yaw system saturation
        self.asv_yaw_tolerance = 0.1 # ASV yaw controller tolerance
        self.asv_follower_clearance = 1 # xy distance at which the ASV attempts to follow the AROV
        self.arov_linear_kP = 0.3 # AROV x and y drive controllers P
        self.arov_linear_kI = 0.0 # AROV x and y drive controllers I
        self.arov_linear_kD = 0.1 # AROV x and y drive controllers D
        self.arov_linear_saturation = 0.07 # AROV x and y drive systems saturation
        self.arov_linear_tolerance = 0.2 # AROV x and y drive controllers tolerance
        self.arov_dive_kP = 0.2 # AROV z drive controller P
        self.arov_dive_kI = 0.01 # AROV z drive controller I
        self.arov_dive_kD = 0.8 # AROV z drive controller D
        self.arov_dive_saturation = 0.04 # AROV z drive system saturation
        self.arov_dive_tolerance = 0.3 # AROV z drive controller tolerance
        self.arov_yaw_kP = 0.25 # AROV yaw controller P
        self.arov_yaw_kI = 0 # AROV yaw controller I
        self.arov_yaw_kD = 0.3 # AROV yaw controller D
        self.arov_yaw_saturation = 0.25 # AROV yaw system saturation
        self.arov_yaw_tolerance = 0.26 # AROV yaw controller tolerance
        self.arov_follower_clearance = 1 # xy distance at which the AROV attempts to follow the ASV
        self.arov_move_then_turn = False # AROV does not attempt to reach yaw target until it has reached its xyz target. Mainly for cases in which delta-yaw results in unreliable localization
        self.cleaning_routine_apriltag_x_offset = 0 # x offset from AprilTag pose in AprilTag frame at which to perform cleaning routine
        self.cleaning_routine_apriltag_y_offset = 0 # y offset from AprilTag pose in AprilTag frame at which to perform cleaning routine
        self.cleaning_routine_apriltag_clearance = 0.75 # z offset from AprilTag pose in AprilTag frame at which to perform cleaning routine
        self.cleaning_routine_depth = -0.5 # Depth (maximum map z - minimum map z) of cleaning routine
        self.cleaning_routine_width = 1 # Width of cleaning routine
        self.cleaning_routine_strip_width = 0.5 # Width of individual strip (strafe command) of cleaning routine
        self.asv_pose_targets = [Pose()] # Poses at which the ASV should idle while AROV performs cleaning routine
        self.arov_fence_frames = [["", ""]] # AprilTag frame IDs at which the AROV should perform cleaning routine. Should have a pair for each ASV pose, but an empty ID string will be skipped
        self.cleaning_cycles = 6 # Number of pairs of cleaning routines the AROV should perform before driving to mort trap pose. Will loop if fewer pose targets than this number
        self.run_arov = True # Run the AROV. If False, no commands will be sent to the AROV and alternate behaviors for when the ASV would follow the AROV will be used
        self.run_asv = False # Run the ASV. If False, no commands will be sent to the ASV and alternate behaviors for when the AROV would follow the ASV will be used

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

        # Construct cleaning routine in AprilTag frame (-z, -x, y)
        for i in range(0, math.ceil(self.cleaning_routine_width / self.cleaning_routine_strip_width)) :
            previous_pos = self.fence_frame_cleaning_routine_poses[-1].position
            strip_depth = self.cleaning_routine_depth + self.cleaning_routine_apriltag_y_offset if previous_pos.y == self.cleaning_routine_apriltag_y_offset else self.cleaning_routine_apriltag_y_offset
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
        self.state = ControlState.NAVIGATING # Current state of the demo state machine
        self.asv_pose_id = 0 # ID in the self.asv_pose_targets array the ASV will navigate to next
        self.arov_routine_pose_id = 0 # ID in the self.fence_frame_cleaning_routine_poses array the AROV will navigate to next
        self.arov_fence_switch = 0 # If the AROV is cleaning the first or second fence in a self.arov_fence_frames pair
        self.asv_x = 0 # Current x position of ASV in map frame
        self.asv_y = 0 # Current y position of ASV in map frame
        self.asv_yaw = 0 # Current yaw of ASV in map frame
        self.arov_x = 0 # Current x position of AROV in map frame
        self.arov_y = 0 # Current y position of AROV in map frame
        self.arov_z = 0 # Current z position of AROV in map frame
        self.arov_yaw = 0 # Current yaw of AROV in map frame
        self.do_set_asv_angular_target = True # State machine should set ASV yaw target if it reaches a set target
        self.do_set_asv_linear_target = True # State machine should set ASV linear target if it reaches a set target
        self.do_set_arov_target = True # State machine should set AROV target if it reaches a set target
        self.transformed_fence_poses = [] # Cleaning routine transformed from specific AprilTag frame to map frame
        self.cleaning_cycles_completed = 0 # Count of complete cleaning cycles (pairs of fences)

    def reset_instance_variables(self) :
        """
        Reset node instance variables to their defaults before changing task. Does not reset poses.
        """
        self.state = ControlState.NAVIGATING
        self.asv_pose_id = 0
        self.arov_routine_pose_id = 0
        self.arov_fence_switch = 0
        self.do_set_asv_angular_target = True
        self.do_set_asv_linear_target = True
        self.do_set_arov_target = True
        self.transformed_fence_poses = []
        self.cleaning_cycles_completed = 0

    def set_arov_target(self, x=None, y=None, z=None, theta=None) :
        """
        Helper function to set AROV controller targets

        :param x: x in map coordinates to drive to
        :param y: y in map coordinates to drive to
        :param z: z in map coordinates to drive to
        :param theta: yaw to turn to
        """
        if x is not None :
            self.arov_x_controller.set_target(x)
        if y is not None :
            self.arov_y_controller.set_target(y)
        if z is not None :
            self.arov_z_controller.set_target(z)
        if theta is not None :
            self.arov_yaw_controller.set_target(theta)

    def arov_go_to_target_position(self, twist) :
        """
        Helper function for constructing AROV commands, builds linear vector

        :param twist: twist to modify
        """
        twist.linear.x = self.arov_x_controller.calculate(self.arov_x)
        twist.linear.y = self.arov_y_controller.calculate(self.arov_y)
        twist.linear.z = self.arov_z_controller.calculate(self.arov_z)

    def toggle_cleaners(self, active) :
        """
        Runs client to toggle the AROV's cleaners

        :param active: set if cleaners active
        """
        self.get_logger().info("Activating cleaner pump" if active else "Deactivating cleaner pump")
        if not self.use_sim and self.run_arov :
            request = SetPump.Request()
            request.set_pump = active
            self.pump_client.wait_for_service()
            self.pump_client.call_async(request)

    def asv_done(self) :
        """
        Helper function to check if all ASV controllers are done

        :return: all system states are within tolerance
        """
        return not self.run_asv or (self.asv_x_controller.is_done() and self.asv_yaw_controller.is_done())
    
    def arov_move_done(self) :
        """
        Helper function to check if all AROV linear controllers are done

        :return: all linear system states are within tolerance
        """
        return not self.run_arov or (self.arov_x_controller.is_done() and self.arov_y_controller.is_done() and self.arov_z_controller.is_done())
    
    def arov_done(self) :
        """
        Helper function to check if all AROV controllers are done

        :return: all system states are within tolerance
        """
        return not self.run_arov or (self.arov_move_done() and self.arov_yaw_controller.is_done())

    def get_asv_linear_distance(self, target) :
        """
        Computes xy distance between ASV and target pose

        :param target: target to compute distance from
        :return: distance from target
        """
        return math.sqrt((target.position.x - self.asv_x)**2 + (target.position.y - self.asv_y)**2)
    
    def arov_follower_pose(self) :
        """
        Generates target pose for AROV when following ASV

        :return: arov target x, arov target y, arov target z
        """
        dx = self.asv_x - self.arov_x
        dy = self.asv_y - self.arov_y
        d = math.sqrt(dx**2 + dy**2) if self.run_asv else 0
        if d == 0 :
            return self.arov_x, self.arov_y, self.arov_follow_z
        clear_d = (d - self.arov_follower_clearance) / d
        return self.arov_x + dx * clear_d, self.arov_y + dy * clear_d, self.arov_follow_z
    
    def asv_follower_pose(self) :
        """
        Generates target pose for ASV when following AROV

        :return: asv target x, asv target y, asv target yaw
        """
        dx = self.arov_x - self.asv_x
        dy = self.arov_y - self.asv_y
        d = math.sqrt(dx**2 + dy**2) if self.run_arov else 0
        if d == 0 :
            return self.asv_x, self.asv_y, self.asv_yaw
        clear_d = (d - self.asv_follower_clearance) / d
        theta = math.atan2(dy, dx)
        return self.asv_x + dx * clear_d, self.asv_y + dy * clear_d, theta
    
    def optimized_asv_target(self, target) :
        """
        Optimize ASV target pose by determining if target could be more quickly reached by turning away from it and then driving backwards

        :param target: target Pose
        :return: linear distance to drive (negative in backwards case), target yaw to reach
        """
        asv_target_yaw_towards = math.atan2(target.position.y - self.asv_y, target.position.x - self.asv_x)
        asv_target_yaw_away = normalize_angle(math.atan2(target.position.y - self.asv_y, target.position.x - self.asv_x) + math.pi)
        asv_linear_distance = self.get_asv_linear_distance(target)
        away_yaw_error = abs(normalize_angle(asv_target_yaw_away - self.asv_yaw))
        towards_yaw_error = abs(normalize_angle(asv_target_yaw_towards - self.asv_yaw))
        if asv_linear_distance > self.asv_linear_tolerance :
            if away_yaw_error < towards_yaw_error :
                return -asv_linear_distance, asv_target_yaw_away
            else :
                return asv_linear_distance, asv_target_yaw_towards
        else :
            return 0, self.asv_yaw
    
    def global_to_relative_arov_command(self, twist) :
        """
        Helper function to convert a map frame AROV command to a base_link one using current yaw

        :param twist: AROV command in map frame
        :return: AROV command in base_link frame
        """
        new_twist = Twist()
        new_twist.linear.x = twist.linear.x * math.cos(-self.arov_yaw) - twist.linear.y * math.sin(-self.arov_yaw)
        new_twist.linear.y = twist.linear.x * math.sin(-self.arov_yaw) + twist.linear.y * math.cos(-self.arov_yaw)
        new_twist.linear.z = twist.linear.z
        new_twist.angular.z = twist.angular.z
        return new_twist
    
    def stop(self) :
        """
        Send a 0 command to both robots and disable cleaners
        """
        self.asv_cmd_vel_publisher.publish(Twist())
        self.arov_cmd_vel_publisher.publish(Twist())
        self.toggle_cleaners(False)

    def publish_poses(self, pose_list) :
        """
        Publish a list of poses for visualization and debugging purposes
        """
        msg = PoseArray()
        msg.header.frame_id = 'map'
        msg.poses = pose_list
        self.pose_pub.publish(msg)

    def setup_routine_in_frame(self, frame_id) :
        """
        Transform the cleaning routine from AptilTag frame to map frame
        """
        if len(frame_id) == 0 :
            return [build_pose([self.arov_x, self.arov_y, self.arov_z, 0, 0, self.arov_yaw])]
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
        return out
    
    def get_poses(self) :
        """
        Helper function to get ASV and AROV poses from TF tree to set pose instance variables. Called at the start of each run of the state machine
        """
        while True :
            if self.run_asv :
                try :
                    asv_t = self.tf_buffer.lookup_transform('map', self.asv_base_link, rclpy.time.Time())
                    self.asv_x = asv_t.transform.translation.x
                    self.asv_y = asv_t.transform.translation.y
                    self.asv_yaw = euler_from_quaternion(asv_t.transform.rotation)[2]
                    if not self.run_arov :
                        break
                except TransformException as ex :
                    self.get_logger().warn(f'Could not get ASV pose: {ex}')
            if self.run_arov :
                try :
                    arov_t = self.tf_buffer.lookup_transform('map', self.arov_base_link, rclpy.time.Time())
                    self.arov_x = arov_t.transform.translation.x
                    self.arov_y = arov_t.transform.translation.y
                    self.arov_z = arov_t.transform.translation.z
                    self.arov_yaw = euler_from_quaternion(arov_t.transform.rotation)[2]
                    break
                except TransformException as ex :
                    self.get_logger().warn(f'Could not get AROV pose: {ex}')
    
    def go_home(self) :
        """
        Drives ASV to home pose. AROV follows if enabled. If ASV is disabled, instead drives AROV to home pose.
        """
        self.get_poses()
        asv_twist = Twist()
        arov_twist = Twist()
        # Set ASV target to home pose
        asv_target_distance, asv_target_yaw = self.optimized_asv_target(self.home)
        if self.do_set_asv_linear_target :
            self.asv_x_controller.set_target(asv_target_distance)
            self.do_set_asv_linear_target = False
        if self.do_set_asv_angular_target :
            self.asv_yaw_controller.set_target(normalize_angle(asv_target_yaw))
            self.do_set_asv_angular_target = False
        # Build ASV command
        asv_twist.angular.z = self.asv_yaw_controller.calculate(self.asv_yaw)
        if self.asv_yaw_controller.is_done() :
            # ASV only drives forward if yaw is within tolerance
            asv_twist.linear.x = self.asv_x_controller.calculate(self.asv_x_controller.get_target() - asv_target_distance)
        # Set AROV target
        if self.run_asv :
            # If ASV is running, AROV follows it
            arov_target_x, arov_target_y, arov_target_z = self.arov_follower_pose()
            self.set_arov_target(x=arov_target_x, y=arov_target_y, z=arov_target_z)
        elif self.do_set_arov_target :
            # If ASV is not running, AROV goes to home pose on its own
            self.set_arov_target(x=self.home.position.x, y=self.home.position.y, z=self.home.position.z)
            self.do_set_arov_target = False
        # Build AROV command
        self.arov_go_to_target_position(arov_twist)
        if self.asv_done() :
            self.do_set_asv_angular_target = True
            self.do_set_asv_linear_target = True
            self.stop()
            self.timer.destroy()
        # Publish commands
        if self.run_asv :
            self.asv_cmd_vel_publisher.publish(asv_twist)
        if self.run_arov :
            self.arov_cmd_vel_publisher.publish(self.global_to_relative_arov_command(arov_twist))

    def run_demo(self) :
        """
        Runs demo routine. If ASV is disabled, AROV will clean each fence without pausing between pairs. If AROV is disabled, ASV will loop through target poses a number of times equal to self.cleaning_cycles
        """
        self.get_logger().info(f"{self.state}")
        self.get_poses()
        asv_twist = Twist()
        arov_twist = Twist()
        if self.state == ControlState.EMPTYING :
            # Set AROV target to mort trap pose
            if self.do_set_arov_target :
                self.set_arov_target(x=self.arov_mort_trap_pose.position.x, 
                                     y=self.arov_mort_trap_pose.position.y, 
                                     z=self.arov_mort_trap_pose.position.z, 
                                     theta=euler_from_quaternion(self.arov_mort_trap_pose.orientation)[2])
                self.do_set_arov_target = False
            # Build AROV command
            self.arov_go_to_target_position(arov_twist)
            if not self.arov_move_then_turn or self.arov_move_done() :
                arov_twist.angular.z = self.arov_yaw_controller.calculate(self.arov_yaw)
            # Generate ASV following pose from AROV pose
            asv_target_x, asv_target_y, asv_target_theta = self.asv_follower_pose()
            asv_target_distance, asv_target_yaw = self.optimized_asv_target(build_pose([asv_target_x, asv_target_y, 0, 0, 0, asv_target_theta]))
            # Set ASV target to following pose
            self.asv_x_controller.set_target(asv_target_distance)
            self.asv_yaw_controller.set_target(normalize_angle(asv_target_yaw))
            # Build ASV command
            asv_twist.angular.z = self.asv_yaw_controller.calculate(self.asv_yaw)
            # Stop robots and kill command generator when done
            if self.asv_yaw_controller.is_done() :
                asv_twist.linear.x = self.asv_x_controller.calculate(self.asv_x_controller.get_target() - asv_target_distance)
            if self.arov_done() :
                self.do_set_arov_target = True
                self.stop()
                self.timer.destroy()
        else :
            # Set ASV target pose to next pose in self.asv_pose_targets. This happens in both the NAVIGATING and CLEANING states, since the ASV should maintain its pose in the CLEANING state
            asv_target_distance, asv_target_yaw = self.optimized_asv_target(self.asv_pose_targets[self.asv_pose_id])
            if self.do_set_asv_linear_target :
                self.asv_x_controller.set_target(asv_target_distance)
                self.do_set_asv_linear_target = False
            if self.do_set_asv_angular_target :
                self.asv_yaw_controller.set_target(normalize_angle(asv_target_yaw))
                self.do_set_asv_angular_target = False
            # Build ASV command
            asv_twist.angular.z = self.asv_yaw_controller.calculate(self.asv_yaw)
            if self.asv_yaw_controller.is_done() :
                # ASV only drives forward if yaw is within tolerance
                asv_twist.linear.x = self.asv_x_controller.calculate(self.asv_x_controller.get_target() - asv_target_distance)
            if self.state == ControlState.NAVIGATING :
                # Generate AROV following pose from ASV pose
                arov_target_x, arov_target_y, arov_target_z = self.arov_follower_pose()
                self.set_arov_target(x=arov_target_x, y=arov_target_y, z=arov_target_z)
                # Build AROV command
                self.arov_go_to_target_position(arov_twist)
                if self.asv_done() :
                    # If ASV has reached its idling pose for cleaning routine, switch to CLEANING state
                    self.state = ControlState.CLEANING
                    # Build the cleaning routine in the map frame around the AprilTag frame pair at this pose
                    self.transformed_fence_poses = []
                    self.transformed_fence_poses.append(self.setup_routine_in_frame(self.arov_fence_frames[self.asv_pose_id][0]))
                    self.transformed_fence_poses.append(self.setup_routine_in_frame(self.arov_fence_frames[self.asv_pose_id][1]))
                    self.publish_poses(self.transformed_fence_poses[self.arov_fence_switch])
                    self.do_set_asv_angular_target = True
                    self.do_set_asv_linear_target = True
            elif self.state == ControlState.CLEANING :
                # Set AROV target pose to next pose in cleaning routine
                if self.do_set_arov_target :
                    self.set_arov_target(x=self.transformed_fence_poses[self.arov_fence_switch][self.arov_routine_pose_id].position.x, 
                                         y=self.transformed_fence_poses[self.arov_fence_switch][self.arov_routine_pose_id].position.y, 
                                         z=self.transformed_fence_poses[self.arov_fence_switch][self.arov_routine_pose_id].position.z, 
                                         theta=euler_from_quaternion(self.transformed_fence_poses[self.arov_fence_switch][self.arov_routine_pose_id].orientation)[2])
                    self.do_set_arov_target = False
                # Only run cleaners when at desired xy pose. Compensates for backwards thrust produced by cleaners.
                if self.arov_x_controller.is_done() and self.arov_y_controller.is_done() :
                    self.toggle_cleaners(True)
                else :
                    self.toggle_cleaners(False)
                # Build AROV command
                self.arov_go_to_target_position(arov_twist)
                if not self.arov_move_then_turn or self.arov_move_done() :
                    arov_twist.angular.z = self.arov_yaw_controller.calculate(self.arov_yaw)
                # When pose in routine is reached, increment AROV target ID
                if self.arov_done() :
                    self.toggle_cleaners(False)
                    self.do_set_arov_target = True
                    self.arov_routine_pose_id += 1
                    # If ID is outside of routine, routine has been completed and ID is reset.
                    if self.arov_routine_pose_id >= len(self.transformed_fence_poses[self.arov_fence_switch]) :
                        self.arov_routine_pose_id = 0
                        # If the cleaned fence was the second in a pair, the cleaning around this pair is complete.
                        if self.arov_fence_switch == 1 :
                            self.do_set_asv_linear_target = True
                            self.do_set_asv_angular_target = True
                            # Increment ASV target ID and completed cleaning cycles
                            self.asv_pose_id = (self.asv_pose_id + 1) % len(self.asv_pose_targets)
                            self.arov_fence_switch = 0
                            self.cleaning_cycles_completed += 1
                            # If completed all cycles, switch to EMPTYING state. Otherwise, switch to NAVIGATING state.
                            self.state = ControlState.NAVIGATING if self.cleaning_cycles_completed < self.cleaning_cycles else ControlState.EMPTYING
                        # If the cleaned fence was the first in a pair, switch to second
                        else :
                            self.arov_fence_switch = 1
                        self.publish_poses(self.transformed_fence_poses[self.arov_fence_switch])
        # Publish commands
        if self.run_asv :
            self.asv_cmd_vel_publisher.publish(asv_twist)
        if self.run_arov :
            self.arov_cmd_vel_publisher.publish(self.global_to_relative_arov_command(arov_twist))
    
    def service_callback(self, request, response) :
        """
        Service callback to handle setting control mode
        """
        mode_string = "HOME" if request.mode == 2 else "START" if request.mode == 1 else "STOP"
        self.get_logger().info(f"Receiving task with mode {mode_string}")
        self.reset_instance_variables()
        if self.timer is not None :
            self.timer.destroy()
        self.stop()
        if request.mode == 1 :
            self.timer = self.create_timer(self.dt, self.run_demo)
        elif request.mode == 2 :
            self.timer = self.create_timer(self.dt, self.go_home)
        return response

def main(args=None) :
    rclpy.init()
    node = Navigation0()
    rclpy.spin(node)

if __name__ == '__main__' :
    main()
