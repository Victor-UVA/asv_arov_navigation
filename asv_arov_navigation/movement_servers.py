import rclpy
from rclpy.node import Node
import math
import numpy as np
from rclpy.action import ActionServer, ActionClient, CancelResponse, GoalResponse
from asv_arov_interfaces.action import NavigationAction
from geometry_msgs.msg import PoseStamped, Pose, Quaternion
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from nav_msgs.msg import Odometry
from robot_guidance_interfaces.action import GoToDepth, GoToSide, NavigateAprilTags # Custom action
from tf2_ros import TransformException, Buffer, TransformListener
from scipy.spatial.transform import Rotation
from asv_arov_navigation.utils import build_pose_stamped

class MovementServerManager(Node) :
    def between_fence_init(self, arov_navigator) :
        self.action_server = ActionServer(self, NavigationAction, 'navigation_action', self.navigation_callback)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.declare_parameter('use_sim', False)
        self.use_sim: bool = self.get_parameter('use_sim').get_parameter_value().bool_value

        self.asv_nav = BasicNavigator(namespace="asv")
        self.asv_nav.waitUntilNav2Active(localizer="/asv/pose_publisher")
        self.arov_nav = arov_navigator

        self.follow_distance = 1.0
    
    def fence_clean_init(self, navigator) :
        # Get topic nameSs
        #self.declare_parameter('tag_detections_topic', '/tag_detections')
        self.declare_parameter('odom_topic', '/odom')
        #tag_topic = self.get_parameter('tag_detections_topic').get_parameter_value().string_value
        odom_topic = self.get_parameter('odom_topic').get_parameter_value().string_value

        self.goal_tolerance = 0.05
        # Subscribers 
        #self.tag_sub = self.create_subscription(
        #    AprilTagDetectionArray,
        #    tag_topic,
        #    self.tag_callback,
        #    10
        #)
        self.odom_sub = self.create_subscription(
            Odometry,
            odom_topic,
            self.odom_callback,
            10
        )

        # Basic Navigator for moving to a pose
        self.navigator = navigator
        # waiting for Nav2Active
        #self.navigator.waitUntilNav2Active()  # The BasicNavigator.waitUntilNav2Active() function explicitly checks for /amcl/get_state. If you're not using AMCL and that service doesn't exist, this call will block forever.
        
        self._action_server = ActionServer(
            self,
            NavigateAprilTags,
            'navigate_apriltags',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback
        )
        # GoToDepth  and DriveOnHeading action clients
        self.depth_control_client = ActionClient(self, GoToDepth, 'go_to_depth')
        self.strafe_control_client = ActionClient(self, GoToSide, 'go_to_side')
        # Transform Listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        # Variable Init
        #self.current_detections = {}
        self.current_pose = None
        # April Tag Type
        self.tag_family = 'tag36h11'
        #self.goal_offset = 0.5

    def navigation_callback(self, msg):
        if msg.request.mode == 0:   # stop all tasks
            self.send_goal(PoseStamped(), PoseStamped(), 0) # AROV cancel task
            self.asv_nav.cancelTask()

        else:
            leader_goal_pose = msg.request.goal
            self.get_logger().info("Defined goal pose")

            if (msg.request.mode % 2) == 1:
                asv_initial_pose = self._get_initial_pose('asv')
                self.get_logger().info("Defined initial poses")
                leader_goal_pose.pose.position.z = asv_initial_pose.pose.position.z
                if msg.request.mode == 1:   # ASV leads, AROV follows
                    return self._goto('asv', asv_initial_pose, leader_goal_pose, True)
                else:   # only ASV moves
                    return self._goto('asv', asv_initial_pose, leader_goal_pose, False)
            else:
                arov_initial_pose = self._get_initial_pose('arov')
                self.get_logger().info("Defined initial poses")
                leader_goal_pose.pose.position.z = arov_initial_pose.pose.position.z
                if msg.request.mode == 2:   # AROV leads, ASV follows
                    return self._goto('arov', arov_initial_pose, leader_goal_pose, True)
                else:   # only AROV moves
                    return self._goto('arov', arov_initial_pose, leader_goal_pose, False)

    def _goto(self, leader, leader_initial_pose, leader_goal_pose, follower_enabled):
        leader_success = False
        follower_success = True
        if leader == 'asv' and follower_enabled:
            lead_nav = self.asv_nav
            self.get_logger().info("ASV set as leader")
        else:
            lead_nav = self.arov_nav
            self.get_logger().info("AROV set as leader")

        lead_nav.setInitialPose(leader_initial_pose)
        self.get_logger().info(f"Set {leader}'s initial pose")
        lead_nav.goToPose(leader_goal_pose)
        self.get_logger().info(f"Completed {leader}'s goToPose call")

        if follower_enabled:
            if leader == 'asv':
                follower = 'arov'
                follow_nav = self.arov_nav
                self.get_logger().info("AROV set as follower")
            else:
                follower = 'asv'
                follow_nav = self.asv_nav
                self.get_logger().info("ASV set as follower")
                
            while not lead_nav.isTaskComplete():
                leader_current_pose = self._get_initial_pose(leader)
                follower_current_pose = self._get_initial_pose(follower)
                follower_goal_pose = self._calculate_pose(follower_current_pose, leader_current_pose, self.follow_distance)

                follow_nav.setInitialPose(follower_current_pose)
                # self.get_logger().info(f"Got {follower}'s initial pose")
                follow_nav.cancelTask()
                # self.get_logger().info(f"Canceled {follower}'s current goto")
                follow_nav.goToPose(follower_goal_pose)
                # self.get_logger().info(f"Completed {follower}'s goToPose call")

            if lead_nav.isTaskComplete():
                leader_success = True if lead_nav.getResult() == 0 else False
                self.get_logger().info(f"Leader finished moving - success: {leader_success}")

                follower_goal_pose = self._get_initial_pose(leader)
                follower_current_pose = self._get_initial_pose(follower)
                follower_goal_pose.pose.position.z = follower_current_pose.pose.position.z

                follow_nav.setInitialPose(follower_current_pose)
                # self.get_logger().info(f"Got {follower}'s initial pose")
                follow_nav.cancelTask()
                # self.get_logger().info(f"Canceled {follower}'s current goto")
                follow_nav.goToPose(follower_goal_pose)
                # self.get_logger().info(f"Completed {follower}'s goToPose call")

                while not follow_nav.isTaskComplete():
                    pass
                follower_success = True if self.asv_nav.getResult() == 0 else False
                self.get_logger().info(f"Follower finished moving - success: {follower_success}")

        else:
            while not lead_nav.isTaskComplete():
                pass
            leader_success = True if lead_nav.getResult() == 0 else False
            self.get_logger().info(f"Leader finished moving - success: {leader_success}")

        self.get_logger().info(f"Done with navigation action(s) - success: {leader_success and follower_success}")
        result = NavigationAction.Result()
        result.goal_reached = leader_success and follower_success
        return result

    def _get_initial_pose(self, vehicle):
        while True :
            try:
                transform = self.tf_buffer.lookup_transform('map', vehicle + '/base_link', rclpy.time.Time())
                return build_pose_stamped(self.get_clock().now(), "map", [transform.transform.translation.x, transform.transform.translation.y, transform.transform.translation.z], transform.transform.rotation)
            except TransformException as initial_pose_ex:
                self.get_logger().warning(f'Could not get {vehicle} initial pose: {initial_pose_ex}')

    def _calculate_pose(self, follower_pose, leader_pose, follower_clearance):
        dx = leader_pose.pose.position.x - follower_pose.pose.position.x
        dy = leader_pose.pose.position.y - follower_pose.pose.position.y
        psi = math.atan2(dy, dx)
        goal_x = leader_pose.pose.position.x - math.cos(psi) * follower_clearance
        goal_y = leader_pose.pose.position.y - math.sin(psi) * follower_clearance

        return build_pose_stamped(self.get_clock().now(), "map", [goal_x, goal_y, follower_pose.pose.position.z, 0, 0, psi - math.pi])
    
    def odom_callback(self, msg: Odometry):
        #self.get_logger().info('Getting Odometry...')
        self.current_pose = msg.pose.pose

    #def tag_callback(self, msg: AprilTagDetectionArray):
    #    self.current_detections = {
    #        f'{detection.family}:{detection.id}': detection for detection in msg.detections
    #        }

    def goal_callback(self, goal_request):
        self.get_logger().info('Received goal request')
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        self.get_logger().info('Received cancel request')
        return CancelResponse.ACCEPT
    
    def get_strafe_heading(self, start_pose: Pose, goal_pose: Pose, stafe_direction) -> Quaternion:
        strafe_left = False
        if stafe_direction == "left":
            strafe_left = True
        
        dx = goal_pose.position.x - start_pose.position.x
        dy = goal_pose.position.y - start_pose.position.y

        # Direction from start to goal
        theta = math.atan2(dy, dx)

        # Rotate to face perpendicular
        if strafe_left:
            yaw = theta + math.pi / 2 
        else: # strafe right
            yaw = theta - math.pi / 2

        quat = Rotation.from_euler('z', yaw).as_quat()  # [x, y, z, w]

        q = Quaternion()
        q.x, q.y, q.z, q.w = quat
        return q
    
    def compute_distance(self, pose1, pose2):
        dx = pose1.position.x - pose2.position.x
        dy = pose1.position.y - pose2.position.y
        #dz = pose1.position.z - pose2.position.z
        return math.sqrt(dx*dx + dy*dy) #+ dz*dz)

    async def execute_callback(self, goal_handle):
        goal = goal_handle.request
        goals_list = goal.goals
        command_list = goal.commands

        for i, tag_id in enumerate(goals_list[:-1]): # Loop excludes last element
            current_goal = goals_list[i]
            next_goal = goals_list[i+1]

            self.get_logger().info(f'Going to pose number {i}')

            # 3. Send initial goToPose
            # Build navigation goal from transform
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.pose.position.x = current_goal.pose.position.x
            pose.pose.position.y = current_goal.pose.position.y
            pose.pose.orientation = current_goal.pose.orientation
            self.navigator.goToPose(pose)

            start_time = self.get_clock().now()
            # 4. Spin while tag is not yet detected and goal not reached
            while rclpy.ok() and not self.navigator.isTaskComplete():
                feedback = self.navigator.getFeedback()
                elapsed_time = (self.get_clock().now() - start_time).nanoseconds / 1e9  # seconds
                if elapsed_time > 600:
                # If task taking too long or april tag detected
                    self.navigator.cancelTask()
                    self.get_logger().warn(f'Goal timeout without being reached {tag_id}')
                    goal_handle.abort()
                    return NavigateAprilTags.Result(navigation_completed=False) # What is this?
                
            if self.navigator.getResult() != TaskResult.SUCCEEDED:
                self.get_logger().warn(f"Failed to reach a goal.")
                goal_handle.abort()
                return NavigateAprilTags.Result(navigation_completed=False)

            # Execute command depending on Tag Pattern
            self.get_logger().info(f'Reached pose number {i}, executing command...')

            command = command_list[i]

            self.get_logger().info(f'Command {command}')

            if command == 'vertical': 
                self.get_logger().info(f'Vertical Command')
                desired_depth = next_goal.pose.position.z
                depth_goal = GoToDepth.Goal()
                depth_goal.target_depth = desired_depth

                self.get_logger().info(f'Waiting for depth control server to start')
                # Wait for server to be ready
                if not self.depth_control_client.wait_for_server(timeout_sec=5.0):
                    self.get_logger().error("GoToDepth action server not available after 5 seconds!")
                    goal_handle.abort()
                    return NavigateAprilTags.Result(navigation_completed=False)
                # Send the goal
                self.get_logger().info(f"Sending GoToDepth goal to {desired_depth:.2f} meters")
                
                send_goal_future = self.depth_control_client.send_goal_async(depth_goal)
                depth_goal_handle = await send_goal_future

                if not depth_goal_handle.accepted:
                    self.get_logger().warn("Depth goal rejected")
                    goal_handle.abort()
                    return NavigateAprilTags.Result(navigation_completed = False)
            
                result_response = await depth_goal_handle.get_result_async()
                
                if not result_response.result.reached_final_depth:
                    self.get_logger().warn("Depth goal failed")
                    goal_handle.abort()
                    return NavigateAprilTags.Result(navigation_completed = False)
                
                self.get_logger().info("Depth goal succeeded")
            
            elif command == 'horizontal':
                self.get_logger().info(f'Horizontal Command')
                distance = np.inf
                
                while distance > self.goal_tolerance:
                    distance = self.compute_distance(self.current_pose, next_goal.pose)
                    self.get_logger().info(f"Distance {distance:.2f} meters")

                    self.get_logger().info(f'Spin in place to stafing angle')
                    q = self.get_strafe_heading(self.current_pose, next_goal.pose, 'right')

                    # Build navigation goal from transform
                    pose = PoseStamped()
                    pose.header.frame_id = 'map'
                    pose.header.stamp = self.get_clock().now().to_msg()
                    pose.pose.position.x = self.current_pose.position.x
                    pose.pose.position.y = self.current_pose.position.y
                    pose.pose.orientation = q
                    self.navigator.goToPose(pose)

                    start_time = self.get_clock().now()
                    # 4. Spin while tag is not yet detected and goal not reached
                    while rclpy.ok() and not self.navigator.isTaskComplete():
                        elapsed_time = (self.get_clock().now() - start_time).nanoseconds / 1e9  # seconds
                        if elapsed_time > 600:
                        # If task taking too long or april tag detected
                            self.navigator.cancelTask()
                            self.get_logger().warn(f'Strafing heading goal timeout without being reached')
                            goal_handle.abort()
                            return NavigateAprilTags.Result(navigation_completed = False)
                        
                    if self.navigator.getResult() != TaskResult.SUCCEEDED:
                        self.get_logger().warn(f"Failed to reach desired heading.")
                        goal_handle.abort()
                        return NavigateAprilTags.Result(navigation_completed = False)
                    
                    strafe_goal = GoToSide.Goal()
                    strafe_goal.target_pose = next_goal

                    self.get_logger().info(f'Waiting for strafe control server to start')
                    # Wait for server to be ready
                    if not self.strafe_control_client.wait_for_server(timeout_sec=5.0):
                        self.get_logger().error("GoToSide action server not available after 5 seconds!")
                        goal_handle.abort()
                        return NavigateAprilTags.Result(navigation_completed=False)
                    # Send the goal
                    self.get_logger().info(f"Sending GoToSide goal")
                    
                    send_goal_future = self.strafe_control_client.send_goal_async(strafe_goal)
                    strafe_goal_handle = await send_goal_future

                    if not strafe_goal_handle.accepted:
                        self.get_logger().warn("Strafe goal rejected")
                        goal_handle.abort()
                        return NavigateAprilTags.Result(navigation_completed = False)
                
                    result_response = await strafe_goal_handle.get_result_async()
                    
                    if not result_response.result.target_reached:
                        self.get_logger().warn("Strafe goal failed")
                        goal_handle.abort()
                        return NavigateAprilTags.Result(navigation_completed = False)
                    
                    self.get_logger().info("Strafe goal succeeded")
                            
        self.get_logger().info("Navigation succeeded")
        goal_handle.succeed()
        return NavigateAprilTags.Result(navigation_completed = True)

    def __init__(self) :
        super().__init__('movement_server_manager')
        arov_navigator = BasicNavigator("arov")
        arov_navigator.waitUntilNav2Active(localizer="/arov/pose_publisher")
        self.between_fence_init(arov_navigator)
        self.fence_clean_init(arov_navigator)

def main(args=None) :
    rclpy.init()
    manager_server = MovementServerManager()
    
    try:
        rclpy.spin(manager_server)
    except KeyboardInterrupt:
        pass
    finally:
        manager_server.destroy_node()
        rclpy.shutdown()