import rclpy
from enum import Enum
import numpy as np
from rclpy.node import Node
from rclpy.action import ActionServer
from sensor_msgs.msg import Image
from asv_arov_interfaces.action import DumbCleanAction
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import cv2

class CleaningState(Enum) :
    FOLLOWING_BAR = 1
    STRAFING = 2
    LOST_BAR = 3

class DumbCleaner(Node) :
    def __init__(self) :
        super().__init__('dumb_cleaner')

        self.publisher = self.create_publisher(Twist, 'arov/cmd_vel')
        self.camera_subscriber = self.create_subscription(Image, '/image_rect', self.camera_feed_callback, 10)
        self.action_server = ActionServer(self, DumbCleanAction, 'dumb_cleaner', self.execute_callback)

        self.COLORS = [[(), ()], [(), ()]]
        self.END_COLORS = [[(), ()], [(), ()]]

        self.spin_speed = 0
        self.vertical_speed = 0
        self.strafing_speed = 0
        self.rotation_centering_kP = 0
        self.strafing_centering_kP = 0
        self.clearance_kP = 0
        self.bar_count = 0
        self.cleaning_height = 0
        self.cleaning_bar_width = 0
        self.strafing_bar_width = 0

        self.bar_distance_from_center = 0
        self.bar_width_error = 0
        self.bar_counter = 0
        self.vertical_switch = -1
        self.detecting_end = False

        self.br = CvBridge()

        self.state = CleaningState.LOST_BAR

    def camera_feed_callback(self, data) :
        frame = self.br.imgmsg_to_cv2(data)
        color_id = 0 if self.vertical_switch == -1 else 1
        self.bar_lower = np.array(self.COLORS[color_id][0], dtype=np.uint8)
        self.bar_upper = np.array(self.COLORS[color_id][1], dtype=np.uint8)
        self.end_lower = np.array(self.END_COLORS[color_id][0], dtype=np.uint8)
        self.end_upper = np.array(self.END_COLORS[color_id][1], dtype=np.uint8)
        bar_mask = cv2.inRange(frame, self.bar_lower, self.bar_upper)
        end_mask = cv2.inRange(frame, self.end_lower, self.end_upper)
        bar_contours = None
        end_contours = None
        cv2.findContours(bar_mask, bar_contours, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        cv2.findContours(end_mask, end_contours, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if bar_contours :
            largest_bar_contour = max(bar_contours, cv2.contourArea)
        if end_contours :
            largest_end_contour = max(end_contours, cv2.contourArea)
        if end_contours and self.state == CleaningState.FOLLOWING_BAR :
            self.detecting_end = True
            x,_,w,_ = cv2.boundingRect(largest_end_contour)
            self.bar_width_error = self.strafing_bar_width - w
            self.bar_distance_from_center = len(frame[0]) / 2 - (x + w / 2)
        elif bar_contours :
            self.detecting_end = False
            x,_,w,_ = cv2.boundingRect(largest_bar_contour)
            self.bar_width_error = self.cleaning_bar_width - w
            self.bar_distance_from_center = len(frame[0]) / 2 - (x + w / 2)
        else :
            self.detecting_end = False
            self.bar_width_error = None

    def execute_callback(self, msg) :
        self.bar_counter = 0
        self.bar_count = msg.request.bars
        self.vertical_switch = msg.request.switch_start_state
        self.last_time = None

        while self.bar_counter < self.bar_count :
            cmd_vel = Twist()
            if self.state == CleaningState.LOST_BAR :
                if self.bar_distance_from_center is None :
                    cmd_vel.angular.z = self.spin_speed
                else :
                    self.state = CleaningState.FOLLOWING_BAR
            elif self.state == CleaningState.FOLLOWING_BAR :
                if self.detecting_end :
                    cmd_vel.linear.x = self.clearance_kP * self.bar_width_error
                    self.state = CleaningState.STRAFING
                    self.bar_counter += 1
                    self.vertical_switch = 1 if self.vertical_switch == -1 else -1
                else :
                    cmd_vel.angular.z = self.rotation_centering_kP * self.bar_distance_from_center
                    cmd_vel.linear.x = self.clearance_kP * self.bar_width_error
                    cmd_vel.linear.y = self.strafing_centering_kP * self.bar_distance_from_center
                    cmd_vel.linear.z = self.vertical_switch * self.vertical_speed
            elif self.state == CleaningState.STRAFING :
                if self.bar_width_error is not None :
                    cmd_vel.linear.x = self.clearance_kP * self.bar_width_error
                    self.state = CleaningState.FOLLOWING_BAR
                else :
                    cmd_vel.linear.y = self.strafing_speed
            self.publisher.publish(cmd_vel)
            rclpy.spin_once(self)
        
        return DumbCleanAction.Result()
    
def main(args=None) :
    rclpy.init()
    server = DumbCleaner()
    rclpy.spin(server)

if __name__ == '__main__' :
    main()