# Exercise 4 - following a colour (green) and stopping upon sight of another (blue).

#from __future__ import division
import threading
import sys, time
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Vector3
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from rclpy.exceptions import ROSInterruptException
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
import signal
from math import sin, cos


# Need to do three things:
# 1. Make sure the package builds
# 2. Use goto point to navigate to the boxes
# 3. Use the camera to detect the all three RGB colours highlighted with bounding boxes        
class Robot(Node):
    def __init__(self):
        super().__init__('robot')
        # Subscribe to Robot base and camera topics
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.rate = self.create_rate(10)  # 10 Hz
        self.subscription = self.create_subscription(Image, '/camera/image_raw', self.processImage, 10)
        self.bridge = CvBridge()

        # Colour detection flags
        self.gflag = 0
        self.bflag = 0
        self.rflag = 0
        self.sensitivity = 90

        # GOTO points
        self.red_box = (-5.0, -2.0, 2.47)
        self.green_box = (3.0, -6.0, -0.48)
        self.blue_box = (-5.8, -8.8, -0.48)

        # action client
        self.action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        
        # area of the bounding box
        self.big_area = 121852.0 # big
        self.medium_area = 10000.0 

        # goal handles
        self.current_goal_handle = None
        self.timer = self.create_timer(0.5, self.check_flags)


    def processImage(self, data):
        image = self.bridge.imgmsg_to_cv2(data, 'bgr8')

        # Use masks to detect the three colours
        hsv_green_lower = np.array([60 - self.sensitivity, 100, 100])
        hsv_green_upper = np.array([60 + self.sensitivity, 255, 255])
        hsv_blue_lower = np.array([120 - self.sensitivity, 100, 100])
        hsv_blue_upper = np.array([120 + self.sensitivity, 255, 255])   
        hsv_red_lower1 = np.array([0, 100, 100])
        hsv_red_upper1 = np.array([self.sensitivity, 255, 255])
        hsv_red_lower2 = np.array([180 - self.sensitivity, 100, 100])
        hsv_red_upper2 = np.array([180, 255, 255])

        # Convert the rgb image into a hsv image
        hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        red_mask1 = cv2.inRange(hsv_image, hsv_red_lower1, hsv_red_upper1)
        red_mask2 = cv2.inRange(hsv_image, hsv_red_lower2, hsv_red_upper2)

        red_mask = cv2.bitwise_or(red_mask1, red_mask2)
        green_mask = cv2.inRange(hsv_image, hsv_green_lower, hsv_green_upper)
        blue_mask = cv2.inRange(hsv_image, hsv_blue_lower, hsv_blue_upper)

        full_mask = cv2.bitwise_or(green_mask, blue_mask)
        full_mask = cv2.bitwise_or(full_mask, red_mask)

        filtered_img = cv2.bitwise_and(image, image, mask=full_mask)

        # Use countours to find the bounding boxes
        # make a different countour for each colour
        red_countours, _ = cv2.findContours(red_mask, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
        green_countours, _ = cv2.findContours(green_mask, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
        blue_countours, _ = cv2.findContours(blue_mask, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
        flags = [self.rflag, self.gflag, self.bflag]
        # zip
        countours = [red_countours, green_countours, blue_countours]
        countours = zip(countours, flags)
        for i, (cnts, flag) in enumerate(countours):
            if len(cnts) > 0:
                # Using the area of the countours, determine if the robot is close enough to the object
                for cnt in cnts:
                    area = cv2.contourArea(cnt)
                    print(f'OBJECT {i} AREA: {area}')
                    if area < self.big_area: # if we are not close enough
                        # make a bounding box
                        x, y, w, h = cv2.boundingRect(cnt)
                        cv2.rectangle(image, (x, y), (x + w, y + h), (255, 165, 0), 2)

                        if i == 2 and self.bflag == 0:
                            if area > self.medium_area: # if blue is in sight but not close enough
                                # Special handling for blue
                                self.get_logger().info("Centering blue object...")
                                if self.center_object(x, w, image.shape[1]):
                                    self.get_logger().info("Centered. Moving forward...")
                                    self.move_forward()

                    else: # if we are close enough
                        # set the flag to 1
                        flag = 1
                        # make a green bounding box
                        x, y, w, h = cv2.boundingRect(cnt)
                        cv2.rectangle(image, (x, y), (x + w, y + h), (0, 255, 0), 2)

        # Display
        cv2.namedWindow("Filtered Image", cv2.WINDOW_NORMAL)
        cv2.imshow("Filtered Image", filtered_img)
        cv2.resizeWindow("Filtered Image", 320, 240)
        cv2.namedWindow('Camera Feed',cv2.WINDOW_NORMAL)
        cv2.imshow('Camera Feed', image)
        cv2.resizeWindow('Camera Feed',320,240)
        cv2.waitKey(3)
        # Check if the area of the shape you want is big enough to be considered

    def goto(self, coords):
        x, y, yaw = coords
        # from lab 4
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        
        # Position
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        
        # Orientation
        goal_msg.pose.pose.orientation.z = sin(yaw / 2)
        goal_msg.pose.pose.orientation.w = cos(yaw / 2)

        self.action_client.wait_for_server()
        self.send_goal_future = self.action_client.send_goal_async(goal_msg)
        self.send_goal_future.add_done_callback(self.goal_response_callback)

    # from lab 4
    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            return

        self.get_logger().info('Goal accepted')
        self.get_result_future = goal_handle.get_result_async()
        self.get_result_future.add_done_callback(self.get_result_callback)

    def center_object(self, x, w, frame_width):
        # Center of the object
        object_center = x + w // 2
        frame_center = frame_width // 2
        error = object_center - frame_center

        twist = Twist()
        if abs(error) > 20:
            # Turn robot to center object
            twist.angular.z = -0.002 * error  # Negative because image x increases to right
            self.publisher.publish(twist)
            return False  # Not centered yet
        else:
            self.publisher.publish(Twist())  # Stop rotation
            return True  # Centered

    def move_forward(self):
        twist = Twist()
        twist.linear.x = 0.1
        self.publisher.publish(twist)


    def check_flags(self):
        flags = [self.gflag, self.rflag, self.bflag]
        if self.current_goal_index < len(flags):
            # if we are looking for the blue box and we see it but it is not close enough
            if self.current_goal_index == 2 and self.bflag == 1 and self.current_goal_handle:
                self.get_logger().info("Blue object found. Cancelling goal for visual tracking.")
                self.current_goal_handle.cancel_goal_async()
                self.current_goal_handle = None
                return  # Let processImage handle the movement

            # Handle for the red and blue boxes
            if flags[self.current_goal_index] == 1 and self.current_goal_handle:
                self.get_logger().info("Target found! Cancelling current goal.")
                self.current_goal_handle.cancel_goal_async()
                self.current_goal_handle = None

                self.current_goal_index += 1
                if self.current_goal_index < len(self.goal_sequence):
                    next_coords = self.goal_sequence[self.current_goal_index]
                    self.get_logger().info(f'Going to next goal: {next_coords}')
                    self.goto(next_coords)
                else:
                    self.get_logger().info("All goals reached.")
                    self.stop()
                    self.destroy_node()
                    rclpy.shutdown()
                    cv2.destroyAllWindows()

    def goal_response_callback(self, future):
        self.current_goal_handle = future.result()
        if not self.current_goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            return

        self.get_logger().info('Goal accepted')
        self.get_result_future = self.current_goal_handle.get_result_async()
        self.get_result_future.add_done_callback(self.get_result_callback)


    def stop(self):
        # Use what you learnt in lab 3 to make the robot stop
        desired_velocity = Twist()


        self.publisher.publish(desired_velocity)

def main():
    import signal

    rclpy.init(args=None)
    robot = Robot()

    def signal_handler(sig, frame):
        print("Stopping robot...")
        robot.stop()
        print("Robot stopped.")
        robot.destroy_node()
        print("Node destroyed.")
        rclpy.shutdown()
        print("ROS2 shutdown.")
        cv2.destroyAllWindows()
        print("OpenCV windows destroyed.")

    signal.signal(signal.SIGINT, signal_handler)

    print("Press Ctrl+C to stop the robot")
    print("Going to green box...")

    robot.goal_sequence = [robot.green_box, robot.red_box, robot.blue_box]
    robot.current_goal_index = 0

    def new_result_callback(future):
        result = future.result().result
        robot.get_logger().info(f'Navigation result: {result}')

        robot.current_goal_index += 1
        if robot.current_goal_index < len(robot.goal_sequence):
            next_coords = robot.goal_sequence[robot.current_goal_index]
            robot.get_logger().info(f'Going to next goal: {next_coords}')
            robot.goto(next_coords)
        else:
            robot.get_logger().info("All goals reached.")
            robot.stop()
            robot.destroy_node()
            rclpy.shutdown()
            cv2.destroyAllWindows()

    robot.get_result_callback = new_result_callback

    robot.goto(robot.goal_sequence[0])

    try:
        rclpy.spin(robot)
    except KeyboardInterrupt:
        pass
        robot.stop()
        robot.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
