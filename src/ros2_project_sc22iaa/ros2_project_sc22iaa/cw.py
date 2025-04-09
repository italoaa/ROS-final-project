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
        self.sensitivity = 10

        # GOTO points
        self.red_box = (-5.0, -2.0, 2.47)
        self.green_box = (3.0, -6.0, -0.48)
        self.blue_box = (-4.5, -8.4, -0.48)

        # action client
        self.action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        
        # area of the bounding box
        # far away from green box to red red box is 14500 in area
        self.big_area =   10000.0 # big
        self.medium_area = 1000.0 # we see an object

        # goal handles
        self.current_goal_handle = None

    def processImage(self, data):
        image = self.bridge.imgmsg_to_cv2(data, 'bgr8')
        cv2.namedWindow('Camera Feed',cv2.WINDOW_NORMAL)
        cv2.imshow('Camera Feed', image)
        cv2.resizeWindow('Camera Feed',320,240)
        cv2.waitKey(3)

        # Use masks to detect the three colours
        hsv_green_lower = np.array([60 - self.sensitivity, 100, 100])
        hsv_green_upper = np.array([60 + self.sensitivity, 255, 255])
        hsv_blue_lower = np.array([120 - self.sensitivity, 100, 100])
        hsv_blue_upper = np.array([120 + self.sensitivity, 255, 255])   
        hsv_red_lower = np.array([0 - self.sensitivity, 100, 100])
        hsv_red_upper = np.array([0 + self.sensitivity, 255, 255])

        # Convert the rgb image into a hsv image
        hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        red_mask = cv2.inRange(hsv_image, hsv_red_lower, hsv_red_upper)
        green_mask = cv2.inRange(hsv_image, hsv_green_lower, hsv_green_upper)
        blue_mask = cv2.inRange(hsv_image, hsv_blue_lower, hsv_blue_upper)

        full_mask = cv2.bitwise_or(green_mask, blue_mask)
        full_mask = cv2.bitwise_or(full_mask, red_mask)

        filtered_img = cv2.bitwise_and(image, image, mask=full_mask)

        # Use countours to find the bounding boxes
        # make a different countour for each colour
        countours = [
            cv2.findContours(mask, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)[0]
            for mask in [red_mask, green_mask, blue_mask]
        ]
        # zip
        # flags 0 = red 1 = green 2 = blue
        for i, cnts in enumerate(countours):
            if len(cnts) > 0:
                # Using the area of the countours, determine if the robot is close enough to the object
                c = max(cnts, key=cv2.contourArea)
                # if the area is big enough, set the flag to 1                
                print(f'Flags: {[self.rflag, self.gflag, self.bflag]} - Color: {["red", "green", "blue"][i]} - Area: {cv2.contourArea(c)}', end='\r')

                # If the area is medium size
                area = cv2.contourArea(c)
                if area > self.medium_area:
                    (x, y), radius = cv2.minEnclosingCircle(c)
                    center = (int(x), int(y))
                    radius = int(radius)
                    cv2.circle(image, center, radius, (0, 255, 0), 2)

                    if area > self.big_area:
                        # if we see and it is our goal, we must go to the goal
                        if i == 0 and self.current_goal_index == 0:
                            self.rflag = 1
                            self.get_logger().info("Red object detected. Stopping...")
                            self.stop()
                            self.cancel_goal()
                        elif i == 1 and self.current_goal_index == 1:
                            self.gflag = 1
                            self.get_logger().info("Green object detected. Stopping...")
                            self.stop()
                            self.cancel_goal()
                        elif i == 2: # for the blue object we dont mind if it is not our goal
                            if self.bflag == 1: continue
                            # we must cancel the goal and center the blue box and go forward
                            self.bflag = 1
                            self.get_logger().info("Blue object detected. Stopping...")
                            self.stop()
                            self.cancel_goal()
                            self.get_logger().info("Centering blue object...")
                            x, y, w, h = cv2.boundingRect(c)
                            if self.center_object(x, w, image.shape[1]):
                                self.get_logger().info("Centered. Moving forward...")
                                self.move_forward()

        # Display
        cv2.namedWindow("Filtered Image", cv2.WINDOW_NORMAL)
        cv2.imshow("Filtered Image", filtered_img)
        cv2.resizeWindow("Filtered Image", 320, 240)
        cv2.namedWindow('Detection Camera Feed',cv2.WINDOW_NORMAL)
        cv2.imshow('Detection Camera Feed', image)
        cv2.resizeWindow('Detection Camera Feed',320,240)
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

    # Method to cancel the goal
    def cancel_goal(self):
        if self.current_goal_handle is not None:
            self.current_goal_handle.cancel_goal_async()
            self.get_logger().info('Goal cancelled')
            self.current_goal_handle = None
        else:
            self.get_logger().info('No goal to cancel')

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
        twist.linear.x = 0.2
        self.publisher.publish(twist)

    def goal_response_callback(self, future):
        self.current_goal_handle = future.result()
        if not self.current_goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            return

        self.get_logger().info('Goal accepted')
        self.get_result_future = self.current_goal_handle.get_result_async()
        self.get_result_future.add_done_callback(self.get_result_callback) # defined in main

    # from lab 4
    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            return

        self.get_logger().info('Goal accepted')
        self.get_result_future = goal_handle.get_result_async()
        self.get_result_future.add_done_callback(self.get_result_callback)



    def stop(self):
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


    # Gets the result of the navigation and goes to the next goal
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

    robot.goal_sequence = [robot.green_box, robot.red_box, robot.blue_box]
    robot.current_goal_index = 0
    robot.get_result_callback = new_result_callback
    robot.goto(robot.goal_sequence[robot.current_goal_index])

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
