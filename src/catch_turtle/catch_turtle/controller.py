import math
from functools import partial

import rclpy
from rclpy.node import Node

from catch_turtle_interfaces.msg import AliveTurtles, TurtleInfo
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from turtlesim.srv import Kill


class Controller(Node):
    def __init__(self):
        super().__init__("controller")
        self.curx = 0.0
        self.cury = 0.0
        self.curtheta = 0.0
        self.alive_turtles_ = []

        self.goal_turtle = None
        self.goal_x = None
        self.goal_y = None

        self.turtle_pose_subscriber_ = self.create_subscription(Pose, "turtle1/pose", self.turtle_pose_callback, 10)
        self.alive_turtles_subscriber_ = self.create_subscription(AliveTurtles, "alive_turtles", self.alive_turtles_callback, 10)
        self.cmd_vel_publisher_ = self.create_publisher(Twist, "turtle1/cmd_vel", 10)
        self.catch_turtle_client = self.create_client(Kill, "catch_turtle")
        
        self.go_to_goal_timer_ = self.create_timer(0.1, self.go_to_goal)
        self.set_goal_timer_ = self.create_timer(0.1, self.set_goal)

    def turtle_pose_callback(self, msg):
        self.curx = msg.x
        self.cury = msg.y
        self.curtheta = msg.theta

    def alive_turtles_callback(self, msg):
        self.alive_turtles_ = []
        for turtle in msg.alive_turtles:
            turtle_info = TurtleInfo()
            turtle_info.name = turtle.name
            turtle_info.x = turtle.x
            turtle_info.y = turtle.y
            self.alive_turtles_.append(turtle_info)

        self.get_logger().info("Alive turtles: " + str(self.alive_turtles_))

    def go_to_goal(self):
        if self.goal_x is None or self.goal_y is None:
            return

        distance = abs(math.sqrt(((self.goal_x - self.curx) ** 2) + ((self.goal_y - self.cury) ** 2)))

        if distance < 0.01:
            self.get_logger().info("Goal reached!")
            self.goal_x = None
            self.goal_y = None
            msg = Twist()
            self.cmd_vel_publisher_.publish(msg)  # Stop the robot
            while not self.catch_turtle_client.wait_for_service(timeout_sec=1.0):
                self.get_logger().info("Waiting for catch_turtle service...")
            request = Kill.Request()
            request.name = self.goal_turtle
            future = self.catch_turtle_client.call_async(request)
            future.add_done_callback(
                partial(self.catch_turtle_callback, turtle_name=self.goal_turtle)
            )
            return

        Beta = 0.9
        speed = distance * Beta

        Phi = 4.0
        ang_dist = math.atan2((self.goal_y - self.cury), (self.goal_x - self.curx))
        if ang_dist < 0:
            ang_dist += 2 * math.pi
        elif ang_dist > 2 * math.pi:
            ang_dist -= 2 * math.pi
        ang_speed = Phi * (ang_dist - self.curtheta)

        msg = Twist()
        msg.linear.x = speed
        msg.angular.z = ang_speed

        self.cmd_vel_publisher_.publish(msg)

    def catch_turtle_callback(self, future, turtle_name):
        if future.result() is not None:
            self.get_logger().info(f"Turtle caught: {turtle_name}")
            self.goal_turtle = None
            self.goal_x = None
            self.goal_y = None
        else:
            self.get_logger().error("Failed to catch turtle: " + str(future.exception()))

    def set_goal(self):
        if self.goal_turtle:
            return
        if not self.alive_turtles_:
            return
        self.goal_turtle = self.alive_turtles_[0].name
        self.goal_x = self.alive_turtles_[0].x
        self.goal_y = self.alive_turtles_[0].y
        self.get_logger().info("New goal set: " + str(self.goal_turtle))

def main(args=None):
    rclpy.init(args=args)
    node = Controller()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()