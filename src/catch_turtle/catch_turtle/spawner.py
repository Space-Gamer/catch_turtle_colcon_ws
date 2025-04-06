#!/usr/bin/env python3
import random
import functools

import rclpy
from rclpy.node import Node

from catch_turtle_interfaces.msg import AliveTurtles, TurtleInfo
from turtlesim.srv import Spawn


class TurtleSpawner(Node):
    def __init__(self):
        super().__init__("turtle_spawner")
        self.alive_turtles_ = []
        self.turtle_count_ = 0
        self.turtle_prefix_ = "turtle"

        spawn_timer_ = self.create_timer(1.0, self.spawn_turtle)
        publisher_timer_ = self.create_timer(1.0, self.publish_turtles)

        self.spawn_client_ = self.create_client(Spawn, "/spawn")
        self.alive_turtles_publisher_ = self.create_publisher(AliveTurtles, "/alive_turtles", 10)

    def spawn_turtle(self):

        while not self.spawn_client_.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Waiting for spawn service...")

        turtle_name = f"{self.turtle_prefix_}_{self.turtle_count_}"
        self.get_logger().info("Spawning turtle: " + turtle_name)
        x,y = random.random() * 11.0, random.random() * 11.0

        request = Spawn.Request()
        request.x = x
        request.y = y
        request.theta = 0.0
        request.name = turtle_name

        future = self.spawn_client_.call_async(request)
        future.add_done_callback(
            functools.partial(self.spawn_turtle_callback, x=x, y=y)
        )

    def spawn_turtle_callback(self, future, x, y):
            if future.result() is not None:
                self.get_logger().info(f"Turtle spawned: {future.result().name}")
                self.alive_turtles_.append((future.result().name, x, y))
                self.turtle_count_ += 1
                self.publish_turtles()
            else:
                self.get_logger().error("Failed to spawn turtle: " + str(future.exception()))

    def publish_turtles(self):
        msg = AliveTurtles()
        for turtle in self.alive_turtles_:
            turtle_info = TurtleInfo()
            turtle_info.name = turtle[0]
            turtle_info.x = turtle[1]
            turtle_info.y = turtle[2]
            msg.alive_turtles.append(turtle_info)
        self.alive_turtles_publisher_.publish(msg)
        


def main(args=None):
    rclpy.init(args=args)
    node = TurtleSpawner()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()