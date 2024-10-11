# turtle_teleop/turtle_teleop.py

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.srv import SetPen
import sys
import termios
import tty
import time

class TurtleTeleop(Node):
    def __init__(self):
        super().__init__('turtle_teleop')
        self.publisher_ = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.set_pen_client = self.create_client(SetPen, '/turtle1/set_pen')
        while not self.set_pen_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for set_pen service...')
        
        self.speed = 1.0  # Default speed
        self.setup_terminal()
        self.get_logger().info("Turtle Teleoperation started. Press keys to control.")

    def setup_terminal(self):
        self.old_settings = termios.tcgetattr(sys.stdin)
        tty.setraw(sys.stdin.fileno())

    def restore_terminal(self):
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.old_settings)

    def get_key(self):
        try:
            return sys.stdin.read(1)
        except KeyboardInterrupt:
            return None

    def set_color(self, r, g, b):
        req = SetPen.Request()
        req.r = r
        req.g = g
        req.b = b
        req.width = 2  # Pen width
        req.off = 0    # Pen down
        future = self.set_pen_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)

    def draw_circle(self, radius):
        twist = Twist()
        twist.linear.x = self.speed  # Set linear speed
        twist.angular.z = self.speed / radius  # Set angular speed based on radius
        duration = 2 * 3.14159 * radius / self.speed  # Calculate time to draw the circle

        # Start drawing the circle
        self.publisher_.publish(twist)
        time.sleep(duration)  # Draw for the calculated duration

        # Stop the turtle after drawing the circle
        self.publisher_.publish(Twist())  # Stop

    def draw_concentric_circles(self):
        for radius in [30, 40, 50]:  # Different radii for circles
            self.draw_circle(radius)

    def run(self):
        try:
            while True:
                key = self.get_key()
                if key == 'w':  # Forward
                    self.move_turtle(1.0)
                elif key == 's':  # Backward
                    self.move_turtle(-1.0)
                elif key == 'a':  # Turn left
                    self.turn_turtle(1.0)
                elif key == 'd':  # Turn right
                    self.turn_turtle(-1.0)
                elif key == '1':  # Speed 1
                    self.speed = 1.0
                elif key == '2':  # Speed 2
                    self.speed = 2.0
                elif key == '3':  # Speed 3
                    self.speed = 3.0
                elif key == 'r':  # Red color
                    self.set_color(255, 0, 0)
                elif key == 'g':  # Green color
                    self.set_color(0, 255, 0)
                elif key == 'b':  # Blue color
                    self.set_color(0, 0, 255)
                elif key == 'c':  # Draw concentric circles
                    self.draw_concentric_circles()
                elif key == 'q':  # Quit
                    break

        finally:
            self.restore_terminal()

    def move_turtle(self, direction):
        twist = Twist()
        twist.linear.x = self.speed * direction
        twist.angular.z = 0.0
        self.publisher_.publish(twist)

    def turn_turtle(self, direction):
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = self.speed * direction
        self.publisher_.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    turtle_teleop = TurtleTeleop()
    turtle_teleop.run()
    turtle_teleop.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()