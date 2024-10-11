import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.srv import SetPen
import math
import time

class TurtleCircleDrawer(Node):
    def __init__(self):
        super().__init__('turtle_circle_drawer')
        self.publisher_ = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.set_pen_client = self.create_client(SetPen, '/turtle1/set_pen')

        while not self.set_pen_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for set_pen service...')

        # Set pen color and width
        self.set_pen_color(255, 0, 0, 2)  # Red color with width 2

        # Circle parameters
        self.radii = [0.5, 1.0, 1.5]  # Radii of the circles
        self.speed = 3.0               # Linear speed of the turtle
        self.current_circle = 0        # To keep track of which circle we're drawing
        self.set_orientation(2.0)

        # Start drawing the first circle
        self.draw_circle()

    def set_pen_color(self, r, g, b, width):
        req = SetPen.Request()
        req.r = r
        req.g = g
        req.b = b
        req.width = width
        req.off = 0  # Pen down
        future = self.set_pen_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)

    def draw_circle(self):
        radius = self.radii[self.current_circle]
        angular_velocity = self.speed / radius
        duration = (2 * math.pi * radius) / self.speed

        # Prepare to draw the circle
        twist = Twist()
        twist.linear.x = self.speed
        twist.angular.z = angular_velocity

        # Draw the circle
        for _ in range(int(duration / 0.1)):
            self.publisher_.publish(twist)  # Publish movement command
            time.sleep(0.1)  # Sleep for a short time

        # Stop the turtle after completing the circle
        self.publisher_.publish(Twist())  # Stop the turtle
        self.get_logger().info(f'Finished drawing circle {self.current_circle + 1}.')

        # Move to the next circle
        self.current_circle += 1
        if(self.current_circle == 1):

            self.set_pen_color(0, 255, 0, 2)  # Red color with width 2
        if(self.current_circle == 2):

            self.set_pen_color(0, 0, 255, 2)  # Red color with width 2
        if self.current_circle < len(self.radii):
            self.draw_circle()  # Draw the next circle
        else:
            self.get_logger().info('Finished drawing all circles.')
    
    def set_orientation(self, angle):
        twist = Twist()
        twist.angular.z = angle  # Set the angular velocity
        self.publisher_.publish(twist)  # Publish orientation command
        time.sleep(1)  # Wait for a second to reach the desired orientation
        self.publisher_.publish(Twist())  # Stop

def main(args=None):
    rclpy.init(args=args)
    turtle_circle_drawer = TurtleCircleDrawer()
    rclpy.spin(turtle_circle_drawer)  # Keep the node alive
    turtle_circle_drawer.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
