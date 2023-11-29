#!/usr/bin/env python

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from rclpy.qos import QoSProfile
import math
from rclpy.qos import qos_profile_sensor_data
from statistics import mean

class MazeSolver(Node):
    def __init__(self):
        super().__init__('maze_solver')
        #self.velocity_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        #self.laser_subscriber = self.create_subscription(LaserScan, '/scan', self.laser_callback, 10)

        qos = QoSProfile(depth=10)

        self.velocity_publisher = self.create_publisher(Twist, 'cmd_vel', qos)
        self.laser_subscriber = self.create_subscription(
            LaserScan, 'scan', self.laser_callback,
            qos_profile=qos_profile_sensor_data
        )
        self.robot_speed = 0.5  # Adjust as needed
        self.wall_following = True
        self.half_square = -1

        self.done = False
        self.get_logger().info('MazeSolver node initialized')


    def laser_callback(self, data):
        if self.done:
            return
        # Turtlebot comeca no meio do labirinto, configura a distancia para uma parede na primeira iteracao do laser
        if self.half_square == -1:
            self.half_square = min(data.ranges)
            self.robot_speed = self.half_square

        # Check left, front, and right distances from the laser scan data
        front_distance = min([*data.ranges[0:20], *data.ranges[-20:]])
        #front_left_distance = min(data.ranges[35:45])
        left_distance = mean(data.ranges[60:90])
        #front_right_distcance = min(data.ranges[305:325])
        right_distance = mean(data.ranges[270:290])


        #ACABOOOOOOOOOOOOOOO
        if min(data.ranges) >= self.half_square * 8.0:
            print("SAIU :)")
            self.done = True
            self.move_robot(0.0, 2.0)
            return

        # Se estiver MUITO perto da parede sai de perto primeiro
        if min(data.ranges) < self.half_square / 4:
            self.move_robot(0.0, -math.radians(data.ranges.index(min(data.ranges))))
            self.move_robot(self.robot_speed, 0.0)
        # Adjust the robot's behavior based on the distances
        elif front_distance < self.half_square:
            if abs(left_distance - right_distance) < self.half_square/20:
                self.move_robot(0.0, -self.half_square * 1.9)
            elif left_distance < right_distance:
                self.move_robot(0.0, -self.half_square)
            else:
                self.move_robot(0.0, self.half_square)
        else:
            # Follow the left wall
            self.move_robot(self.robot_speed, 0.0)

    def move_robot(self, linear_speed=0.2, angular_speed=0.0):
        # print(f'MOVE L={linear_speed} A={angular_speed}')
        twist = Twist()
        twist.linear.x = float(linear_speed)
        twist.angular.z = float(angular_speed)
        self.velocity_publisher.publish(twist)
        self.get_logger().info('Robot movement command: Linear={}, Angular={}'.format(linear_speed, angular_speed))

    def solve_maze(self):
        rate = self.create_rate(10)
        self.get_logger().info('Start solving maze')
        #while rclpy.ok():
            # Continue solving the maze until the node is stopped
            #rate.sleep()

        # self.get_logger().info('Maze solving stopped')

def main(args=None):
    rclpy.init(args=args)
    maze_solver = MazeSolver()
    # maze_solver.solve_maze()
    rclpy.spin(maze_solver)
    maze_solver.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
