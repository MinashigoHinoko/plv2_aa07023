from math import pi
from enum import Enum, auto
import rclpy
import signal
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from transforms3d.euler import quat2euler

class States(Enum):
    DRIVE = auto()
    ROTATE = auto()
    DriAgain = auto()
    STOP = auto()

class MoveCloserToWall(Node):

    def __init__(self):
        super().__init__('odomexample')
        #self.subscription = self.create_subscription(
        #    LaserScan,
        #    'scan',
        #    self.scan_callback,
        #    1)
        self.subscription_odom = self.create_subscription(
            Odometry,
            'odom',
            self.odom_callback,
            1)
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 1)
        self.red_wall = False
        self.MAX_LIN_VEL = 0.26 # m/s
        self.MAX_ANG_VEL = 1.82 # rad/s
        self.MIN_WALL_DISTANCE = 0.17
        self.MIN_WALL_DISTANCE_TURNING = 22
        self.lin_vel_percent = 0
        self.state = States.DRIVE
        self.drive_coord = []
        self.rotation_goal = 0

    def vel(self, lin_vel_percent, ang_vel_percent=0):
        """ publishes linear and angular velocities in percent
        """

        cmd_vel_msg = Twist()
        cmd_vel_msg.linear.x = self.MAX_LIN_VEL * lin_vel_percent / 100
        cmd_vel_msg.angular.z = self.MAX_ANG_VEL * ang_vel_percent / 100

        self.cmd_vel_pub.publish(cmd_vel_msg)
        self.ang_vel_percent = ang_vel_percent
        self.lin_vel_percent = lin_vel_percent

    def odom_callback(self, odom):
        if self.state == States.DRIVE:
            speed = self.calc_speed(odom, 0.15)
            self.vel(speed)
            if not speed:
                self.state = States.ROTATE
        if self.state == States.ROTATE:
            rotate = self.calc_rotation(odom, -90)
            self.vel(0, rotate)
            if not rotate:
                self.state = States.DriAgain
        if self.state == States.DriAgain:
            speed = self.calc_speed(odom, 0.15)
            self.vel(speed)
            if not speed:
                self.state = States.STOP

    def scan_callback(self, msg):
        ranges = msg.ranges
        if len(ranges) > 0:
            self.move_closer_to_wall(ranges[0])

    def calc_speed(self, odom, distance):
        if not self.drive_coord:
            self.drive_coord = odom.pose.pose.position.x, odom.pose.pose.position.y
        x_init, y_init = self.drive_coord
        x_now = odom.pose.pose.position.x
        y_now = odom.pose.pose.position.y
        driven_already = ((abs(x_init - x_now))**2 + (abs(y_init - y_now)**2))**0.5
        print(driven_already)
        if distance < 0:
            # Driving backwards
            remaining_dist = abs(distance) - driven_already
        else:
            remaining_dist = distance - driven_already

        if remaining_dist > 0.15:
            ret = 100
        elif remaining_dist > 0.05:
            ret = 75
        elif remaining_dist > 0.02:
            ret = 35
        elif remaining_dist > 0.001:
            ret = 10
        else:
            ret = 0
            self.drive_coord = []

        if distance < 0:
            ret = -ret
        return ret

    def calc_rotation(self, odom, angle):
        current_angle = odom.pose.pose.orientation
        current_angle = quat2euler([
                                    current_angle.x,
                                    current_angle.y,
                                    current_angle.z,
                                    current_angle.w
                                    ])

        if self.rotation_goal:
            goal = self.rotation_goal
        else:
            rad = (angle / 360) * pi * 2
            goal = current_angle[0] + rad
            print(goal, current_angle[0])

            if goal > pi:
                goal = (goal-pi*2)
            elif goal < -pi:
                goal = (goal + pi*2)
            self.rotation_goal = goal

        if goal < 0 and current_angle[0] < 0:
            if current_angle[0] < goal:
                rot_dist = abs(goal - current_angle[0])
                direction = "counter"
            else:
                rot_dist = abs(current_angle[0] - goal)
                direction = "clock"
        elif goal > 0 and current_angle[0] > 0:
            if current_angle[0] < goal:
                rot_dist = goal - current_angle[0]
                direction = "counter"
            else:
                rot_dist = current_angle[0] - goal
                direction = "clock"
        else:
            clockwise_dist = abs(goal) + abs(current_angle[0])
            if goal > 0:
                clockwise_dist = 2*pi - clockwise_dist
            anticlockwise_dist = pi * 2 - clockwise_dist

            if clockwise_dist < anticlockwise_dist:
                rot_dist = clockwise_dist
                direction = "clock"
            else:
                rot_dist = anticlockwise_dist
                direction = "counter"

        ret = 0
        if rot_dist > 0.5:
            ret = 100
        elif rot_dist > 0.25:
            ret = 40
        elif rot_dist > 0.05:
            ret = 3
        elif rot_dist > 0.0003:
            ret = 1
        else:
            print(goal, current_angle[0])
            ret = 0
            self.rotation_goal = 0

        if direction == "clock":
            ret = ret * -1
        return ret

    def move_closer_to_wall(self, distance):
        speed = self.speedcalc(distance)
        self.vel(speed, 0)

    def speedcalc(self, distance):
        left_distance = distance - self.MIN_WALL_DISTANCE
        if left_distance <= self.MAX_LIN_VEL and left_distance >= 0:
            speed = (100 / (self.MAX_LIN_VEL)**2)*left_distance**2
        elif left_distance < 0:
            speed = -(100 / (self.MAX_LIN_VEL) ** 2) * left_distance ** 2
        elif self.lin_vel_percent < 100:
            if self.lin_vel_percent <= 90:
                speed = self.lin_vel_percent + 10
        return speed

def main(args=None):
    rclpy.init(args=args)

    move_closer_to_wall = MoveCloserToWall()

    def stop_robot(sig, frame):
        move_closer_to_wall.vel(0, 0)
        move_closer_to_wall.destroy_node()
        rclpy.shutdown()

    signal.signal(signal.SIGINT, stop_robot)  # Stop on SIGINT

    rclpy.spin(move_closer_to_wall)

    move_closer_to_wall.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()




