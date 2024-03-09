from enum import Enum, auto
from math import pi

import rclpy  # ROS client library
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

import message_filters
from sensor_msgs.msg import LaserScan, Image
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

from transforms3d.euler import quat2euler
from Flowmanager import States

from Flowmanager import Flowmanager, States



class Colors(Enum):
    WHITE = auto()
    WOOD = auto()
    RED = auto()


class Tb3(Node):
    def __init__(self):
        super().__init__('tb3')

        self.cmd_vel_pub = self.create_publisher(
                Twist,      # message type
                'cmd_vel',  # topic name
                qos_profile_sensor_data)          # history depth

        self.scan_subs = message_filters.Subscriber(self, LaserScan, 'scan')
        self.odom_subs = message_filters.Subscriber(self, Odometry, 'odom')
        self.image_subs = message_filters.Subscriber(self, Image, 'camera/image_raw')

        ts = message_filters.ApproximateTimeSynchronizer([self.scan_subs, self.odom_subs, self.image_subs], 10, 0.75)
        ts.registerCallback(self.callback)

        self.ang_vel_percent = 0
        self.lin_vel_percent = 0
        self.flowmanager = Flowmanager()
        self.rotation_goal = 0
        self.drive_coord = []
        self.state = States.INIT


    def vel(self, lin_vel_percent, ang_vel_percent=0):
        """ publishes linear and angular velocities in percent
        """
        # for TB3 Waffle
        MAX_LIN_VEL = 0.26  # m/s
        MAX_ANG_VEL = 1.82  # rad/s

        cmd_vel_msg = Twist()
        cmd_vel_msg.linear.x = MAX_LIN_VEL * lin_vel_percent / 100
        cmd_vel_msg.angular.z = MAX_ANG_VEL * ang_vel_percent / 100

        self.cmd_vel_pub.publish(cmd_vel_msg)
        self.ang_vel_percent = ang_vel_percent
        self.lin_vel_percent = lin_vel_percent


    def callback(self, scan, odom, image):

        if self.state == States.INIT:
            self.state = States.DRIVE_ONE_HALF
        elif self.state == States.ROTATE_RIGHT:
            rotate_val = self.calc_rotation(odom, -90)
            self.vel(0, rotate_val)
            if not rotate_val:
                self.state = States.DRIVE_ONE
        elif self.state == States.ROTATE_LEFT:
            rotate_val = self.calc_rotation(odom, 90)
            self.vel(0, rotate_val)
            if not rotate_val:
                self.state = States.DRIVE_ONE
        elif self.state == States.DRIVE_TILL_WALL:
            speed_val = self.calc_distance(scan)
            self.vel(speed_val)
            if not speed_val:
                self.state = States.STOP
        elif self.state == States.DRIVE_ONE_HALF:
            speed_val = self.calc_speed(odom, 0.35)
            self.vel(speed_val)
            if not speed_val:
                self.state = States.ROTATE_LEFT
        elif self.state == States.DRIVE_ONE:
            speed_val = self.calc_speed(odom, 0.4)
            self.vel(speed_val)
            if not speed_val:
                self.state = States.STOP

        print(self.get_colors(image))


    def get_colors(self, image):
        """
        :param image: Takes the whole image from the camera
        :return: Returns array with three elements from the Color Enum
        """

        data = image.data
        left = []
        mid = []
        right = []

        for i in range(480):
            left.append(self.decode_col(data[i * 1920 + 0:i * 1920 + 3]))
            mid.append(self.decode_col(data[i * 1920 + 960:i * 1920 + 963]))
            right.append(self.decode_col(data[i * 1920 + 1917:i * 1920 + 1920]))

        left = self.color_filter(left)
        mid = self.color_filter(mid)
        right = self.color_filter(right)

        return left, mid, right

    def color_filter(self, col):
        s = set(col)
        if Colors.RED in s:
            return Colors.RED
        elif Colors.WOOD in s:
            return Colors.WOOD
        elif Colors.WHITE in s:
            return Colors.WHITE
        else:
            return None

    def decode_col(self, col):
        if abs(max(col)-min(col)) < 5:
            return Colors.WHITE
        elif col[2] < col[1]/1.25:
            print(col)
            return Colors.WOOD
        elif col[0] > 2*(col[1] + col[2]):
            return Colors.RED
        else:
            print(col)


    def calc_distance(self, scan):
        remaining_dist = scan.ranges[0]

        if remaining_dist > 0.50:
            ret = 50
        elif remaining_dist > 0.40:
            ret = 10
        elif remaining_dist > 0.34:
            ret = 1
        else:
            ret = 0

        return ret

    def calc_speed(self, odom, distance):
        if not self.drive_coord:
            self.drive_coord = odom.pose.pose.position.x, odom.pose.pose.position.y
        x_init, y_init = self.drive_coord
        x_now = odom.pose.pose.position.x
        y_now = odom.pose.pose.position.y
        driven_already = ((abs(x_init- x_now))**2 + (abs(y_init - y_now)**2))**0.5
        print(driven_already)
        if distance < 0:
            # Driving backwards
            remaining_dist = abs(distance) - driven_already
        else:
            remaining_dist = distance - driven_already


        if remaining_dist > 0.15:
            ret = 50
        elif remaining_dist > 0.05:
            ret = 25
        elif remaining_dist > 0.02:
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
            input_in_rad = (angle / 360) * pi * 2 #Looks to me like it is not more precise in sim
            goal = current_angle[0] + input_in_rad
            print(goal, current_angle[0])

            if goal > pi:
                goal = (goal-pi*2)
            elif goal < -pi:
                goal = (goal + pi*2)
            self.rotation_goal = goal
            print(goal, current_angle[0])

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
        elif rot_dist > 0.0001:
            ret = 1
        else:
            print(goal, current_angle[0])
            ret = 0
            self.rotation_goal = 0

        if direction == "clock":
            ret = ret * -1
        return ret


def main(args=None):
    rclpy.init(args=args)

    tb3 = Tb3()
    print('waiting for messages...')

    try:
        rclpy.spin(tb3)  # Execute tb3 node
        # Blocks until the executor (spin) cannot work
    except KeyboardInterrupt:
        pass

    tb3.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
