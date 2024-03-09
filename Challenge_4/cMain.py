from math import pi
from enum import Enum, auto
import math
import utils as utils
import rclpy
import signal
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from transforms3d.euler import quat2euler
from math import atan2, sqrt
from rclpy.qos import qos_profile_sensor_data

class States(Enum):
    ORIENTATE = auto()
    DRIVE = auto()
    ROTATE = auto()
    FINALDRIVE = auto()
    STOP = auto()
class Statess(Enum):
    ROTATED_FORWARD = auto()
    ROTATED_RIGHT = auto()
    ROTATED_LEFT = auto()
    ROTATED_BACK = auto()

class MoveCloserToWall(Node):

    def __init__(self):
        super().__init__('odomexample')
        self.subscription = self.create_subscription(
            LaserScan,
            'scan',
            self.scan_callback,
            qos_profile_sensor_data)
        self.subscription_odom = self.create_subscription(
            Odometry,
            'odom',
            self.odom_callback,
            qos_profile_sensor_data)
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 1)
        self.is_innit=False
        self.transform = None
        self.odom_current = None
        self.red_wall = False
        self.MAX_LIN_VEL = 0.26 # m/s
        self.MAX_ANG_VEL = 1.82 # rad/s
        self.MIN_WALL_DISTANCE = 0.17
        self.MIN_WALL_DISTANCE_TURNING = 22
        self.x = 0
        self.y = 0
        self.y_turn_counter = 0
        self.x_turn_counter = 0
        self.was_x = False
        self.was_y = False
        self.y_didnt_work = False
        self.x_didnt_work = False
        self.is_Lost = False
        self.lin_vel_percent = 0
        self.state = States.STOP
        self.direction = Statess.ROTATED_FORWARD
        self.Offset_Yaw = 0
        self.yaw = False
        self.drive_coord = []
        self.rotation_goal = 0
        self.distance_wall =0
        self.ranges =[]
        self.collect_scans =[]

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
        print("We currently are at : ",odom.pose.pose.orientation)
        print("We currently are at x: ",odom.pose.pose.position.x)
        print("We currently are at y: ",odom.pose.pose.position.y)
        if not self.is_innit:
            self.is_innit = True
            self.x = odom.pose.pose.position.x+1
            self.y = odom.pose.pose.position.y+1
            self.Offset_Yaw = self.quaternion_to_yaw(odom.pose.pose.orientation.w,odom.pose.pose.orientation.x,odom.pose.pose.orientation.y,odom.pose.pose.orientation.z)
            self.Offset_Yaw = self.Offset_Yaw 
        self.yaw = self.quaternion_to_yaw(odom.pose.pose.orientation.w,odom.pose.pose.orientation.x,odom.pose.pose.orientation.y,odom.pose.pose.orientation.z)
        self.yaw = self.yaw - self.Offset_Yaw
        _,self.transform=utils.create_transformations_between_odom_start_and_odom((self.x,self.y),self.yaw)
        self.odom_current = self.transform((odom.pose.pose.position.x,odom.pose.pose.position.y))
        print("We currently are at x: ",self.x)
        print("We currently are at y: ",self.y)
        print("Momentaner Korrektur: ", self.Offset_Yaw* (180 / math.pi))
        print("Momentaner ausrichtung: ", self.yaw* (180 / math.pi))
        print("Drive To Goal: ",self.odom_current)
        self.state_manager(odom)

    def state_manager(self, odom):
        if (len(self.ranges) >= 5):
            print("State : ", self.state)
            print("Distance to Wall: ",self.ranges[0])
            if self.ranges[0] < 1.0 and self.state == States.STOP:
                self.state = States.ROTATE
            if self.ranges[0]>1.0 and self.state == States.STOP:
                while(True):
                    try:
                        if round(self.odom_current[1])<0:
                            print("----------Start-Driving----------")
                            self.state = States.DRIVE
                            break
                        if round(self.odom_current[0])<0 :
                            self.was_x = True
                            print("----------Start-Driving----------")
                            self.state = States.DRIVE
                            break
                        if round(self.odom_current[0]) > 0 or round(self.odom_current[1]) > 0:
                            print("----------Start-Orientate----------")
                            self.state = States.ROTATE
                            break
                        else:
                            print("----------Start-FALSE----------")
                            self.state = States.STOP
                            break
                    except:
                        self.vel(0, 0)
                        self.destroy_node()
                        rclpy.shutdown()
                        return
            if (self.ranges[0]<0.5 and self.state == States.STOP):
                    self.state = States.FINALDRIVE
                    self.distance_wall = self.ranges[0]*0.5
            
            if self.state == States.ORIENTATE:
                rotate = self.calc_rotation(odom, -90)
                self.vel(0, rotate)
                if not rotate:
                    self.state = States.DRIVE
            if self.state == States.DRIVE:
                speed = self.calc_speed(odom, 1)
                self.vel(speed)
                if not speed:
                    self.state = States.STOP
                    self.y_didnt_work = False
                    self.x_didnt_work = False
            if self.state == States.ROTATE:
                if self.was_x:
                    self.x_turn_counter += 1
                if self.was_y:
                    self.y_turn_counter += 1
                rotate = self.calc_rotation(odom, +90)
                self.vel(0, rotate)
                if not rotate:
                    self.state = States.STOP
            if self.state == States.FINALDRIVE:
                speed = self.calc_speed(odom, self.distance_wall)
                self.vel(speed)
                if not speed:
                    self.state = States.STOP
            
    def quaternion_to_yaw(self,w, x, y, z):
        # Berechne den Yaw-Winkel
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y**2 + z**2)
        yaw = atan2(siny_cosp, cosy_cosp)
        
        return yaw
    def scan_callback(self, msg):
        ranges = msg.ranges
        ranges = [0 if x == float('inf') or math.isnan(x) else x for x in ranges]
        # Füge den neuen Scan hinzu
        if len(self.collect_scans) < 5:
            self.collect_scans.append(ranges)
        else:
            self.collect_scans.pop(0)  # Entferne den ältesten Scan
            self.collect_scans.append(ranges)
        
        # Berechne den Durchschnitt, wenn wir 5 Scans haben
        if len(self.collect_scans) == 5:
            self.ranges = self.kombiniere_scans(self.collect_scans)

    def kombiniere_scans(self,scans):
        y =360
        for scan in scans:
            if len(scan)>y:
                y = len(scan)
        summen = [0.0] * y
        anzahl_scans = len(scans)
        
        for scan in scans:
            x = len(scan)
            for i in range(x):
                summen[i] += scan[i]
        
        durchschnittswerte = [summe / anzahl_scans for summe in summen]
        return durchschnittswerte

    def calc_speed(self, odom, distance):
        if not self.drive_coord:
            self.drive_coord = odom.pose.pose.position.x, odom.pose.pose.position.y
        x_init, y_init = self.drive_coord
        x_now = odom.pose.pose.position.x
        y_now = odom.pose.pose.position.y
        driven_already = ((abs(x_init - x_now))**2 + (abs(y_init - y_now)**2))**0.5
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
            ret = 25
        elif remaining_dist > 0.0001:
            ret = 5
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
            ret = 75
        elif rot_dist > 0.05:
            ret = 50
        elif rot_dist > 0.0003:
            ret = 2
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




