import math
import signal

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import time
from rclpy.qos import qos_profile_sensor_data


class MoveCloserToWall(Node):

    def __init__(self):
        super().__init__('move_closer_to_wall')
        self.subscription = self.create_subscription(
            LaserScan,
            'scan',
            self.scan_callback,qos_profile_sensor_data)
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 1)
        self.red_wall = False
        self.goal_max= None
        self.stop_count = 0
        self.has_turned = False # Flag, ob der Bot sich bereits gedreht hat
        self.hasnt_turned = True # Flag, ob der Bot sich noch nicht gedreht hat
        self.current_speed = 0.0  # Hinzufügen einer Variablen für die aktuelle Geschwindigkeit
        self.rotate_speed =0.0
        self.rotate_accel=0.02
        self.max_speed = 0.6      # Maximale Geschwindigkeit
        self.acceleration = 0.05# Beschleunigungsrate
        self.range_readings = []  # Eine Liste zur Speicherung der letzten Messungen

    def scan_callback(self, msg):
        ranges = msg.ranges
        ranges = [0 if x == float('inf') or math.isnan(x) else x for x in ranges]
        if len(ranges) > 0:
            reduced_noise = self.reduce_noise(ranges)
            print(reduced_noise)
            if reduced_noise < 0.17*2: # Schwellenwert zum Stoppen
                if self.red_wall == True:
                    if(self.hasnt_turned == True):
                        i = 0
                        readings = []
                        
                        while i<3:
                            readings.append((ranges.index(max(ranges)),max(ranges)))
                            if len(readings) > 3:  # Behalte nur die letzten X Messungen
                                readings.pop(0)
                            i+=1
                        max_reading = max(readings, key=lambda item: item[1])
                        y = max_reading[1]
                        x = max_reading[0]
                                                                        
                        self.goal_max = (x-90)%360
                        self.hasnt_turned = False
                    current_max = None
                    i=0
                    readings = []
                        
                    while i<3:
                        readings.append((ranges.index(max(ranges)),max(ranges)))
                        if len(readings) > 3:  # Behalte nur die letzten X Messungen
                            readings.pop(0)
                        i+=1
                    max_reading = max(readings, key=lambda item: item[1])
                    y = max_reading[1]
                    x = max_reading[0]
                    current_max = x % 360
                        
                    self.turn_in_Direction(current_max,self.goal_max)
                    
                    if (self.has_turned == True):
                        if ranges[0]<0.20:
                            self.stop_moving()
                        else:
                            self.adjust_speed(ranges[0])
                else:
                    self.red_wall = True
                    self.stop_moving()
            else:
                self.adjust_speed(reduced_noise)
    
    def reduce_noise(self, ranges):
        filtered_list = [element for element in ranges if element != 0]
        if filtered_list !=0:
            self.range_readings.append(min(filtered_list))
            if len(self.range_readings) > 3:  # Behalte nur die letzten X Messungen
                self.range_readings.pop(0)

        average_range = sum(self.range_readings) / len(self.range_readings)
        return average_range
    
    def reduce_noise_max(self, ranges):
        self.range_readings.append(max(ranges))
        if len(self.range_readings) > 3:  # Behalte nur die letzten X Messungen
            self.range_readings.pop(0)
        average_range = sum(self.range_readings) / len(self.range_readings)
        return average_range
    
    def adjust_speed(self, distance):
        if distance < 1.0:
            # Verringert die Geschwindigkeit basierend auf dem Abstand
            self.current_speed = distance / 2  #Halbieren der Geschwindigkeit
        else:
            # Erhöht die Geschwindigkeit bis zum Maximalwert, falls nötig
            if self.current_speed < self.max_speed:
                self.current_speed += self.acceleration
                # Stellt sicher, dass die Geschwindigkeit nicht über das Maximum hinausgeht
                self.current_speed = min(self.current_speed, self.max_speed)
        
        self.move_closer_to_wall()

        
    def move_closer_to_wall(self):
        twist = Twist()
        twist.linear.x = self.current_speed
        twist.angular.z = 0.0
        self.publisher_.publish(twist)
        self.get_logger().info(f'Moving Turtlebot3 closer to wall with speed: {self.current_speed}')
        
    def turn_in_Direction(self,current_max,goal_max):
        distance = abs(current_max - goal_max) %360
        if current_max == goal_max:
            self.get_logger().info("Größter Wert ist jetzt an der richtigen Stelle.")
            self.has_turned = True
            self.stop_moving()
        else:
            if not self.has_turned:
                self.get_logger().info(f"current max: {current_max}")
                self.get_logger().info(f"goal max: {goal_max}")
                # Entscheidungslogik für die Drehgeschwindigkeit
                if current_max > goal_max:  # stop_threshold muss definiert sein
                    if self.rotate_speed < self.max_speed:
                        self.rotate_speed += self.rotate_accel
                else:
                    # Verlangsamen
                    if distance < 30:
                        self.rotate_speed /2
                    else:
                        self.rotate_speed =0
                        self.has_turned = True
                
                # Stelle sicher, dass die Geschwindigkeit korrekt ist
                self.rotate_speed = min(max(self.rotate_speed, 0), self.max_speed)
                
                # Erstelle und sende Twist-Nachricht
                twist = Twist()
                twist.linear.x = 0.0
                twist.angular.z = float(self.rotate_speed)
                self.publisher_.publish(twist)
        
    def stop_moving(self):
        self.current_speed = 0.0  # Geschwindigkeit zurücksetzen
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.publisher_.publish(twist)
        self.get_logger().info('Stopping Turtlebot3')

def main(args=None):
    rclpy.init(args=args)
    move_closer_to_wall = MoveCloserToWall()
    
    def stop_robot(sig, frame):
        move_closer_to_wall.stop_moving()
        move_closer_to_wall.destroy_node()
        rclpy.shutdown()
    signal.signal(signal.SIGINT, stop_robot)  # Stop on SIGINT
    rclpy.spin(move_closer_to_wall)
    
    move_closer_to_wall.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()
