import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

class MoveCloserToWall(Node):

    def __init__(self):
        super().__init__('move_closer_to_wall')
        self.subscription = self.create_subscription(
            LaserScan,
            'scan',
            self.scan_callback,
            1)
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 1)
        self.red_wall = False
        self.current_speed = 0.0  # Hinzufügen einer Variablen für die aktuelle Geschwindigkeit
        self.max_speed = 0.5      # Maximale Geschwindigkeit
        self.acceleration = 0.035  # Beschleunigungsrate
        self.range_readings = []  # Eine Liste zur Speicherung der letzten Messungen

    def scan_callback(self, msg):
        ranges = msg.ranges
        if len(ranges) > 0:
            reduced_noise = self.reduce_noise(ranges)
            if reduced_noise < 0.17: # Schwellenwert zum Stoppen
                self.red_wall = True
                self.stop_moving()
            else:
                self.adjust_speed(reduced_noise)
                
    def reduce_noise(self, ranges):
        self.range_readings.append(min(ranges))
        if len(self.range_readings) > 3:  # Behalte nur die letzten X Messungen
            self.range_readings.pop(0)
        average_range = sum(self.range_readings) / len(self.range_readings)
        return average_range
                
    def adjust_speed(self, distance):
        if distance < 1.0:
            # Verringert die Geschwindigkeit basierend auf dem Abstand
            speed = distance / 2  #Halbieren der Geschwindigkeit bei 1.1 Meter
        else:
            if self.current_speed < self.max_speed:
                self.current_speed += self.acceleration
            
        self.current_speed = min(speed, self.max_speed)
        self.move_closer_to_wall()
            
    def move_closer_to_wall(self):
        twist = Twist()
        twist.linear.x = self.current_speed
        twist.angular.z = 0.0
        self.publisher_.publish(twist)
        self.get_logger().info(f'Moving Turtlebot3 closer to wall with speed: {self.current_speed}')



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

    rclpy.spin(move_closer_to_wall)

    move_closer_to_wall.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()




