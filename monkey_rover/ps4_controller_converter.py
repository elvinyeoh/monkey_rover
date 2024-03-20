import rclpy
import time
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist

class PS4ControllerConverter(Node):
    def __init__(self):
        super().__init__('ps4_controller_converter')
        self.subscription = self.create_subscription(Joy, 'joy', self.controller_callback, 10)
        # self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.publisher = self.create_publisher(Twist, '/diff_cont/cmd_vel_unstamped', 10)


        self.scale = 1
        self.prev_speed_button = 0
        self.prev_time = time.time()
        self.twist_msg = Twist()
        self.timer = self.create_timer(0.1, self.timer_callback)
        
    def controller_callback(self, msg):
        # cmd_vel control
        twist_msg = Twist()
    
        # linear_speed = msg.axes[1] * 0.5 * self.scale
        # angular_speed = msg.axes[3] * (0.5 + (0.1 * (self.scale - 1)))

        linear_speed = msg.axes[1] * 0.2 * self.scale
        # angular_speed = msg.axes[3] * (0.5 + (0.2 * (self.scale - 1)))
        angular_speed = msg.axes[3] * (0.5 + (0.1 * (self.scale - 1)))
    
        twist_msg.linear.x = linear_speed
        twist_msg.angular.z = angular_speed
        
        print(f"linear: {twist_msg.linear.x}, angular: {twist_msg.angular.z}")
        #print(twist_msg.angular.z)

        self.twist_msg = twist_msg
        #self.publisher.publish(twist_msg)

        # Speed control
        speed_button = msg.axes[7]
        if (speed_button != self.prev_speed_button):
            self.scale += speed_button
            if (self.scale > MAX_SCALE):
                self.scale = MAX_SCALE
            elif (self.scale < MIN_SCALE):
                self.scale = MIN_SCALE
        
        self.prev_speed_button = speed_button
        self.prev_time = time.time()

    def timer_callback(self):
        if (time.time() - self.prev_time) <= 1:
            self.publisher.publish(self.twist_msg)
        else:
            self.get_logger().warn("Disconnected")


    
def main(args=None):
    # Constants
    global MIN_SCALE
    global MAX_SCALE
    MIN_SCALE = 1
    MAX_SCALE = 5

    rclpy.init(args=args)
    ps4_controller_converter = PS4ControllerConverter()
    rclpy.spin(ps4_controller_converter)
    rclpy.shutdown()
    
    ps1 = PS4ControllerConverter()
    ps2 = PS4ControllerConverter()
    
    ps1.scale = 10
    
if __name__ == '__main__':
    main()
