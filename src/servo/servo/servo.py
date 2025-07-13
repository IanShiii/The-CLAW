from gpiozero import Servo as GPIOServo
from gpiozero.pins.lgpio import LGPIOFactory
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64


class Servo(Node):
    MIN_ANGLE_DEGREES: float = 0.0
    MAX_ANGLE_DEGREES: float = 180.0

    factory = LGPIOFactory()

    def __init__(self):
        super().__init__('servo')
        self.declare_parameter('gpio_pin', 0)
        self.declare_parameter('id', 0)

        self.__gpio_pin: int = self.get_parameter('gpio_pin').get_parameter_value().integer_value
        self.__id: int = self.get_parameter('id').get_parameter_value().integer_value

        self.__gpio_servo = GPIOServo(self.__gpio_pin, pin_factory=Servo.factory)
        
        self.__target_angle_subcriber = self.create_subscription(Float64, ('servo_' + str(self.__id) + '_target_angle_degrees'), self.target_angle_callback, 10)

    def set_servo_angle_degrees(self, angle_degrees: float):
        angle_degrees = clamp(angle_degrees, Servo.MIN_ANGLE_DEGREES, Servo.MAX_ANGLE_DEGREES)
        position = map_value(angle_degrees, Servo.MIN_ANGLE_DEGREES, Servo.MAX_ANGLE_DEGREES, -1, 1)
        self.__gpio_servo.value = position
    
    def target_angle_callback(self, msg: Float64) -> None:
        self.set_servo_angle_degrees(msg.data)
    
    def destroy_node(self):
        self.get_logger().info('Stopping servo and detaching')
        self.__gpio_servo.detach()
        super().destroy_node()
    
def clamp(x, minimum, maximum):
    return max(minimum, min(x, maximum))

def map_value(value, in_min, in_max, out_min, out_max):
    return (value - in_min) / (in_max - in_min) * (out_max - out_min) + out_min

def main(args=None):
    rclpy.init(args=args)
    servo_node = Servo()
    rclpy.spin(servo_node)
    servo_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


    
    

