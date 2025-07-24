from typing import List
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from servo.servo import Servo
from trajectory_msgs.msg import JointTrajectory
import math


SERVOS = [
    {'name': 'joint_1', 'gpio_pin': -1},
    {'name': 'joint_2', 'gpio_pin': -1},
    {'name': 'joint_3', 'gpio_pin': -1},
    {'name': 'joint_4', 'gpio_pin': -1},
    {'name': 'joint_5', 'gpio_pin': -1},
]

class ArmController(Node):

    def __init__(self):
        super().__init__('servo')
        self.__servos = [Servo(servo['name'], servo['gpio_pin']) for servo in SERVOS]
        self.__joint_state_publisher = self.create_publisher(JointState, '/joint_states', 10)
        self.__trajectory_listener = self.create_subscription(JointTrajectory, '/joint_trajectory', self.trajectory_callback, 10)
        self.__timer = self.create_timer(0.1, self.timer_callback)

    def timer_callback(self):
        self.publish_angles_to_joint_states([servo['name'] for servo in SERVOS], [math.radians(90)] * len(SERVOS))
    
    def trajectory_callback(self, msg: JointTrajectory) -> None:
        for point in msg.points:
            angles_radians: List[float] = point.positions
            servo_angle_degrees = [math.degrees(angle_radians) for angle_radians in angles_radians]
            for (servo, angle_degrees) in zip(self.__servos, servo_angle_degrees):
                servo.set_angle_degrees(angle_degrees)
            self.publish_angles_to_joint_states([servo['name'] for servo in SERVOS], angles_radians)

    def publish_angles_to_joint_states(self, joint_names: List[str], angles_radians: List[float]) -> None:
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = joint_names
        msg.position = angles_radians
        self.__joint_state_publisher.publish(msg)
    
    def destroy_node(self) -> None:
        self.get_logger().info('Stopping servo and detaching')
        for servo in self.__servos:
            servo.destroy()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    arm_controller = ArmController()
    rclpy.spin(arm_controller)
    arm_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


    
    

