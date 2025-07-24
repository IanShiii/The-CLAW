from gpiozero import Servo as GPIOServo
from gpiozero.pins.lgpio import LGPIOFactory

def is_raspberry_pi():
    try:
        with open('/proc/cpuinfo', 'r') as f:
            cpuinfo = f.read()
        return 'BCM' in cpuinfo or 'Raspberry Pi' in cpuinfo
    except Exception:
        return False

class Servo():
    LGPIO_FACTORY = LGPIOFactory() if is_raspberry_pi() else None

    MIN_ANGLE_DEGREES = 0.0
    MAX_ANGLE_DEGREES = 180.0

    def __init__(self, name: str, gpio_pin: int) -> None:
        self.__servo = None if Servo.LGPIO_FACTORY == None else GPIOServo(pin = gpio_pin)
        self.set_angle_degrees(90)
        self.__name = name

    def set_angle_degrees(self, angle_degrees: float) -> None:
        if self.__servo != None:
            angle_degrees = clamp(angle_degrees, Servo.MIN_ANGLE_DEGREES, Servo.MAX_ANGLE_DEGREES)
            position = map(angle_degrees, Servo.MIN_ANGLE_DEGREES, Servo.MAX_ANGLE_DEGREES, -1, 1)
            self.__servo.value = position

    def get_name(self) -> str:
        return self.__name

    def destroy(self) -> None:
        if self.__servo != None:
            self.__servo.detach()

def clamp(x, minimum, maximum):
    return max(minimum, min(x, maximum))

def map(value, in_min, in_max, out_min, out_max):
    return (value - in_min) / (in_max - in_min) * (out_max - out_min) + out_min
