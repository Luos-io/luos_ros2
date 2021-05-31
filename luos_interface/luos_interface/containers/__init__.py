from .state import LuosStatePublisher
from .color import LuosColorPublisher
from .imu import LuosImuPublisher
from .angle import LuosAnglePublisher
from .controlled_motor import LuosControlledMotorPublisher
from .dc_motor import LuosDcMotorPublisher
from .angle import LuosAnglePublisher
from .distance import LuosDistancePublisher
from .light import LuosLightPublisher
from .servo_motor import LuosServoMotorPublisher
from .voltage import LuosVoltagePublisher
from .dxl import LuosDxlMotorPublisher
from .stepper import LuosStepperMotorPublisher

_make = {
    'State': LuosStatePublisher,
    'Color': LuosColorPublisher,
    'Imu': LuosImuPublisher,
    'Angle': LuosAnglePublisher,
    'ControlledMotor': LuosControlledMotorPublisher,
    'DCMotor': LuosDcMotorPublisher,
    'Angle': LuosAnglePublisher,
    'Distance': LuosDistancePublisher,
    'LightSensor': LuosLightPublisher,
    'Servo': LuosServoMotorPublisher,
    'Voltage': LuosVoltagePublisher,
    'DynamixelMotor': LuosDxlMotorPublisher,
    'Stepper': LuosStepperMotorPublisher,
}

def make_container_interface_factory(node, container, rate):
    if container.type in _make:
        return _make[container.type](node, container, rate)
    elif container.type == 'Void':
        node.get_logger().warn("A DynamixelMotor container has been found but no motor has been connected. Please connect a motor first.")
    elif container.type != 'Gate':
        # Gate has no publisher or subscriber
        node.get_logger().warn("Luos container type '{}' is unknown to luos_interface and will be ignored".format(container.type))
