from std_msgs.msg import UInt32, Float32, Bool
from geometry_msgs.msg import Vector3, Accel
from sensor_msgs.msg import Imu, MagneticField
from .serializers import serializeUInt32, serializeFloat32, serializeVector3
from .serializers import serializeAccel, serializeImu, serializeMagneticField
from .deserializers import deserializeBool
from .generic import LuosGenericPublisher

class LuosImuPublisher(LuosGenericPublisher):
    def __init__(self, node, container, rate):
        variables = {
            "pedometer": {"read_type": UInt32, "write_type": Bool,
                          "serialize": serializeUInt32, "deserialize": deserializeBool},
            "walk_time": {"read_type": Float32, "write_type": Bool,
                          "serialize": serializeFloat32, "deserialize": deserializeBool},
            "gravity_vector": {"read_type": Vector3, "write_type": Bool,
                               "serialize": serializeVector3, "deserialize": deserializeBool},
            # TODO: is heading a [Float, Float, Float]?
            "heading": {"read_type": UInt32, "write_type": Bool,
                        "serialize": serializeUInt32, "deserialize": deserializeBool},
        }
        events = {}
        aggregates = {
            "acceleration": {"type": Accel, "serialize": serializeAccel},
            "imu": {"type": Imu, "serialize": serializeImu},
            "compass": {"type": MagneticField, "serialize": serializeMagneticField},
        }
        super(LuosImuPublisher, self).__init__(node, container, rate, variables, events, aggregates)

"""
Aggregate 1
gyro 	X, Y, Z axis rotational acceleration data in degrees per second 	read only: [Float, Float, Float]
gyro 	Starts/Stops gyro measurement actualization 	write only: Boolean (True or False)
acceleration 	X, Y, Z axis linear acceleration data in G 	read only: [Float, Float, Float]
acceleration 	Starts/Stops acceleration measurement actualization 	write only: Boolean (True or False)

Aggregate 2
quaternion 	Sensor fused w, x, y, z rotational angles 	read only: [Float, Float, Float, Float]
quaternion 	Starts/Stops quaternion measurement actualization 	write only: Boolean (True or False)
linear_acceleration 	Linear acceleration in body frame coordinates 	read only: [Float, Float, Float]
linear_acceleration 	Starts/Stops linear_acceleration measurement actualization 	write only: Boolean (True or False)

# Variables
heading 	360 degrees from North with Y+ axis as the pointer 	read only: [Float, Float, Float]
heading 	Starts/Stops heading measurement actualization 	write only: Boolean (True or False)
compass 	Magnetic field data in micro-tesla on each axis 	read only: [Float, Float, Float]
compass 	Starts/Stops compass measurement actualization 	write only: Boolean (True or False)
gravity_vector 	Which access gravity effects 	read only: [Float, Float, Float]
gravity_vector 	Starts/Stops gravity_vector measurement actualization 	write only: Boolean (True or False)
pedometer 	Step number 	read only: int
pedometer 	Starts/Stops pedometer measurement actualization 	write only: Boolean (True or False)
walk_time 	Duration (second) of the walk 	read only: Float
walk_time 	Starts/Stops walk_time measurement actualization 	write only: Boolean (True or False)
"""