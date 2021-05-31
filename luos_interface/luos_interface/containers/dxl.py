from std_msgs.msg import Float32, Bool
from geometry_msgs.msg import Vector3
from .serializers import serializeFloat32, serializeFloat32DegToRad, serializeBool, serializeVector3, serializeVector3MinMax
from .deserializers import deserializeFloat32, deserializeFloat32RadToDeg, deserializeBool, deserializeVector3, deserializeVector3MinMax
from .generic import LuosGenericPublisher

class LuosDxlMotorPublisher(LuosGenericPublisher):
    def __init__(self, node, container, rate):
        variables = {
            "compliant": {
                "read_type": Bool, "write_type": Bool,
                "serialize": serializeBool, "deserialize": deserializeBool,
            },
            "target_rot_position": {
                "read_type": Float32, "write_type": Float32,
                "serialize": serializeFloat32DegToRad, "deserialize": deserializeFloat32RadToDeg,
            },
            "target_rot_speed": {
                "read_type": Float32, "write_type": Float32,
                "serialize": serializeFloat32DegToRad, "deserialize": deserializeFloat32RadToDeg,
            },
            "wheel_mode": {
                "read_type": Bool, "write_type": Bool,
                "serialize": serializeBool, "deserialize": deserializeBool,
            },
            "rot_position": {
                "read_type": Float32, "write_type": Float32,
                "serialize": serializeFloat32DegToRad, "deserialize": deserializeFloat32RadToDeg,
            },
            "temperature": {
                "read_type": Float32, "write_type": Float32,
                "serialize": serializeFloat32, "deserialize": deserializeFloat32,
            },
            "positionPid": {
                "read_type": Vector3, "write_type": Vector3,
                "serialize": serializeVector3, "deserialize": deserializeVector3,
            },
            "power_ratio_limit": {
                "read_type": Float32, "write_type": Float32,
                "serialize": serializeFloat32, "deserialize": deserializeFloat32,
            },
            "rot_position_limit": {
                "read_type": Vector3, "write_type": Vector3,
                "serialize": serializeVector3MinMax, "deserialize": deserializeVector3MinMax,
            },
        }
        events = {}
        aggregates = {}
        super(LuosDxlMotorPublisher, self).__init__(node, container, rate, variables, events, aggregates)


