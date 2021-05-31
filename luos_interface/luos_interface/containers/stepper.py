from std_msgs.msg import Float32, Bool
from .serializers import serializeFloat32, serializeFloat32DegToRad, serializeBool
from .deserializers import deserializeFloat32, deserializeFloat32RadToDeg, deserializeBool
from .generic import LuosGenericPublisher

class LuosStepperMotorPublisher(LuosGenericPublisher):
    def __init__(self, node, container, rate):
        variables = {
            "compliant": {
                "read_type": Bool, "write_type": Bool,
                "serialize": serializeBool, "deserialize": deserializeBool,
            },
            # "rot_position_mode": {
            #     "read_type": Bool, "write_type": Bool,
            #     "serialize": serializeBool, "deserialize": deserializeBool,
            # },
            # "rot_speed_mode": {
            #     "read_type": Bool, "write_type": Bool,
            #     "serialize": serializeBool, "deserialize": deserializeBool,
            # },
            # "trans_position_mode": {
            #     "read_type": Bool, "write_type": Bool,
            #     "serialize": serializeBool, "deserialize": deserializeBool,
            # },
            # "trans_speed_mode": {
            #     "read_type": Bool, "write_type": Bool,
            #     "serialize": serializeBool, "deserialize": deserializeBool,
            # },
            "stepPerTurn": {
                "read_type": Float32, "write_type": Float32,
                "serialize": serializeFloat32, "deserialize": deserializeFloat32,
            },
            "wheel_size": {
                "read_type": Float32, "write_type": Float32,
                "serialize": serializeFloat32, "deserialize": deserializeFloat32,
            },
            "target_rot_position": {
                "read_type": Float32, "write_type": Float32,
                "serialize": serializeFloat32, "deserialize": deserializeFloat32,
            },
            "target_rot_speed": {
                "read_type": Float32, "write_type": Float32,
                "serialize": serializeFloat32, "deserialize": deserializeFloat32,
            },
            "target_trans_position": {
                "read_type": Float32, "write_type": Float32,
                "serialize": serializeFloat32, "deserialize": deserializeFloat32,
            },
            "target_trans_speed": {
                "read_type": Float32, "write_type": Float32,
                "serialize": serializeFloat32, "deserialize": deserializeFloat32,
            },
        }
        events = {}
        aggregates = {}
        super(LuosStepperMotorPublisher, self).__init__(node, container, rate, variables, events, aggregates)
