from luos_msgs.msg import FloatChange
from std_msgs.msg import Float32
from .serializers import serializeFloat32, serializeFloat32DegToRad
from .deserializers import deserializeFloat32, deserializeFloat32RadToDeg
from .generic import LuosGenericPublisher

class LuosServoMotorPublisher(LuosGenericPublisher):
    def __init__(self, node, container, rate):
        variables = {
            "rot_position": {"read_type": Float32, "write_type": Float32,
                      "serialize": serializeFloat32DegToRad, "deserialize": deserializeFloat32RadToDeg,
                      },
            "max_angle": {"read_type": Float32, "write_type": Float32,
                      "serialize": serializeFloat32DegToRad, "deserialize": deserializeFloat32RadToDeg,
                      },
            "min_pulse": {"read_type": Float32, "write_type": Float32,
                      "serialize": serializeFloat32, "deserialize": deserializeFloat32,
                      },
            "max_pulse": {"read_type": Float32, "write_type": Float32,
                      "serialize": serializeFloat32, "deserialize": deserializeFloat32,
                      },
        }
        events = {}
        aggregates = {}
        super(LuosServoMotorPublisher, self).__init__(node, container, rate, variables, events, aggregates)
