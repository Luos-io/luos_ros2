from std_msgs.msg import Float32, ColorRGBA
from .serializers import serializeColor, serializeFloat32
from .deserializers import deserializeColor, deserializeFloat32
from .generic import LuosGenericPublisher

class LuosColorPublisher(LuosGenericPublisher):
    def __init__(self, node, container, rate):
        variables = {
            "color": {"read_type": ColorRGBA, "write_type": ColorRGBA,
                      "serialize": serializeColor, "deserialize": deserializeColor,
                     },
            "time": {"read_type": Float32, "write_type": Float32,
                     "serialize": serializeFloat32, "deserialize": deserializeFloat32,
                    }
        }
        events = {}
        aggregates = {}
        super(LuosColorPublisher, self).__init__(node, container, rate, variables, events, aggregates)
