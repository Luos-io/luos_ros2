from std_msgs.msg import Float32
from .serializers import serializeFloat32
from .deserializers import deserializeFloat32
from .generic import LuosGenericPublisher

class LuosDcMotorPublisher(LuosGenericPublisher):
    def __init__(self, node, container, rate):
        variables = {
            "power_ratio": {
                      "read_type": Float32, "write_type": Float32,
                      "serialize": serializeFloat32, "deserialize": deserializeFloat32,
                      },
        }
        events = {}
        aggregates = {}
        super(LuosDcMotorPublisher, self).__init__(node, container, rate, variables, events, aggregates)
