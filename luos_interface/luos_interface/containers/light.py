from luos_msgs.msg import FloatChange
from std_msgs.msg import Float32
from .serializers import serializeFloat32, serializeFloatChange
from .deserializers import deserializeFloat32
from .generic import LuosGenericPublisher

class LuosLightPublisher(LuosGenericPublisher):
    def __init__(self, node, container, rate):
        variables = {
            "lux": {
                "read_type": Float32,
                "serialize": serializeFloat32
                },
            "threshold": {
                "read_type": Float32, "write_type": Float32, 
                "serialize": serializeFloat32, "deserialize": deserializeFloat32,
                },
        }
        events = {
            "changed": {"type": FloatChange, "serialize": serializeFloatChange},
            "filter_changed": {"type": FloatChange, "serialize": serializeFloatChange},
        }
        aggregates = {}
        super(LuosLightPublisher, self).__init__(node, container, rate, variables, events, aggregates)
