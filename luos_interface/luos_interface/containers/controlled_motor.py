from std_msgs.msg import Float32, Bool
from geometry_msgs.msg import Vector3
from .serializers import serializeVector3, serializeFloat32, serializeBool, serializeFloat32DegToRad
from .deserializers import deserializeVector3, deserializeFloat32, deserializeBool, deserializeFloat32RadToDeg
from .generic import LuosGenericPublisher


class LuosControlledMotorPublisher(LuosGenericPublisher):
    def __init__(self, node, container, rate):
        variables = {
            "positionPid": {
                "read_type": Vector3, "write_type": Vector3,
                "serialize": serializeVector3, "deserialize": deserializeVector3},
            "speedPid": {
                "read_type": Vector3, "write_type": Vector3,
                "serialize": serializeVector3, "deserialize": deserializeVector3},
            "encoder_res": {
                "read_type": Float32, "write_type": Float32,
                "serialize": serializeFloat32, "deserialize": deserializeFloat32},
            "reduction": {
                "read_type": Float32,  "write_type": Bool,
                "serialize": serializeFloat32, "deserialize": deserializeFloat32},
            "wheel_size": {
                "read_type": Float32, "write_type": Float32 ,
                "serialize": serializeFloat32, "deserialize": deserializeFloat32},
            "compliant": {
                "read_type": Bool,  "write_type": Bool,
                "serialize": serializeBool, "deserialize": deserializeBool},
            "power_mode": {
                "read_type": Bool,  "write_type": Bool,
                "serialize": serializeBool, "deserialize": deserializeBool},
            "rot_position_mode": {
                "read_type": Bool,  "write_type": Bool,
                "serialize": serializeBool, "deserialize": deserializeBool},
            "rot_speed_mode": {
                "read_type": Bool,  "write_type": Bool,
                "serialize": serializeBool, "deserialize": deserializeBool},
            "trans_position_mode": {
                "read_type": Bool,  "write_type": Bool,
                "serialize": serializeBool, "deserialize": deserializeBool},
            "trans_speed_mode": {
                "read_type": Bool,  "write_type": Bool,
                "serialize": serializeBool, "deserialize": deserializeBool},
            "rot_position": {
                "read_type": Float32,  "write_type": Bool,
                "serialize": serializeFloat32, "deserialize": deserializeBool},
            "rot_speed": {
                "read_type": Float32,  "write_type": Bool,
                "serialize": serializeFloat32, "deserialize": deserializeBool},
            "trans_position": {
                "read_type": Float32,  "write_type": Bool,
                "serialize": serializeFloat32, "deserialize": deserializeBool},
            "trans_speed": {
                "read_type": Float32,  "write_type": Bool,
                "serialize": serializeFloat32, "deserialize": deserializeBool},
            "current": {
                "read_type": Float32,  "write_type": Bool,
                "serialize": serializeFloat32, "deserialize": deserializeBool},
            "power_ratio": {
                "read_type": Float32,  "write_type": Float32,
                "serialize": serializeFloat32, "deserialize": deserializeFloat32},
            "target_rot_position": {
                "read_type": Float32, "write_type": Float32,
                "serialize": serializeFloat32DegToRad, "deserialize": deserializeFloat32RadToDeg},
            "target_rot_speed": {
                "read_type": Float32, "write_type": Float32,
                "serialize": serializeFloat32DegToRad, "deserialize": deserializeFloat32RadToDeg},
            "target_trans_position": {
                "read_type": Float32, "write_type": Float32,
                "serialize": serializeFloat32, "deserialize": deserializeFloat32},
            "target_trans_speed": {
                "read_type": Float32, "write_type": Float32,
                "serialize": serializeFloat32, "deserialize": deserializeFloat32},
        }
        events = {}
        aggregates = {}
        super(LuosControlledMotorPublisher, self).__init__(node, container, rate, variables, events, aggregates)
