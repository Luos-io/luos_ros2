"""
Deserialization from ROS standard or custom message types and units to Luos containers
There are:
  * container_agnostic deserializers
  * container-dependent deserialiers
"""

RAD_TO_DEG = 57.29577951308232

def deserializeBool(msg):
    return bool(msg.data)

def deserializeFloat32(msg):
    return float(msg.data)

def deserializeFloat32RadToDeg(msg):
    msg.data *= RAD_TO_DEG
    return deserializeFloat32(msg)

def deserializeUInt32(msg):
    return int(msg.data)

def deserializeVector3(msg):
    return [msg.x, msg.y, msg.z]

def deserializeVector3MinMax(msg):
    # Vector 3 is used to represent (min, max) couples with x=min, y=max, z=<whatever>
    return [msg.x, msg.y]

def deserializeColor(msg):
    return [msg.r, msg.g, msg.b]

