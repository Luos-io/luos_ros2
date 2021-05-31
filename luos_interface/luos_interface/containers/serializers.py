"""
Serialization from Luos containers to ROS standard or custom message types and units
There are:
  * container_agnostic serializers, that only take the data in input
  * container-dependent serialiers, that only take the container in input
"""

from std_msgs.msg import Bool, Float32, UInt32, ColorRGBA
from geometry_msgs.msg import Vector3, Accel
from sensor_msgs.msg import Imu, MagneticField
from luos_msgs.msg import BoolChange, FloatChange
from rclpy.clock import Clock

DEG_TO_RAD=0.017453292519943295
_clock = Clock()

def serializeBool(data):
    return Bool(data=bool(data))

def serializeFloat32(data):
    return Float32(data=float('nan' if data is None else data))

def serializeFloat32DegToRad(data):
    return Float32(data=float('nan' if data is None else data*DEG_TO_RAD))

def serializeUInt32(data):
    return UInt32(data=int(data))

def serializeVector3(data):
    return(Vector3(x=float('nan' if data[0] is None else data[0]),
                   y=float('nan' if data[1] is None else data[1]),
                   z=float('nan' if data[2] is None else data[2])))

def serializeVector3MinMax(data):
    # Vector 3 is used to represent (min, max) couples with x=min, y=max, z=<whatever>
    return(Vector3(x=float('nan' if data[0] is None else data[0]),
                   y=float('nan' if data[1] is None else data[1]),
                   z=float('nan')))

"""
Variable serialization from Color Luos container to ColorRGBA ROS type
"""
def serializeColor(data):
    return ColorRGBA(
        r=float('nan' if data is None else data[0]),
        g=float('nan' if data is None else data[1]),
        b=float('nan' if data is None else data[2]),
        a=float('nan'),                
    )

"""
Aggregated serialization from Imu Luos container to MagneticField ROS type
"""
def serializeMagneticField(container):
    magn = MagneticField()
    magn.header.frame_id = container.alias
    magn.header.stamp = _clock.now().to_msg()
    if container.compass is not None:
        magn.magnetic_field.x = float(container.compass[0])
        magn.magnetic_field.y = float(container.compass[1])
        magn.magnetic_field.z = float(container.compass[2])
    return magn

"""
Aggregated serialization from Imu Luos container to Accel ROS type
"""
def serializeAccel(container):
    accel = Accel()
    if container.acceleration is not None:
        accel.linear.x = float(container.acceleration[0])
        accel.linear.y = float(container.acceleration[1])
        accel.linear.z = float(container.acceleration[2])
    if container.gyro is not None:
        accel.angular.x = float(container.gyro[0]*DEG_TO_RAD)
        accel.angular.y = float(container.gyro[1]*DEG_TO_RAD)
        accel.angular.z = float(container.gyro[2]*DEG_TO_RAD)
    return accel

"""
Aggregated serialization from Imu Luos container to Imu ROS type
"""
def serializeImu(container):
    imu = Imu()
    imu.header.frame_id = container.alias
    imu.header.stamp = _clock.now().to_msg()
    if container.linear_acceleration is not None:
        imu.linear_acceleration.x = float(container.linear_acceleration[0])
        imu.linear_acceleration.y = float(container.linear_acceleration[1])
        imu.linear_acceleration.z = float(container.linear_acceleration[2])
    if container.quaternion is not None:
        imu.orientation.x = float(container.quaternion[0])
        imu.orientation.y = float(container.quaternion[1])
        imu.orientation.z = float(container.quaternion[2])
        imu.orientation.w = float(container.quaternion[3])
    return imu

"""
Event serialization from State Luos container to State ROS type
"""
def serializeBoolChange(container, event):
    return BoolChange(
        old_value=event.old_value,
        new_value=event.new_value
    )

"""
Event serialization 
"""
def serializeFloatChange(container, event):
    return FloatChange(
        old_value=event.old_value,
        new_value=event.new_value
    )