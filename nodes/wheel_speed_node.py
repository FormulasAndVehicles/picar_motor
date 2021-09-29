#!/usr/bin/env python
import serial
import time
import struct
from picar_msgs.msg import WheelSpeedStamped
import rospy

F_CPU = 16000000.0
PRESCALER = 256.0 # needs to be the same as for the microcontroller code
BAUD = 250000 # needs to be the same for microcontroller and raspi

ticks_per_sec = F_CPU/PRESCALER
pulses_per_rev = 10


class Node(object):
    def __init__(self):
        rospy.init_node("wheel_speed_node")
        connected = False
        while not connected:
            try:
                self.serial = serial.Serial("/dev/ttyUSB0", BAUD, timeout=1.0)
            except serial.SerialException as error:
                rospy.logwarn("[{}] {}".format(rospy.get_name(), error))
                time.sleep(1)
            else:
                rospy.loginfo("[{}] Serial connection succesfull!".format(
                    rospy.get_name()))
                connected = True
        time.sleep(2.0)
        self.publisher = rospy.Publisher(
            "~wheel_speed",
            WheelSpeedStamped,
            queue_size=1)





if __name__ == "__main__":
    n = Node()
    index = 0
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        # write arbitrary byte
        try:
            n.serial.write(b"\x01")
            data = n.serial.read(8)
        except Exception as error:
            rospy.logerr("[{}] {}".format(
                rospy.get_name(),
                error))
        else:
            data = struct.unpack('>HHHH', data)
            speed = []
            for ticks in data:
                speed.append(ticks_per_sec/ticks/pulses_per_rev*2*3.1416 if ticks > 0 else 0.0)
            message = WheelSpeedStamped()
            message.header.stamp = rospy.Time.now()
            message.front_left = speed[0]
            message.front_right = speed[1]
            message.rear_left = speed[2]
            message.rear_right = speed[3]
            n.publisher.publish(message)
        finally:
            rate.sleep()
