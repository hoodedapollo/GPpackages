#!/usr/bin/python

from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Quaternion
import bridge
import rospy
import signal
import math
from math import pi,e

class imu_bridge(bridge.bridge):

    def msg_process(self, msg):
        msg_list = msg.split(";")
        
        pkgSizeAcc = int(msg_list[0])
        moduleSizeAcc = 4*pkgSizeAcc+2
        acc_msg = msg_list[2:moduleSizeAcc]

        pkgSizeGyro = int(msg_list[moduleSizeAcc])
        moduleSizeGyro = 4*pkgSizeGyro+2
        gyro_msg = msg_list[moduleSizeAcc+2:moduleSizeAcc+moduleSizeGyro]

        pkgSizeOrient = int(msg_list[moduleSizeAcc+moduleSizeGyro])
        moduleSizeOrient = 4*pkgSizeOrient+2
        orient_msg = msg_list[moduleSizeAcc+moduleSizeGyro+2:moduleSizeAcc+moduleSizeGyro+moduleSizeOrient]

        biggerpkg = max([pkgSizeAcc, pkgSizeGyro, pkgSizeOrient])

        imu_list = []
        acceleration = Vector3()
        velocity = Vector3()
        orientation = Quaternion()

        for i in range(0,biggerpkg):
            time = ""
            if i < pkgSizeGyro:
                velocity = Vector3()
                velocity.x = float(gyro_msg[3*i].replace(',','.'))
                velocity.y = float(gyro_msg[3*i+1].replace(',','.'))
                velocity.z = float(gyro_msg[3*i+2].replace(',','.'))
                time = gyro_msg[3*pkgSizeGyro+i]

            if i < pkgSizeAcc:
                acceleration = Vector3()
                acceleration.x = float(acc_msg[3*i].replace(',','.'))
                acceleration.y = float(acc_msg[3*i+1].replace(',','.'))
                acceleration.z = float(acc_msg[3*i+2].replace(',','.'))
                time = acc_msg[3*pkgSizeAcc+i]

            if i < pkgSizeOrient:
                orientation = Quaternion()
                orientation.x = float(orient_msg[3*i].replace(',','.'))*10**-6
                orientation.y = float(orient_msg[3*i+1].replace(',','.'))*10**-6
                orientation.z = float(orient_msg[3*i+2].replace(',','.'))*10**-6
                orientation.w = 0
                time = orient_msg[3*pkgSizeOrient+i]

            imu_message = Imu()

            now = rospy.get_rostime()
            imu_message.header.stamp.secs = now.secs
            imu_message.header.stamp.nsecs = now.nsecs

            imu_message.header.frame_id = time
            imu_message.angular_velocity = velocity
            imu_message.linear_acceleration = acceleration
            imu_message.orientation = orientation
            imu_list.append(imu_message)
        return imu_list


def main():
    rospy.init_node('imu_bridge', anonymous=True)
    device = rospy.get_param('~device_name','sensors')
    mqtt_topic = device + '/imu'
    print mqtt_topic
    imu_publisher = rospy.Publisher('/imu_data', Imu, queue_size=1)
    imu_sub = imu_bridge(imu_publisher, mqtt_topic, 'bridge_imu_'+ device)
    rospy.on_shutdown(imu_sub.hook)

    while not rospy.is_shutdown():
        imu_sub.looping()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
