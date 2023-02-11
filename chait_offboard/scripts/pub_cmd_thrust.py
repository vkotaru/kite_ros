#!/usr/bin/env python
from concurrent.futures import thread
import rospy
from chait_msgs.msg import QCommandStamped
from geometry_msgs.msg import Vector3
from nav_msgs.msg import Odometry
import numpy as np

position = np.zeros(3)
velocity = np.zeros(3)
def callback(data):
    global position, velocity
    # print('position', position, 'velocity', velocity)
    position = np.array([data.pose.pose.position.x, data.pose.pose.position.y, data.pose.pose.position.z])
    velocity = np.array([data.twist.twist.linear.x, data.twist.twist.linear.y, data.twist.twist.linear.z])

def publish_thrust():
    global position, velocity
    pub  = rospy.Publisher('/falcon/command', Command, queue_size=10)
    rospy.Subscriber('/falcon/falcon/qrotor_plugin/odometry', Odometry, callback)
    rospy.init_node('command', anonymous=True)
    rospy.loginfo("Initialzing publish command rosnode")
    rate = rospy.Rate(100)  # 10hz

    setpoint = np.array([1., -1., 1.0])
    mass = 0.75
    gvec = np.array([0., 0., 9.81])
    kx = np.array([4,4,6])
    kv = np.array([2.5, 2.5, 3])

    while not rospy.is_shutdown():
        thurst = -np.multiply(position-setpoint, kx) - np.multiply(velocity, kv) + gvec*mass

        msg = Command()
        msg.mode = Command.MODE_THRUST_YAW
        t = Vector3()
        t.x, t.y, t.z = thurst[0],  thurst[1],  thurst[2]
        msg.command.append(t)
        msg.yaw.append(0)
        # print(thurst)
        pub.publish(msg)
        rate.sleep()


if __name__ == '__main__':
    try:
        publish_thrust()
    except rospy.ROSInterruptException:
        pass
