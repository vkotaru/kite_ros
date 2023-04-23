#!/usr/bin/env python
import rospy
from kite_msgs.msg import QCommandStamped, QCommand
from geometry_msgs.msg import Vector3


def publish_cmds():
    pub = []
    pub.append(rospy.Publisher('/falcon/kite_quadrotor_plugin/command', QCommandStamped, queue_size=10))
    rospy.init_node('command', anonymous=True)
    rospy.loginfo("Initialzing publish command rosnode")
    rate = rospy.Rate(10)  # 10hz
    while not rospy.is_shutdown():
        msg = QCommandStamped()
        msg.command.mode = QCommand.MODE_POSITION
        position = Vector3()
        position.x, position.y, position.z = 1, 0, 2
        msg.command.values.append(position)
        pub[0].publish(msg)
        rate.sleep()


if __name__ == '__main__':
    try:
        publish_cmds()
    except rospy.ROSInterruptException:
        pass
