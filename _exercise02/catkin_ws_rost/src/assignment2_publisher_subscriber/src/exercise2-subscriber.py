#!/usr/bin/env python

import rospy
from autominy_msgs.msg import Speed


class BasicSubscriber:

    def __init__(self):
        rospy.init_node("basic_subscriber")
        self.speed_subscriber = rospy.Subscriber("/sensors/speed", Speed, self.on_speed, queue_size=10)

        rospy.spin()

    def on_speed(self, msg):
        print(msg.value)


if __name__ == "__main__":
    BasicSubscriber()
