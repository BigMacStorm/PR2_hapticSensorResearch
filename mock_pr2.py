#!/usr/bin/env python

import rospy
from std_msgs.msg import String

def MockPR2():
    pub = rospy.Publisher('PR2_data', String, queue_size=10)
    rospy.init_node('PR2', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        hello_str = "hello world %s" % rospy.get_time()
        rospy.loginfo(hello_str)
        pub.publish(hello_str)
        rate.sleep()

if __name__ == '__main__':
    try:
        MockPR2()
    except rospy.ROSInterruptException:
        pass
