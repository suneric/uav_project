#!/usr/bin/env python
import numpy as np
import rospy
from geometry_msgs.msg import Twist
from gazebo_msgs.msg import ModelStates
import tf.transformations as tft

class MobileDriver:
    def __init__(self, name):
        self.name = name
        self.topic = '/'+name+'/cmd_vel'
        self.velPub = rospy.Publisher(self.topic, Twist, queue_size=1)
        self.poseSub = rospy.Subscriber('/gazebo/model_states', ModelStates, self._pose_cb)
        self.ugvPose = None
        self.check_publisher_connection()
        self.check_subscriber_connection()

    def drive(self,vx,vyaw):
        msg = Twist()
        msg.linear.x=vx
        msg.linear.y=0
        msg.linear.z=0
        msg.angular.x=0
        msg.angular.y=0
        msg.angular.z=vyaw
        self.velPub.publish(msg)

    def stop(self):
        self.drive(0,0)

    def pose(self):
        gps = self.ugvPose
        x = gps.position.x
        y = gps.position.y
        q = gps.orientation
        e = tft.euler_from_quaternion([q.x,q.y,q.z,q.w])
        return (x,y,e[2])

    def _pose_cb(self,data):
        self.ugvPose = data.pose[data.name.index(self.name)]

    def check_publisher_connection(self):
        rate=rospy.Rate(10)
        while self.velPub.get_num_connections() == 0 and not rospy.is_shutdown():
            rospy.logdebug("no subscriber to "+self.topic+" yet, so wait and try again")
            try:
                rate.sleep()
            except rospy.ROSInterruptException:
                pass
        rospy.logdebug(self.topic+" Publisher connected")

    def check_subscriber_connection(self):
        rospy.logdebug("waiting for /gazebo/model_states to be ready.")
        data = None
        while data is None and not rospy.is_shutdown():
            try:
                data = rospy.wait_for_message("/gazebo/model_states", ModelStates, timeout=5.0)
                rospy.logdebug("current /gazebo/model_states is ready.")
            except:
                rospy.logdebug("current /gazebo/model_states is not ready, retrying.")
