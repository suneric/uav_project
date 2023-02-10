#!/usr/bin/env python
import sys
sys.path.append('.')
sys.path.append('..')
import rospy
import argparse
import numpy as np
from camera import RSD435
from ugv_driver import MobileDriver

np.random.seed(111)

def out_search(ugvPose,len=50):
    x, y = ugvPose[0], ugvPose[1]
    if x > len or x < -len or y > len or y < -len:
        return True
    else:
        return False

def random_moving(driver, base):
    rate = rospy.Rate(0.5)
    status, yaw = 'forward', None
    while not rospy.is_shutdown():
        try:
            ugvPose = driver.pose()
            # print("ugv status {}, position ({:.3f}, {:.3f}), yaw {:.3f}".format(status,ugvPose[0],ugvPose[1],ugvPose[2]))
            if out_search(ugvPose):
                if status == 'forward':
                    status = 'turning'
                    yaw = ugvPose[2] + np.pi
                    if yaw > np.pi:
                        yaw -= 2*np.pi
            else:
                if status == 'turning':
                    status = 'forward'

            if status == 'turning':
                status = "turn"
                rate1 = rospy.Rate(10)
                while abs(driver.pose()[2]-yaw) > 0.1:
                    driver.drive(0.0,base[1])
                # print("turned")
                while out_search(driver.pose()):
                    driver.drive(base[0],0.0)
                driver.drive(base[0],0.0)
                rospy.sleep(5)
                status = 'forward'
            else:
                rad = np.random.uniform(size=2)
                vx = base[0]+base[0]*(rad[0]-0.5)
                vz = base[1]*(rad[1]-0.5)
                # print("ugv random velocity ({:.3f}, {:.3f})".format(vx,vz))
                driver.drive(vx,vz)

        except rospy.ROSInterruptException:
            pass
        rate.sleep()

if __name__ == "__main__":
    print("start random route for ugv")
    rospy.init_node("ugv_route", anonymous=True)
    rospy.sleep(2) # waiting for robot to spawn
    # cam1 = RSD435('uav1')
    # cam2 = RSD435('uav2')
    base = (20,4*np.pi)
    driver = MobileDriver('ugv1')
    random_moving(driver, base)
