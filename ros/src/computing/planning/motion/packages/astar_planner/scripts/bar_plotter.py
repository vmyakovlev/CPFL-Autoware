#!/usr/bin/env python

import numpy as np
import matplotlib.pyplot as plt
import rospy, cv_bridge, cv2
from sensor_msgs.msg import Image
from autoware_msgs.msg import lane

class BarPlotter(object):

    def __init__(self):
        self.sub = rospy.Subscriber("/final_waypoints", lane, self.callback)
        self.pub = rospy.Publisher("/velocity_bar", Image, queue_size=1)
        self.rate = rospy.Rate(10)
        self.bridge = cv_bridge.CvBridge()
        self.x, self.y = None, None
        self.fig, self.axis = plt.subplots()

    def callback(self, msg):
        x, y = [], []
        for idx, wp in enumerate(msg.waypoints):
            x.append(idx)
            y.append(wp.twist.twist.linear.x)  # velocity
        self.x = np.array(x)
        self.y = np.array(y) * 3.6  # mps -> kmph

    def spin(self):
        plt.ion()
        while not rospy.is_shutdown():
            if self.x is None or self.y is None:
                rospy.logdebug("waiting for callback ...")
                self.rate.sleep()
                continue
            self.axis.set_xlim(np.min(self.x), np.max(self.x))
            self.axis.set_ylim(0., 60.)
            self.axis.bar(self.x, self.y)
            self.fig.canvas.draw()
            self.fig.canvas.flush_events()
            img = np.fromstring(self.fig.canvas.tostring_rgb(), dtype=np.uint8, sep='')
            img = img.reshape(self.fig.canvas.get_width_height()[::-1] + (3,))
            img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
            self.pub.publish(self.bridge.cv2_to_imgmsg(img, "bgr8"))
            self.axis.clear()
            self.rate.sleep()

if __name__ == "__main__":
    rospy.init_node("bar_plotter", anonymous=True)
    bp = BarPlotter()
    bp.spin()
