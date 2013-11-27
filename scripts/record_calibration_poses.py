#!/usr/bin/env python

import rospy
import tf
import std_msgs.msg as std_msgs
import sensor_msgs.msg as sensor_msgs
import geometry_msgs.msg as geometry_msgs
import cv2
import numpy
from cv_bridge import CvBridge, CvBridgeError

class SampleRecorder(object):
    def __init__(self):

        self.listener = tf.TransformListener()

        self.image_sub = rospy.Subscriber('/camera/image_raw',sensor_msgs.Image, self.image_cb)
        self.sample_sub = rospy.Subscriber('/cameracalibrator/added_sample',std_msgs.Int16, self.sampled_cb)
        self.poses = []
        self.last_image = None
        self.poselog = open('poses.txt','w')

    def __del__(self):
        self.poselog.close()

    def image_cb(self, msg):
        self.last_image = msg

    def sampled_cb(self, msg):
        try:
            (trans,rot) = self.listener.lookupTransform('/board_link','/camera_link',rospy.Time(0))
        except:
            trans = [0,0,0]
            rot = [1,0,0,0]

        self.poses.append([trans,rot])
        rospy.loginfo("Recorded sample: " + str(trans) + " | " + str(rot) )

        if self.last_image:
            bridge_ = CvBridge()
            cvim = bridge_.imgmsg_to_cv(self.last_image, "rgb8")
            cvarr = numpy.asarray(cvim)

            cvarr_lap = cv2.Laplacian(cvarr, cv2.CV_8UC3, 7)
            cvarr_gray = cv2.cvtColor(cvarr_lap, cv2.COLOR_BGR2GRAY)
            cvarr_overlay = numpy.zeros((self.last_image.height,self.last_image.width,3), numpy.uint8)
            cvarr_overlay[:,:,1] = cvarr_gray;


            #cv2.imshow('%d.png' % msg.data, cvim)
            cv2.imwrite('%d.png' % (msg.data), cvarr_overlay)
            self.poselog.write("%d: { position: [%g, %g, %g], orientation: [%g, %g, %g, %g] }\n" % (msg.data, trans[0], trans[1], trans[2], rot[0], rot[1], rot[2], rot[3]))




def main():
    rospy.init_node('record_calibration_pose')

    recorder = SampleRecorder()

    rospy.spin()


    


if __name__ == '__main__':
    main()
