#!/usr/bin/env python

import rospy
import tf
import std_msgs.msg as std_msgs
import geometry_msgs.msg as geometry_msgs

class SampleRecorder(object):
    def __init__(self):

        self.listener = tf.TransformListener()

        self.sample_sub = rospy.Subscriber('/cameracalibrator/added_sample',std_msgs.Int32, self.sampled)

    def sampled(self, msg):
        try:
            (trans,rot) = self.listener.lookupTransform('/camera_link','/board_link')
            rospy.loginfo("Recorded sample: " + str(trans) + " | " + str(rot) )
        except:
            pass



def main():
    rospy.init_node('record_calibration_pose')



if __name__ == '__main__':
    main()
