#!/usr/bin/env python3

import rospy
import tf
from tf import TransformListener, TransformBroadcaster

class TransformDuplicator:
    def __init__(self):
        rospy.init_node('transform_duplicator', anonymous=True)

        self.tf_listener = TransformListener()
        self.tf_broadcaster = TransformBroadcaster()

        self.source_frame = rospy.get_param('~source_frame', 'map')
        self.target_frame = rospy.get_param('~target_frame', 'base_link')
        self.new_parent_frame = rospy.get_param('~new_parent_frame', 'odom')
        self.new_child_frame = rospy.get_param('~new_child_frame', 'base_laser')

    def duplicate_transform(self):
        rate = rospy.Rate(10.0)  # 10 Hz
        while not rospy.is_shutdown():
            try:
                self.tf_listener.waitForTransform(self.source_frame, self.target_frame, rospy.Time(), rospy.Duration(1.0))
                (trans, rot) = self.tf_listener.lookupTransform(self.source_frame, self.target_frame, rospy.Time(0))

                current_time = rospy.Time.now()
                self.tf_broadcaster.sendTransform(
                    trans,
                    rot,
                    current_time,
                    self.new_child_frame,
                    self.new_parent_frame
                )
                rospy.loginfo(f"Duplicated transform from {self.source_frame}->{self.target_frame} to {self.new_parent_frame}->{self.new_child_frame}")

            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
                rospy.logwarn(f"Transform lookup failed: {e}")

            rate.sleep()

if __name__ == '__main__':
    try:
        duplicator = TransformDuplicator()
        duplicator.duplicate_transform()
    except rospy.ROSInterruptException:
        pass