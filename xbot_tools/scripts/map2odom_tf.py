#!/usr/bin/env python
import rospy, math
import tf
from nav_msgs.msg import Odometry




def sendTF(msg):
  br = tf.TransformBroadcaster()
  br.sendTransform((msg.pose.pose.position.x, msg.pose.pose.position.y, 0),
                   tf.transformations.quaternion_from_euler(0, 0, msg.pose.pose.orientation.z),
                   rospy.Time.now(),
                   'odom',
                   "map")

if __name__ == '__main__':
  rospy.init_node('map2odom_tf_broadcaster')
  rospy.Subscriber('/odom', Odometry, sendTF)
  rospy.spin()
