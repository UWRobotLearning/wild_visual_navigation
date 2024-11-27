#!/usr/bin/env python
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Vector3Stamped
import tf
import tf2_ros
import tf2_geometry_msgs

def twist2vec3(twist):
    vector_linear = Vector3Stamped()
    vector_linear.vector.x = twist.linear.x
    vector_linear.vector.y = twist.linear.y
    vector_linear.vector.z = twist.linear.z
    
    vector_angular = Vector3Stamped()
    vector_angular.vector.x = twist.angular.x
    vector_angular.vector.y = twist.angular.y
    vector_angular.vector.z = twist.angular.z
    return vector_linear, vector_angular

def vec32twist(vlinear, vangular):
    twist = Twist()
    twist.linear.x = vlinear.vector.x
    twist.linear.y = vlinear.vector.y
    twist.linear.z = vlinear.vector.z

    twist.angular.x = vangular.vector.x
    twist.angular.y = vangular.vector.y
    twist.angular.z = vangular.vector.z
    return twist

def zero_trans(transform):
    transform.transform.translation.x = 0
    transform.transform.translation.y = 0
    transform.transform.translation.z = 0
    return transform

def odom_callback(msg, tf_buffer):
    
    try:
        # Get the transform from base_link to vision
        trans_base_to_vision = zero_trans(tf_buffer.lookup_transform('base_link', 'vision', rospy.Time(0)))
        # Get the transform from odom to vision
        trans_odom_to_vision = zero_trans(tf_buffer.lookup_transform('odom', 'vision', rospy.Time(0)))
        # Get the transform from vision to base_link
        trans_vision_to_base = zero_trans(tf_buffer.lookup_transform('vision', 'base_link', rospy.Time(0)))

        # Apply the transforms to the twist
        twist = msg.twist.twist
        twist_vec, twist_rot = twist2vec3(twist) 

        # Apply the base_link to vision transformation
        twist_vec = tf2_geometry_msgs.do_transform_vector3(twist_vec, trans_base_to_vision)
        twist_rot = tf2_geometry_msgs.do_transform_vector3(twist_rot, trans_base_to_vision)

        # Apply the odom to vision transformation
        twist_vec = tf2_geometry_msgs.do_transform_vector3(twist_vec, trans_odom_to_vision)
        twist_rot = tf2_geometry_msgs.do_transform_vector3(twist_rot, trans_odom_to_vision)

        # Apply the vision to base_link transformation
        twist_vec = tf2_geometry_msgs.do_transform_vector3(twist_vec, trans_vision_to_base)
        twist_rot = tf2_geometry_msgs.do_transform_vector3(twist_rot, trans_vision_to_base)

        # Create a new Odometry message with the corrected twist
        corrected_odom = Odometry()
        corrected_odom.header = msg.header
        corrected_odom.header.frame_id = 'vision'
        corrected_odom.child_frame_id = 'body'
        corrected_odom.pose = msg.pose
        corrected_odom.twist.twist = vec32twist(twist_vec, twist_rot)

        # Publish the corrected odometry
        odom_pub.publish(corrected_odom)

    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        rospy.logwarn("Transformation lookup failed.")

def main():
    rospy.init_node('odometry_corrector')

    # Initialize tf2 buffer and listener
    tf_buffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tf_buffer)

    # Create the publisher for the corrected odometry
    global odom_pub
    odom_pub = rospy.Publisher('/spot/odometry_clean_corrected', Odometry, queue_size=10)

    # Subscribe to the /spot/odometry_corrected topic
    rospy.Subscriber('/spot/odometry_corrected', Odometry, odom_callback, tf_buffer)

    rospy.spin()

if __name__ == '__main__':
    main()

