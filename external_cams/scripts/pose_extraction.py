#!/usr/bin/python3

import rospy
import numpy as np
import tf
from geometry_msgs.msg import (
    PointStamped,
    TransformStamped,
    Quaternion,
    PoseWithCovarianceStamped,
)

from nav_msgs.msg import Odometry

from std_msgs.msg import Float64

import tf
import tf2_ros
from tf.transformations import (
    euler_from_quaternion,
    quaternion_from_euler,
    quaternion_multiply,
)

from aruco_msgs.msg import MarkerArray


class DistanceCalculation(object):
    def __init__(self):
        # Get topics
        self.makers_topic = rospy.get_param(
            "~markers_topic", "/aruco_marker_publisher/markers"
        )
        self.auv_position_topic = rospy.get_param("~auv_position_topic", "auv_position")
        self.ds_position_topic = rospy.get_param("~ds_position_topic", "ds_position")
        self.distance_topic = rospy.get_param("~distance_topic", "distance")
        self.relative_pose_topic = rospy.get_param(
            "~relative_pose_topic", "relative_pose"
        )

        self.header_frame = rospy.get_param("~reference_frame", "cm_station/uw_camera")

        # self.camera_frame = rospy.get_param("~base_frame_2d", "cm_station/uw_camera")
        # self.cm_base_frame = rospy.get_param("~base_frame_2d", "cm_station/uw_camera")

        # Init tf
        self.listener = tf.TransformListener()
        # self.static_tf_bc = tf2_ros.StaticTransformBroadcaster()
        self.tf_bc = tf2_ros.TransformBroadcaster()
        # self.br = tf.TransformBroadcaster()
        self.transform_stamped = TransformStamped()

        self.tf_buffer = tf2_ros.Buffer()
        tf2_ros.TransformListener(self.tf_buffer)

        # Init subscribers
        self.thrust_cmd_sub = rospy.Subscriber(
            self.makers_topic, MarkerArray, self.markers_cb
        )

        # Publishers
        self.auv_position_pub = rospy.Publisher(
            self.auv_position_topic, Odometry, queue_size=1
        )
        self.ds_position_pub = rospy.Publisher(
            self.ds_position_topic, Odometry, queue_size=1
        )
        self.distance_pub = rospy.Publisher(self.distance_topic, Float64, queue_size=1)
        self.relative_pose_pub = rospy.Publisher(
            self.relative_pose_topic, Odometry, queue_size=1
        )

        # Init variables
        self.ds_position = None
        self.auv_position = None
        self.relative_position = Odometry()
        self.relative_position.twist.twist.linear.x = 0.0
        self.relative_position.twist.twist.linear.y = 0.0
        self.relative_position.twist.twist.linear.z = 0.0
        self.relative_position.twist.twist.angular.x = 0.0
        self.relative_position.twist.twist.angular.y = 0.0
        self.relative_position.twist.twist.angular.z = 0.0
        self.relative_position.twist.covariance = np.zeros(36)

        self.detected_pose = Odometry()
        self.detected_pose.twist.twist.linear.x = 0.0
        self.detected_pose.twist.twist.linear.y = 0.0
        self.detected_pose.twist.twist.linear.z = 0.0
        self.detected_pose.twist.twist.angular.x = 0.0
        self.detected_pose.twist.twist.angular.y = 0.0
        self.detected_pose.twist.twist.angular.z = 0.0
        self.detected_pose.twist.covariance = np.zeros(36)

        rospy.loginfo("Distance node started")

        rospy.spin()

    # region Call backs
    def markers_cb(self, markers_msg):
        """
        Callback for the markers
        Reads out the maker pose, publishes it as a static transform
        and calculates the differnece between the DS and AUV.
        """
        for marker in markers_msg.markers:
            if self.get_marker_link(marker.id) is not None:
                self.transform_stamped.transform.translation.x = (
                    marker.pose.pose.position.x
                )
                self.transform_stamped.transform.translation.y = (
                    marker.pose.pose.position.y
                )
                self.transform_stamped.transform.translation.z = (
                    marker.pose.pose.position.z
                )
                self.transform_stamped.transform.rotation = marker.pose.pose.orientation
                self.transform_stamped.header.frame_id = self.header_frame
                self.transform_stamped.child_frame_id = self.get_marker_link(marker.id)
                self.transform_stamped.header.stamp = rospy.Time.now()
                self.tf_bc.sendTransform(self.transform_stamped)

                # Copy detected marker into odometry message
                self.detected_pose.header.frame_id = self.header_frame
                self.detected_pose.child_frame_id = self.get_marker_link(marker.id)
                self.detected_pose.header.stamp = rospy.Time.now()
                self.detected_pose.pose = marker.pose

                if self.get_marker_link(marker.id) == "dc_station/base_link":
                    self.ds_position = np.array(
                        [
                            marker.pose.pose.position.x,
                            marker.pose.pose.position.y,
                            marker.pose.pose.position.z,
                        ]
                    )
                    self.ds_position_pub.publish(self.detected_pose)

                if self.get_marker_link(marker.id) == "sam/qr_link_optical_0":
                    self.auv_position = np.array(
                        [
                            marker.pose.pose.position.x,
                            marker.pose.pose.position.y,
                            marker.pose.pose.position.z,
                        ]
                    )
                    self.auv_position_pub.publish(self.detected_pose)
                    print("AUV detected")

                    relative_pose = self.tf_buffer.lookup_transform(
                        "sam/base_link_qr",
                        "ds/base_link",
                        rospy.Time(0),
                        rospy.Duration(1.0),
                    )

                    # Calculate relative pose
                    self.relative_position.header.frame_id = "sam/base_link_qr"
                    self.relative_position.child_frame_id = "ds/base_link"
                    self.relative_position.header.stamp = rospy.Time.now()
                    self.relative_position.pose.pose.position.x = (
                        relative_pose.transform.translation.x
                    )
                    self.relative_position.pose.pose.position.y = (
                        relative_pose.transform.translation.y
                    )
                    self.relative_position.pose.pose.position.z = (
                        relative_pose.transform.translation.z
                    )
                    self.relative_position.pose.pose.orientation.x = (
                        relative_pose.transform.rotation.x
                    )
                    self.relative_position.pose.pose.orientation.y = (
                        relative_pose.transform.rotation.y
                    )
                    self.relative_position.pose.pose.orientation.z = (
                        relative_pose.transform.rotation.z
                    )
                    self.relative_position.pose.pose.orientation.w = (
                        relative_pose.transform.rotation.w
                    )


                    # Publish relative pose
                    self.relative_pose_pub.publish(self.relative_position)

                    if self.auv_position.any() and self.ds_position.any():
                        distance_auv_ds = np.linalg.norm(
                            self.ds_position - self.auv_position
                        )
                        print("Distance DS to AUV {0}".format(distance_auv_ds))

                        
                        self.distance_pub.publish(distance_auv_ds)

    # endregion

    # 102 back port
    # 107 back starboard
    def get_marker_link(self, x):
        """
        Returns the link name of the marker
        """
        return {
            110: "sam/qr_link_optical_0",
            103: "sam/qr_link_1",
            108: "dc_station/base_link",
            109: "test",
        }.get(x, None)


if __name__ == "__main__":
    rospy.init_node("distance_calculation")
    try:
        DistanceCalculation()
    except rospy.ROSInterruptException:
        rospy.logerr("Couldn't launch distnace_calculation node.")
