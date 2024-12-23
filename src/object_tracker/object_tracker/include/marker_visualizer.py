import rclpy
import numpy as np
from rclpy.node import Node
from smrr_interfaces.msg import Entities
from visualization_msgs.msg import Marker, MarkerArray

class MarkerVisualizer:
    def __init__(self, node, subscriber_topic, publisher_topic, marker_frame, marker_color):
        self.node = node
        self.marker_frame = marker_frame
        self.marker_color = marker_color
        self.sub_ = self.node.create_subscription(Entities, subscriber_topic, self.entities_callback, 10)
        self.pub_ = self.node.create_publisher(MarkerArray, publisher_topic, 10)

        # Track the marker IDs to delete old markers
        self.previous_marker_ids = set()

    def entities_callback(self, msg):
        x_ = np.array(msg.x)
        y_ = np.array(msg.y)

        coords = np.vstack((x_, y_)).T

        marker_array = MarkerArray()
        current_marker_ids = set()

        # Delete all markers before adding new ones
        delete_marker = Marker()
        delete_marker.action = Marker.DELETEALL
        marker_array.markers.append(delete_marker)

        # Iterate through the coordinates and create markers
        for i, (person_x, person_y) in enumerate(coords):
            current_marker_ids.add(i)

            marker = Marker()
            marker.header.frame_id = self.marker_frame
            marker.header.stamp = self.node.get_clock().now().to_msg()
            marker.id = i
            marker.type = Marker.CYLINDER
            marker.action = Marker.ADD
            marker.pose.position.x = float(person_x)
            marker.pose.position.y = float(person_y)
            marker.pose.position.z = 0.0
            marker.scale.x = 0.25
            marker.scale.y = 0.25
            marker.scale.z = 0.001

            if (self.marker_color == "red"):
                marker.color.a = 1.0
                marker.color.r = 1.0
                marker.color.g = 0.0
                marker.color.b = 0.0
            elif (self.marker_color == "green"):
                marker.color.a = 1.0
                marker.color.r = 0.0
                marker.color.g = 1.0
                marker.color.b = 0.0

            marker_array.markers.append(marker)

        # Remove markers that are not in the current set of markers
        for old_marker_id in self.previous_marker_ids.difference(current_marker_ids):
            marker = Marker()
            marker.header.frame_id = self.marker_frame  # Use consistent frame for both adding and deleting
            marker.header.stamp = self.node.get_clock().now().to_msg()
            marker.id = old_marker_id
            marker.action = Marker.DELETE

            marker_array.markers.append(marker)

        # Publish the marker array
        self.pub_.publish(marker_array)

        # Update the previous marker IDs for the next callback
        self.previous_marker_ids = current_marker_ids

