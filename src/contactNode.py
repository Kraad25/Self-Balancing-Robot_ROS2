#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from sbr_pjt.msg import Contacts
from std_msgs.msg import Bool

class ContactNode(Node):
    def __init__(self):
        super().__init__('contact_node')
        self.get_logger().info("Filtering Specific Contacts")

        self.contact_sub = self.create_subscription(Contacts, '/contacts', self.contact_callback, 10)
        self.cleaned_contact_pub = self.create_publisher(Bool, '/base_contact', 10)

        self.game_over = False
        self.start_time = self.get_clock().now()

    def contact_callback(self, msg):
        current_time = self.get_clock().now()
        elapsed = (current_time - self.start_time).nanoseconds / 1e9

        self.get_logger().info(f"[DEBUG] Skipping check for 7s")
        if elapsed < 7.0: # in seconds
            return
        
        self.publish_if_base_contacts_ground(msg)

    def publish_if_base_contacts_ground(self, rawData):
        if not self.game_over and self.check_contact(rawData):
            self.game_over=True

        self.cleaned_contact_pub.publish(Bool(data=self.game_over))

    def check_contact(self, rawData):
        for contact in rawData.states:
            if ("base_link" in contact.info and "ground_plane" in contact.info):
                return True
        return False



if __name__ == '__main__':
    rclpy.init()
    node = ContactNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()