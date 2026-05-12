import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2


class LidarCloudNode(Node):
    def __init__(self):
        super().__init__("lidar_cloud_node")

        self.declare_parameter("input_topic", "/overhead_lidar/points")
        self.declare_parameter("output_topic", "/lidar/points_fixed")
        self.declare_parameter("frame_id", "overhead_lidar/link/overhead_lidar_sensor")

        self.input_topic = self.get_parameter("input_topic").value
        self.output_topic = self.get_parameter("output_topic").value
        self.frame_id = self.get_parameter("frame_id").value

        self.pub = self.create_publisher(PointCloud2, self.output_topic, 10)
        self.sub = self.create_subscription(
            PointCloud2,
            self.input_topic,
            self.cloud_cb,
            10
        )

        self.get_logger().info(f"Listening: {self.input_topic}")
        self.get_logger().info(f"Publishing: {self.output_topic}")
        self.get_logger().info(f"Using frame_id: {self.frame_id}")

    def cloud_cb(self, msg):
        # Gazebo bridge sometimes gives empty frame_id, RViz hates that
        msg.header.frame_id = self.frame_id
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = LidarCloudNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()