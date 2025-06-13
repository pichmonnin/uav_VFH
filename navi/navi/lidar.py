import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSDurabilityPolicy, QoSReliabilityPolicy

class LidarRelay(Node):
    def __init__(self):
        super().__init__('lidar_relay')

        self.sub = self.create_subscription(
            LaserScan, '/world/default/model/x500_lidar_2d_0/link/link/sensor/lidar_2d_v2/scan', self.scan_callback, 10  # <-- Queue size here
        )

    def scan_callback(self, msg):
        self.get_logger().info(f"Recieved scan with {(msg.ranges)} ranges"
                               f'min angle{msg.angle_min} , max angle :{msg.angle_max}'
                               )

def main():
    rclpy.init()
    node = LidarRelay()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()