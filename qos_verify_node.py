import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import signal
import sys

count = 0
file_path = '/tmp/qos_verify_count.txt'

def signal_handler(sig, frame):
    with open(file_path, 'w') as f:
        f.write(str(count))
    print(f"[QoS验证] 收到消息数: {count}", flush=True)
    sys.exit(0)

class QoSVeifyNode(Node):
    def __init__(self):
        super().__init__('qos_verify_node')
        self.subscription = self.create_subscription(
            PointCloud2,
            '/wamv/sensors/lidars/front_lidar_sensor/points',
            self.listener_callback,
            rclpy.qos.QoSProfile(depth=10, reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT))
        self.subscription

    def listener_callback(self, msg):
        global count
        count += 1

def main(args=None):
    signal.signal(signal.SIGTERM, signal_handler)
    signal.signal(signal.SIGINT, signal_handler)
    rclpy.init(args=args)
    node = QoSVeifyNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
