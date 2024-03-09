import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import urllib.request
import time
import threading
import socket

class CameraDriver(Node):
    def __init__(self):
        super().__init__('fishbot_camera')
        self.publisher_ = self.create_publisher(Image, 'fishbot_camera_raw', 10)
        self.cv_bridge = CvBridge()
        self.last_recv_img_time = 0

        self.thread_wait_connect = threading.Thread(target=self.wait_connect)
        self.thread_wait_connect.start()

    
    def wait_connect(self):
        self.udp_port = 8887
        self.udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.udp_socket.bind(('', self.udp_port))


        while True:
            data, addr = self.udp_socket.recvfrom(1024)
            # print(f"Received message from {addr}: {data.decode('utf-8')}")

            if time.time()-self.last_recv_img_time>=6.0:
                self.url = f'http://{addr[0]}:81/stream'
                print(f"start read image thread {self.url}")
                self.thread_readimg = threading.Thread(target=self.stream_image)
                self.thread_readimg.start()
                self.rate = 1

    def stream_image(self):
        self.stream = urllib.request.urlopen(self.url)
        self.stream_bytes = b' '
        self.count = 0
        self.last_time = time.time()
        while rclpy.ok():
            self.stream_bytes += self.stream.read(128)
            first = self.stream_bytes.find(b'\xff\xd8')
            last = self.stream_bytes.find(b'\xff\xd9')
            if first != -1 and last != -1:
                if last < first:
                    self.stream_bytes = self.stream_bytes[last + 2:]
                else:
                    jpg = self.stream_bytes[first:last + 2]
                    self.stream_bytes = self.stream_bytes[last + 2:]
                    nimage = np.frombuffer(jpg, dtype=np.uint8)
                    image = cv2.imdecode(nimage, cv2.IMREAD_COLOR)
                    self.count += 1
                    self.last_recv_img_time = time.time()
                    if int(time.time() - self.last_time) >= 5:
                        self.rate = self.count / (time.time() - self.last_time)
                        self.get_logger().info("平均帧率 %s" % str(self.rate))
                        self.last_time = time.time()
                        self.count = 0
                     

                    # 将图像转换为ROS图像消息并发布
                    ros_image = self.cv_bridge.cv2_to_imgmsg(image, "bgr8")
                    self.publisher_.publish(ros_image)


def main(args=None):
    rclpy.init(args=args)
    node = CameraDriver()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
