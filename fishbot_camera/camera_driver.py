import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from rcl_interfaces.msg import ParameterDescriptor,IntegerRange
from rcl_interfaces.msg import SetParametersResult
import cv2
import numpy as np
import urllib.error  
import urllib.request
import time
import threading
import socket
  

class CameraDriver(Node):
    def __init__(self):
        super().__init__('fishbot_camera') # node 节点初始化命名
        self.publisher_ = self.create_publisher(Image, 'fishbot_camera_raw', 10)
        self.cv_bridge = CvBridge()
        self.last_recv_img_time = 0
        self.camera_ip = None
        # 声明参数
        # 图像质量
        self.declare_param_with_range('quality', 10 ,4,64)
        # 图像大小
        self.declare_param_with_range('framesize', 5 ,0,13)
        # LED亮度
        self.declare_param_with_range('led_intensity', 0 ,0,255)
        self.declare_param_with_range('hmirror', 0 ,0,1)
        self.declare_param_with_range('vflip', 0 ,0,1)
        self.declare_param_with_range('face_detect', 0 ,0,1)
        self.declare_param_with_range('special_effect', 0 ,0,6)
        self.thread_wait_connect = threading.Thread(target=self.wait_connect)
        self.thread_wait_connect.start()
        #self.print_p()
        self.print_param()
        #add_on_set_parameters_callback 是NOde 类方法, 每当为节点设置参数时调用的函数 parameter_callback
        self.add_on_set_parameters_callback(self.parameter_callback)

    # add by yyx for test in 20250608
    def print_param(self):
        parameters=self._parameters.keys()
        if not parameters:
            self.get_logger().info(f'param is null,类型 为{parameters}')
        for key in parameters:
            self.get_logger().info(f'参数名为{self._parameters[key].name},数值为：{self._parameters[key].value}')
    
    # add by yyx for test in 20250608
    def print_p(self):
        self.get_logger().info(f'{self._parameters["quality"],dir(self._parameters["quality"])}')



    def parameter_callback(self, parameters):
        for parameter in parameters:
            self.get_logger().info(f'参数 {parameter.name} 设置为：{parameter.value}')
            self.sync_param(parameter.name,parameter.value)
        return SetParametersResult(successful=True)
    
    # 收到参数改变请求，发送request同步该参数
    def sync_param(self,name,value):
        if self.camera_ip:
            url = f'http://{self.camera_ip}/control?var={name}&val={value}'
            try:  
                with urllib.request.urlopen(url) as response:  
                    return response.read().decode('utf-8')  # 读取并解码响应内容  
            except urllib.error.URLError as e:  
                self.get_logger().error(f"发生错误：{e}")  
                return None  
            time.sleep(0.1)

    #声明参数范围  
    def declare_param_with_range(self,name,value,start,end):
        pd = ParameterDescriptor()
        pd_range = IntegerRange()
        pd_range.from_value = start
        pd_range.to_value = end
        pd_range.step = 1
        pd.integer_range.append(pd_range)
        self.declare_parameter(name,value,pd) 
    
    #向camera 同步所有设置的参数，可以理解为与camera 连接后，初始化camera的参数
    def sync_all_param(self):
        self.sync_param('led_intensity',self.get_parameter('led_intensity').value)
        self.sync_param('quality',self.get_parameter('quality').value)
        self.sync_param('framesize',self.get_parameter('framesize').value)
        self.sync_param('hmirror',self.get_parameter('hmirror').value)
        self.sync_param('vflip',self.get_parameter('vflip').value)
        self.sync_param('face_detect',self.get_parameter('face_detect').value)
        self.sync_param('special_effect',self.get_parameter('special_effect').value)
        self.get_logger().info(f"sync_all_param start")

    def wait_connect(self):
        self.udp_port = 8887
        #创建UDP 套接字 AF_INET(IPV4)+SOCK_DGRAM(UDP) AF_INET6：IPV6，SOCK_STREAM：TCP
        self.udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.udp_socket.bind(('', self.udp_port))


        while True:
            data, addr = self.udp_socket.recvfrom(1024)
            # print(f"Received message from {addr}: {data.decode('utf-8')}")
            if time.time()-self.last_recv_img_time>=30.0:
                self.last_recv_img_time = time.time()
                self.camera_ip = addr[0]
                self.url = f'http://{self.camera_ip}:81/stream'
                self.get_logger().info(f"start read image thread {self.url}")
                # syc param 
                self.sync_all_param()
                #启动图像数据流，进行图像传输
                self.thread_readimg = threading.Thread(target=self.stream_image)
                self.thread_readimg.start()
                self.rate = 1

    def stream_image(self):
        self.stream = urllib.request.urlopen(self.url)
        self.stream_bytes = b' '
        self.count = 0
        self.last_time = time.time()
        self.last_recv_img_time = time.time()
        while rclpy.ok():
            # 读取图像数据流
            # 1种.如果一帧的数据超过128字节，则需要通过+进行数据连接，才能读取到图像数据的结束标志位
            # 2种.如果一帧的数据小于128字节，通过+的方式会混入下一帧图像的开始标志位
            self.stream_bytes += self.stream.read(128)
            #查找 JPEG 图像的开始和结束标志位
            # JPEG 图像的开始标志位为 b'\xff\xd8'，结束标志位为 b'\xff\xd9
            first = self.stream_bytes.find(b'\xff\xd8')
            last = self.stream_bytes.find(b'\xff\xd9')
            if first != -1 and last != -1:
                if last < first:
                    #如果结束标志在起始标志前，说明数据有问题，丢弃前面的数据
                    self.stream_bytes = self.stream_bytes[last + 2:]
                else:
                    # 提取完整的JPEG图像数据
                    jpg = self.stream_bytes[first:last + 2]
                    # 更新缓存，去掉已处理的数据(去掉已经读取的图像数据)
                    # 注意：如果数据流中有多帧图像，可能会出现多帧图像的情况
                    # 如前面所说的第二种情况，可能会混入下一帧图像的开始标志位
                    self.stream_bytes = self.stream_bytes[last + 2:]
                    # 将JPEG字节流转为numpy数组
                    nimage = np.frombuffer(jpg, dtype=np.uint8)
                    # 解码JPEG图像
                    # cv2.imdecode() 函数将JPEG字节流解码为OpenCV图像格式
                    image = cv2.imdecode(nimage, cv2.IMREAD_COLOR)
                    self.count += 1
                    self.last_recv_img_time = time.time()
                    # 每2秒统计一次平均帧率
                    if int(time.time() - self.last_time) >= 2:
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
