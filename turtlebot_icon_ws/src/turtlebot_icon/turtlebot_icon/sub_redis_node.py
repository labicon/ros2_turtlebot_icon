import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge
import cv2
from rclpy.qos import qos_profile_sensor_data
import numpy as np
import sys
import redis
import pickle  # For serializing the image data
from geometry_msgs.msg import PoseStamped


class sub_redis_node(Node):
    def __init__(self, bot_name):
        super().__init__('sub_redis_node')
        # define some variable
        self.bridge = CvBridge()
        self.maxDepth = 10*1000

        # Redis connection
        self.redis_client = redis.Redis(host='localhost', port=6379, db=0, decode_responses=False)

        # list for synchronization
        self.rgb_list = []
        self.rgb_time_list = []
        self.pose_list = []
        self.pose_time_list = []

        # images + VICON sub
        self.subscription1 = self.create_subscription(
            CompressedImage, f'/{bot_name}/oakd/rgb/image_raw/compressed', self.rgb_image_callback, qos_profile_sensor_data)
        self.subscription2 = self.create_subscription(
            Image, f'/{bot_name}/oakd/stereo/image_raw', self.depth_image_callback, qos_profile_sensor_data)
        self.subscription3 = self.create_subscription(
            PoseStamped, f'/vicon/{bot_name}/{bot_name}/pose', self.vicon_data_callback, qos_profile_sensor_data)

        

    def rgb_image_callback(self, msg: CompressedImage):
        rgb_image = self.bridge.compressed_imgmsg_to_cv2(msg, "bgr8") # get uint8

        self.rgb_list.append(rgb_image)
        self.rgb_time_list.append(msg.header.stamp.sec + 1e-9 * msg.header.stamp.nanosec)

        # Keep only the latest 50 images
        if len(self.rgb_list) > 20:
            self.rgb_list.pop(0)
            self.rgb_time_list.pop(0)




    def vicon_data_callback(self, msg: PoseStamped):
        self.pose_list.append(msg)
        self.pose_time_list.append(msg.header.stamp.sec + 1e-9 * msg.header.stamp.nanosec)

        # Keep only the latest xxx poses
        if len(self.pose_list) > 100:
            self.pose_list.pop(0)
            self.pose_time_list.pop(0)




    def depth_image_callback(self, msg: Image):
        depth_image = self.bridge.imgmsg_to_cv2(msg, "16UC1") # get unint16
        depth_image = np.clip(depth_image, 0, self.maxDepth)
        self.send_data_to_redis('depth_image', depth_image)
        
        depth_time = msg.header.stamp.sec + 1e-9 * msg.header.stamp.nanosec
        # Find closest timestamp for RGB image
        if len(self.rgb_time_list) > 0:
            closest_rgb_idx = np.argmin( np.abs(np.array(self.rgb_time_list) - depth_time) )
            closest_rgb_image = self.rgb_list[closest_rgb_idx]
            self.send_data_to_redis('rgb_image', cv2.cvtColor(closest_rgb_image, cv2.COLOR_BGR2RGB) )
            self.blend_and_show(closest_rgb_image, depth_image)

            print(f'depth:{msg.header.stamp.sec + 1e-9 * msg.header.stamp.nanosec}') #TODO:debug
            print(f'rgb:{self.rgb_time_list[closest_rgb_idx]}') #TODO:debug

        # Find closest timestamp for pose
        if len(self.pose_time_list) > 0:
            closest_pose_idx = np.argmin( np.abs(np.array(self.pose_time_list) - depth_time) )
            closest_pose = self.pose_list[closest_pose_idx]
            self.send_data_to_redis('vicon_data', closest_pose)

            print(f'depth:{msg.header.stamp.sec + 1e-9 * msg.header.stamp.nanosec}') #TODO:debug
            print(f'pose:{self.pose_time_list[closest_pose_idx]}') #TODO:debug

    


    def send_data_to_redis(self, key, data):
        # Serialize the message using pickle
        pickled_data = pickle.dumps(data)
        # Store in Redis
        self.redis_client.set(key, pickled_data)


    def blend_and_show(self, rgb_image, depth_image):
        if rgb_image is not None and depth_image is not None:
            depth_image_8bit = (depth_image * 255. / self.maxDepth).astype(np.uint8)
            depth_3ch = cv2.cvtColor(depth_image_8bit, cv2.COLOR_GRAY2BGR)
            depth_3ch = cv2.applyColorMap(depth_3ch, cv2.COLORMAP_JET)
            blended_image = cv2.addWeighted(rgb_image, 0.7, depth_3ch, 0.3, 0)
            cv2.imshow('Blended Image', blended_image)
            cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)

    bot_name = 'miriel'  # Default bot name
    if len(sys.argv) > 1:
        bot_name = sys.argv[1]

    image_blender = sub_redis_node(bot_name)
    rclpy.spin(image_blender)
    image_blender.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()