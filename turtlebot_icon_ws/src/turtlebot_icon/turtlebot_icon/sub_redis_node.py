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
import os
from collections import deque



class sub_redis_node(Node):
    def __init__(self, bot_name, save_path):
        super().__init__('sub_redis_node')
        # define some variable
        self.bridge = CvBridge()

        # Redis connection
        self.redis_client = redis.Redis(host='localhost', port=6379, db=0, decode_responses=False)

        # list for synchronization
        self.rgb_list = deque(maxlen=20)
        self.rgb_time_list = deque(maxlen=20)
        self.pose_list = deque(maxlen=20)
        self.pose_time_list = deque(maxlen=20)
        self.rimg_list = deque(maxlen=20)
        self.rimg_time_list =deque(maxlen=20)
        self.limg_list = deque(maxlen=20)
        self.limg_time_list = deque(maxlen=20)

        # images + VICON sub
        self.subscription1 = self.create_subscription(
            CompressedImage, f'/{bot_name}/oakd/rgb/image_raw/compressed', self.rgb_image_callback, qos_profile_sensor_data)
        self.subscription2 = self.create_subscription(
            Image, f'/{bot_name}/oakd/stereo/image_raw', self.depth_image_callback, qos_profile_sensor_data)
        self.subscription3 = self.create_subscription(
            PoseStamped, f'/vicon/{bot_name}/{bot_name}/pose', self.vicon_data_callback, qos_profile_sensor_data)
        
        self.subscription4 = self.create_subscription(
            CompressedImage, f'/{bot_name}/oakd/left/image_rect/compressed', self.left_image_callback, qos_profile_sensor_data)
        self.subscription5 = self.create_subscription(
            CompressedImage, f'/{bot_name}/oakd/right/image_rect/compressed', self.right_image_callback, qos_profile_sensor_data)

        self.save_path = save_path
        self.counter = 0

        self.maxDepth = 10*1000 # mm
        

    def rgb_image_callback(self, msg: CompressedImage):
        rgb_image = self.bridge.compressed_imgmsg_to_cv2(msg, "bgr8") # get uint8
        time = msg.header.stamp.sec + 1e-9 * msg.header.stamp.nanosec
        self.rgb_list.append(rgb_image)
        self.rgb_time_list.append(time)

    def left_image_callback(self, msg: CompressedImage):
        img = self.bridge.compressed_imgmsg_to_cv2(msg, "bgr8") # get uint8
        time = msg.header.stamp.sec + 1e-9 * msg.header.stamp.nanosec
        self.limg_list.append(img)
        self.limg_time_list.append(time)
    
    def right_image_callback(self, msg: CompressedImage):
        img = self.bridge.compressed_imgmsg_to_cv2(msg, "bgr8") # get uint8
        time = msg.header.stamp.sec + 1e-9 * msg.header.stamp.nanosec
        self.rimg_list.append(img)
        self.rimg_time_list.append(time)

    def vicon_data_callback(self, msg: PoseStamped):
        time = msg.header.stamp.sec + 1e-9 * msg.header.stamp.nanosec
        self.pose_list.append(msg)
        self.pose_time_list.append(time)



    def depth_image_callback(self, msg: Image):
        depth_image = self.bridge.imgmsg_to_cv2(msg, "16UC1") # get unint16
        depth_time = msg.header.stamp.sec + 1e-9 * msg.header.stamp.nanosec
        
        # Save depth_image
        self.counter += 1 
        if self.save_path is not None:
            cv2.imwrite(self.save_path+f'/depth_{self.counter}.png', depth_image)

        # Find closest timestamp for RGB image
        if len(self.rgb_time_list) > 0 and len(self.limg_time_list) > 0 and len(self.rimg_time_list) > 0:
            closest_rgb_idx = np.argmin( np.abs(np.array(self.rgb_time_list) - depth_time) )
            closest_rgb_image = self.rgb_list[closest_rgb_idx]

            closest_limg_idx = np.argmin( np.abs(np.array(self.limg_time_list) - depth_time) )
            closest_limg = self.limg_list[closest_limg_idx]

            closest_rimg_idx = np.argmin( np.abs(np.array(self.rimg_time_list) - depth_time) )
            closest_rimg = self.rimg_list[closest_rimg_idx]
            
            self.send_data_to_redis('rgb_image', cv2.cvtColor(closest_rgb_image, cv2.COLOR_BGR2RGB) )
            self.send_data_to_redis('depth_image', depth_image)
            self.send_data_to_redis('l_img', closest_limg)
            self.send_data_to_redis('r_img', closest_rimg)

            self.blend_and_show(closest_rgb_image, depth_image, closest_limg, closest_rimg)

            if self.save_path is not None:
                cv2.imwrite(self.save_path+f'/rgb_{self.counter}.png', closest_rgb_image )

        # Find closest timestamp for pose
        if len(self.pose_time_list) > 0:
            closest_pose_idx = np.argmin( np.abs(np.array(self.pose_time_list) - depth_time) )
            closest_pose = self.pose_list[closest_pose_idx]
            self.send_data_to_redis('vicon_data', closest_pose)
            
            # Save closest_pose (assuming you have a way to serialize it)
            if self.save_path is not None:
                pose_filename =  self.save_path+f'/pose_{self.counter}.pkl' 
                with open(pose_filename, 'wb') as f:
                    pickle.dump(closest_pose, f)
        

    def send_data_to_redis(self, key, data):
        # Serialize the message using pickle
        pickled_data = pickle.dumps(data)
        # Store in Redis
        self.redis_client.set(key, pickled_data)


    def blend_and_show(self, rgb_image, depth_image, left_img, right_img):
        if rgb_image is not None and depth_image is not None:
            depth_image_8bit = (depth_image * 255. / self.maxDepth).astype(np.uint8)
            colorDepth = cv2.applyColorMap(depth_image_8bit, cv2.COLORMAP_HOT)
            blended_image = cv2.addWeighted(rgb_image, 0.4, colorDepth, 0.6, 0)
            cv2.imshow('Depth Image', colorDepth)
            cv2.imshow('RGB', rgb_image)
            cv2.imshow('Blended', blended_image)
            cv2.imshow('Left', left_img)
            cv2.imshow('Right', right_img)
            cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    bot_name = sys.argv[1] if len(sys.argv) > 1 else 'miriel'
    save_path = os.path.join('./saved_data', sys.argv[2]) if len(sys.argv) > 2 else None
    if save_path is not None:
        os.makedirs(save_path, exist_ok=True)

    image_blender = sub_redis_node(bot_name, save_path)
    
    rclpy.spin(image_blender)
    image_blender.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()