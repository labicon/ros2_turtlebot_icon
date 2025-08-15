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
import math 

class wlsFilter:
    def __init__(self, _lambda, _sigma):
        """
            @ _lambda: Larger values force filtered disparity map edges 
            to adhere more to source image edges. Typical value is 8000. 
            @param _sigma: how sensitive the filtering process is to source image edges. 
            Large values can lead to disparity leakage.  Typical values range from 0.8 to 2.0. 
        """
        self._lambda = _lambda
        self._sigma = _sigma
        self.wlsFilter = cv2.ximgproc.createDisparityWLSFilterGeneric(False)

    def filter(self, disparity, guide_img, depthScaleFactor):
        self.wlsFilter.setLambda(self._lambda)
        self.wlsFilter.setSigmaColor(self._sigma)
        filteredDisp = self.wlsFilter.filter(disparity, guide_img)
        filteredDisp[filteredDisp == np.inf] = 0
        
        # Compute depth from disparity (32 levels)
        with np.errstate(divide='ignore'): # Should be safe to ignore div by zero here
            # raw depth values
            depthFrame = (depthScaleFactor / filteredDisp.astype(np.float32)).astype(np.uint16)

        return filteredDisp, depthFrame
    


class sub_redis_node(Node):
    def __init__(self, bot_name, save_path, filter_flag):
        super().__init__('sub_redis_node')
        # define some variable
        self.bridge = CvBridge()
        self.maxDepth = 5*1000
        self.baseline = 75 # mm 
        self.focal = 3000 # RGB original f, px
        #self.fov = 72.9 # deg

        # Redis connection
        self.redis_client = redis.Redis(host='localhost', port=6379, db=0, decode_responses=False)

        # list for synchronization
        self.rgb_list = []
        self.rgb_time_list = []
        self.pose_list = []
        self.pose_time_list = []
        self.rimg_list = deque(maxlen=20)
        self.rimg_time_list = deque(maxlen=20)

        # images + VICON sub
        self.subscription1 = self.create_subscription(
            CompressedImage, f'/{bot_name}/oakd/rgb/image_raw/compressed', self.rgb_image_callback, qos_profile_sensor_data)
        self.subscription2 = self.create_subscription(
            Image, f'/{bot_name}/oakd/stereo/image_raw', self.depth_image_callback, qos_profile_sensor_data)
        self.subscription3 = self.create_subscription(
            PoseStamped, f'/vicon/{bot_name}/{bot_name}/pose', self.vicon_data_callback, qos_profile_sensor_data)

        self.save_path = save_path
        self.counter = 0

        # for filter 
        self.wlsfilter =  wlsFilter(_lambda=7*1000, _sigma=5)
        self.filter_flag = filter_flag
        

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
        
        depth_time = msg.header.stamp.sec + 1e-9 * msg.header.stamp.nanosec
        
        # Save depth_image
        self.counter += 1 
        #cv2.imwrite(self.save_path+f'/depth_{self.counter}.png', depth_image)

        # Find closest timestamp for RGB image
        if len(self.rgb_time_list) > 0:
            closest_rgb_idx = np.argmin( np.abs(np.array(self.rgb_time_list) - depth_time) )
            closest_rgb_image = self.rgb_list[closest_rgb_idx]
            # apply filter 
            #focal = depth_image.shape[1] / (2. * math.tan(math.radians(self.fov / 2)))
            depthScaleFactor = self.baseline * self.focal
            filteredDisp, depthFrame = self.wlsfilter.filter(depth_image, closest_rgb_image, depthScaleFactor)
            #depthFrame = np.clip(depthFrame, a_min=0, a_max=self.maxDepth)
            # send data and vis 
            self.send_data_to_redis('rgb_image', cv2.cvtColor(closest_rgb_image, cv2.COLOR_BGR2RGB) )
            if self.filter_flag:
                depthFrame = filteredDisp #TODO
                depthFrame = np.clip(depthFrame, a_min=0, a_max=self.maxDepth)
                self.send_data_to_redis('depth_image', 
                                    (depthFrame).astype(np.uint16)) # convert to m
                self.blend_and_show(closest_rgb_image, depthFrame)
                #TODO
                print(f'mean = {np.mean(depthFrame)}')
            else: 
                self.send_data_to_redis('depth_image', depth_image)
                self.blend_and_show(closest_rgb_image, depth_image)
                print(f'mean = {np.mean(depth_image)}')
            # Save closest_rgb_image
            #cv2.imwrite(self.save_path+f'/rgb_{self.counter}.png', closest_rgb_image )




        # Find closest timestamp for pose
        if len(self.pose_time_list) > 0:
            closest_pose_idx = np.argmin( np.abs(np.array(self.pose_time_list) - depth_time) )
            closest_pose = self.pose_list[closest_pose_idx]
            self.send_data_to_redis('vicon_data', closest_pose)
            
            # Save closest_pose (assuming you have a way to serialize it)
            pose_filename =  self.save_path+f'/pose_{self.counter}.pkl' 
            with open(pose_filename, 'wb') as f:
                pickle.dump(closest_pose, f)
    

    def send_data_to_redis(self, key, data):
        # Serialize the message using pickle
        pickled_data = pickle.dumps(data)
        # Store in Redis
        self.redis_client.set(key, pickled_data)


    def blend_and_show(self, rgb_image, depth_image):
        if rgb_image is not None and depth_image is not None:
            depth_image_8bit = (depth_image * 255. / self.maxDepth).astype(np.uint8)
            colorDepth = cv2.applyColorMap(depth_image_8bit, cv2.COLORMAP_HOT)
            blended_image = cv2.addWeighted(rgb_image, 0.4, colorDepth, 0.6, 0)
            cv2.imshow('Depth Image', colorDepth)
            cv2.imshow('RGB', rgb_image)
            cv2.imshow('Blended', blended_image)
            cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    bot_name = sys.argv[1] if len(sys.argv) > 1 else 'miriel'
    filter_flag = int(sys.argv[2]) if len(sys.argv) > 2 else True
    save_path = sys.argv[3] if len(sys.argv) > 3 else 'default_exp'

    if len(sys.argv) > 1: # argv[0] is occupied
        bot_name = sys.argv[1]
    if len(sys.argv) > 2: 
        save_path = sys.argv[2]

    save_path = os.path.join('./saved_data', save_path)
    os.makedirs(save_path, exist_ok=True)
    image_blender = sub_redis_node(bot_name, save_path, filter_flag)
    rclpy.spin(image_blender)
    image_blender.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()