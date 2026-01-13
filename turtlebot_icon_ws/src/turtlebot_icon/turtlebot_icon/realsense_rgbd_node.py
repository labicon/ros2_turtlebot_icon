import rclpy
from rclpy.node import Node
from realsense2_camera_msgs.msg import RGBD
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge
import cv2
from rclpy.qos import qos_profile_sensor_data
import numpy as np
import sys
import redis
import pickle
import os
from collections import deque

class RealsenseRGBDNode(Node):
    def __init__(self, bot_name, save_path):
        super().__init__('realsense_rgbd_node')
        
        self.bridge = CvBridge()
        
        # Redis connection
        try:
            self.redis_client = redis.Redis(host='localhost', port=6379, db=0, decode_responses=False)
        except Exception as e:
            self.get_logger().error(f"Failed to connect to Redis: {e}")
            self.redis_client = None

        # Lists for synchronization (only needed for Vicon pose)
        self.pose_list = deque(maxlen=20)
        self.pose_time_list = deque(maxlen=20)

        # Subscriptions
        # RGBD message contains both RGB and Depth images
        self.rgbd_sub = self.create_subscription(
            RGBD, 
            f'/{bot_name}/D435_1/rgbd', 
            self.rgbd_callback, 
            qos_profile_sensor_data
        )
        
        self.vicon_sub = self.create_subscription(
            PoseStamped, 
            f'/vicon/{bot_name}/{bot_name}/pose', 
            self.vicon_data_callback, 
            qos_profile_sensor_data
        )
    
        self.save_path = save_path
        self.counter = 0
        self.maxDepth = 10 * 1000 # mm

        self.get_logger().info(f"RealsenseRGBDNode started for {bot_name}")

    def vicon_data_callback(self, msg: PoseStamped):
        time = msg.header.stamp.sec + 1e-9 * msg.header.stamp.nanosec
        self.pose_list.append(msg)
        self.pose_time_list.append(time)

    def rgbd_callback(self, msg: RGBD):
        try:
            rgb_image = self.bridge.imgmsg_to_cv2(msg.rgb, "bgr8")
            depth_image = self.bridge.imgmsg_to_cv2(msg.depth, "16UC1")
            
            current_time = msg.header.stamp.sec + 1e-9 * msg.header.stamp.nanosec
            
            self.counter += 1
            
            # Save images to disk
            if self.save_path is not None:
                cv2.imwrite(os.path.join(self.save_path, f'rgb_{self.counter}.png'), rgb_image)
                cv2.imwrite(os.path.join(self.save_path, f'depth_{self.counter}.png'), depth_image)

            # Send to Redis
            if self.redis_client:
                self.send_data_to_redis('rgb_image', cv2.cvtColor(rgb_image, cv2.COLOR_BGR2RGB))
                self.send_data_to_redis('depth_image', depth_image)

            # Find closest timestamp for pose
            if len(self.pose_time_list) > 0:
                closest_pose_idx = np.argmin(np.abs(np.array(self.pose_time_list) - current_time))
                closest_pose = self.pose_list[closest_pose_idx]
                
                if self.redis_client:
                    self.send_data_to_redis('vicon_data', closest_pose)
                
                # Save closest_pose
                if self.save_path is not None:
                    pose_filename = os.path.join(self.save_path, f'pose_{self.counter}.pkl')
                    with open(pose_filename, 'wb') as f:
                        pickle.dump(closest_pose, f)

            # Visualize
            self.blend_and_show(rgb_image, depth_image)

        except Exception as e:
            self.get_logger().error(f"Error in rgbd_callback: {e}")

    def send_data_to_redis(self, key, data):
        try:
            pickled_data = pickle.dumps(data)
            self.redis_client.set(key, pickled_data)
        except Exception as e:
            self.get_logger().error(f"Error sending data to Redis: {e}")

    def blend_and_show(self, rgb_image, depth_image):
        if rgb_image is not None and depth_image is not None:
            depth_image_8bit = (depth_image * 255. / self.maxDepth).astype(np.uint8)
            colorDepth = cv2.applyColorMap(depth_image_8bit, cv2.COLORMAP_HOT)
            
            # Ensure sizes match (resize depth to rgb if needed, though they should be same)
            if rgb_image.shape[:2] != colorDepth.shape[:2]:
                colorDepth = cv2.resize(colorDepth, (rgb_image.shape[1], rgb_image.shape[0]))

            blended_image = cv2.addWeighted(rgb_image, 0.4, colorDepth, 0.6, 0)
            
            cv2.imshow('Depth Image', colorDepth)
            cv2.imshow('RGB', rgb_image)
            cv2.imshow('Blended', blended_image)
            cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    
    bot_name = sys.argv[1] if len(sys.argv) > 1 else 'miriel'
    
    save_path = None
    if len(sys.argv) > 2:
        save_path = os.path.join('./saved_data', sys.argv[2])
        os.makedirs(save_path, exist_ok=True)

    node = RealsenseRGBDNode(bot_name, save_path)
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
