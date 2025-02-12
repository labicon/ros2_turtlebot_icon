import redis # for reading images from ROS2 nodes
import pickle # for loading data from redis
import torch 
import numpy as np

redis_client = redis.StrictRedis(host='localhost', port=6379, db=0, decode_responses=False)

# Get RGB image from Redis
rgb_data = redis_client.get('rgb_image')
rgb_image = pickle.loads(rgb_data)
rgb_image = torch.from_numpy(rgb_image.astype(np.float32) / 255.) # Normalize to [0, 1]

# Get depth image from Redis
depth_data = redis_client.get('depth_image')
depth_image = pickle.loads(depth_data)
depth_image = torch.from_numpy(depth_image.astype(np.float32) / 1000.) # mm to m

# Get pose from Redis 
pickled_vicon_data = redis_client.get('vicon_data') 
vicon_data = pickle.loads(pickled_vicon_data)  # vicon_data is now a PoseStamped object