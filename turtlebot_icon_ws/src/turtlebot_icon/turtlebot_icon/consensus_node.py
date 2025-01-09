import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
import cv2
from rclpy.qos import qos_profile_sensor_data, QoSProfile,  ReliabilityPolicy, HistoryPolicy, DurabilityPolicy 
import numpy as np
import sys
import redis
import pickle  # For serializing the image data
from std_msgs.msg import Float32MultiArray, MultiArrayDimension
import datetime
import time
class consensus_node(Node):
    def __init__(self, bot_name, other_bot_name):
        super().__init__('consensus_node')
        # define some variable
        self.consensus_publish_waitTime = 60. # don't send it fastetr than it can be processed
        self.agent_i_old = None

        # Redis connection
        self.redis_client = redis.Redis(host='localhost', port=6379, db=0, decode_responses=False)

        # for consensus
        qos_profile = QoSProfile(
                history=HistoryPolicy.KEEP_LAST, 
                reliability=ReliabilityPolicy.RELIABLE, 
                depth=1  ) 
        self.subscription4 = self.create_subscription(
            Float32MultiArray, f'/{other_bot_name}/consensus', self.consensus_callback, qos_profile)          
        self.publisher = self.create_publisher(Float32MultiArray, f'/{bot_name}/consensus', qos_profile)
        self.timer = self.create_timer(self.consensus_publish_waitTime, self.publish_data)  # calls the publish_data functionck every x seconds

    
    def consensus_callback(self, msg: Float32MultiArray):
        data = np.array(msg.data).reshape((msg.layout.dim[0].size, msg.layout.dim[1].size))
        theta_j = data[0, :]
        uncertainty_j = data[1, :]
        agent_j = {'theta_j':theta_j, 'uncertainty_j':uncertainty_j}
        agent_j_pickled = pickle.dumps(agent_j)
        self.redis_client.set('agent_j', agent_j_pickled)
        print(f"{datetime.datetime.now()}:    consensus receive")



    def publish_data(self):
        # publish
        agent_i =  self.redis_client.get('agent_i')
        if agent_i != self.agent_i_old:
            agent_i_pickled = pickle.loads(agent_i)
            theta_i = agent_i_pickled['theta_i']
            uncertainty_i = agent_i_pickled['uncertainty_i']

            # Combine the arrays into a 2D array
            combined_data = np.vstack((theta_i, uncertainty_i))  # Stack vertically

            # Create Float32MultiArray message
            msg = Float32MultiArray()
            msg.data = combined_data.flatten().tolist()  # Flatten and convert to list

            # Set the dimensions (assuming theta_i and uncertainty_i have the same length)
            """
                Given 2D array:
                    1 2 3
                    4 5 6
                    7 8 9
                Flatten it into 1D: 1 2 3 4 5 6 7 8 9
                dim[0] row: size = 3, stride = 3 (to get to next row, need to step over 3 elements in 1D array)
                dim[1] col: size = 3, stride = 1
            """
            msg.layout.dim = [
                MultiArrayDimension(label="row", size=2, stride=len(theta_i)),
                MultiArrayDimension(label="column", size=len(theta_i), stride=1)
            ]
            # Publish the message
            self.publisher.publish(msg)
            print(f"{datetime.datetime.now()}:    consensus send")
            self.agent_i_old = agent_i
    
    


    def send_data_to_redis(self, key, data):
        # Serialize the message using pickle
        pickled_data = pickle.dumps(data)
        # Store in Redis
        self.redis_client.set(key, pickled_data)




def main(args=None):
    rclpy.init(args=args)

    bot_name = 'miriel'  # Default bot name
    if len(sys.argv) > 1:
        bot_name = sys.argv[1]
        ohter_bot_name = sys.argv[2]

    my_node = consensus_node(bot_name, ohter_bot_name)

    try:
        rclpy.spin(my_node)
    except KeyboardInterrupt:
        my_node.destroy_node()
        rclpy.shutdown()
    

if __name__ == '__main__':
    main()