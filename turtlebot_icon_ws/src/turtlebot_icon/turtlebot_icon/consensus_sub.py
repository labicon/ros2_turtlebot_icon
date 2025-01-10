import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
import sys
import redis
from std_msgs.msg import Float32MultiArray
import datetime
import time
import numpy  as np
import pickle  # For serializing the image data


class consensus_sub(Node):
    def __init__(self, other_bot_name):
        super().__init__('consensus_sub')
        # define some variable
        self.consensus_subwaitTime = 0.5

        # Redis connection
        self.redis_client = redis.Redis(host='localhost', port=6379, db=0, decode_responses=False)

        self.subscription4 = self.create_subscription(
            Float32MultiArray, f'/{other_bot_name}/consensus', self.consensus_callback, qos_profile_sensor_data )
    
    def consensus_callback(self, msg: Float32MultiArray):
        data = np.array(msg.data).reshape((msg.layout.dim[0].size, msg.layout.dim[1].size))
        theta_j = data[0, :]
        uncertainty_j = data[1, :]
        agent_j = {'theta_j':theta_j, 'uncertainty_j':uncertainty_j}
        agent_j_pickled = pickle.dumps(agent_j)
        self.redis_client.set('agent_j', agent_j_pickled)
        time.sleep(self.consensus_subwaitTime)
        print(f"{datetime.datetime.now()}:    consensus receive")



def main(args=None):
    rclpy.init(args=args)

    bot_name = 'miriel'  # Default bot name
    if len(sys.argv) > 1:
        other_bot_name = sys.argv[1]

    my_node = consensus_sub(other_bot_name)

    rclpy.spin(my_node)
    my_node.destroy_node()
    rclpy.shutdown()
    

if __name__ == '__main__':
    main()