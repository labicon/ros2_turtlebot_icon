import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, qos_profile_services_default
import numpy as np
import sys
import redis
import pickle  # For serializing the image data
from std_msgs.msg import Float32MultiArray, MultiArrayDimension
import datetime


class consensus_pub(Node):
    def __init__(self, bot_name):
        super().__init__('consensus_pub')
        # define some variable
        self.consensus_publish_waitTime = 15
        self.agent_i_old = None

        # Redis connection
        self.redis_client = redis.Redis(host='localhost', port=6379, db=0, decode_responses=False)
        
        self.publisher = self.create_publisher(Float32MultiArray, f'/{bot_name}/consensus', qos_profile_services_default)
        self.timer = self.create_timer(self.consensus_publish_waitTime, self.publish_data)  # calls the publish_data functionck every x seconds


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


def main(args=None):
    rclpy.init(args=args)

    bot_name = 'miriel'  # Default bot name
    if len(sys.argv) > 1:
        bot_name = sys.argv[1]
    
    my_node = consensus_pub(bot_name)

    print("consensus publisher is now running...")
    rclpy.spin(my_node)
    my_node.destroy_node()
    rclpy.shutdown()
    

if __name__ == '__main__':
    main()