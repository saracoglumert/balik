import random
import rclpy
from rclpy.node import Node

from std_msgs.msg import String

import time

def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node('random')
    publisher = node.create_publisher(String, 'balik_path', 10)
    
    types = ['TF','TB','RR','RL']
    while True:
        mytype  = random.choice(types)
        myvalue = random.randint(250,750)
        cmd = str(mytype + str(myvalue))
        msg = String()
        msg.data = cmd
        publisher.publish(msg)
        print(cmd)
        time.sleep(10)




if __name__ == "__main__":
    main()